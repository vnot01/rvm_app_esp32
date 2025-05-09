#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h" // Jika perlu untuk debug LED
#include "esp_log.h"

#define UART_NUM UART_NUM_0 // Menggunakan UART0
#define BUF_SIZE (1024)
#define TASK_STACK_SIZE (2048)

static const char *TAG = "RVM_ESP32";

// Task untuk membaca data dari UART (dari RPi)
static void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    while (1) {
        // Menunggu event UART
        if (uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0) == ESP_OK) {
             ESP_LOGI(TAG, "UART driver installed for task.");
        } else {
            // Jika sudah terinstal oleh monitor atau task lain, ini mungkin error, tidak apa-apa lanjut
            // ESP_LOGW(TAG, "UART driver might already be installed.");
        }


        // Loop membaca data
        while(1) {
            // Baca data dari UART
            int len = uart_read_bytes(UART_NUM, dtmp, BUF_SIZE -1 , pdMS_TO_TICKS(20)); // Timeout 20ms
            if (len > 0) {
                dtmp[len] = '\0'; // Null terminate
                ESP_LOGI(TAG, "Diterima dari RPi/PC: %s", (char*)dtmp);

                // Proses perintah
                char* command = (char*)dtmp;
                // Hilangkan newline jika ada (sering dari Python .write(cmd + '\n'))
                command[strcspn(command, "\r\n")] = 0;


                if (strcmp(command, "PING_FROM_RPI") == 0) {
                    const char* pong_msg = "PONG_TO_RPI\n";
                    uart_write_bytes(UART_NUM, pong_msg, strlen(pong_msg));
                    ESP_LOGI(TAG, "Mengirim ke RPi/PC: %s", pong_msg);
                } else if (strcmp(command, "LED_ON_SIM_RPI") == 0) {
                    ESP_LOGI(TAG, "SIM: LED Menyala (diperintahkan RPi)");
                    const char* ack_msg = "ACK_LED_ON_RPI\n";
                    uart_write_bytes(UART_NUM, ack_msg, strlen(ack_msg));
                } else if (strcmp(command, "LED_OFF_SIM_RPI") == 0) {
                    ESP_LOGI(TAG, "SIM: LED Mati (diperintahkan RPi)");
                    const char* ack_msg = "ACK_LED_OFF_RPI\n";
                    uart_write_bytes(UART_NUM, ack_msg, strlen(ack_msg));
                } else {
                    const char* unk_msg = "UNKNOWN_CMD_RPI\n";
                    uart_write_bytes(UART_NUM, unk_msg, strlen(unk_msg));
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10)); // Cek setiap 10ms
        }
        if (dtmp) {
            free(dtmp);
        }
        // Hapus driver jika sudah tidak digunakan oleh task ini (biasanya tidak sampai sini jika loop terus)
        // uart_driver_delete(UART_NUM);
        // vTaskDelete(NULL);
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "ESP32 RVM Controller Startup.");
    ESP_LOGI(TAG, "Menggunakan UART%d untuk komunikasi RPi dan Monitor", UART_NUM);

    // Konfigurasi parameter UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT, // Atau UART_SCLK_APB
    };
    
    // Instalasi driver UART (sekali di app_main untuk logging awal)
    // Parameter terakhir (intr_alloc_flags) bisa 0 atau ESP_INTR_FLAG_IRAM
    // Jika task uart_event_task akan menginstal/menghapus driver, jangan instal di sini
    // atau pastikan hanya satu yang mengontrol.
    // Untuk kesederhanaan awal, kita biarkan task yang mencoba instalasi.
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
    // Atur pin UART (GPIO1 untuk TX0, GPIO3 untuk RX0 adalah default untuk UART0)
    // Jika Anda menggunakan UART lain, sesuaikan pinnya di sini
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    // UART_PIN_NO_CHANGE berarti menggunakan pin default yang sudah dikonfigurasi untuk UART tersebut
    // Untuk UART0: TXD_PIN (GPIO1), RXD_PIN (GPIO3)
    // Jika ingin eksplisit: uart_set_pin(UART_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    ESP_LOGI(TAG, "ESP32 Serial Slave (ESP-IDF) Ready.");
    ESP_LOGI(TAG, "Menunggu perintah dari Raspberry Pi...");

    // Buat task untuk menangani event UART
    xTaskCreate(uart_event_task, "uart_event_task", TASK_STACK_SIZE, NULL, 10, NULL);
}