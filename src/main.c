#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h" // Diperlukan untuk kontrol GPIO LED
#include "esp_log.h"

#define COMM_UART_NUM UART_NUM_0 
// Pin default untuk UART0 adalah GPIO1 (TX) dan GPIO3 (RX).

#define BUF_SIZE (1024)
#define TASK_STACK_SIZE (2048 * 2) // Naikkan sedikit untuk string ops dan logging
#define LED_BUILTIN_GPIO GPIO_NUM_2 // LED Built-in biasanya di GPIO2

static const char *TAG = "ESP32_RVM_LED_COMM";

static void uart_communication_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t install_result = uart_driver_install(COMM_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    if (install_result == ESP_OK) {
        ESP_LOGI(TAG, "UART%d driver installed for communication task.", COMM_UART_NUM);
    } else if (install_result == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "UART%d driver already installed.", COMM_UART_NUM);
    } else {
        ESP_LOGE(TAG, "Failed to install UART%d driver, error: %s", COMM_UART_NUM, esp_err_to_name(install_result));
        vTaskDelete(NULL);
        return;
    }
    
    ESP_ERROR_CHECK(uart_param_config(COMM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t* data_buffer = (uint8_t*) malloc(BUF_SIZE);
    if (data_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for UART buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART%d communication task started. Waiting for data from RPi...", COMM_UART_NUM);

    while (1) {
        int len = uart_read_bytes(COMM_UART_NUM, data_buffer, BUF_SIZE - 1, pdMS_TO_TICKS(100));
        
        if (len > 0) {
            data_buffer[len] = '\0';
            ESP_LOGI(TAG, "Diterima dari RPi: [%s] (len: %d)", (char*)data_buffer, len);

            char* command = (char*)data_buffer;
            size_t cmd_len = strlen(command);
            while (cmd_len > 0 && (command[cmd_len - 1] == '\n' || command[cmd_len - 1] == '\r')) {
                command[cmd_len - 1] = '\0';
                cmd_len--;
            }
            ESP_LOGI(TAG, "Perintah setelah diproses: [%s]", command);

            if (strcmp(command, "PING_FROM_RPI") == 0) {
                const char* pong_msg = "PONG_TO_RPI\n";
                uart_write_bytes(COMM_UART_NUM, pong_msg, strlen(pong_msg));
                ESP_LOGI(TAG, "Mengirim ke RPi: %s", pong_msg);
            } else if (strcmp(command, "LED_ON") == 0) { // Ganti nama perintah agar lebih generik
                gpio_set_level(LED_BUILTIN_GPIO, 1); // Nyalakan LED (biasanya HIGH untuk nyala)
                ESP_LOGI(TAG, "LED Menyala (diperintahkan RPi)");
                const char* ack_msg = "ACK_LED_ON\n";
                uart_write_bytes(COMM_UART_NUM, ack_msg, strlen(ack_msg));
            } else if (strcmp(command, "LED_OFF") == 0) { // Ganti nama perintah
                gpio_set_level(LED_BUILTIN_GPIO, 0); // Matikan LED (biasanya LOW untuk mati)
                ESP_LOGI(TAG, "LED Mati (diperintahkan RPi)");
                const char* ack_msg = "ACK_LED_OFF\n";
                uart_write_bytes(COMM_UART_NUM, ack_msg, strlen(ack_msg));
            } else {
                ESP_LOGI(TAG, "Perintah tidak dikenal: [%s]", command);
                const char* unk_msg = "UNKNOWN_CMD\n"; // Ganti nama perintah
                uart_write_bytes(COMM_UART_NUM, unk_msg, strlen(unk_msg));
            }
        }
    }
    free(data_buffer);
    uart_driver_delete(COMM_UART_NUM);
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32 RVM Controller Startup (ESP-IDF with LED Test).");
    ESP_LOGI(TAG, "Menggunakan UART%d untuk komunikasi dengan RPi.", COMM_UART_NUM);
    // Konfigurasi GPIO untuk LED Built-in
    gpio_reset_pin(LED_BUILTIN_GPIO);
    gpio_set_direction(LED_BUILTIN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_BUILTIN_GPIO, 0); // Pastikan LED mati saat startup
    ESP_LOGI(TAG, "LED Built-in (GPIO%d) dikonfigurasi sebagai OUTPUT dan dimatikan.", LED_BUILTIN_GPIO);

    xTaskCreate(uart_communication_task, "uart_comm_task", TASK_STACK_SIZE, NULL, 10, NULL); 
    ESP_LOGI(TAG, "app_main selesai. Task komunikasi UART berjalan.");
}

// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/uart.h"
// #include "driver/gpio.h" // Jika perlu untuk debug LED
// #include "esp_log.h"

// // Biasanya UART0 untuk logging/monitor, UART1 atau UART2 untuk komunikasi aplikasi
// // Mari kita coba gunakan UART1 untuk komunikasi dengan RPi
// // Jika Anda menggunakan UART0, maka monitor serial PlatformIO mungkin konflik.
// // Untuk ESP32 WROVER, pin default UART0 adalah GPIO1 (TX) dan GPIO3 (RX)
// // Pin default UART1 adalah GPIO10 (TX) dan GPIO9 (RX)
// // Pin default UART2 adalah GPIO17 (TX) dan GPIO16 (RX)
// // GANTI PIN SESUAI KEBUTUHAN JIKA MENGGUNAKAN UART SELAIN UART0
// #define UART_NUM UART_NUM_0 // Menggunakan UART0
// #define RPI_UART_NUM UART_NUM_1 // Pilih UART untuk komunikasi dengan RPi
// #define RPI_UART_TXD_PIN (GPIO_NUM_10) // Sesuaikan jika perlu
// #define RPI_UART_RXD_PIN (GPIO_NUM_9)  // Sesuaikan jika perlu
// #define RPI_UART_RTS_PIN (UART_PIN_NO_CHANGE)
// #define RPI_UART_CTS_PIN (UART_PIN_NO_CHANGE)

// #define BUF_SIZE (1024)
// #define TASK_STACK_SIZE (2048)
// #define LED_GPIO GPIO_NUM_2 // LED_BUILTIN pada banyak board ESP32

// static const char *TAG = "RVM_ESP32";

// // Task untuk membaca data dari UART (dari RPi)
// static void uart_event_task(void *pvParameters) {
//     uart_event_t event;
//     uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
//     while (1) {
//         // Menunggu event UART
//         if (uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0) == ESP_OK) {
//              ESP_LOGI(TAG, "UART driver installed for task.");
//         } else {
//             // Jika sudah terinstal oleh monitor atau task lain, ini mungkin error, tidak apa-apa lanjut
//             // ESP_LOGW(TAG, "UART driver might already be installed.");
//         }


//         // Loop membaca data
//         while(1) {
//             // Baca data dari UART
//             int len = uart_read_bytes(UART_NUM, dtmp, BUF_SIZE -1 , pdMS_TO_TICKS(20)); // Timeout 20ms
//             if (len > 0) {
//                 dtmp[len] = '\0'; // Null terminate
//                 ESP_LOGI(TAG, "Diterima dari RPi/PC: %s", (char*)dtmp);

//                 // Proses perintah
//                 char* command = (char*)dtmp;
//                 // Hilangkan newline jika ada (sering dari Python .write(cmd + '\n'))
//                 command[strcspn(command, "\r\n")] = 0;


//                 if (strcmp(command, "PING_FROM_RPI") == 0) {
//                     const char* pong_msg = "PONG_TO_RPI\n";
//                     uart_write_bytes(UART_NUM, pong_msg, strlen(pong_msg));
//                     ESP_LOGI(TAG, "Mengirim ke RPi/PC: %s", pong_msg);
//                 } else if (strcmp(command, "LED_ON_SIM_RPI") == 0) {
//                     ESP_LOGI(TAG, "SIM: LED Menyala (diperintahkan RPi)");
//                     const char* ack_msg = "ACK_LED_ON_RPI\n";
//                     uart_write_bytes(UART_NUM, ack_msg, strlen(ack_msg));
//                 } else if (strcmp(command, "LED_OFF_SIM_RPI") == 0) {
//                     ESP_LOGI(TAG, "SIM: LED Mati (diperintahkan RPi)");
//                     const char* ack_msg = "ACK_LED_OFF_RPI\n";
//                     uart_write_bytes(UART_NUM, ack_msg, strlen(ack_msg));
//                 } else {
//                     const char* unk_msg = "UNKNOWN_CMD_RPI\n";
//                     uart_write_bytes(UART_NUM, unk_msg, strlen(unk_msg));
//                 }
//             }
//             vTaskDelay(pdMS_TO_TICKS(10)); // Cek setiap 10ms
//         }
//         if (dtmp) {
//             free(dtmp);
//         }
//         // Hapus driver jika sudah tidak digunakan oleh task ini (biasanya tidak sampai sini jika loop terus)
//         // uart_driver_delete(UART_NUM);
//         // vTaskDelete(NULL);
//     }
// }


// void app_main(void) {
//     ESP_LOGI(TAG, "ESP32 RVM Controller Startup.");
//     ESP_LOGI(TAG, "Menggunakan UART%d untuk komunikasi RPi dan Monitor", UART_NUM);

//     // Konfigurasi parameter UART
//     uart_config_t uart_config = {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity    = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_DEFAULT, // Atau UART_SCLK_APB
//     };
    
//     // Instalasi driver UART (sekali di app_main untuk logging awal)
//     // Parameter terakhir (intr_alloc_flags) bisa 0 atau ESP_INTR_FLAG_IRAM
//     // Jika task uart_event_task akan menginstal/menghapus driver, jangan instal di sini
//     // atau pastikan hanya satu yang mengontrol.
//     // Untuk kesederhanaan awal, kita biarkan task yang mencoba instalasi.
//     // ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
//     ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    
//     // Atur pin UART (GPIO1 untuk TX0, GPIO3 untuk RX0 adalah default untuk UART0)
//     // Jika Anda menggunakan UART lain, sesuaikan pinnya di sini
//     ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
//     // UART_PIN_NO_CHANGE berarti menggunakan pin default yang sudah dikonfigurasi untuk UART tersebut
//     // Untuk UART0: TXD_PIN (GPIO1), RXD_PIN (GPIO3)
//     // Jika ingin eksplisit: uart_set_pin(UART_NUM, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

//     ESP_LOGI(TAG, "ESP32 Serial Slave (ESP-IDF) Ready.");
//     ESP_LOGI(TAG, "Menunggu perintah dari Raspberry Pi...");

//     // Buat task untuk menangani event UART
//     xTaskCreate(uart_event_task, "uart_event_task", TASK_STACK_SIZE, NULL, 10, NULL);
// }