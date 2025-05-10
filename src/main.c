#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

// --- Konfigurasi UART untuk Komunikasi dengan Raspberry Pi ---
#define COMM_UART_NUM UART_NUM_0 // Menggunakan UART0 (TX0=GPIO1, RX0=GPIO3)
#define UART_BUF_SIZE (256)      // Ukuran buffer UART

// --- Konfigurasi Pin GPIO untuk LED (Simulasi Aktuator & Indikator) ---
#define LED_INTERNAL_LIGHT GPIO_NUM_2 // LED Built-in (misalnya, untuk simulasi lampu internal kamera)
#define LED_SLOT_STATUS GPIO_NUM_4    // LED untuk status slot/operasi (misal, HIJAU jika OK, MERAH jika error)
#define LED_MECHANISM_A GPIO_NUM_5    // LED untuk simulasi mekanisme A (misal, sortir valid)
#define LED_MECHANISM_B GPIO_NUM_18   // LED untuk simulasi mekanisme B (misal, return item)
// Catatan: Sesuaikan nomor GPIO ini dengan pin yang benar-benar Anda gunakan jika memasang LED eksternal.
// Jika hanya menggunakan LED_BUILTIN_GPIO (GPIO2), maka hanya itu yang akan aktif.

#define TASK_STACK_SIZE (2048 * 2) // Ukuran stack untuk task UART

static const char *TAG = "ESP32_RVM_ACTUATOR"; // Tag untuk logging ESP32

/**
 * @brief Mengirim pesan respons/acknowledgment ke Raspberry Pi melalui UART.
 *
 * @param message Pesan string yang akan dikirim (akan ditambahkan newline).
 */
void send_response_to_rpi(const char *message)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s\n", message);
    uart_write_bytes(COMM_UART_NUM, buffer, strlen(buffer));
    ESP_LOGI(TAG, "TX to RPi: [%s]", message);
}

/**
 * @brief Task FreeRTOS untuk menangani komunikasi UART dan kontrol aktuator (LED).
 */
static void uart_actuator_control_task(void *pvParameters)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t install_result = uart_driver_install(COMM_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (install_result != ESP_OK && install_result != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Gagal instalasi driver UART%d: %s", COMM_UART_NUM, esp_err_to_name(install_result));
        vTaskDelete(NULL);
        return;
    }
    ESP_ERROR_CHECK(uart_param_config(COMM_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(COMM_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    uint8_t *data_buffer = (uint8_t *)malloc(UART_BUF_SIZE);
    if (data_buffer == NULL)
    {
        ESP_LOGE(TAG, "Gagal alokasi memori untuk data_buffer UART");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART Actuator Task dimulai. Menunggu perintah dari RPi...");

    while (1)
    {
        int len = uart_read_bytes(COMM_UART_NUM, data_buffer, UART_BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0)
        {
            data_buffer[len] = '\0';
            char *command_full = (char *)data_buffer;

            size_t cmd_full_len = strlen(command_full);
            while (cmd_full_len > 0 && (command_full[cmd_full_len - 1] == '\n' || command_full[cmd_full_len - 1] == '\r'))
            {
                command_full[cmd_full_len - 1] = '\0';
                cmd_full_len--;
            }
            ESP_LOGI(TAG, "RX from RPi: [%s]", command_full);

            char command_prefix[32] = {0};
            char command_payload[64] = {0};

            char *colon_ptr = strchr(command_full, ':');
            if (colon_ptr != NULL)
            {
                size_t prefix_len = colon_ptr - command_full;
                if (prefix_len < sizeof(command_prefix))
                {
                    strncpy(command_prefix, command_full, prefix_len);
                    command_prefix[prefix_len] = '\0';
                }
                else
                { // Prefix terlalu panjang, ambil sebagian
                    strncpy(command_prefix, command_full, sizeof(command_prefix) - 1);
                    command_prefix[sizeof(command_prefix) - 1] = '\0';
                }
                strncpy(command_payload, colon_ptr + 1, sizeof(command_payload) - 1);
                ESP_LOGD(TAG, "Parsed Prefix: [%s], Payload: [%s]", command_prefix, command_payload);
            }
            else
            {
                strncpy(command_prefix, command_full, sizeof(command_prefix) - 1);
                ESP_LOGD(TAG, "Parsed Command (no payload): [%s]", command_prefix);
            }

            // --- Logika Pemrosesan Perintah ---
            // Matikan LED status umum sebelum set yang baru (jika LED_SLOT_STATUS dipakai untuk banyak hal)
            // gpio_set_level(LED_SLOT_STATUS, 0);

            if (strcmp(command_prefix, "PING_RPI") == 0)
            {
                send_response_to_rpi("PONG_ESP32");
            }
            // --- Perintah Kontrol Slot (Simulasi dengan LED_SLOT_STATUS) ---
            else if (strcmp(command_prefix, "SLOT_OPEN") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Membuka Slot");
                gpio_set_level(LED_SLOT_STATUS, 1); // Nyalakan LED Slot (misal, HIJAU)
                send_response_to_rpi("ACK_SLOT_OPEN");
            }
            else if (strcmp(command_prefix, "SLOT_CLOSE") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Menutup Slot");
                gpio_set_level(LED_SLOT_STATUS, 0); // Matikan LED Slot
                send_response_to_rpi("ACK_SLOT_CLOSED");
            }
            // --- Perintah Kontrol Lampu Internal (Simulasi dengan LED_INTERNAL_LIGHT) ---
            else if (strcmp(command_prefix, "INTERNAL_LIGHT_ON") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Lampu Internal ON");
                gpio_set_level(LED_INTERNAL_LIGHT, 1);
                send_response_to_rpi("ACK_LIGHT_ON");
            }
            else if (strcmp(command_prefix, "INTERNAL_LIGHT_OFF") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Lampu Internal OFF");
                gpio_set_level(LED_INTERNAL_LIGHT, 0);
                send_response_to_rpi("ACK_LIGHT_OFF");
            }
            // --- Perintah Kontrol Mekanisme (Simulasi dengan LED_MECHANISM_A/B) ---
            else if (strcmp(command_prefix, "SORT_VALID_ITEM") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Sortir Item Valid");
                gpio_set_level(LED_MECHANISM_A, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_MECHANISM_A, 0);
                send_response_to_rpi("ACK_SORTED");
            }
            else if (strcmp(command_prefix, "RETURN_REJECTED_ITEM") == 0)
            {
                ESP_LOGI(TAG, "AKSI: Kembalikan Item Ditolak");
                gpio_set_level(LED_MECHANISM_B, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_MECHANISM_B, 0);
                send_response_to_rpi("ACK_ITEM_RETURNED");
            }
            // --- Perintah Indikasi Status (Mengontrol LED_INTERNAL_LIGHT atau LED_SLOT_STATUS) ---
            else if (strcmp(command_prefix, "INDICATE_STATUS_IDLE") == 0)
            {
                ESP_LOGI(TAG, "STATUS: IDLE");
                gpio_set_level(LED_INTERNAL_LIGHT, 0); // Lampu proses mati
                gpio_set_level(LED_SLOT_STATUS, 0);    // LED Slot (misal, hijau) mati atau LED status khusus IDLE
                send_response_to_rpi("ACK_STATUS_INDICATED");
            }
            else if (strcmp(command_prefix, "INDICATE_STATUS_PROCESSING") == 0)
            {
                ESP_LOGI(TAG, "STATUS: PROCESSING");
                gpio_set_level(LED_INTERNAL_LIGHT, 1); // Lampu proses nyala (simulasi)
                send_response_to_rpi("ACK_STATUS_INDICATED");
            }
            else if (strcmp(command_prefix, "INDICATE_STATUS_SUCCESS") == 0)
            {
                ESP_LOGI(TAG, "STATUS: SUCCESS");
                gpio_set_level(LED_INTERNAL_LIGHT, 0);
                gpio_set_level(LED_SLOT_STATUS, 1); // Misal, LED Slot Hijau nyala
                vTaskDelay(pdMS_TO_TICKS(1000));    // Tampilkan sebentar
                gpio_set_level(LED_SLOT_STATUS, 0); // Lalu matikan atau kembali ke idle
                send_response_to_rpi("ACK_STATUS_INDICATED");
            }
            else if (strcmp(command_prefix, "INDICATE_STATUS_REJECTED") == 0)
            {
                ESP_LOGI(TAG, "STATUS: REJECTED");
                gpio_set_level(LED_INTERNAL_LIGHT, 0);
                gpio_set_level(LED_SLOT_STATUS, 1); // Misal, LED Slot Merah nyala (jika pakai RGB atau LED terpisah)
                                                    // Jika hanya satu LED status, mungkin berkedip
                vTaskDelay(pdMS_TO_TICKS(1000));
                gpio_set_level(LED_SLOT_STATUS, 0);
                send_response_to_rpi("ACK_STATUS_INDICATED");
            }
            else if (strcmp(command_prefix, "INDICATE_STATUS_ERROR") == 0)
            {
                ESP_LOGI(TAG, "STATUS: ERROR");
                gpio_set_level(LED_INTERNAL_LIGHT, 0);
                gpio_set_level(LED_SLOT_STATUS, 1); // Misal, LED Slot Merah berkedip cepat
                // TODO: Implementasi kedipan LED jika diperlukan
                send_response_to_rpi("ACK_STATUS_INDICATED");
            }
            else
            {
                ESP_LOGW(TAG, "Perintah tidak dikenal dari RPi: [%s]", command_full);
                send_response_to_rpi("UNKNOWN_CMD_ESP");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    free(data_buffer);
    uart_driver_delete(COMM_UART_NUM);
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 Actuator Controller v0.5 (ESP-IDF).");

    gpio_config_t io_conf = {}; // Inisialisasi dengan zero
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    // Gabungkan semua pin LED yang akan digunakan sebagai output
    io_conf.pin_bit_mask = (1ULL << LED_INTERNAL_LIGHT) | (1ULL << LED_SLOT_STATUS) | (1ULL << LED_MECHANISM_A) | (1ULL << LED_MECHANISM_B);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // Set semua LED ke kondisi awal (mati)
    gpio_set_level(LED_INTERNAL_LIGHT, 0);
    gpio_set_level(LED_SLOT_STATUS, 0);
    gpio_set_level(LED_MECHANISM_A, 0);
    gpio_set_level(LED_MECHANISM_B, 0);

    ESP_LOGI(TAG, "GPIOs LEDs initialized (OFF).");

    xTaskCreate(uart_actuator_control_task, "uart_actuator_task", TASK_STACK_SIZE, NULL, 10, NULL);
    ESP_LOGI(TAG, "app_main finished. UART Actuator Task is running.");
}