#include "gy25.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "GY25";

#define CMD_ASCII_OUTPUT   0x53
#define CMD_TILT_CALIB     0x54
#define CMD_HEADING_CALIB  0x55

static int gy25_uart_num = -1;
static char line_buffer[64];
static int line_index = 0;

bool gy25_init(int uart_num, int tx_pin, int rx_pin, int baud_rate) {
    gy25_uart_num = uart_num;
    
    // Конфигурация UART
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART pin config failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = uart_driver_install(uart_num, 1024, 0, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    gy25_calibrate_tilt();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    gy25_calibrate_heading();
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Установка режима ASCII вывода
    uint8_t cmd_ascii[2] = {0xA5, CMD_ASCII_OUTPUT};
    uart_write_bytes(gy25_uart_num, (const char*)cmd_ascii, sizeof(cmd_ascii));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "GY-25 initialized successfully");
    return true;
}

bool gy25_get_data(gy25_data_t* data) {
    if (gy25_uart_num < 0 || data == NULL) {
        return false;
    }
    
    uint8_t rx_buffer[128];
    int length = uart_read_bytes(gy25_uart_num, rx_buffer, sizeof(rx_buffer) - 1, 20 / portTICK_PERIOD_MS);
    
    if (length > 0) {
        for (int i = 0; i < length; i++) {
            if (rx_buffer[i] == '\n') {
                line_buffer[line_index] = '\0';
                
                if (strncmp(line_buffer, "#YPR=", 5) == 0) {
                    char* token = strtok(line_buffer + 5, ",");
                    if (token) data->yaw = atof(token);
                    
                    token = strtok(NULL, ",");
                    if (token) data->pitch = atof(token);
                    
                    token = strtok(NULL, ",");
                    if (token) data->roll = atof(token);
                    
                    line_index = 0;
                    return true;
                }
                line_index = 0;
            } else if (line_index < sizeof(line_buffer) - 1) {
                line_buffer[line_index++] = rx_buffer[i];
            }
        }
    }
    
    return false;
}

void gy25_calibrate_tilt(void) {
    if (gy25_uart_num < 0) return;
    
    uint8_t cmd[2] = {0xA5, CMD_TILT_CALIB};
    uart_write_bytes(gy25_uart_num, (const char*)cmd, sizeof(cmd));
    ESP_LOGI(TAG, "Tilt calibration command sent");
}

void gy25_calibrate_heading(void) {
    if (gy25_uart_num < 0) return;
    
    uint8_t cmd[2] = {0xA5, CMD_HEADING_CALIB};
    uart_write_bytes(gy25_uart_num, (const char*)cmd, sizeof(cmd));
    ESP_LOGI(TAG, "Heading calibration command sent");
}

void gy25_deinit(void) {
    if (gy25_uart_num >= 0) {
        uart_driver_delete(gy25_uart_num);
        gy25_uart_num = -1;
    }
    ESP_LOGI(TAG, "GY-25 deinitialized");
}
static float prevyaw = 0.0f;
static bool firstread = true;
static float absangleaccum = 0.0f;

float gy25_get_absolute_angle()
{
    gy25_data_t data;
    if (!gy25_get_data(&data)) return absangleaccum;
    float yaw = data.yaw;
    if (firstread) {
        prevyaw = yaw;
        absangleaccum = yaw; 
        firstread = false;
        return absangleaccum;
    }

    float delta = yaw - prevyaw;
    if (delta > 180.0f)      delta -= 360.0f;
    else if (delta < -180.0f) delta += 360.0f;
    absangleaccum += delta;
    prevyaw = yaw;
    return absangleaccum;
}
