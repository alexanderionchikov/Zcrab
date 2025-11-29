
#include "apds9960.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define APDS9960_REG_ENABLE  0x80
#define APDS9960_REG_ATIME   0x81
#define APDS9960_REG_WTIME   0x83
#define APDS9960_REG_CONFIG1 0x8D
#define APDS9960_REG_CONTROL 0x8F
#define APDS9960_REG_CONFIG2 0x90
#define APDS9960_REG_ID      0x92
#define APDS9960_REG_STATUS  0x93
#define APDS9960_REG_CDATAL  0x94 
#define APDS9960_REG_CDATAH  0x95
#define APDS9960_REG_RDATAL  0x96
#define APDS9960_REG_RDATAH  0x97
#define APDS9960_REG_GDATAL  0x98
#define APDS9960_REG_GDATAH  0x99
#define APDS9960_REG_BDATAL  0x9A
#define APDS9960_REG_BDATAH  0x9B

#define APDS9960_ENABLE_PON   (1 << 0) 
#define APDS9960_ENABLE_AEN   (1 << 1) 
#define APDS9960_STATUS_AVALID (1 << 0)

static const char *TAG = "APDS9960";
static apds9960_t apds9960_dev; 


static esp_err_t apds9960_write8(apds9960_t *dev, uint8_t reg, uint8_t value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err;
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, reg, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, value, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
cleanup:
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t apds9960_read8(apds9960_t *dev, uint8_t reg, uint8_t *value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t err;
    // Запись адреса регистра
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_WRITE, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, reg, true);
    if (err != ESP_OK) goto cleanup;
    // Повторный старт и чтение
    err = i2c_master_start(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_write_byte(cmd, (dev->i2c_addr << 1) | I2C_MASTER_READ, true);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_stop(cmd);
    if (err != ESP_OK) goto cleanup;
    err = i2c_master_cmd_begin(dev->i2c_port, cmd, pdMS_TO_TICKS(100));
cleanup:
    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t apds9960_read16(apds9960_t *dev, uint8_t reg_l, uint16_t *value) {
    uint8_t lo = 0, hi = 0;
    esp_err_t err;
    err = apds9960_read8(dev, reg_l, &lo);
    if (err != ESP_OK) return err;
    err = apds9960_read8(dev, reg_l + 1, &hi);
    if (err != ESP_OK) return err;
    *value = ((uint16_t)hi << 8) | lo;
    return ESP_OK;
}

esp_err_t apds9960_set_again(apds9960_t *dev, apds9960_again_t gain) {
    uint8_t control = 0;
    esp_err_t err = apds9960_read8(dev, APDS9960_REG_CONTROL, &control);
    if (err != ESP_OK) return err;
    control &= ~0x03; // Сброс битов AGAIN[1:0]
    control |= (gain & 0x03);
    return apds9960_write8(dev, APDS9960_REG_CONTROL, control);
}

esp_err_t apds9960_init(apds9960_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio, uint32_t i2c_freq_hz) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    dev->i2c_port = port;
    dev->i2c_addr = APDS9960_I2C_ADDR_DEFAULT;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_freq_hz,
        .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }
    // Проверка ID датчика
    uint8_t id = 0;
    err = apds9960_read8(dev, APDS9960_REG_ID, &id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ID reg: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "APDS-9960 ID: 0x%02X", id);
    err = apds9960_write8(dev, APDS9960_REG_ATIME, 0xDB);
    if (err != ESP_OK) return err;
    err = apds9960_set_again(dev, APDS9960_AGAIN_16X);
    if (err != ESP_OK) return err;
    uint8_t enable = 0;
    err = apds9960_read8(dev, APDS9960_REG_ENABLE, &enable);
    if (err != ESP_OK) return err;
    enable |= (APDS9960_ENABLE_PON | APDS9960_ENABLE_AEN);
    err = apds9960_write8(dev, APDS9960_REG_ENABLE, enable);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(120)); // Дать время на первый цикл
    return ESP_OK;
}

esp_err_t apds9960_read_color(apds9960_t *dev, uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    if (!dev || !r || !g || !b || !c) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    uint8_t status = 0;
    const TickType_t timeout_ticks = pdMS_TO_TICKS(200);
    TickType_t start = xTaskGetTickCount();
    do {
        err = apds9960_read8(dev, APDS9960_REG_STATUS, &status);
        if (err != ESP_OK) return err;
        if (status & APDS9960_STATUS_AVALID) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    } while ((xTaskGetTickCount() - start) < timeout_ticks);
    if (!(status & APDS9960_STATUS_AVALID)) return ESP_ERR_TIMEOUT;
    err = apds9960_read16(dev, APDS9960_REG_RDATAL, r);
    if (err != ESP_OK) return err;
    err = apds9960_read16(dev, APDS9960_REG_GDATAL, g);
    if (err != ESP_OK) return err;
    err = apds9960_read16(dev, APDS9960_REG_BDATAL, b);
    if (err != ESP_OK) return err;
    err = apds9960_read16(dev, APDS9960_REG_CDATAL, c);
    if (err != ESP_OK) return err;
    return ESP_OK;
}


const char* detect_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c)
{
    if (r > 700 && g > 700 && b > 700 && c > 700) return "white";
    // --- Чёрный ---
    if (r < 220 && g < 200 && b < 200 && c < 600) return "black";

    // --- Синий ---
    if (r > 400 && g > 400 && b > 500 && c > 700) return "blue";
    // --- Белый ---


    return "other";
}


void apds9960_global_init(void) {
    apds9960_init(&apds9960_dev, 0, 8, 9, 400000);
}

const char* get_robot_color(void) {
    uint16_t r, g, b, c;
    esp_err_t err = apds9960_read_color(&apds9960_dev, &r, &g, &b, &c);
    if (err == ESP_OK) {
        return detect_color(r, g, b, c);
    }
    return "error";
}
