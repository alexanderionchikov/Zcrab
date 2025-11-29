#include "vl53l0x.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

#define I2C_MASTER_SCL_IO 9
#define I2C_MASTER_SDA_IO 8
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define VL53L0X_ADDR_DEFAULT 0x29

#define REG_SYSRANGE_START 0x00
#define REG_SYSTEM_SEQUENCE_CONFIG 0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD 0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO 0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0B
#define REG_RESULT_INTERRUPT_STATUS 0x13
#define REG_RESULT_RANGE_STATUS 0x14
#define REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define REG_GPIO_HV_MUX_ACTIVE_HIGH 0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV 0x89
#define REG_I2C_SLAVE_DEVICE_ADDRESS 0x8A
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 0xB0
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_1 0xB1
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_2 0xB2
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_3 0xB3
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_4 0xB4
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_5 0xB5
#define REG_GLOBAL_CONFIG_REF_EN_START_SELECT 0xB6
#define REG_IDENTIFICATION_MODEL_ID 0xC0
#define REG_OSC_CALIBRATE_VAL 0xF8
#define REG_MSRC_CONFIG_CONTROL 0x60
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD 0x50
#define REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x51
#define REG_MSRC_CONFIG_TIMEOUT_MACROP 0x46
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI 0x71

const gpio_num_t VL53L0X_XSHUT_PINS[VL53L0X_NUM_SENSORS] = { GPIO_NUM_13, GPIO_NUM_12, GPIO_NUM_11 }; // справа, спереди, слева
const uint8_t VL53L0X_I2C_ADDRS[VL53L0X_NUM_SENSORS] = { 0x30, 0x31, 0x32 };

static uint8_t g_stop_var[VL53L0X_NUM_SENSORS] = {0};

// --- I2C helpers ---
static esp_err_t i2c_write_reg8(uint8_t addr7, uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write_reg16(uint8_t addr7, uint8_t reg, uint16_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, (uint8_t)(val >> 8), true);
    i2c_master_write_byte(cmd, (uint8_t)(val & 0xFF), true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_reg(uint8_t addr7, uint8_t reg, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static uint8_t read_u8(uint8_t addr7, uint8_t reg) { uint8_t v=0; i2c_read_reg(addr7, reg, &v, 1); return v; }
static uint16_t read_u16(uint8_t addr7, uint8_t reg) { uint8_t b[2]={0}; i2c_read_reg(addr7, reg, b, 2); return ((uint16_t)b[0]<<8)|b[1]; }

// --- I2C init ---
static void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void xshut_all_low(void) {
    for (size_t i = 0; i < VL53L0X_NUM_SENSORS; ++i) {
        gpio_reset_pin(VL53L0X_XSHUT_PINS[i]);
        gpio_set_direction(VL53L0X_XSHUT_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(VL53L0X_XSHUT_PINS[i], 0);
    }
}
static void xshut_one_high(size_t idx) {
    gpio_set_level(VL53L0X_XSHUT_PINS[idx], 1);
}

static bool vl53l0x_init_full(uint8_t addr7, uint8_t *out_stop_var) {
    if (read_u8(addr7, REG_IDENTIFICATION_MODEL_ID) != 0xEE) return false;
    i2c_write_reg8(addr7, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, read_u8(addr7, REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    i2c_write_reg8(addr7, 0x88, 0x00);
    i2c_write_reg8(addr7, 0x80, 0x01);
    i2c_write_reg8(addr7, 0xFF, 0x01);
    i2c_write_reg8(addr7, 0x00, 0x00);
    uint8_t stop_variable = read_u8(addr7, 0x91);
    i2c_write_reg8(addr7, 0x00, 0x01);
    i2c_write_reg8(addr7, 0xFF, 0x00);
    i2c_write_reg8(addr7, 0x80, 0x00);
    i2c_write_reg8(addr7, REG_MSRC_CONFIG_CONTROL, read_u8(addr7, REG_MSRC_CONFIG_CONTROL) | 0x12);
    i2c_write_reg16(addr7, REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7)));
    i2c_write_reg8(addr7, REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    if (out_stop_var) *out_stop_var = stop_variable;
    return true;
}

static bool assign_addresses_and_init(void) {
    xshut_all_low();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    for (size_t i = 0; i < VL53L0X_NUM_SENSORS; ++i) {
        xshut_one_high(i);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        i2c_write_reg8(VL53L0X_ADDR_DEFAULT, REG_I2C_SLAVE_DEVICE_ADDRESS, VL53L0X_I2C_ADDRS[i] & 0x7F);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (!vl53l0x_init_full(VL53L0X_I2C_ADDRS[i], &g_stop_var[i])) {
            ESP_LOGE("VL53L0X", "Init failed for sensor idx=%d with addr=0x%02X", (int)i, VL53L0X_I2C_ADDRS[i]);
            return false;
        }
        ESP_LOGI("VL53L0X", "Sensor idx=%d initialized with addr=0x%02X", (int)i, VL53L0X_I2C_ADDRS[i]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    return true;
}


// --- Single-shot read ---
static uint16_t vl53l0x_read_single_mm(uint8_t addr7, uint8_t stop_variable) {
    i2c_write_reg8(addr7, 0x80, 0x01);
    i2c_write_reg8(addr7, 0xFF, 0x01);
    i2c_write_reg8(addr7, 0x00, 0x00);
    i2c_write_reg8(addr7, 0x91, stop_variable);
    i2c_write_reg8(addr7, 0x00, 0x01);
    i2c_write_reg8(addr7, 0xFF, 0x00);
    i2c_write_reg8(addr7, 0x80, 0x00);

    i2c_write_reg8(addr7, REG_SYSRANGE_START, 0x01);

    int timeout = 300;
    while (timeout-- > 0 && (read_u8(addr7, REG_SYSRANGE_START) & 0x01)) vTaskDelay(2 / portTICK_PERIOD_MS);

    timeout = 400;
    while (timeout-- > 0 && ((read_u8(addr7, REG_RESULT_INTERRUPT_STATUS) & 0x07) == 0)) vTaskDelay(2 / portTICK_PERIOD_MS);

    uint16_t range = read_u16(addr7, REG_RESULT_RANGE_STATUS + 10);
    i2c_write_reg8(addr7, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}

// --- Публичный API ---
void vl53l0x_init_all(void) {
    i2c_master_init();
    vTaskDelay(50 / portTICK_PERIOD_MS);
    if (!assign_addresses_and_init()) {
        ESP_LOGE("VL53L0X", "VL53L0X init failed");
        while (1) vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

bool vl53l0x_read_mm(uint8_t sensor_idx, uint16_t *distance_mm) {
    if (sensor_idx >= VL53L0X_NUM_SENSORS) return false;
    uint16_t dist = vl53l0x_read_single_mm(VL53L0X_I2C_ADDRS[sensor_idx], g_stop_var[sensor_idx]);
    if (distance_mm) *distance_mm = dist;
    return true;
}

void vl53l0x_start_continuous(uint8_t sensor_idx, uint16_t period_ms) {
    uint8_t addr = VL53L0X_I2C_ADDRS[sensor_idx];
    i2c_write_reg8(addr, REG_SYSTEM_INTERMEASUREMENT_PERIOD, (uint8_t)period_ms);
    i2c_write_reg8(addr, REG_SYSRANGE_START, 0x02); 
}
bool vl53l0x_read_mm_fast(uint8_t sensor_idx, uint16_t *distance_mm) {
    uint8_t addr = VL53L0X_I2C_ADDRS[sensor_idx];
    uint16_t range = read_u16(addr, REG_RESULT_RANGE_STATUS + 10);
    i2c_write_reg8(addr, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    if (distance_mm) *distance_mm = range;
    return true;
}

