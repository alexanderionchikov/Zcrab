
// components/apds9960/apds9960.h
#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Стандартный I2C-адрес APDS-9960 (0x39 из даташита)
#define APDS9960_I2C_ADDR_DEFAULT  0x39

typedef struct {
    i2c_port_t i2c_port;
    uint8_t    i2c_addr;
} apds9960_t;

// Инициализация I2C и самого датчика в режиме RGBC
esp_err_t apds9960_init(apds9960_t *dev,
                        i2c_port_t port,
                        gpio_num_t sda_gpio,
                        gpio_num_t scl_gpio,
                        uint32_t i2c_freq_hz);

// Чтение 16-битных значений R/G/B/C
esp_err_t apds9960_read_color(apds9960_t *dev,
                              uint16_t *r,
                              uint16_t *g,
                              uint16_t *b,
                              uint16_t *c);

// Опционально: установка усиления (1/4/16/64x)
typedef enum {
    APDS9960_AGAIN_1X  = 0,
    APDS9960_AGAIN_4X  = 1,
    APDS9960_AGAIN_16X = 2,
    APDS9960_AGAIN_64X = 3,
} apds9960_again_t;

esp_err_t apds9960_set_again(apds9960_t *dev, apds9960_again_t gain);
// const char* detect_color(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
void read_and_print_color(apds9960_t *dev);
void apds9960_global_init(void);
const char* get_robot_color(void);

#ifdef __cplusplus
}
#endif
