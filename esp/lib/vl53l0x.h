#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#define VL53L0X_NUM_SENSORS 3

extern const gpio_num_t VL53L0X_XSHUT_PINS[VL53L0X_NUM_SENSORS];
extern const uint8_t VL53L0X_I2C_ADDRS[VL53L0X_NUM_SENSORS];

void vl53l0x_init_all(void);
bool vl53l0x_read_mm(uint8_t sensor_idx, uint16_t *distance_mm);
void vl53l0x_start_continuous(uint8_t sensor_idx, uint16_t period_ms);
bool vl53l0x_read_mm_fast(uint8_t sensor_idx, uint16_t *distance_mm);
