// mf_motor.h
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "gy25.h"



#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int gpio_tx;           // ESP32 TX -> TJA1050 TXD
    int gpio_rx;           // ESP32 RX <- TJA1050 RXD
    uint32_t bitrate_bps;  // 1'000'000
} mf_can_config_t;

typedef struct {
    uint8_t left_ids[2];   // {2, 1}
    uint8_t right_ids[2];  // {3, 5}
    int8_t left_sign;      // +1
    int8_t right_sign;     // -1
    float wheel_diam_mm;   // 73.0
    float track_mm;        // 116.0
} mf_robot_config_t;

// CAN/Motor base
esp_err_t mf_can_init(const mf_can_config_t *cfg);
void      mf_can_deinit(void);

// Motor basic
esp_err_t mf_motor_run(uint8_t id);       // 0x88
esp_err_t mf_motor_stop(uint8_t id);      // 0x81
esp_err_t mf_motor_shutdown(uint8_t id);  // 0x80

esp_err_t mf_set_speed_dps(uint8_t id, float dps);
esp_err_t robot_init(const mf_robot_config_t *cfg);
esp_err_t robot_set_side_speed_dps(float left_dps, float right_dps);
esp_err_t robot_set_side_speed_cmps(float left_cm_s, float right_cm_s);
esp_err_t robot_set_linear_speed_cmps(float v_cm_s);
esp_err_t robot_drive_forward_time_cm(float cm, float v_cm_s);
esp_err_t robot_stop_all(void);
esp_err_t robot_drive_until_mean_deg(double target_deg, float speed_dps, uint32_t timeout_ms);
esp_err_t robot_drive_until_mean_cm(float target_cm, float speed_cm_s, uint32_t timeout_ms);
esp_err_t robot_turn_to_angle(float target_angle, float turn_speed_dps, float tolerance_deg, uint32_t timeout_ms);
esp_err_t robot_turn_by_absolute_angle(float delta_deg, float turnspeed_dps, float tolerance_deg, uint32_t timeout_ms);
esp_err_t robot_turn_by_absolute_angle(float delta_deg, float turnspeed_dps, float tolerance_deg, uint32_t timeout_ms);
esp_err_t robot_drive_cm_profile(float cm, float max_speed, float acc_dist_frac);
esp_err_t robot_drive_align_cm(float target_cm, float base_speed, uint32_t timeout_ms);
esp_err_t robot_turn_enc_then_gyro(float delta_deg,
                                   float turn_speed_dps,
                                   float enc_tol_deg,
                                   uint32_t align_ms,
                                   float kp, float kd,
                                   uint32_t timeout_ms);
void maze_step();




#ifdef __cplusplus
}
#endif
