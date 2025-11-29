#include "mf_motor.h"
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "esp_check.h"
#include "vl53l0x.h" 
#include "apds9960.h"

#ifndef ESP_RETURN_ON_ERROR
#define ESP_RETURN_ON_ERROR(_expr, _tag, _msg) do {                    \
    esp_err_t __e = (_expr);                                           \
    if (__e != ESP_OK) {                                               \
        ESP_LOGE(_tag, "%s: %s", (_msg), esp_err_to_name(__e));        \
        return __e;                                                    \
    }                                                                  \
} while (0)
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TAG "MF"

#define MF_CAN_BASE_CMD 0x140U
#define MF_CAN_STD_ID(id) (MF_CAN_BASE_CMD + ((uint32_t)(id) & 0x7F))

static mf_robot_config_t g_robot = {
    .left_ids = {2, 1},
    .right_ids = {3, 5},
    .left_sign = +1,
    .right_sign = -1,      
    .wheel_diam_mm = 69.0f,
    .track_mm = 160.0f,
};

static bool g_can_started = false;


static esp_err_t can_send(uint32_t std_id, const uint8_t data[8])
{
    twai_message_t m = {0};
    m.identifier = std_id;
    m.data_length_code = 8;
    m.extd = 0; // standard frame
    m.rtr = 0;  // data frame
    memcpy(m.data, data, 8);
    return twai_transmit(&m, pdMS_TO_TICKS(50));
}

static esp_err_t can_send_cmd(uint8_t id, uint8_t cmd, const uint8_t payload7[7])
{
    uint8_t d[8] = {0};
    d[0] = cmd;
    if (payload7) memcpy(&d[1], payload7, 7);
    return can_send(MF_CAN_STD_ID(id), d);
}

static esp_err_t can_recv_match(uint8_t id, uint8_t expect_cmd, twai_message_t *out, uint32_t timeout_ms)
{
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (xTaskGetTickCount() < deadline) {
        twai_message_t m = {0};
        if (twai_receive(&m, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (!m.extd && !m.rtr && m.data_length_code == 8 &&
                m.identifier == MF_CAN_STD_ID(id) && m.data[0] == expect_cmd) {
                if (out) *out = m;
                return ESP_OK;
            }
        }
    }
    return ESP_ERR_TIMEOUT;
}


esp_err_t mf_can_init(const mf_can_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(cfg->gpio_tx, cfg->gpio_rx, TWAI_MODE_NORMAL);
    twai_timing_config_t  t = TWAI_TIMING_CONFIG_1MBITS(); 
    twai_filter_config_t  f = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g, &t, &f));
    ESP_ERROR_CHECK(twai_start());
    g_can_started = true;
    ESP_LOGI(TAG, "TWAI started: TX=%d RX=%d 1Mbps", cfg->gpio_tx, cfg->gpio_rx);
    return ESP_OK;
}

void mf_can_deinit(void)
{
    if (g_can_started) {
        twai_stop();
        twai_driver_uninstall();
        g_can_started = false;
    }
}

esp_err_t mf_motor_run(uint8_t id)
{
    uint8_t p[7] = {0};
    return can_send_cmd(id, 0x88, p);
}

esp_err_t mf_motor_stop(uint8_t id)
{
    uint8_t p[7] = {0};
    return can_send_cmd(id, 0x81, p);
}

esp_err_t mf_motor_shutdown(uint8_t id)
{
    uint8_t p[7] = {0};
    return can_send_cmd(id, 0x80, p);
}
esp_err_t mf_set_speed_dps(uint8_t id, float dps)
{
   
    int32_t sp = (int32_t)llroundf(dps * 100.0f);
    uint8_t p[7] = {0};
    p[3] = (uint8_t)(sp & 0xFF);
    p[4] = (uint8_t)((sp >> 8) & 0xFF);
    p[5] = (uint8_t)((sp >> 16) & 0xFF);
    p[6] = (uint8_t)((sp >> 24) & 0xFF);
    return can_send_cmd(id, 0xA2, p);
}

esp_err_t mf_read_angle_multi_deg(uint8_t id, double *deg)
{
    uint8_t p[7] = {0};
    ESP_RETURN_ON_ERROR(can_send_cmd(id, 0x92, p), TAG, "send 0x92 failed");
    twai_message_t m = {0};
    ESP_RETURN_ON_ERROR(can_recv_match(id, 0x92, &m, 100), TAG, "recv 0x92 timeout");

    uint64_t uv = 0;
    for (int i = 0; i < 7; ++i) {
        uv |= ((uint64_t)m.data[1 + i]) << (8 * i);
    }
    int64_t sv = ((int64_t)(uv << 8)) >> 8; 
    if (deg) *deg = (double)sv / 100.0;     
    return ESP_OK;
}

static esp_err_t mf_read_state2_speed_dps(uint8_t id, int16_t *speed_dps)
{
    uint8_t p[7] = {0};
    ESP_RETURN_ON_ERROR(can_send_cmd(id, 0x9C, p), TAG, "send 0x9C failed");
    twai_message_t m = {0};
    ESP_RETURN_ON_ERROR(can_recv_match(id, 0x9C, &m, 100), TAG, "recv 0x9C timeout");
    if (speed_dps) {
        int16_t sp = (int16_t)((int16_t)m.data[4] | ((int16_t)m.data[5] << 8)); // 1 dps LSB
        *speed_dps = sp;
    }
    return ESP_OK;
}


esp_err_t robot_init(const mf_robot_config_t *cfg)
{
    if (cfg) g_robot = *cfg;

    // Перевести все моторы в RUN
    for (int i = 0; i < 2; ++i) {
        mf_motor_run(g_robot.left_ids[i]);
        mf_motor_run(g_robot.right_ids[i]);
    }
    return ESP_OK;
}

static float cmps_to_dps(float cm_s, float wheel_diam_mm)
{
    // dps = 360 * v / (pi * D_cm)
    const float D_cm = wheel_diam_mm * 0.1f;
    return (float)(360.0f * (cm_s / (float)(M_PI * D_cm)));
}

esp_err_t robot_set_side_speed_dps(float left_dps, float right_dps)
{
    float l = left_dps  * (float)g_robot.left_sign;
    float r = right_dps * (float)g_robot.right_sign;

    esp_err_t e = ESP_OK;
    for (int i = 0; i < 2 && e == ESP_OK; ++i) {
        e = mf_set_speed_dps(g_robot.left_ids[i], l);
        if (e == ESP_OK) e = mf_set_speed_dps(g_robot.right_ids[i], r);
    }
    return e;
}

esp_err_t robot_set_side_speed_cmps(float left_cm_s, float right_cm_s)
{
    float ld = cmps_to_dps(left_cm_s,  g_robot.wheel_diam_mm);
    float rd = cmps_to_dps(right_cm_s, g_robot.wheel_diam_mm);
    return robot_set_side_speed_dps(ld, rd);
}

esp_err_t robot_set_linear_speed_cmps(float v_cm_s)
{
    return robot_set_side_speed_cmps(v_cm_s, v_cm_s);
}

esp_err_t robot_stop_all(void)
{
    esp_err_t e = ESP_OK;
    for (int i = 0; i < 2 && e == ESP_OK; ++i) {
        e = mf_motor_stop(g_robot.left_ids[i]);
        if (e == ESP_OK) e = mf_motor_stop(g_robot.right_ids[i]);
    }
    return e;
}

static esp_err_t read_all_angles_deg(double a[4])
{
    esp_err_t e = ESP_OK;
    e = mf_read_angle_multi_deg(g_robot.left_ids[0],  &a[0]); if (e != ESP_OK) return e;
    e = mf_read_angle_multi_deg(g_robot.left_ids[1],  &a[1]); if (e != ESP_OK) return e;
    e = mf_read_angle_multi_deg(g_robot.right_ids[0], &a[2]); if (e != ESP_OK) return e;
    e = mf_read_angle_multi_deg(g_robot.right_ids[1], &a[3]); return e;
}

esp_err_t robot_drive_until_mean_deg(double target_deg, float speed_dps, uint32_t timeout_ms)
{
    if (fabsf(speed_dps) < 1e-3f || fabs(target_deg) < 1e-6) return ESP_ERR_INVALID_ARG;

    // 1) Снять стартовые углы
    double a0[4] = {0};
    esp_err_t e = read_all_angles_deg(a0);
    if (e != ESP_OK) return e;

    // 2) Пустить постоянную скорость по 0xA2
    int dir = (target_deg >= 0.0) ? +1 : -1;
    float set_dps = fabsf(speed_dps) * (float)dir;
    e = robot_set_side_speed_dps(set_dps, set_dps);
    if (e != ESP_OK) return e;

    const double target_abs = fabs(target_deg);
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 60000);
    while (xTaskGetTickCount() < deadline) {
        double a[4] = {0};
        e = read_all_angles_deg(a);
        if (e != ESP_OK) break;

        double dL0 = (a[0] - a0[0]) * (double)g_robot.left_sign;
        double dL1 = (a[1] - a0[1]) * (double)g_robot.left_sign;
        double dR0 = (a[2] - a0[2]) * (double)g_robot.right_sign;
        double dR1 = (a[3] - a0[3]) * (double)g_robot.right_sign;

        double mean_fwd = 0.25 * (dL0 + dL1 + dR0 + dR1);

        if ((dir > 0 && mean_fwd >= target_abs) ||
            (dir < 0 && -mean_fwd >= target_abs)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    robot_stop_all();

    if (xTaskGetTickCount() >= (TickType_t)(deadline)) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

static double wheel_deg_for_cm(double cm, double wheel_diam_mm)
{
    const double D_cm = wheel_diam_mm * 0.1;
    return 360.0 * (cm / (M_PI * D_cm));
}

esp_err_t robot_drive_until_mean_cm(float target_cm, float speed_cm_s, uint32_t timeout_ms)
{
    if (fabsf(speed_cm_s) < 1e-3f || fabs(target_cm) < 1e-6f) return ESP_ERR_INVALID_ARG;
    int dir = (target_cm >= 0.0f) ? +1 : -1;

    double tgt_deg = wheel_deg_for_cm(fabs((double)target_cm), (double)g_robot.wheel_diam_mm) * (double)dir;
    float  sp_dps  = cmps_to_dps(fabsf(speed_cm_s), g_robot.wheel_diam_mm);
    return robot_drive_until_mean_deg(tgt_deg, sp_dps, timeout_ms);
}

float robot_get_gyro_yaw(void)
{
    gy25_data_t data = {0};
    if (gy25_get_data(&data))
        return data.yaw;
    return 0.0f;
}

static inline float wrap180f(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

static inline float ang_err_shortest(float desired_deg, float current_deg) {
    return wrap180f(desired_deg - current_deg);
}

esp_err_t robot_turn_gyro_yaw(float delta_angle_deg, float turn_speed_dps, uint32_t timeout_ms) {

    const float kP = 3.0f;            
    const float speed_min = 60.0f;     
    const float speed_max = fabsf(turn_speed_dps); 
    const float deadband_deg = 1.0f;   
    const uint32_t settle_ms = 200;    
    const TickType_t dt = pdMS_TO_TICKS(10);

    // Старт и цель
    float yaw_start = robot_get_gyro_yaw();
    float yaw_des   = wrap180f(yaw_start + delta_angle_deg);

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 60000);
    TickType_t stable_since = 0;

    while (xTaskGetTickCount() < deadline) {
        float yaw_now = robot_get_gyro_yaw();
        float err = ang_err_shortest(yaw_des, yaw_now);
        float aerr = fabsf(err);

        if (aerr <= deadband_deg) {
            if (stable_since == 0) stable_since = xTaskGetTickCount();
            robot_set_side_speed_dps(0.0f, 0.0f);
            if ((xTaskGetTickCount() - stable_since) >= pdMS_TO_TICKS(settle_ms)) break;
            vTaskDelay(dt);
            continue;
        } else {
            stable_since = 0;
        }

        float cmd = fminf(speed_max, fmaxf(speed_min, kP * aerr));
        float sgn = (err >= 0.0f) ? +1.0f : -1.0f;

        esp_err_t e = robot_set_side_speed_dps(-sgn * cmd, +sgn * cmd);
        if (e != ESP_OK) { robot_stop_all(); return e; }

        vTaskDelay(dt);
    }

    robot_set_side_speed_dps(0.0f, 0.0f);

    if (xTaskGetTickCount() >= deadline) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

esp_err_t robot_turn_to_angle(float target_angle, float turn_speed_dps, float tolerance_deg, uint32_t timeout_ms) {
    float current_angle = gy25_get_absolute_angle();
    ESP_LOGI("ROBOT", "Turning to %.2f°, start at %.2f°", target_angle, current_angle);

    // Определяем направление вращения
    int dir = (target_angle > current_angle) ? 1 : -1;

    // Запускаем вращение
    esp_err_t e = robot_set_side_speed_dps(-dir * turn_speed_dps, dir * turn_speed_dps);
    if (e != ESP_OK) return e;

    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (xTaskGetTickCount() < deadline) {
        current_angle = gy25_get_absolute_angle();
        float diff = fabsf(target_angle - current_angle);

        ESP_LOGI("ROBOT", "Current: %.2f°, Target: %.2f°, Δ=%.2f°", current_angle, target_angle, diff);

        // Проверяем, достиг ли угол цели в пределах допустимой погрешности
        if (diff <= tolerance_deg) {
            ESP_LOGI("ROBOT", "Target reached (%.2f° ±%.2f°)", current_angle, tolerance_deg);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    robot_stop_all();
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI("ROBOT", "Final angle: %.2f°", gy25_get_absolute_angle());
    return ESP_OK;
}


esp_err_t robot_turn_by_absolute_angle(float delta_deg, float turnspeed_dps, float tolerance_deg, uint32_t timeout_ms)
{
    float start_angle = gy25_get_absolute_angle();
    float target_angle = start_angle + delta_deg;

    return robot_turn_to_angle(target_angle, turnspeed_dps, tolerance_deg, timeout_ms);
}

#define MPI 3.14159265358979323846f

esp_err_t robot_drive_cm_profile(float cm, float max_speed, float acc_dist_frac) {
    if (cm < 1e-3f || max_speed < 1e-3f || acc_dist_frac < 0.01f || acc_dist_frac > 0.45f)
        return ESP_ERR_INVALID_ARG;

    float acc_dist = cm * acc_dist_frac;
    float const_dist = cm - 2 * acc_dist;

    uint32_t dt_ms = 10; // — очень маленький шаг для максимальной плавности
    float acc_time = acc_dist / (0.5f * max_speed); // t = 2s/v
    int acc_steps = (int)(acc_time * 1000.0f / dt_ms);

    // S-профиль разгон
    for (int i = 0; i < acc_steps; i++) {
        float phase = (float)(i + 1) / acc_steps; // от 0 до 1
        float sp = max_speed * sinf(0.5f * MPI * phase);
        robot_set_linear_speed_cmps(sp);
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
    // Равномерный участок
    float const_time = (const_dist > 0) ? const_dist / max_speed : 0;
    int const_steps = (int)(const_time * 1000.0f / dt_ms);
    for (int i = 0; i < const_steps; i++) {
        robot_set_linear_speed_cmps(max_speed);
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
    // S-профиль торможение
    for (int i = acc_steps; i > 0; i--) {
        float phase = (float)(i) / acc_steps; // от 1 до 0
        float sp = max_speed * sinf(0.5f * MPI * phase);
        robot_set_linear_speed_cmps(sp);
        vTaskDelay(pdMS_TO_TICKS(dt_ms));
    }
    robot_stop_all();
    return ESP_OK;
}

esp_err_t robot_drive_align_cm(float target_cm, float base_speed, uint32_t timeout_ms)
{
    double a0[4] = {0}, a[4] = {0};
    esp_err_t e = read_all_angles_deg(a0);
    if (e != ESP_OK) return e;

    float kp = 0.12f, kd = 0.3f;
    float prev_error = 0.0f;
    float traveled = 0.0f;
    const float deadband = 2.0f;
    static uint16_t prev_left = 100, prev_right = 100;

    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t deadline = start_tick + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 60000);

    while (traveled < target_cm && xTaskGetTickCount() < deadline) {
        const char* color = get_robot_color();
        if (color && strcmp(color, "black") == 0) {
            robot_set_side_speed_cmps(0.0f, 0.0f);
            robot_turn_enc_then_gyro(-180.0f, 420.0f, 2.0f, 500.0f, 1.0f, 9.8f, 3000); 
            robot_drive_until_mean_cm(8.0f, 45.0f, 3000);
            ESP_LOGI("ALIGN", "Чёрное обнаружено, остановка и выход");
            return ESP_OK; 
        }

        uint16_t left_raw = 0, right_raw = 0;
        vl53l0x_read_mm_fast(2, &left_raw);
        vl53l0x_read_mm_fast(0, &right_raw);

        uint16_t left = left_raw == 20 ? 180 : left_raw;
        uint16_t right = right_raw == 20 ? 180 : right_raw;
        prev_left = left;
        prev_right = right;

        if (left > 250) left = 180;
        if (right > 250) right = 180;
        // left = (left > 30) ? left - 50 : 0; 
        // right = (right > 70) ? right - 70 : 0;

        float error = (float)left - (float)right;
        float d_error = error - prev_error;
        prev_error = error;

        float u = (fabsf(error) < deadband) ? 0.0f : kp * error + kd * d_error;
        if (u > 25.0f) u = 25.0f;
        if (u < -25.0f) u = -25.0f;

        float left_speed = base_speed - u;
        float right_speed = base_speed + u;
        robot_set_side_speed_cmps(left_speed, right_speed);

        vTaskDelay(pdMS_TO_TICKS(10));

        read_all_angles_deg(a);
        double dL0 = (a[0] - a0[0]) * (double)g_robot.left_sign;
        double dL1 = (a[1] - a0[1]) * (double)g_robot.left_sign;
        double dR0 = (a[2] - a0[2]) * (double)g_robot.right_sign;
        double dR1 = (a[3] - a0[3]) * (double)g_robot.right_sign;
        double mean_fwd = 0.25 * (dL0 + dL1 + dR0 + dR1);

        float wheel_diam_mm = g_robot.wheel_diam_mm;
        float cm = mean_fwd * (M_PI * (wheel_diam_mm * 0.1f)) / 360.0f;
        traveled = fabsf(cm);
    }

    // robot_set_side_speed_cmps(0.0f, 0.0f);
    // vTaskDelay(pdMS_TO_TICKS(10));


    return (traveled >= target_cm) ? ESP_OK : ESP_FAIL;
}


esp_err_t robot_turn_enc_then_gyro(float delta_deg,
                                   float turn_speed_dps,
                                   float enc_tol_deg,
                                   uint32_t align_ms,
                                   float kp, float kd,
                                   uint32_t timeout_ms)
{
    if (fabsf(turn_speed_dps) < 1e-2f || fabsf(delta_deg) < 1e-3f) return ESP_ERR_INVALID_ARG;

    float yaw_start = gy25_get_absolute_angle();
    float yaw_target = yaw_start + delta_deg;

    double a0[4] = {0};
    esp_err_t e = read_all_angles_deg(a0);
    if (e != ESP_OK) return e;

    int dir = (delta_deg >= 0.0f) ? +1 : -1; // +CCW
    e = robot_set_side_speed_dps(-dir * fabsf(turn_speed_dps), +dir * fabsf(turn_speed_dps));
    if (e != ESP_OK) return e;

    const TickType_t dt = pdMS_TO_TICKS(10);
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms ? timeout_ms : 60000);

    while (xTaskGetTickCount() < deadline) {
        double a[4] = {0};
        e = read_all_angles_deg(a);
        if (e != ESP_OK) break;

        double dL0 = (a[0] - a0[0]) * (double)g_robot.left_sign;
        double dL1 = (a[1] - a0[1]) * (double)g_robot.left_sign;
        double dR0 = (a[2] - a0[2]) * (double)g_robot.right_sign;
        double dR1 = (a[3] - a0[3]) * (double)g_robot.right_sign;

        double phi_L = 0.5 * (dL0 + dL1);
        double phi_R = 0.5 * (dR0 + dR1);

        float K = (float)g_robot.wheel_diam_mm / (2.0f * (float)g_robot.track_mm); 
        float theta_odo = K * (float)(phi_R - phi_L);

        float err_enc = delta_deg - theta_odo;
        if (fabsf(err_enc) <= enc_tol_deg) break; 
        vTaskDelay(dt);
    }


    float prev_err = 0.0f;
    const float speed_max = fabsf(turn_speed_dps);
    const float speed_min = fminf(0.5f * speed_max, 80.0f); 

    TickType_t t0 = xTaskGetTickCount();
    while ((xTaskGetTickCount() - t0) < pdMS_TO_TICKS(align_ms)) {
        float yaw_now = gy25_get_absolute_angle();
        float err = ang_err_shortest(yaw_target, yaw_now); 
        float d_err = err - prev_err;
        prev_err = err;

        float cmd = kp * err + kd * d_err;
        float mag = fminf(speed_max, fmaxf(speed_min, fabsf(cmd)));
        float sgn = (cmd >= 0.0f) ? 1.0f : -1.0f;

        e = robot_set_side_speed_dps(-sgn * mag, +sgn * mag);
        if (e != ESP_OK) { robot_stop_all(); return e; }

        vTaskDelay(dt);
    }

    robot_set_side_speed_dps(0.0f, 0.0f);
    return (xTaskGetTickCount() < (TickType_t)(deadline)) ? ESP_OK : ESP_ERR_TIMEOUT;
}

void wall_align_pid_step(float basespeed) {
    uint16_t left_dist, right_dist;
    vl53l0x_read_mm_fast(2, &left_dist);
    vl53l0x_read_mm_fast(0, &right_dist);
    // защита от выбросов
    if (left_dist > 300) left_dist = 180;
    if (right_dist > 300) right_dist = 180;
    float kp = 0.35f, kd = 4.3f;
    float error = (float)left_dist - (float)right_dist;
    static float preverror = 0.0f;
    float derror = error - preverror;
    preverror = error;
    float u = fabsf(error) < 3.0f ? 0.0f : kp * error + kd * derror;
    if (u > 25.0f) u = 25.0f;
    if (u < -25.0f) u = -25.0f;
    float leftspeed = basespeed - u;
    float rightspeed = basespeed + u;
    robot_set_side_speed_cmps(leftspeed, rightspeed);
}



typedef struct { bool front; bool right; bool left; } WallState;

#define SENSOR_NO_WALL_VALUE 25    
#define SENSOR_NO_WALL_COUNT_THRESH 7 

WallState check_walls() {
    WallState ws;
    uint16_t dist_front_max = 0, dist_right_max = 0, dist_left_max = 0;
    uint16_t dist = 0;
    bool front_any20 = false, right_any20 = false, left_any20 = false;

    // Фронт
    for (int i = 0; i < 10; ++i) {
        vl53l0x_read_mm_fast(1, &dist);
        if (dist > dist_front_max) dist_front_max = dist;
        if (dist == 20) front_any20 = true;
    }
    // Право
    for (int i = 0; i < 10; ++i) {
        vl53l0x_read_mm_fast(0, &dist);
        if (dist > dist_right_max) dist_right_max = dist;
        if (dist == 20) right_any20 = true;
    }
    // Лево
    for (int i = 0; i < 10; ++i) {
        vl53l0x_read_mm_fast(2, &dist);
        if (dist > dist_left_max) dist_left_max = dist;
        if (dist == 20) left_any20 = true;
    }

    ws.front = (dist_front_max >= 190) || front_any20;
    ws.right = (dist_right_max >= 250) || right_any20;
    ws.left  = (dist_left_max  >= 250) || left_any20;
    return ws;
}

void maze_step() {
    robot_drive_align_cm(24.0f, 40.0f, 5000);
    WallState ws = check_walls();
    const char* color = get_robot_color();
    if (color && strcmp(color, "other") == 0 ) {
        // Если увидели синий:
        robot_set_side_speed_cmps(0.0f, 0.0f);
        vTaskDelay(pdMS_TO_TICKS(5000));       
    }

    if (ws.right) { 
        robot_turn_enc_then_gyro(-90.0f, 520.0f, 2.0f, 400.0f, 30.0f, 5.8f, 3000);
    }
    else if (ws.front) {
    }
    else if (ws.left) { 
        robot_turn_enc_then_gyro(90.0f, 520.0f, 2.0f, 400.0f, 30.0f, 5.8f, 3000); 
    }
    else { 
        // Тупик - разворот
        robot_turn_enc_then_gyro(-180.0f, 520.0f, 2.0f, 400.0f, 30.0f, 5.8f, 3000); 
    }
}

