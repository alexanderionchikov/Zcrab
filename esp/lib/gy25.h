#ifndef GY25_H
#define GY25_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float yaw;
    float pitch;
    float roll;
} gy25_data_t;

/**
 * @brief Инициализация датчика GY-25
 * 
 * @param uart_num Номер UART (UART_NUM_1, UART_NUM_2 и т.д.)
 * @param tx_pin GPIO для TX
 * @param rx_pin GPIO для RX
 * @param baud_rate Скорость обмена (9600 или 115200)
 * @return true - успех, false - ошибка
 */
bool gy25_init(int uart_num, int tx_pin, int rx_pin, int baud_rate);

/**
 * @brief Получение данных с датчика
 * 
 * @param data Указатель на структуру для записи данных
 * @return true - данные получены, false - ошибка чтения
 */
bool gy25_get_data(gy25_data_t* data);

/**
 * @brief Калибровка наклона (крен, тангаж)
 */
void gy25_calibrate_tilt(void);

/**
 * @brief Калибровка курса (рыскание)
 */
void gy25_calibrate_heading(void);

/**
 * @brief Деинициализация датчика
 */
void gy25_deinit(void);
float gy25_get_absolute_angle(void);

#ifdef __cplusplus
}
#endif

#endif // GY25_H