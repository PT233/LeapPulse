#ifndef LEAPPULSE_STM32_PORT_H
#define LEAPPULSE_STM32_PORT_H

#include "stm32f4xx_hal.h"
#include "board_port.h"

struct lp_stm32_pin {
    GPIO_TypeDef *port;
    uint16_t pin;
    bool active_low;
};

struct lp_stm32_config {
    I2C_HandleTypeDef *i2c;
    UART_HandleTypeDef *hc05;
    IWDG_HandleTypeDef *watchdog;
    struct lp_stm32_pin start;
    struct lp_stm32_pin encoder_a;
    struct lp_stm32_pin encoder_b;
    struct lp_stm32_pin hc05_state;
    int (*battery_mv)(void);
    bool (*allow_update)(void);
    void (*report_sensor_health)(bool motion, bool motion_wake, bool optical);
};

/* Call once after Cube has initialized GPIO, I2C, UART and any IWDG. */
bool lp_stm32_port_init(const struct lp_stm32_config *config);
bool lp_stm32_uart_start(void);
void lp_stm32_uart_rx_complete(UART_HandleTypeDef *uart);
void lp_stm32_uart_error(UART_HandleTypeDef *uart);
bool lp_stm32_allow_update(void);
void lp_stm32_report_sensor_health(bool motion, bool motion_wake, bool optical);

#endif
