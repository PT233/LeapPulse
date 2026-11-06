#ifndef LEAPPULSE_APP_H
#define LEAPPULSE_APP_H

#include <stdbool.h>

/* Call from generated Cube main after lp_stm32_port_init and before starting FreeRTOS. */
bool lp_app_start(void);
/* Forward BMI270 INT1 and MAX30102 FIFO interrupts from HAL_GPIO_EXTI_Callback. */
void lp_app_sensor_wake_isr(void);

#endif
