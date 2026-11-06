#ifndef LEAPPULSE_RTC_STOP_H
#define LEAPPULSE_RTC_STOP_H

#include "stm32f4xx_hal.h"

/* Optional: RTC at 32.768 kHz, wake IRQ, FreeRTOS custom tickless mode. */
void lp_stm32_stop_init(RTC_HandleTypeDef *rtc, void (*restore_clock)(void));

#endif
