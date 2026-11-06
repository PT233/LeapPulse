#ifndef LEAPPULSE_PERIPHERALS_H
#define LEAPPULSE_PERIPHERALS_H

#include <stdbool.h>
#include <stdint.h>

/* Call init, update, mode selection and haptic requests from the UI task. */
void lp_peripherals_init(void);
void lp_peripherals_update(uint32_t jumps, uint16_t bpm, bool pulse_valid, uint8_t mode,
                           bool complete);
int lp_peripherals_take_mode_delta(void);
void lp_peripherals_goal(void);
void lp_peripherals_alert(void);
int lp_peripherals_battery_mv(void);

#endif
