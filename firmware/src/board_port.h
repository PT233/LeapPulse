#ifndef LEAPPULSE_BOARD_PORT_H
#define LEAPPULSE_BOARD_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int lp_board_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *bytes, size_t len);
int lp_board_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *bytes, size_t len);
uint32_t lp_board_ms(void);
void lp_board_delay_ms(uint32_t ms);
int lp_board_encoder_ab(void);
int lp_board_battery_mv(void);
bool lp_board_start_pressed(void);
bool lp_board_hc05_connected(void);
int lp_board_uart_read(uint8_t *data, size_t max_len);
int lp_board_uart_write(const uint8_t *data, size_t len);
void lp_board_watchdog(void);

#endif
