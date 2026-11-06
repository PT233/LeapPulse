#include "stm32_port.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "task.h"

static struct lp_stm32_config hw;
static SemaphoreHandle_t i2c_mutex;
static StreamBufferHandle_t uart_rx;
static uint8_t uart_byte;

bool lp_stm32_port_init(const struct lp_stm32_config *config)
{
    if (!config || !config->i2c || !config->hc05 || !config->start.port ||
        !config->encoder_a.port || !config->encoder_b.port) {
        return false;
    }
    hw = *config;
    i2c_mutex = xSemaphoreCreateMutex();
    uart_rx = xStreamBufferCreate(256, 1);
    if (!i2c_mutex || !uart_rx) {
        if (i2c_mutex) {
            vSemaphoreDelete(i2c_mutex);
        }
        if (uart_rx) {
            vStreamBufferDelete(uart_rx);
        }
        return false;
    }
    return true;
}

static bool pin_active(struct lp_stm32_pin pin)
{
    return pin.port && ((HAL_GPIO_ReadPin(pin.port, pin.pin) == GPIO_PIN_SET) != pin.active_low);
}

int lp_board_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *bytes, size_t len)
{
    if (len > UINT16_MAX || xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return -1;
    }
    HAL_StatusTypeDef result = HAL_I2C_Mem_Read(hw.i2c, (uint16_t)addr7 << 1, reg,
                                               I2C_MEMADD_SIZE_8BIT, bytes, (uint16_t)len, 8);
    xSemaphoreGive(i2c_mutex);
    return result == HAL_OK ? 0 : -1;
}

int lp_board_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *bytes, size_t len)
{
    if (len > UINT16_MAX || xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return -1;
    }
    HAL_StatusTypeDef result = HAL_I2C_Mem_Write(hw.i2c, (uint16_t)addr7 << 1, reg,
                                                I2C_MEMADD_SIZE_8BIT, (uint8_t *)bytes,
                                                (uint16_t)len, 8);
    xSemaphoreGive(i2c_mutex);
    return result == HAL_OK ? 0 : -1;
}

uint32_t lp_board_ms(void)
{
    return (uint32_t)xTaskGetTickCount();
}

void lp_board_delay_ms(uint32_t ms)
{
    if (ms) {
        vTaskDelay(pdMS_TO_TICKS(ms) + 1u);
    }
}

int lp_board_encoder_ab(void)
{
    return (pin_active(hw.encoder_b) << 1) | pin_active(hw.encoder_a);
}

int lp_board_battery_mv(void)
{
    return hw.battery_mv ? hw.battery_mv() : -1;
}

bool lp_board_start_pressed(void)
{
    return pin_active(hw.start);
}

bool lp_board_hc05_connected(void)
{
    return !hw.hc05_state.port || pin_active(hw.hc05_state);
}

bool lp_stm32_uart_start(void)
{
    return HAL_UART_Receive_IT(hw.hc05, &uart_byte, 1) == HAL_OK;
}

void lp_stm32_uart_rx_complete(UART_HandleTypeDef *uart)
{
    if (uart != hw.hc05) {
        return;
    }
    BaseType_t wake = pdFALSE;
    xStreamBufferSendFromISR(uart_rx, &uart_byte, 1, &wake);
    lp_stm32_uart_start();
    portYIELD_FROM_ISR(wake);
}

void lp_stm32_uart_error(UART_HandleTypeDef *uart)
{
    if (uart == hw.hc05 && uart->RxState == HAL_UART_STATE_READY) {
        lp_stm32_uart_start();
    }
}

int lp_board_uart_read(uint8_t *data, size_t max_len)
{
    return (int)xStreamBufferReceive(uart_rx, data, max_len, pdMS_TO_TICKS(10));
}

int lp_board_uart_write(const uint8_t *data, size_t len)
{
    if (len > UINT16_MAX) {
        return -1;
    }
    return HAL_UART_Transmit(hw.hc05, (uint8_t *)data, (uint16_t)len, 100) == HAL_OK ? 0 : -1;
}

bool lp_stm32_allow_update(void)
{
    return hw.allow_update && hw.allow_update();
}

void lp_board_watchdog(void)
{
    if (hw.watchdog) {
        HAL_IWDG_Refresh(hw.watchdog);
    }
}

void lp_stm32_report_sensor_health(bool motion, bool motion_wake, bool optical)
{
    if (hw.report_sensor_health) {
        hw.report_sensor_health(motion, motion_wake, optical);
    }
}
