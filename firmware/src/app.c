#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "board_port.h"
#include "flash_slots.h"
#include "peripherals.h"
#include "sensors.h"
#include "stm32_port.h"
#include "wireless.h"
#include "workout.h"

#define IMU_PERIOD_MS 5u
#define UI_PERIOD_MS 5u
#define DEBOUNCE_MS 20u
#define ALERT_BPM 180u
#define ALERT_CLEAR_BPM 170u

_Static_assert(configTICK_RATE_HZ == 1000, "LeapPulse requires a 1 ms FreeRTOS tick");
_Static_assert(sizeof(TickType_t) == 4, "LeapPulse requires 32-bit FreeRTOS ticks");

static TaskHandle_t sensor_handle;
static volatile bool sensor_irq_ready;
static bool exercising;
static bool ota_exclusive;
static struct lp_sensor_snapshot latest;
static struct lp_workout workout;
static uint16_t wireless_count;
static uint8_t wireless_mode;
static uint8_t wireless_bpm;

static bool set_exercising(bool enabled)
{
    taskENTER_CRITICAL();
    bool allowed = !enabled || !ota_exclusive;
    if (allowed) {
        exercising = enabled;
    }
    taskEXIT_CRITICAL();
    if (allowed) {
        xTaskNotifyGive(sensor_handle);
    }
    return allowed;
}

static void sensor_task(void *context)
{
    (void)context;
    struct lp_sensor_health health = lp_sensors_init();
    lp_stm32_report_sensor_health(health.motion, health.motion_wake, health.optical);
    sensor_irq_ready = true;

    for (;;) {
        uint32_t now = lp_board_ms();
        taskENTER_CRITICAL();
        bool active = exercising;
        taskEXIT_CRITICAL();

        lp_sensors_sample(now, active);
        struct lp_sensor_snapshot snapshot;
        lp_sensors_snapshot(&snapshot);
        taskENTER_CRITICAL();
        latest = snapshot;
        taskEXIT_CRITICAL();

        uint32_t period = active ? IMU_PERIOD_MS : lp_sensors_idle_delay(now);
        uint32_t elapsed = lp_board_ms() - now;
        if (elapsed < period) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(period - elapsed));
        } else {
            taskYIELD();
        }
    }
}

static bool start_button_edge(uint32_t now)
{
    static bool stable;
    static bool candidate;
    static uint32_t changed;
    bool pressed = lp_board_start_pressed();
    if (pressed != candidate) {
        candidate = pressed;
        changed = now;
    }
    if (pressed != stable && now - changed >= DEBOUNCE_MS) {
        stable = pressed;
        return pressed;
    }
    return false;
}

static void ui_task(void *context)
{
    (void)context;
    bool alert_sent = false;
    lp_peripherals_init();

    for (;;) {
        uint32_t now = lp_board_ms();
        taskENTER_CRITICAL();
        struct lp_sensor_snapshot data = latest;
        taskEXIT_CRITICAL();

        if (lp_workout_update(&workout, now, data.jumps)) {
            set_exercising(false);
            lp_peripherals_goal();
        }
        lp_workout_select(&workout, lp_peripherals_take_mode_delta());

        if (start_button_edge(now)) {
            if (set_exercising(workout.state != LP_ACTIVE)) {
                lp_workout_toggle(&workout, now, data.jumps);
                alert_sent = false;
            }
        }

        bool valid = workout.state == LP_ACTIVE && data.bpm_valid;
        uint16_t bpm = valid ? (uint16_t)(data.bpm + 0.5f) : 0;
        if (valid && bpm < ALERT_CLEAR_BPM) {
            alert_sent = false;
        }
        if (valid && bpm >= ALERT_BPM && !alert_sent) {
            alert_sent = true;
            lp_peripherals_alert();
        }
        lp_peripherals_update(workout.jumps, bpm, valid, workout.mode, workout.state == LP_COMPLETE);
        taskENTER_CRITICAL();
        wireless_count = workout.jumps <= UINT16_MAX ? (uint16_t)workout.jumps : UINT16_MAX;
        wireless_mode = (uint8_t)workout.mode;
        wireless_bpm = bpm <= UINT8_MAX ? (uint8_t)bpm : UINT8_MAX;
        taskEXIT_CRITICAL();
        lp_board_watchdog();
        vTaskDelay(pdMS_TO_TICKS(UI_PERIOD_MS));
    }
}

static bool claim_ota(void)
{
    bool unlocked = lp_stm32_allow_update();
    taskENTER_CRITICAL();
    bool allowed = !exercising && (ota_exclusive || unlocked);
    if (allowed) {
        ota_exclusive = true;
    }
    taskEXIT_CRITICAL();
    return allowed;
}

static int begin_ota(void *context, uint32_t size, uint32_t crc)
{
    if (!claim_ota()) {
        return -1;
    }
    lp_board_watchdog();
    return lp_flash_begin(context, size, crc);
}

static int resume_ota(void *context, uint32_t *size, uint32_t *crc, uint32_t *offset)
{
    return claim_ota() ? lp_flash_resume(context, size, crc, offset) : -1;
}

static int uart_send(void *context, const uint8_t *bytes, size_t length)
{
    (void)context;
    return lp_board_uart_write(bytes, length);
}

static void wireless_task(void *context)
{
    (void)context;
    const struct lp_ota_ops ota = {
        .capacity = LP_SLOT_SIZE,
        .begin = begin_ota,
        .write = lp_flash_write,
        .read = lp_flash_read,
        .save = lp_flash_checkpoint,
        .load = resume_ota,
        .activate = lp_flash_activate,
    };
    struct lp_wire wire;
    lp_wire_init(&wire, lp_flash_slots_available() ? &ota : NULL, uart_send, NULL);
    if (!lp_stm32_uart_start()) {
        vTaskDelete(NULL);
        return;
    }
    bool connected = false;
    uint32_t last_rx = 0;
    uint32_t last_identity = 0;
    uint32_t id = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();

    for (;;) {
        uint8_t bytes[LP_WIRE_FRAME_SIZE];
        int received = lp_board_uart_read(bytes, sizeof(bytes));
        uint32_t now = lp_board_ms();
        bool link = lp_board_hc05_connected();

        if (!link || now - last_rx >= 5000u) {
            if (lp_wire_updating(&wire) || !link) {
                lp_wire_abort(&wire);
            }
        }
        if (link && received > 0) {
            last_rx = now;
            lp_wire_rx(&wire, bytes, (size_t)received);
            if (wire.activated) {
                /* lp_wire_rx has attempted the final ACK on the blocking UART. */
                NVIC_SystemReset();
            }
        }
        if (!lp_wire_updating(&wire)) {
            taskENTER_CRITICAL();
            ota_exclusive = false;
            taskEXIT_CRITICAL();
        }
        if (link && !lp_wire_updating(&wire)) {
            if (!connected || now - last_identity >= 30000u) {
                int mv = lp_peripherals_battery_mv();
                unsigned percentage = mv >= 4200 ? 100u : mv <= 3300 ? 0u : (unsigned)(mv - 3300) / 9u;
                lp_wire_identity(&wire, id, mv >= 0 ? (uint8_t)percentage : LP_WIRE_BATTERY_UNKNOWN);
                last_identity = now;
            }
            taskENTER_CRITICAL();
            uint8_t mode = wireless_mode;
            uint16_t count = wireless_count;
            uint8_t bpm = wireless_bpm;
            taskEXIT_CRITICAL();
            lp_wire_publish(&wire, now, mode, count, bpm);
        }
        connected = link;
        vTaskDelay(1);
    }
}

void lp_app_sensor_wake_isr(void)
{
    if (sensor_irq_ready) {
        BaseType_t wake = pdFALSE;
        vTaskNotifyGiveFromISR(sensor_handle, &wake);
        portYIELD_FROM_ISR(wake);
    }
}

bool lp_app_start(void)
{
    TaskHandle_t ui_handle;
    if (xTaskCreate(sensor_task, "lp_sensor", 768, NULL, tskIDLE_PRIORITY + 3, &sensor_handle)
        != pdPASS) {
        return false;
    }
    if (xTaskCreate(ui_task, "lp_ui", 1024, NULL, tskIDLE_PRIORITY + 1, &ui_handle) != pdPASS) {
        vTaskDelete(sensor_handle);
        sensor_handle = NULL;
        return false;
    }
    if (xTaskCreate(wireless_task, "lp_spp", 768, NULL, tskIDLE_PRIORITY + 2, NULL) != pdPASS) {
        vTaskDelete(ui_handle);
        vTaskDelete(sensor_handle);
        sensor_handle = NULL;
        return false;
    }
    return true;
}
