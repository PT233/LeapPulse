#include "sensors.h"

#include "board_port.h"
#include "leappulse.h"
#include "bmi270.h"

#ifndef LP_BMI270_I2C_ADDRESS
#define LP_BMI270_I2C_ADDRESS 0x68
#endif

#define PPG_ADDRESS 0x57
#define PPG_INT_ENABLE 0x02
#define PPG_FIFO_WRITE 0x04
#define PPG_FIFO_DATA 0x07
#define PPG_FIFO_CONFIG 0x08
#define PPG_MODE 0x09
#define PPG_SPO2_CONFIG 0x0a
#define PPG_RED_CURRENT 0x0c
#define PPG_PART_ID 0xff
#define PPG_FIFO_MASK 31u
#define PPG_SHUTDOWN 0x80
#define PPG_RESET 0x40
#define PPG_MODE_RED_IR 3
#define PPG_SAMPLE_MS 10u
#define REST_SAMPLE_MS 1000u
#define REST_WINDOW_MS 40u
#define RAW_ACCEL_TO_G (8.0f / 32768.0f)
#define RAW_GYRO_TO_RADIANS (1000.0f * 0.01745329252f / 32768.0f)
#define TEMP_DRIFT_RAD_PER_C (0.05f * 0.01745329252f)

static uint8_t bmi_address = LP_BMI270_I2C_ADDRESS;
static struct bmi2_dev bmi;
static struct lp_motion motion;
static struct lp_ppg pulse;
static struct lp_sensor_health health;
static bool was_exercising;
static bool gyro_on;
static bool optical_awake;
static bool temperature_valid;
static float temperature_reference;
static float gyro_correction;
static uint32_t last_temperature_ms;
static uint32_t rest_started_ms;
static uint32_t optical_sample_ms;
static uint32_t last_poll_ms;
static bool optical_clock_valid;

static BMI2_INTF_RETURN_TYPE bmi_read(uint8_t reg, uint8_t *bytes, uint32_t len, void *context)
{
    return lp_board_i2c_read(*(const uint8_t *)context, reg, bytes, len);
}

static BMI2_INTF_RETURN_TYPE bmi_write(uint8_t reg, const uint8_t *bytes, uint32_t len, void *context)
{
    return lp_board_i2c_write(*(const uint8_t *)context, reg, bytes, len);
}

static void bmi_delay(uint32_t us, void *context)
{
    (void)context;
    lp_board_delay_ms((us + 999u) / 1000u);
}

static bool configure_motion(void)
{
    bmi = (struct bmi2_dev){
        .intf = BMI2_I2C_INTF,
        .intf_ptr = &bmi_address,
        .read = bmi_read,
        .write = bmi_write,
        .delay_us = bmi_delay,
        .read_write_len = 32,
    };
    if (bmi270_init(&bmi) != BMI2_OK) {
        return false;
    }

    struct bmi2_sens_config settings[] = { { .type = BMI2_ACCEL }, { .type = BMI2_GYRO } };
    if (bmi270_get_sensor_config(settings, 2, &bmi) != BMI2_OK) {
        return false;
    }

    settings[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    settings[0].cfg.acc.range = BMI2_ACC_RANGE_8G;
    settings[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    settings[1].cfg.gyr.range = BMI2_GYR_RANGE_1000;
    uint8_t accel = BMI2_ACCEL;
    return bmi270_set_sensor_config(settings, 2, &bmi) == BMI2_OK &&
           bmi270_sensor_enable(&accel, 1, &bmi) == BMI2_OK;
}

static bool configure_motion_wake(void)
{
    struct bmi2_sens_config feature = { .type = BMI2_ANY_MOTION };
    struct bmi2_int_pin_config pin = { .pin_type = BMI2_INT1, .int_latch = BMI2_INT_NON_LATCH };
    struct bmi2_sens_int_config route = { .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT1 };
    uint8_t motion_feature = BMI2_ANY_MOTION;

    if (bmi270_get_sensor_config(&feature, 1, &bmi) != BMI2_OK) {
        return false;
    }

    /* BMI270 any-motion duration has 20 ms granularity, threshold 0.48 mg/LSB. */
    feature.cfg.any_motion.duration = 1;
    feature.cfg.any_motion.threshold = 1042;
    feature.cfg.any_motion.select_x = 1;
    feature.cfg.any_motion.select_y = 1;
    feature.cfg.any_motion.select_z = 1;
    pin.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    pin.pin_cfg[0].od = BMI2_INT_PUSH_PULL;
    pin.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;

    return bmi270_set_sensor_config(&feature, 1, &bmi) == BMI2_OK &&
           bmi2_set_int_pin_config(&pin, &bmi) == BMI2_OK &&
           bmi270_map_feat_int(&route, 1, &bmi) == BMI2_OK &&
           bmi270_sensor_enable(&motion_feature, 1, &bmi) == BMI2_OK;
}

static bool ppg_write(uint8_t reg, uint8_t value)
{
    return lp_board_i2c_write(PPG_ADDRESS, reg, &value, 1) == 0;
}

static bool ppg_clear_fifo(void)
{
    const uint8_t clear[3] = {0};
    optical_clock_valid = false;
    return lp_board_i2c_write(PPG_ADDRESS, PPG_FIFO_WRITE, clear, sizeof(clear)) == 0;
}

static bool configure_optical(void)
{
    uint8_t id = 0;
    if (lp_board_i2c_read(PPG_ADDRESS, PPG_PART_ID, &id, 1) || id != 0x15 ||
        !ppg_write(PPG_MODE, PPG_RESET)) {
        return false;
    }

    lp_board_delay_ms(18);
    /* ADC 8192 nA, 100 samples/s, 411 us (18 bits), 17-sample IRQ watermark. */
    return ppg_write(PPG_FIFO_CONFIG, 0x0f) && ppg_write(PPG_INT_ENABLE, 0x80) &&
           ppg_write(PPG_SPO2_CONFIG, 0x47) &&
           ppg_write(PPG_RED_CURRENT, 0x24) && ppg_write(PPG_RED_CURRENT + 1u, 0x24) &&
           ppg_clear_fifo() && ppg_write(PPG_MODE, PPG_SHUTDOWN | PPG_MODE_RED_IR);
}

struct lp_sensor_health lp_sensors_init(void)
{
    health.motion = configure_motion();
    health.motion_wake = health.motion && configure_motion_wake();
    health.optical = configure_optical();
    rest_started_ms = lp_board_ms() - REST_SAMPLE_MS;
    return health;
}

static void sample_motion(uint32_t now)
{
    if (!temperature_valid || now - last_temperature_ms >= 1000) {
        int16_t raw;
        last_temperature_ms = now;
        if (bmi2_get_temperature_data(&raw, &bmi) == BMI2_OK && raw != INT16_MIN) {
            float temperature = 23.0f + raw / 512.0f;
            if (!temperature_valid) {
                temperature_reference = temperature;
                temperature_valid = true;
            }
            gyro_correction = (temperature - temperature_reference) * TEMP_DRIFT_RAD_PER_C;
        }
    }

    struct bmi2_sens_data axes;
    if (bmi2_get_sensor_data(&axes, &bmi) == BMI2_OK) {
        lp_motion_update(&motion, now, axes.acc.x * RAW_ACCEL_TO_G,
                         axes.acc.y * RAW_ACCEL_TO_G, axes.acc.z * RAW_ACCEL_TO_G,
                         axes.gyr.x * RAW_GYRO_TO_RADIANS - gyro_correction,
                         axes.gyr.y * RAW_GYRO_TO_RADIANS - gyro_correction,
                         axes.gyr.z * RAW_GYRO_TO_RADIANS - gyro_correction);
    }
}

static uint32_t read_optical_word(const uint8_t *bytes)
{
    return ((uint32_t)(bytes[0] & 3u) << 16) | ((uint32_t)bytes[1] << 8) | bytes[2];
}

static void sample_optical(uint32_t now, bool exercising)
{
    if (!optical_awake && (exercising || now - rest_started_ms >= REST_SAMPLE_MS)) {
        rest_started_ms = now;
        optical_awake = ppg_clear_fifo() && ppg_write(PPG_MODE, PPG_MODE_RED_IR);
    }
    if (!optical_awake || now - last_poll_ms < PPG_SAMPLE_MS) {
        return;
    }

    last_poll_ms = now;
    uint8_t fifo[3];
    if (lp_board_i2c_read(PPG_ADDRESS, PPG_FIFO_WRITE, fifo, sizeof(fifo)) || fifo[1]) {
        ppg_clear_fifo();
        pulse = (struct lp_ppg){0};
    } else {
        unsigned count = (fifo[0] + 32u - fifo[2]) & PPG_FIFO_MASK;
        if (!optical_clock_valid && count) {
            optical_sample_ms = now - count * PPG_SAMPLE_MS;
            optical_clock_valid = true;
        }
        for (unsigned i = 0; i < count; ++i) {
            uint8_t values[6];
            if (lp_board_i2c_read(PPG_ADDRESS, PPG_FIFO_DATA, values, sizeof(values))) {
                ppg_clear_fifo();
                pulse = (struct lp_ppg){0};
                break;
            }
            optical_sample_ms += PPG_SAMPLE_MS;
            if (exercising) {
                lp_ppg_update(&pulse, optical_sample_ms, read_optical_word(values),
                              read_optical_word(values + 3));
            }
        }
    }
    if (!exercising && now - rest_started_ms >= REST_WINDOW_MS &&
        ppg_write(PPG_MODE, PPG_SHUTDOWN | PPG_MODE_RED_IR)) {
        optical_awake = false;
    }
}

void lp_sensors_sample(uint32_t now, bool exercising)
{
    if (was_exercising != exercising) {
        was_exercising = exercising;
        motion = (struct lp_motion){ .jumps = motion.jumps };
        pulse = (struct lp_ppg){0};
        if (health.optical && optical_awake) {
            ppg_clear_fifo();
            rest_started_ms = now - REST_WINDOW_MS;
        }
    }

    if (health.motion) {
        if (gyro_on != exercising) {
            uint8_t gyro = BMI2_GYRO;
            int8_t rc = exercising ? bmi270_sensor_enable(&gyro, 1, &bmi) :
                                       bmi270_sensor_disable(&gyro, 1, &bmi);
            if (rc == BMI2_OK) {
                gyro_on = exercising;
            }
        }
        if (exercising && gyro_on) {
            sample_motion(now);
        }
    }
    if (health.optical) {
        sample_optical(now, exercising);
    }
}

void lp_sensors_snapshot(struct lp_sensor_snapshot *snapshot)
{
    *snapshot = (struct lp_sensor_snapshot){
        .jumps = motion.jumps,
        .bpm = pulse.bpm,
        .hrv_rmssd_ms = pulse.hrv_rmssd_ms,
        .spo2_pct = pulse.spo2_pct,
        .roll_deg = motion.roll_deg,
        .pitch_deg = motion.pitch_deg,
        .yaw_deg = motion.yaw_deg,
        .style = (unsigned)motion.style,
        .bpm_valid = pulse.bpm_valid,
        .hrv_valid = pulse.hrv_valid,
        .spo2_valid = pulse.spo2_valid,
    };
}

uint32_t lp_sensors_idle_delay(uint32_t now)
{
    if (!health.optical && !gyro_on) {
        return REST_SAMPLE_MS;
    }
    if (optical_awake) {
        return PPG_SAMPLE_MS;
    }
    uint32_t elapsed = now - rest_started_ms;
    return elapsed < REST_SAMPLE_MS ? REST_SAMPLE_MS - elapsed : 1;
}
