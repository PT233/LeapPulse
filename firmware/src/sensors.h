#ifndef LEAPPULSE_SENSORS_H
#define LEAPPULSE_SENSORS_H

#include <stdbool.h>
#include <stdint.h>

struct lp_sensor_snapshot {
    uint32_t jumps;
    float bpm;
    float hrv_rmssd_ms;
    float spo2_pct;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    unsigned style;
    bool bpm_valid;
    bool hrv_valid;
    bool spo2_valid;
};

struct lp_sensor_health {
    bool motion;
    bool motion_wake;
    bool optical;
};

struct lp_sensor_health lp_sensors_init(void);
void lp_sensors_sample(uint32_t timestamp_ms, bool exercising);
uint32_t lp_sensors_idle_delay(uint32_t timestamp_ms);
void lp_sensors_snapshot(struct lp_sensor_snapshot *snapshot);

#endif
