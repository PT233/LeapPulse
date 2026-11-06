#ifndef LEAPPULSE_H
#define LEAPPULSE_H

#include <stdbool.h>
#include <stdint.h>

enum lp_jump_style {
    LP_JUMP_UNKNOWN,
    LP_JUMP_SINGLE,
    LP_JUMP_DOUBLE,
    LP_JUMP_CROSS,
};

/* Initialize with {0}. Only the first fields are application outputs. */
struct lp_motion {
    float quaternion[4]; /* w, x, y, z */
    float roll_deg;
    float pitch_deg;
    float yaw_deg; /* Relative heading, subject to drift. */
    uint32_t jumps;
    uint32_t last_cycle_ms;
    enum lp_jump_style style;
    bool orientation_valid;

    float filter_state[3][2];
    float noise[10];
    float cycle_threshold;
    float cycle_roll_deg;
    float cycle_pitch_deg;
    float cycle_roll_range;
    float cycle_pitch_range;
    float yaw_history[21];
    uint32_t yaw_time_ms[21];
    uint32_t last_sample_ms;
    uint32_t cycle_start_ms;
    uint32_t last_count_ms;
    uint8_t noise_index;
    uint8_t noise_count;
    uint8_t yaw_index;
    uint8_t yaw_count;
    bool cycle_active;
    bool cycle_rotation;
    bool cycle_cross;
};

/* Call at 200 Hz. Inputs are acceleration in g and angular rate in rad/s. */
void lp_motion_update(struct lp_motion *motion, uint32_t timestamp_ms,
                      float ax, float ay, float az,
                      float gx, float gy, float gz);

struct lp_ppg {
    float bpm;
    float spo2_pct;
    float hrv_rmssd_ms;
    bool bpm_valid;
    bool spo2_valid;
    bool hrv_valid;

    float red_dc;
    float ir_dc;
    float red_ac;
    float ir_ac;
    float ir_power;
    float mean_red;
    float mean_ir;
    float red_variation;
    float ir_variation;
    float covariance;
    uint32_t last_sample_ms;
    uint32_t start_ms;
    uint32_t last_beat_ms;
    uint16_t intervals_ms[8];
    uint16_t cycle_samples;
    uint8_t interval_index;
    uint8_t interval_count;
    bool initialized;
    bool have_beat;
    bool armed;
};

/* Initialize with {0}. Pass 18-bit raw red and IR readings with sample times. */
void lp_ppg_update(struct lp_ppg *ppg, uint32_t timestamp_ms,
                   uint32_t red, uint32_t ir);

#endif
