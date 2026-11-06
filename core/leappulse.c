#include "leappulse.h"

#include <math.h>
#include <string.h>

#define DEG_PER_RAD 57.2957795131f
#define RAD_PER_DEG 0.01745329252f

static float angle_difference(float a, float b)
{
    const float delta = a - b;
    if (delta > 180.0f) {
        return delta - 360.0f;
    }
    if (delta < -180.0f) {
        return delta + 360.0f;
    }
    return delta;
}

static void gravity_direction(const float q[4], float gravity[3])
{
    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];
    gravity[0] = 2.0f * (x * z - w * y);
    gravity[1] = 2.0f * (y * z + w * x);
    gravity[2] = 1.0f - 2.0f * (x * x + y * y);
}

static void initialize_orientation(float q[4], float ax, float ay, float az)
{
    const float roll = atan2f(ay, az) * 0.5f;
    const float pitch = atan2f(-ax, hypotf(ay, az)) * 0.5f;
    q[0] = cosf(roll) * cosf(pitch);
    q[1] = sinf(roll) * cosf(pitch);
    q[2] = cosf(roll) * sinf(pitch);
    q[3] = -sinf(roll) * sinf(pitch);
}

static void update_orientation(struct lp_motion *motion, float dt,
                               float ax, float ay, float az, float magnitude,
                               float gx, float gy, float gz)
{
    float *q = motion->quaternion;
    if (magnitude >= 0.85f && magnitude <= 1.15f) {
        float gravity[3];
        gravity_direction(q, gravity);
        /* Gravity feedback is gated during linear acceleration. */
        const float gain = 2.5f / magnitude;
        gx += gain * (ay * gravity[2] - az * gravity[1]);
        gy += gain * (az * gravity[0] - ax * gravity[2]);
        gz += gain * (ax * gravity[1] - ay * gravity[0]);
    }

    const float w = q[0];
    const float x = q[1];
    const float y = q[2];
    const float z = q[3];
    const float half_dt = dt * 0.5f;
    q[0] += half_dt * (-x * gx - y * gy - z * gz);
    q[1] += half_dt * (w * gx + y * gz - z * gy);
    q[2] += half_dt * (w * gy + z * gx - x * gz);
    q[3] += half_dt * (w * gz + x * gy - y * gx);

    const float inv_norm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] +
                                       q[2] * q[2] + q[3] * q[3]);
    for (unsigned i = 0; i < 4; ++i) {
        q[i] *= inv_norm;
    }
}

static void update_angles(struct lp_motion *motion, const float gravity[3])
{
    const float *q = motion->quaternion;
    motion->roll_deg = atan2f(gravity[1], gravity[2]) * DEG_PER_RAD;
    motion->pitch_deg = atan2f(-gravity[0], hypotf(gravity[1], gravity[2])) *
                        DEG_PER_RAD;
    motion->yaw_deg = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
                            1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) *
                      DEG_PER_RAD;
}

static float filter_acceleration(struct lp_motion *motion, float input)
{
    /* 15 Hz Butterworth at 200 samples/s, bilinear transform with prewarping.
     * Coefficients are b0, b1, b2, a1, a2 for denominator 1+a1*z^-1+a2*z^-2.
     */
    static const float coefficients[3][5] = {
        {0.19359961f, 0.19359961f, 0.0f, -0.61280079f, 0.0f},
        {0.03985760f, 0.07971520f, 0.03985760f, -1.30332133f, 0.46275173f},
        {0.04779197f, 0.09558393f, 0.04779197f, -1.56277072f, 0.75393859f},
    };
    for (unsigned i = 0; i < 3; ++i) {
        const float *c = coefficients[i];
        float *state = motion->filter_state[i];
        const float output = c[0] * input + state[0];
        state[0] = c[1] * input - c[3] * output + state[1];
        state[1] = c[2] * input - c[4] * output;
        input = output;
    }
    return input;
}

static float motion_threshold(const struct lp_motion *motion)
{
    if (!motion->noise_count) {
        return 0.16f;
    }
    float sum = 0.0f;
    float sum_sq = 0.0f;
    for (unsigned i = 0; i < motion->noise_count; ++i) {
        const float sample = motion->noise[i];
        sum += sample;
        sum_sq += sample * sample;
    }
    const float mean = sum / motion->noise_count;
    const float variance = sum_sq / motion->noise_count - mean * mean;
    return fmaxf(0.16f, fabsf(mean) + 3.0f * sqrtf(fmaxf(0.0f, variance)));
}

static bool rapid_yaw_change(struct lp_motion *motion, uint32_t now)
{
    const uint8_t index = motion->yaw_index;
    bool cross = false;
    if (fabsf(motion->pitch_deg) < 60.0f) {
        for (unsigned i = 0; i < motion->yaw_count; ++i) {
            if (now - motion->yaw_time_ms[i] <= 100u &&
                fabsf(angle_difference(motion->yaw_deg,
                                       motion->yaw_history[i])) > 90.0f) {
                cross = true;
                break;
            }
        }
        motion->yaw_time_ms[index] = now;
        motion->yaw_history[index] = motion->yaw_deg;
        motion->yaw_index = (index + 1u) % 21u;
        if (motion->yaw_count < 21u) {
            ++motion->yaw_count;
        }
    } else {
        motion->yaw_index = 0;
        motion->yaw_count = 0;
    }
    return cross;
}

static enum lp_jump_style jump_style(const struct lp_motion *motion, uint32_t period)
{
    if (motion->cycle_cross) {
        return LP_JUMP_CROSS;
    }
    if (period >= 250u && period <= 500u && motion->cycle_roll_range >= 45.0f) {
        return LP_JUMP_DOUBLE;
    }
    if (period >= 333u && period <= 667u && motion->cycle_pitch_range >= 30.0f) {
        return LP_JUMP_SINGLE;
    }
    return LP_JUMP_UNKNOWN;
}

static void count_jumps(struct lp_motion *motion, uint32_t now,
                        float vertical, float gz)
{
    const float filtered = filter_acceleration(motion, vertical);
    const bool rotating = fabsf(gz) >= 200.0f * RAD_PER_DEG;
    const bool cross = rapid_yaw_change(motion, now);

    if (motion->cycle_active && now - motion->cycle_start_ms > 500u) {
        motion->cycle_active = false;
    }

    if (!motion->cycle_active) {
        const float threshold = motion_threshold(motion);
        if (filtered < threshold) {
            if (!rotating && fabsf(filtered) < 0.12f) {
                motion->noise[motion->noise_index] = filtered;
                motion->noise_index = (motion->noise_index + 1u) % 10u;
                if (motion->noise_count < 10u) {
                    ++motion->noise_count;
                }
            }
            return;
        }
        motion->cycle_active = true;
        motion->cycle_start_ms = now;
        motion->cycle_threshold = threshold;
        motion->cycle_roll_deg = motion->roll_deg;
        motion->cycle_pitch_deg = motion->pitch_deg;
        motion->cycle_roll_range = 0.0f;
        motion->cycle_pitch_range = 0.0f;
        motion->cycle_rotation = rotating;
        motion->cycle_cross = cross;
        return;
    }

    motion->cycle_rotation |= rotating;
    motion->cycle_cross |= cross;
    motion->cycle_roll_range = fmaxf(motion->cycle_roll_range,
        fabsf(angle_difference(motion->roll_deg, motion->cycle_roll_deg)));
    motion->cycle_pitch_range = fmaxf(motion->cycle_pitch_range,
        fabsf(motion->pitch_deg - motion->cycle_pitch_deg));
    if (filtered > -motion->cycle_threshold) {
        return;
    }

    motion->cycle_active = false;
    const uint32_t period = motion->jumps ? now - motion->last_count_ms : 0u;
    if (motion->cycle_rotation && now - motion->cycle_start_ms >= 45u &&
        (!motion->jumps || period >= 300u)) {
        motion->style = jump_style(motion, period);
        motion->last_cycle_ms = period;
        motion->last_count_ms = now;
        if (motion->jumps < UINT32_MAX) {
            ++motion->jumps;
        }
    }
}

void lp_motion_update(struct lp_motion *motion, uint32_t now,
                      float ax, float ay, float az,
                      float gx, float gy, float gz)
{
    const float magnitude = sqrtf(ax * ax + ay * ay + az * az);
    if (!(magnitude <= 30.0f) || !(fabsf(gx) <= 40.0f) ||
        !(fabsf(gy) <= 40.0f) || !(fabsf(gz) <= 40.0f)) {
        return;
    }

    const uint32_t elapsed_ms = now - motion->last_sample_ms;
    if (!motion->orientation_valid || elapsed_ms > 30u) {
        motion->cycle_active = false;
        motion->yaw_count = 0;
        motion->yaw_index = 0;
        motion->orientation_valid = magnitude >= 0.85f && magnitude <= 1.15f;
        memset(motion->filter_state, 0, sizeof(motion->filter_state));
        if (!motion->orientation_valid) {
            return;
        }
        initialize_orientation(motion->quaternion, ax, ay, az);
    } else if (elapsed_ms != 0u) {
        update_orientation(motion, elapsed_ms * 0.001f, ax, ay, az, magnitude,
                           gx, gy, gz);
    } else {
        return;
    }
    motion->last_sample_ms = now;

    float gravity[3];
    gravity_direction(motion->quaternion, gravity);
    update_angles(motion, gravity);
    const float vertical = ax * gravity[0] + ay * gravity[1] +
                           az * gravity[2] - 1.0f;
    count_jumps(motion, now, vertical, gz);
}

static void reset_cycle_statistics(struct lp_ppg *ppg)
{
    ppg->cycle_samples = 0;
    ppg->mean_red = 0.0f;
    ppg->mean_ir = 0.0f;
    ppg->red_variation = 0.0f;
    ppg->ir_variation = 0.0f;
    ppg->covariance = 0.0f;
}

static void invalidate_ppg(struct lp_ppg *ppg)
{
    ppg->bpm_valid = false;
    ppg->spo2_valid = false;
    ppg->hrv_valid = false;
    ppg->interval_count = 0;
}

static void update_cycle_statistics(struct lp_ppg *ppg)
{
    const float red_diff = ppg->red_ac - ppg->mean_red;
    const float ir_diff = ppg->ir_ac - ppg->mean_ir;
    ++ppg->cycle_samples;
    ppg->mean_red += red_diff / ppg->cycle_samples;
    ppg->mean_ir += ir_diff / ppg->cycle_samples;
    ppg->red_variation += red_diff * (ppg->red_ac - ppg->mean_red);
    ppg->ir_variation += ir_diff * (ppg->ir_ac - ppg->mean_ir);
    ppg->covariance += red_diff * (ppg->ir_ac - ppg->mean_ir);
}

static void update_intervals(struct lp_ppg *ppg, uint32_t interval)
{
    ppg->intervals_ms[ppg->interval_index] = (uint16_t)interval;
    ppg->interval_index = (ppg->interval_index + 1u) % 8u;
    if (ppg->interval_count < 8u) {
        ++ppg->interval_count;
    }

    unsigned index = (ppg->interval_index + 8u - ppg->interval_count) % 8u;
    int previous = ppg->intervals_ms[index];
    float squared_differences = 0.0f;
    unsigned total = (unsigned)previous;
    for (unsigned i = 1; i < ppg->interval_count; ++i) {
        index = (index + 1u) % 8u;
        const int current = ppg->intervals_ms[index];
        const float difference = (float)(current - previous);
        squared_differences += difference * difference;
        total += (unsigned)current;
        previous = current;
    }
    ppg->bpm = 60000.0f * ppg->interval_count / total;
    ppg->bpm_valid = ppg->interval_count >= 2u;
    ppg->hrv_valid = ppg->interval_count >= 4u;
    if (ppg->hrv_valid) {
        ppg->hrv_rmssd_ms = sqrtf(squared_differences / (ppg->interval_count - 1u));
    }
}

static void evaluate_beat(struct lp_ppg *ppg, uint32_t interval)
{
    const float red_rms = sqrtf(ppg->red_variation / ppg->cycle_samples);
    const float ir_rms = sqrtf(ppg->ir_variation / ppg->cycle_samples);
    const float red_perfusion = red_rms / ppg->red_dc;
    const float ir_perfusion = ir_rms / ppg->ir_dc;
    const float corr_bound = 0.75f * red_rms * ir_rms * ppg->cycle_samples;

    const bool signal_ok = ppg->cycle_samples >= 10u &&
                           red_perfusion >= 0.001f && red_perfusion <= 0.20f &&
                           ir_perfusion >= 0.001f && ir_perfusion <= 0.20f &&
                           ppg->covariance >= corr_bound;
    ppg->spo2_valid = false;
    if (!signal_ok) {
        invalidate_ppg(ppg);
        return;
    }

    update_intervals(ppg, interval);
    const float ratio = red_perfusion / ir_perfusion;
    if (ratio >= 0.35f && ratio <= 1.30f && ppg->bpm_valid) {
        /* Illustrative Maxim reference curve. Calibrate with the actual optics. */
        const float estimate = 94.845f + ratio * (30.354f - 45.060f * ratio);
        ppg->spo2_pct = fminf(100.0f, estimate);
        ppg->spo2_valid = true;
    }
}

void lp_ppg_update(struct lp_ppg *ppg, uint32_t now, uint32_t red, uint32_t ir)
{
    if (red < 5000u || ir < 5000u || red >= 260000u || ir >= 260000u) {
        *ppg = (struct lp_ppg){0};
        return;
    }

    const uint32_t elapsed_ms = now - ppg->last_sample_ms;
    if (!ppg->initialized || elapsed_ms > 100u) {
        *ppg = (struct lp_ppg){
            .red_dc = (float)red,
            .ir_dc = (float)ir,
            .last_sample_ms = now,
            .start_ms = now,
            .initialized = true,
        };
        return;
    }
    if (elapsed_ms == 0u) {
        return;
    }
    ppg->last_sample_ms = now;

    /* DC baseline, then a low-pass filter on the AC residual. */
    const float dc_alpha = elapsed_ms / (800.0f + elapsed_ms);
    const float ac_alpha = elapsed_ms / (35.0f + elapsed_ms);
    ppg->red_dc += dc_alpha * ((float)red - ppg->red_dc);
    ppg->ir_dc += dc_alpha * ((float)ir - ppg->ir_dc);
    ppg->red_ac += ac_alpha * ((float)red - ppg->red_dc - ppg->red_ac);
    ppg->ir_ac += ac_alpha * ((float)ir - ppg->ir_dc - ppg->ir_ac);
    ppg->ir_power += dc_alpha * (ppg->ir_ac * ppg->ir_ac - ppg->ir_power);

    if (now - ppg->start_ms < 700u) {
        return;
    }
    if (ppg->have_beat && now - ppg->last_beat_ms > 1500u) {
        ppg->have_beat = false;
        invalidate_ppg(ppg);
        reset_cycle_statistics(ppg);
    }

    const float threshold = fmaxf(ppg->ir_dc * 0.001f, sqrtf(ppg->ir_power) * 0.35f);
    if (ppg->ir_ac < -threshold) {
        ppg->armed = true;
    }
    const uint32_t interval = now - ppg->last_beat_ms;
    if (ppg->armed && ppg->ir_ac > threshold) {
        if (ppg->have_beat) {
            if (interval >= 300u && ppg->cycle_samples) {
                evaluate_beat(ppg, interval);
            } else {
                invalidate_ppg(ppg);
            }
        }
        ppg->last_beat_ms = now;
        ppg->have_beat = true;
        ppg->armed = false;
        reset_cycle_statistics(ppg);
    }
    if (ppg->have_beat) {
        update_cycle_statistics(ppg);
    }
}
