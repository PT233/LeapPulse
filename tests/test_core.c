/* Keep these assertions enabled in Release builds. */
#undef NDEBUG
#include <assert.h>
#include <math.h>

#include "leappulse.h"

#define TAU 6.28318530718f

static void test_attitude(void)
{
    struct lp_motion motion = {0};
    for (uint32_t t = 0; t <= 500u; t += 5u) {
        lp_motion_update(&motion, t, -0.5f, 0.0f, 0.8660254f, 0.0f, 0.0f, 0.0f);
    }

    assert(motion.orientation_valid);
    assert(fabsf(motion.pitch_deg - 30.0f) < 0.01f);
    assert(fabsf(motion.roll_deg) < 0.01f);
    assert(motion.jumps == 0u);

    struct lp_motion heading = {0};
    lp_motion_update(&heading, 0u, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    for (uint32_t t = 5u; t <= 500u; t += 5u) {
        lp_motion_update(&heading, t, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, TAU * 0.5f);
    }
    assert(fabsf(heading.yaw_deg - 90.0f) < 0.05f);

    float squared_norm = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        squared_norm += heading.quaternion[i] * heading.quaternion[i];
    }
    assert(fabsf(squared_norm - 1.0f) < 0.0001f);
}

static uint32_t replay_swing(uint32_t start, float frequency, float z_rate)
{
    struct lp_motion motion = {0};
    for (uint32_t elapsed = 0u; elapsed < 5000u; elapsed += 5u) {
        const float phase = TAU * frequency * elapsed * 0.001f;
        const float vertical = 1.0f + 0.42f * sinf(phase);
        lp_motion_update(&motion, start + elapsed, 0.0f, 0.0f, vertical,
                         0.0f, 0.0f, z_rate);
    }
    return motion.jumps;
}

static void test_jump_cycles(void)
{
    assert(replay_swing(0u, 2.0f, 5.0f) == 10u);
    assert(replay_swing(UINT32_MAX - 1000u, 2.0f, 5.0f) == 10u);
    assert(replay_swing(0u, 2.0f, 2.0f) == 0u);
    assert(replay_swing(0u, 45.0f, 5.0f) == 0u);

    /* The refractory period suppresses adjacent cycles only 250 ms apart. */
    assert(replay_swing(0u, 4.0f, 5.0f) == 10u);

    struct lp_motion motion = {0};
    lp_motion_update(&motion, 0u, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5.0f);
    for (uint32_t t = 5u; t <= 100u; t += 5u) {
        lp_motion_update(&motion, t, 0.0f, 0.0f, 1.42f, 0.0f, 0.0f, 5.0f);
    }
    assert(motion.cycle_active);

    lp_motion_update(&motion, 200u, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 5.0f);
    for (uint32_t t = 205u; t <= 300u; t += 5u) {
        lp_motion_update(&motion, t, 0.0f, 0.0f, 0.58f, 0.0f, 0.0f, 5.0f);
    }
    assert(motion.jumps == 0u);
}

static enum lp_jump_style replay_style(float frequency, float roll_amplitude,
                                      float pitch_amplitude, float yaw_rate)
{
    struct lp_motion motion = {0};
    for (uint32_t t = 0u; t < 5000u; t += 5u) {
        const float angular_frequency = TAU * frequency;
        const float phase = angular_frequency * t * 0.001f;
        const float roll = roll_amplitude * sinf(phase);
        const float pitch = pitch_amplitude * sinf(phase);
        const float roll_rate = roll_amplitude * angular_frequency * cosf(phase);
        const float pitch_rate = pitch_amplitude * angular_frequency * cosf(phase);
        const float vertical = 1.0f + 0.42f * sinf(phase);
        lp_motion_update(&motion, t,
                         -vertical * sinf(pitch),
                         vertical * sinf(roll) * cosf(pitch),
                         vertical * cosf(roll) * cosf(pitch),
                         roll_rate - yaw_rate * sinf(pitch),
                         pitch_rate * cosf(roll) + yaw_rate * sinf(roll) * cosf(pitch),
                         -pitch_rate * sinf(roll) + yaw_rate * cosf(roll) * cosf(pitch));
    }
    return motion.style;
}

static void test_jump_styles(void)
{
    assert(replay_style(2.0f, 0.0f, 0.66f, 5.5f) == LP_JUMP_SINGLE);
    assert(replay_style(2.5f, 0.96f, 0.0f, 5.5f) == LP_JUMP_DOUBLE);
    assert(replay_style(2.0f, 0.0f, 0.0f, 17.4f) == LP_JUMP_CROSS);
    assert(replay_style(2.0f, 0.0f, 0.0f, 5.5f) == LP_JUMP_UNKNOWN);
}

static void replay_ppg(struct lp_ppg *ppg, uint32_t start, uint32_t duration,
                       float period_ms, float red_amplitude)
{
    for (uint32_t elapsed = 0u; elapsed < duration; elapsed += 10u) {
        const float wave = sinf(TAU * elapsed / period_ms);
        const uint32_t red = (uint32_t)(50000.0f + red_amplitude * wave);
        const uint32_t ir = (uint32_t)(80000.0f + 2400.0f * wave);
        lp_ppg_update(ppg, start + elapsed, red, ir);
    }
}

static void test_ppg(void)
{
    struct lp_ppg ppg = {0};
    replay_ppg(&ppg, UINT32_MAX - 3000u, 10000u, 800.0f, 1000.0f);
    assert(ppg.bpm_valid && ppg.spo2_valid && ppg.hrv_valid);
    assert(fabsf(ppg.bpm - 75.0f) < 0.5f);
    assert(fabsf(ppg.spo2_pct - 95.054f) < 0.5f);
    assert(ppg.hrv_rmssd_ms < 15.0f);

    lp_ppg_update(&ppg, 7000u, 50000u, 262143u);
    assert(!ppg.bpm_valid && !ppg.spo2_valid && !ppg.hrv_valid);

    replay_ppg(&ppg, 8000u, 6000u, 800.0f, -1000.0f);
    assert(!ppg.bpm_valid && !ppg.spo2_valid && !ppg.hrv_valid);

    replay_ppg(&ppg, 15000u, 6000u, 250.0f, 1000.0f);
    assert(!ppg.bpm_valid && !ppg.spo2_valid && !ppg.hrv_valid);

    replay_ppg(&ppg, 22000u, 10000u, 800.0f, 1000.0f);
    assert(ppg.bpm_valid && ppg.spo2_valid && ppg.hrv_valid);
    for (uint32_t t = 32000u; t < 35000u; t += 10u) {
        lp_ppg_update(&ppg, t, 50000u, 80000u);
    }
    assert(!ppg.bpm_valid && !ppg.spo2_valid && !ppg.hrv_valid);
}

int main(void)
{
    test_attitude();
    test_jump_cycles();
    test_jump_styles();
    test_ppg();
    return 0;
}
