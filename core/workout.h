#ifndef LEAPPULSE_WORKOUT_H
#define LEAPPULSE_WORKOUT_H

#include <stdbool.h>
#include <stdint.h>

enum lp_mode {
    LP_FREE,
    LP_TIMED,
    LP_COUNT,
    LP_MODE_COUNT
};

enum lp_workout_state {
    LP_READY,
    LP_ACTIVE,
    LP_COMPLETE
};

struct lp_workout {
    enum lp_mode mode;
    enum lp_workout_state state;
    uint32_t jumps;
    uint32_t started_ms;
    uint32_t starting_count;
};

void lp_workout_select(struct lp_workout *workout, int direction);
void lp_workout_toggle(struct lp_workout *workout, uint32_t now_ms, uint32_t count);
bool lp_workout_update(struct lp_workout *workout, uint32_t now_ms, uint32_t count);

#endif
