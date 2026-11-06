#include "workout.h"

#define TIMED_GOAL_MS 60000u
#define COUNT_GOAL 100u

void lp_workout_select(struct lp_workout *workout, int direction)
{
    if (workout->state == LP_ACTIVE || direction == 0) {
        return;
    }

    int step = direction > 0 ? 1 : -1;
    workout->mode = (workout->mode + LP_MODE_COUNT + step) % LP_MODE_COUNT;
    workout->state = LP_READY;
    workout->jumps = 0;
}

void lp_workout_toggle(struct lp_workout *workout, uint32_t now_ms, uint32_t count)
{
    if (workout->state == LP_ACTIVE) {
        workout->state = LP_READY;
        return;
    }

    workout->state = LP_ACTIVE;
    workout->started_ms = now_ms;
    workout->starting_count = count;
    workout->jumps = 0;
}

bool lp_workout_update(struct lp_workout *workout, uint32_t now_ms, uint32_t count)
{
    if (workout->state != LP_ACTIVE) {
        return false;
    }

    workout->jumps = count - workout->starting_count;
    if ((workout->mode == LP_TIMED && now_ms - workout->started_ms >= TIMED_GOAL_MS) ||
        (workout->mode == LP_COUNT && workout->jumps >= COUNT_GOAL)) {
        workout->state = LP_COMPLETE;
        return true;
    }
    return false;
}
