#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include "workout.h"

int main(void)
{
    struct lp_workout workout = {0};

    lp_workout_toggle(&workout, 42, 3);
    assert(workout.state == LP_ACTIVE);
    lp_workout_select(&workout, 1);
    assert(workout.mode == LP_FREE);
    assert(!lp_workout_update(&workout, 99999, 104));
    assert(workout.jumps == 101);
    lp_workout_toggle(&workout, 100000, 104);
    assert(workout.state == LP_READY && workout.jumps == 101);

    lp_workout_select(&workout, 1);
    assert(workout.mode == LP_TIMED);
    uint32_t start = UINT32_MAX - 2000;
    lp_workout_toggle(&workout, start, 104);
    assert(!lp_workout_update(&workout, start + 59999u, 150));
    assert(lp_workout_update(&workout, start + 60000u, 151));
    assert(!lp_workout_update(&workout, start + 60001u, 152));
    assert(workout.state == LP_COMPLETE && workout.jumps == 47);

    lp_workout_select(&workout, 1);
    assert(workout.mode == LP_COUNT);
    lp_workout_toggle(&workout, 10, UINT32_MAX - 9);
    assert(!lp_workout_update(&workout, 200, 89));
    assert(lp_workout_update(&workout, 210, 90));
    lp_workout_select(&workout, 1);
    assert(workout.mode == LP_FREE && workout.jumps == 0);
    lp_workout_select(&workout, -1);
    assert(workout.mode == LP_COUNT);
    return 0;
}
