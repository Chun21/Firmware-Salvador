#include "localize.h"

#ifdef ROBOT_MODEL_G1
#include "moveballgoalorder.h"
#include "moveballorder.h"
#endif

MotionCommand LocalizeAgent::proceed(std::shared_ptr<Order> order) {
    const int64_t now = time_us();
    std::optional<LocPosition> loc_position = loc_position_sub.latestIfExists();
    float locQual = loc_position ? loc_position->quality : 0.0f;

#ifdef ROBOT_MODEL_G1
    if (locQual >= minQual) {
        g1_last_good_loc_us = now;
    }
    const std::optional<RelBall> g1_current_ball = rel_ball_sub.latest();
    const bool g1_has_recent_loc =
            g1_last_good_loc_us != 0 && now - g1_last_good_loc_us <= kG1LocTtlUs;
    const bool g1_has_fresh_ball_for_dribble =
            g1_current_ball && g1_current_ball->ball_age_us < 1_s && g1_has_recent_loc &&
            isOrder<MoveBallOrder, MoveBallGoalOrder>(order);
    if (g1_has_fresh_ball_for_dribble) {
        return MotionCommand::Nothing;
    }
#else
    (void)order;
#endif

    if (locQual < minQual || bestQual < minQualHysteresis) {
        int64_t lastLocAge = now - lastLocTime;
        int64_t lastLocTimeCycle = lastLocAge % (standTime + turnTime);
        int cycle_number = lastLocAge / (standTime + turnTime);

        // Just try to find the position by looking around.
        MotionCommand mc = MotionCommand::Stand(HeadFocus::LOC);

        if (cycle_number != 0 && cycle_number % 4 == 0 && lastLocTimeCycle < 3_s) {
            mc = MotionCommand::Walk({.dx = 0.35, .dy = 0, .da = 0}, HeadFocus::LOC);
        }

        // If we stood there and haven't seen anything rotate slowly around ourself.
        if (lastLocTimeCycle > standTime) {
            mc = MotionCommand::Walk({.dx = 0, .dy = 0, .da = 1.f}, HeadFocus::LOC);
        }

        bestQual = locQual;
        return mc;
    }

    lastLocTime = now;
    return MotionCommand::Nothing;
}
