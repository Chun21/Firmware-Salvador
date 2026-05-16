#include "search_ball.h"

#include <robot_time.h>

#include "keepgoalorder.h"
#include "moveballgoalorder.h"
#include "moveballorder.h"
#include "walktopositionorder.h"

#ifdef ROBOT_MODEL_G1
#include "g1_ball_behavior.h"
#endif

MotionCommand BallSearchAgent::proceed(std::shared_ptr<Order> order) {
    std::optional<RelBall> ball = ball_sub.latest();
    updateBallDirectionMemory(ball);
    if (!isOrder<MoveBallOrder, MoveBallGoalOrder, WalkToPositionOrder, KeepGoalOrder>(order))
        return MotionCommand::Nothing;

    const int64_t now = time_us();
    int64_t ballNotSeenTime = now - ballLastSeen;
    bool is_striker = (isOrder<WalkToPositionOrder>(order) &&
                       dynamic_cast<WalkToPositionOrder*>(order.get())->mode ==
                               WalkToPositionOrder::Mode::STRIKER) ||
                      isOrder<MoveBallOrder>(order) || isOrder<MoveBallGoalOrder>(order);
    std::optional<TeamComData> striker = striker_sub.latest();
#ifdef ROBOT_MODEL_G1
    const int64_t kG1BallFallbackTtl = htwk::g1::ballFallbackTtlUs();
    LocPosition g1_loc_position = pos_sub.latest();
    const std::optional<point_2d> g1_fallback_ball = g1_ball_fallback.updateAndSelect(
            ball, g1_loc_position, striker, now, kG1BallFallbackTtl);
    if (auto search_command = g1StrikerLostBallSearchCommand(
                ball.has_value(), is_striker, g1_fallback_ball.has_value(),
                ballLastSide == Side::LEFT)) {
        return *search_command;
    }
    const bool g1_should_defer_fallback_to_dribble =
            !ball && g1_fallback_ball && isOrder<MoveBallOrder, MoveBallGoalOrder>(order);
    if (g1_should_defer_fallback_to_dribble) {
        // Let DribbleAgent consume fresh own/team fallback ball estimates. Keeping BallSearch
        // before Dribble preserves normal lost-ball search priority, but prevents BallSearch from
        // stealing near-ball fallback control when the camera briefly loses the ball at the feet.
        return MotionCommand::Nothing;
    }
    if (g1_fallback_ball) {
        if (auto fallback_command = g1StrikerLostBallFallbackCommand(
                    ball.has_value(), is_striker, *g1_fallback_ball)) {
            return *fallback_command;
        }
    }

    bool g1_force_body_search = false;
#endif
    if (isOrder<WalkToPositionOrder>(order)) {
        auto* wtp_order = dynamic_cast<WalkToPositionOrder*>(order.get());
        if (wtp_order->mode == WalkToPositionOrder::Mode::USE_A ||
            wtp_order->mode == WalkToPositionOrder::Mode::FOCUS_DIRECTION)
            return MotionCommand::Nothing;
        if (wtp_order->mode == WalkToPositionOrder::Mode::STRIKER &&
            pos_sub.latest().position.point().dist(wtp_order->pos.point()) > 1.f) {
            return MotionCommand::Nothing;
        }
#ifdef ROBOT_MODEL_G1
        if (!ball && wtp_order->mode == WalkToPositionOrder::Mode::STRIKER &&
            g1_loc_position.position.point().dist(wtp_order->pos.point()) <= 0.3f) {
            g1_force_body_search = true;
        }
#endif
        if (wtp_order->mode == WalkToPositionOrder::Mode::SUPPORTER && striker && striker->ball)
            return MotionCommand::Nothing;
    }
    if (isOrder<KeepGoalOrder>(order) && striker && striker->ball)
        return MotionCommand::Nothing;

#ifdef ROBOT_MODEL_G1
    if (!ball && g1_fallback_ball) {
        return MotionCommand::Nothing;
    }

    if (g1_force_body_search) {
        return g1BallSearchCommand(ballLastSide == Side::LEFT);
    }
#endif

    const int64_t nearResponseTime = is_striker ? 2._s : .75_s;
    const int64_t farResponseTime = 2._s;
    const int64_t nearFastSearchTime = 2._s;
    const int64_t longResponseTime = 8._s;
    const int64_t hypoNearWaitTime = 0.3_s;
    const int64_t hypoFarWaitTime = 2._s;

    int64_t hypoNotSeenTime = now - hypoLastSeen;

    int64_t responseTime = lastBallDist < maxBallDist ? nearResponseTime : farResponseTime;
    int hypoWaitTime = lastBallDist < maxBallDist ? hypoNearWaitTime : hypoFarWaitTime;

    if (ballNotSeenTime < responseTime)
        return MotionCommand::Nothing;
    // We've just seen a hypothesis, maybe that's the ball, just stop for a bit.
    // if (hypoNotSeenTime < ballNotSeenTime && hypoNotSeenTime < hypoWaitTime)
    //     return MotionCommand::Stand(HeadFocus::BALL);
    if (lastBallDist < maxBallDist && ballNotSeenTime < nearFastSearchTime) {
        // We just lost the ball, rotate fast back where it just was.
        if (ballLastSide == Side::LEFT)
            return MotionCommand::Walk({.dx = 0, .dy = 0.0, .da = 1.9f},
                                       HeadFocus::BALL_SEARCH_LEFT);
        else
            return MotionCommand::Walk({.dx = 0, .dy = 0.0, .da = -1.9f},
                                       HeadFocus::BALL_SEARCH_RIGHT);
    } else if (ballNotSeenTime < longResponseTime) {
        // We just lost the ball for a longer time, rotate slower to find it.
        if (ballLastSide == Side::LEFT)
            return MotionCommand::Walk({.dx = 0, .dy = 0.0, .da = 1.6f},
                                       HeadFocus::BALL_SEARCH_LEFT);
        else
            return MotionCommand::Walk({.dx = 0, .dy = 0.0, .da = -1.6f},
                                       HeadFocus::BALL_SEARCH_RIGHT);
    } else {
        // The ball is lost for quite some time. Search even more slowly so the images are
        // clearer.
        if (ballLastSide == Side::LEFT)
            return MotionCommand::Walk({.dx = 0.0f, .dy = 0.0, .da = 1.0f},
                                       HeadFocus::BALL_SEARCH_LEFT);
        else
            return MotionCommand::Walk({.dx = 0.0f, .dy = 0.0, .da = -1.0f},
                                       HeadFocus::BALL_SEARCH_RIGHT);
    }
}
