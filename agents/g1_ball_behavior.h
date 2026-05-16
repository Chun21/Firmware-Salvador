#pragma once

#ifdef ROBOT_MODEL_G1

#include <algorithm>
#include <cmath>
#include <optional>

#include "motion_command.h"
#include "point_2d.h"

inline MotionCommand g1BallSearchCommand(bool last_seen_left) {
    constexpr float kSearchYawSpeed = 0.60f;
    return MotionCommand::Walk(
            {.dx = 0.0f, .dy = 0.0f, .da = last_seen_left ? kSearchYawSpeed : -kSearchYawSpeed},
            last_seen_left ? HeadFocus::BALL_SEARCH_LEFT : HeadFocus::BALL_SEARCH_RIGHT);
}

inline std::optional<MotionCommand> g1StrikerLostBallSearchCommand(bool has_current_ball,
                                                                   bool is_striker_order,
                                                                   bool has_fresh_fallback,
                                                                   bool last_seen_left) {
    if (!is_striker_order || has_current_ball || has_fresh_fallback) {
        return std::nullopt;
    }
    return g1BallSearchCommand(last_seen_left);
}

inline MotionCommand g1TurnForwardToBallCommand(const point_2d& ball_pos_rel,
                                                HeadFocus focus = HeadFocus::BALL) {
    constexpr float kMaxForward = 0.80f;
    constexpr float kMinForward = 0.08f;
    constexpr float kMaxYaw = 0.50f;
    constexpr float kWalkAngle = 0.35f;  // about 20 deg

    const float ball_distance = ball_pos_rel.norm();
    const float ball_angle = ball_pos_rel.to_direction();
    const float abs_angle = std::abs(ball_angle);

    float dx = 0.0f;
    const float da = std::clamp(ball_angle * 1.35f, -kMaxYaw, kMaxYaw);

    if (abs_angle <= kWalkAngle) {
        const float alignment = std::clamp(1.0f - abs_angle / kWalkAngle, 0.25f, 1.0f);
        dx = std::clamp(ball_pos_rel.x * alignment, kMinForward, kMaxForward);
    }

    // G1 ball approach must never sidestep: rotate the body toward the ball and walk forward.
    return MotionCommand::Walk({.dx = dx, .dy = 0.0f, .da = da}, focus);
}

inline MotionCommand g1ConservativeDribbleCommand(const point_2d& ball_pos_rel) {
    constexpr float kMaxForward = 0.80f;
    constexpr float kPushForward = 0.80f;
    constexpr float kMinForward = 0.08f;
    constexpr float kMaxLateral = 0.08f;
    constexpr float kMaxYaw = 0.45f;
    constexpr float kCenterY = 0.07f;
    constexpr float kAlignAngle = 0.16f;  // about 9 deg
    constexpr float kWalkAngle = 0.30f;   // about 17 deg

    const float ball_distance = ball_pos_rel.norm();
    const float ball_angle = ball_pos_rel.to_direction();
    const float abs_y = std::abs(ball_pos_rel.y);
    const float abs_angle = std::abs(ball_angle);

    float dx = 0.0f;
    float dy = 0.0f;
    float da = std::clamp(ball_angle * 1.15f, -kMaxYaw, kMaxYaw);
    const float lateral_alignment =
            std::clamp(1.0f - std::max(0.0f, abs_y - kCenterY) / 0.20f, 0.35f, 1.0f);

    if (abs_angle > kWalkAngle) {
        // First rotate the body toward the ball. Keep lateral zero while the ball is far off-axis
        // because G1 high-level sideways walking is less reliable than turn-then-forward there.
        dx = 0.0f;
    } else if (ball_distance > 0.55f) {
        // Approach even with a modest angle, while continuously turning toward the ball.
        const float alignment = std::clamp(1.0f - abs_angle / kWalkAngle, 0.25f, 1.0f);
        dx = std::clamp(ball_pos_rel.x * alignment * lateral_alignment, kMinForward, kMaxForward);
        dy = std::clamp(ball_pos_rel.y * 0.45f, -kMaxLateral, kMaxLateral);
    } else if (abs_y <= kCenterY || abs_angle <= kAlignAngle) {
        // Near and centered: push through the ball. Allow only a very small bounded lateral
        // correction so small G1 foot-placement bias does not make it pass the ball on one side.
        dx = kPushForward * lateral_alignment;
        dy = std::clamp(ball_pos_rel.y * 0.45f, -kMaxLateral, kMaxLateral);
        da = std::clamp(ball_angle * 0.70f, -0.25f, 0.25f);
    }

    return MotionCommand::Walk({.dx = dx, .dy = dy, .da = da}, HeadFocus::BALL);
}

inline std::optional<MotionCommand> g1StrikerLostBallFallbackCommand(bool has_current_ball,
                                                                     bool is_striker_order,
                                                                     const point_2d& fallback_ball) {
    if (!is_striker_order || has_current_ball) {
        return std::nullopt;
    }
    if (fallback_ball.norm() < 0.70f && fallback_ball.x > 0.0f) {
        // When the ball is very close it is easy for the bottom of the camera image to lose it for
        // a few frames. Keep pushing through the last fresh estimate instead of handing control
        // back to generic positioning/search, which makes G1 turn away from the ball.
        return MotionCommand::Walk(
                {.dx = 0.60f,
                 .dy = 0.0f,
                 .da = std::clamp(fallback_ball.to_direction() * 0.5f, -0.20f, 0.20f)},
                HeadFocus::BALL);
    }
    return g1ConservativeDribbleCommand(fallback_ball);
}

#endif  // ROBOT_MODEL_G1
