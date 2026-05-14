#pragma once

#ifdef ROBOT_MODEL_G1

#include <algorithm>
#include <cmath>

#include "motion_command.h"
#include "point_2d.h"

inline MotionCommand g1BallSearchCommand(bool last_seen_left) {
    constexpr float kSearchYawSpeed = 0.60f;
    return MotionCommand::Walk(
            {.dx = 0.0f, .dy = 0.0f, .da = last_seen_left ? kSearchYawSpeed : -kSearchYawSpeed},
            last_seen_left ? HeadFocus::BALL_SEARCH_LEFT : HeadFocus::BALL_SEARCH_RIGHT);
}

inline MotionCommand g1TurnForwardToBallCommand(const point_2d& ball_pos_rel,
                                                HeadFocus focus = HeadFocus::BALL) {
    constexpr float kMaxForward = 1.00f;
    constexpr float kMinForward = 0.10f;
    constexpr float kMaxYaw = 0.50f;
    constexpr float kWalkAngle = 0.50f;  // about 29 deg

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
    constexpr float kMaxForward = 1.00f;
    constexpr float kPushForward = 0.80f;
    constexpr float kMinForward = 0.10f;
    constexpr float kMaxYaw = 0.45f;
    constexpr float kCenterY = 0.10f;
    constexpr float kAlignAngle = 0.24f;  // about 14 deg
    constexpr float kWalkAngle = 0.45f;   // about 26 deg

    const float ball_distance = ball_pos_rel.norm();
    const float ball_angle = ball_pos_rel.to_direction();
    const float abs_y = std::abs(ball_pos_rel.y);
    const float abs_angle = std::abs(ball_angle);

    float dx = 0.0f;
    float dy = 0.0f;
    float da = std::clamp(ball_angle * 1.15f, -kMaxYaw, kMaxYaw);

    if (abs_angle > kWalkAngle) {
        // First rotate the body toward the ball. G1 high-level sideways walking is unreliable
        // for ball approach, so do not sidestep here.
        dx = 0.0f;
    } else if (ball_distance > 0.55f) {
        // Approach even with a modest angle, while continuously turning toward the ball.
        const float alignment = std::clamp(1.0f - abs_angle / kWalkAngle, 0.25f, 1.0f);
        dx = std::clamp(ball_pos_rel.x * alignment, kMinForward, kMaxForward);
    } else if (abs_y <= kCenterY || abs_angle <= kAlignAngle) {
        // Near and centered: push through the ball.
        dx = kPushForward;
        da = std::clamp(ball_angle * 0.70f, -0.25f, 0.25f);
    }

    return MotionCommand::Walk({.dx = dx, .dy = dy, .da = da}, HeadFocus::BALL);
}

#endif  // ROBOT_MODEL_G1
