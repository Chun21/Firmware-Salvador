#pragma once

#ifdef ROBOT_MODEL_G1

#include <algorithm>
#include <cmath>

#include "motion_command.h"
#include "point_2d.h"

inline MotionCommand g1ReadyWalkCommand(const point_2d& dest_rel, float final_yaw_error,
                                        HeadFocus focus) {
    constexpr float kMaxForwardSpeed = 0.50f;
    constexpr float kMaxYawSpeed = 0.60f;
    constexpr float kMaxFinalYawSpeed = 0.60f;
    constexpr float kTurnInPlaceAngle = 35.0f * static_cast<float>(M_PI) / 180.0f;
    constexpr float kFinalTurnDistance = 0.35f;
    constexpr float kMinForwardDistance = 0.10f;
    constexpr float kFinalYawDeadband = 8.0f * static_cast<float>(M_PI) / 180.0f;

    const float distance = dest_rel.norm();
    if (distance < kFinalTurnDistance) {
        if (std::abs(final_yaw_error) < kFinalYawDeadband) {
            return MotionCommand::Stand(focus);
        }
        // At the Ready target, always correct the final heading. The target heading in kickoff
        // configs is the opponent-goal direction; do not suppress large yaw errors here, otherwise
        // G1 may arrive at the spot but keep facing sideways/backwards.
        return MotionCommand::Walk(
                {.dx = 0.0f,
                 .dy = 0.0f,
                 .da = std::clamp(final_yaw_error * 1.0f,
                                  -kMaxFinalYawSpeed,
                                  kMaxFinalYawSpeed)},
                focus);
    }

    const float target_angle = dest_rel.to_direction();
    const float abs_target_angle = std::abs(target_angle);

    float dx = 0.0f;
    if (distance > kMinForwardDistance && abs_target_angle < kTurnInPlaceAngle) {
        const float forward_component = distance * std::cos(target_angle);
        dx = std::clamp(forward_component, 0.0f, kMaxForwardSpeed);
    }

    return MotionCommand::Walk(
            {.dx = dx,
             .dy = 0.0f,
             .da = std::clamp(target_angle * 1.4f, -kMaxYawSpeed, kMaxYawSpeed)},
            focus);
}

#endif  // ROBOT_MODEL_G1
