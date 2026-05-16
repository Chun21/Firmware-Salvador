#include "head_control.h"

#include <algorithm>
#include <array>

#include <loguru.hpp>

#include "gc_state.h"
#include "localization_utils.h"
#include "robot_time.h"
#include "soccerfield.h"
#include "stl_ext.h"

namespace htwk {

namespace {

constexpr float kDefaultBallSearchPitchCenter = 29_deg;
constexpr float kDefaultBallSearchPitchAmplitude = 19_deg;
constexpr float kG1BallSearchPitchCenter = -22.5_deg;
constexpr float kG1BallSearchPitchAmplitude = 37.5_deg;

constexpr float ballSearchPitchCenter() {
#ifdef ROBOT_MODEL_G1
    return kG1BallSearchPitchCenter;
#else
    return kDefaultBallSearchPitchCenter;
#endif
}

constexpr float ballSearchPitchAmplitude() {
#ifdef ROBOT_MODEL_G1
    return kG1BallSearchPitchAmplitude;
#else
    return kDefaultBallSearchPitchAmplitude;
#endif
}

#ifdef ROBOT_MODEL_G1
YawPitch g1RobocupDeployStyleLocScan(float scan_time_s) {
    struct Phase {
        YawPitch target;
        float duration_s;
    };

    // Same phase sequence and timing as /home/eai/robocup_deploy roboCup_sdk camFindBall:
    // (0,0) 200ms, then each of (35,-15), (35,15), (-35,-15), (-35,15), (0,0)
    // for 500ms. In robocup_deploy this is a one-shot interpolation; after the scan finishes,
    // the head stays centered while the body may continue searching. Do not continuously
    // oscillate the G1 head.
    const std::array<Phase, 6> kPhases{{
            {YawPitch{0.0f, 0.0f}, 0.2f},
            {YawPitch{35_deg, -15_deg}, 0.5f},
            {YawPitch{35_deg, 15_deg}, 0.5f},
            {YawPitch{-35_deg, -15_deg}, 0.5f},
            {YawPitch{-35_deg, 15_deg}, 0.5f},
            {YawPitch{0.0f, 0.0f}, 0.5f},
    }};

    YawPitch previous{0.0f, 0.0f};
    float elapsed = scan_time_s;
    for (const Phase& phase : kPhases) {
        if (elapsed <= phase.duration_s) {
            const float alpha = std::clamp(elapsed / phase.duration_s, 0.0f, 1.0f);
            return YawPitch{
                    previous.yaw + (phase.target.yaw - previous.yaw) * alpha,
                    previous.pitch + (phase.target.pitch - previous.pitch) * alpha,
            };
        }
        elapsed -= phase.duration_s;
        previous = phase.target;
    }
    return YawPitch{0.0f, 0.0f};
}
#endif

}  // namespace

/**
 * @param w current position
 * @param t slowdown (higher value slows more)
 * @param gain maximum yaw-angle of head
 */
float HeadControl::smoothTriAng(float w, sta_settings s) {
    // TODO: Replace this monster with something understandable. Do we even want to break? Why not
    // linear acceleration/deceleration?
    w *= s.w_fac;
    while (w < 0)
        w += M_PIf * 2.f;
    while (w > M_PIf * 2.f)
        w -= M_PIf * 2.f;
    if ((w > s.t && w < M_PIf - s.t) || (w > M_PIf + s.t && w < M_PIf * 2.f - s.t)) {
        if (w < M_PIf) {
            return s.gain * (w / M_PIf * 2.f - 1);
        } else {
            return s.gain * ((M_PIf * 2.f - w) / M_PIf * 2.f - 1.f);
        }
    } else {
        float r = s.t;
        float a = 1.f / 2.f / r;
        float b = -r / 2.f + s.t;
        if (w <= s.t) {
            float x = w;
            return s.gain * ((a * x * x + b) / M_PIf * 2.f - 1.f);
        } else if (w <= M_PIf) {
            float x = M_PIf - w;
            return s.gain * ((1.f - (a * x * x + b) / M_PIf) * 2.f - 1.f);
        } else if (w >= M_PIf * 2.f - s.t) {
            float x = w - M_PIf * 2.f;
            return s.gain * ((a * x * x + b) / M_PIf * 2.f - 1.f);
        } else if (w > M_PIf) {
            float x = w - M_PIf;
            return s.gain * ((1.f - (a * x * x + b) / M_PIf) * 2.f - 1.f);
        }
        return 0;
    }
}

float HeadControl::smoothTriAngStartTime(YawPitch head_pos, sta_settings sta_yaw,
                                         sta_settings sta_pitch) {
    float min_w = 0;
    float min_dist = std::numeric_limits<float>::infinity();
    for (float w = 0; w < M_PIf * 2.f / std::abs(sta_yaw.w_fac);
         w += 0.1f / std::abs(sta_yaw.w_fac)) {
        float dist = 0;
        if (sta_pitch.w_fac == 0) {
            dist = std::abs(smoothTriAng(w, sta_yaw) - head_pos.yaw);
        } else {
            // TODO: This doesn't work if abs(w_fac) is different in sta_yaw vs. sta_pitch.
            dist = (point_2d(smoothTriAng(w, sta_yaw), smoothTriAng(w, sta_pitch)) -
                    point_2d(head_pos.yaw, head_pos.pitch))
                           .magnitude();
        }
        if (dist < min_dist) {
            min_dist = dist;
            min_w = w;
        }
    }
    return min_w;
}

YawPitch HeadControl::proceed(const MotionCommand& motion_command) {
#if defined(ROBOT_MODEL_T1)
    float pitch_offset = 0;
    float cam_height = 1.05;
    float fps = 5000;
#elif defined(ROBOT_MODEL_G1)
    float pitch_offset = 0;
    float cam_height = 1.05;
    float fps = 50;
#else
    float pitch_offset = 14_deg;
    float cam_height = 0.9;
    float fps = 200;
#endif
    GCState gc_state = gc_state_sub.latest();
    bool may_move = true;
    if (gc_state.state == GameState::Initial || gc_state.state == GameState::Finished ||
        gc_state.state == GameState::Standby ||
        gc_state.my_team.players[gc_state.player_idx].is_penalized) {
        may_move = false;
    }
    LocPosition loc_position = loc_position_sub.latest();
    HeadFocus cur_focus = motion_command.focus;
    int64_t time = time_us();
#ifdef ROBOT_MODEL_G1
    constexpr float kG1LocScanQualityThreshold = 0.7f;
    constexpr int64_t kG1LocScanDelayUs = 1_s;
    const bool g1_loc_unstable = loc_position.quality < kG1LocScanQualityThreshold;
    if (g1_loc_unstable) {
        if (g1_unstable_loc_since_us == 0) {
            g1_unstable_loc_since_us = time;
        }
    } else {
        g1_unstable_loc_since_us = 0;
    }
    const bool g1_needs_loc_scan =
            g1_loc_unstable && time - g1_unstable_loc_since_us >= kG1LocScanDelayUs;
    if (g1_needs_loc_scan) {
        // G1 may run the robocup_deploy-style one-shot head scan in every GameController state,
        // including startup before Ready, but only after localization has been continuously bad
        // for one second. Body motion is still blocked by the motion connector when the state
        // forbids movement.
        cur_focus = HeadFocus::LOC;
    } else if (!may_move) {
        return YawPitch{0, 0};
    } else if (cur_focus == HeadFocus::NOTHING) {
        cur_focus = HeadFocus::BALL;
    }
#else
    if (!may_move)
        return YawPitch{0, 0};
    if (cur_focus == HeadFocus::NOTHING)
        cur_focus = loc_position.quality > 0.7f ? HeadFocus::BALL : HeadFocus::LOC;
#endif
    time_step += (time - last_time) / 1'000'000.f;

#ifdef ROBOT_MODEL_K1
    if (std::shared_ptr<IMUJointState> imu_joint_state = imu_joint_states_sub.latest()) {
        head_pos = {imu_joint_state->serial[static_cast<size_t>(JointIndex::kHeadYaw)].q,
                    imu_joint_state->serial[static_cast<size_t>(JointIndex::kHeadPitch)].q};
    }
#elif defined(ROBOT_MODEL_T1) || defined(ROBOT_MODEL_G1)
    // T1/G1 head state is simulated here. G1 limits and degree conversion are enforced by the
    // G1 servo adapter in the motion connector.
#else
    LOG_F(ERROR, "Unknown robot model in head control");
#endif

    if (cur_focus == HeadFocus::LOC) {
#ifdef ROBOT_MODEL_G1
        if (!g1_needs_loc_scan) {
            g1_loc_scan_was_active = false;
            head_pos = YawPitch{0.0f, 0.0f};
        } else {
            if (!g1_loc_scan_was_active)
                time_step = 0.0f;
            g1_loc_scan_was_active = true;
            head_pos = g1RobocupDeployStyleLocScan(time_step);
        }
#else
        sta_settings sta_yaw{2, 0.1f, 58_deg};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(head_pos, sta_yaw);
        head_pos = {smoothTriAng(time_step, sta_yaw), 15_deg};
#endif
    } else if (cur_focus == HeadFocus::BALL || cur_focus == HeadFocus::BALL_GOALIE) {
#ifdef ROBOT_MODEL_G1
        g1_loc_scan_was_active = false;
        std::optional<RelBall> rel_ball = rel_ball_sub.latest();
        if (rel_ball && rel_ball->ball_age_us < 2_s) {
            ball_found = true;
            const float target_yaw = rel_ball->pos_rel.to_direction();
            const float target_pitch =
                    std::atan(cam_height / std::max(0.01f, rel_ball->pos_rel.magnitude())) -
                    pitch_offset;
            head_pos.yaw = normalizeRotation(
                    head_pos.yaw + normalizeRotation(target_yaw - head_pos.yaw) * 8.f / fps);
            head_pos.pitch =
                    clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 8.f / fps, -20_deg,
                          85_deg);
            last_focus = cur_focus;
            last_time = time;
            return head_pos;
        }
        std::optional<TeamComData> striker = striker_sub.latest();
#else
        std::optional<TeamComData> striker = striker_sub.latest();
#endif
        std::lock_guard<std::mutex> lck(ball_detection_mtx);
        if (last_ball_percept > time - 2_s) {
            ball_found = true;
            // TODO: Rotate the last ball percept based on odometry since we last saw it.
            // float cur_ball_yaw = normalizeRotation(
            //         ball_pos.yaw -
            //         get<Position>(SensorData::instance().getOdometry(last_ball_percept,
            //         time)).a);
            float cur_ball_yaw = normalizeRotation(ball_pos.yaw);
            float target_yaw =
                    cur_ball_yaw;  // clamped_linear_interpolation(std::abs(cur_ball_yaw), 0.f,
                                   //                cur_ball_yaw, 10_deg, 20_deg);
            float target_pitch = ball_pos.pitch;  // clamped_linear_interpolation(48_deg -
                                                  // ball_pos.pitch, 48_deg,
                                                  //               ball_pos.pitch, 10_deg, 20_deg);
            head_pos.yaw = normalizeRotation(
                    head_pos.yaw + normalizeRotation(target_yaw - head_pos.yaw) * 40.f / fps);
            head_pos.pitch = clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 60.f / fps,
                                   10_deg, 48_deg);
        } else if ((cur_focus == HeadFocus::BALL_GOALIE || cur_focus == HeadFocus::BALL) &&
                   striker && striker->ball && striker->ball->ball_age_us < 1_s) {
            ball_found = true;
            point_2d rel_team_ball = LocalizationUtils::absToRel(
                    LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos),
                    loc_position.position);
            // estimate yaw and pitch, this could be done with a relToCam if we have it.
            head_pos.yaw = normalizeRotation(
                    head_pos.yaw +
                    normalizeRotation(rel_team_ball.to_direction() - head_pos.yaw) * 40.f / fps);
            float target_pitch =
                    std::atan(cam_height / std::max(0.01f, rel_team_ball.magnitude())) -
                    pitch_offset;
            head_pos.pitch = clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 30.f / fps,
                                   10_deg, 48_deg);
        } else {
            sta_settings sta_yaw{3, 0.1f, 58_deg};
            if (cur_focus != last_focus || ball_found) {
                time_step = smoothTriAngStartTime(head_pos, sta_yaw);
                ball_found = false;
            }
            // TODO: use last seen to determine direction of this.
            // TODO: use pitch of striker ball
            head_pos.yaw = smoothTriAng(time_step, sta_yaw);
            if (striker && striker->ball) {
                point_2d rel_team_ball = LocalizationUtils::absToRel(
                        LocalizationUtils::relToAbs(striker->ball->pos_rel, striker->pos),
                        loc_position.position);
                float target_pitch =
                        std::atan(cam_height / std::max(0.01f, rel_team_ball.magnitude())) -
                        pitch_offset;
                head_pos.pitch =
                        clamp(head_pos.pitch + (target_pitch - head_pos.pitch) * 30.f / fps, 10_deg,
                              48_deg);
            } else {
                head_pos.pitch = 15_deg;
            }
        }
    } else if (cur_focus == HeadFocus::OBSTACLES) {
        sta_settings sta_yaw{2, 0.1f, 0.5f};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(head_pos, sta_yaw);
        head_pos = {smoothTriAng(time_step, sta_yaw), 15_deg};
    } else if (cur_focus == HeadFocus::BALL_SEARCH_LEFT) {
        sta_settings sta_yaw{3, 0.1f, 30_deg};
        sta_settings sta_pitch{6, 0.1f, ballSearchPitchAmplitude()};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(
                    YawPitch(head_pos.yaw - .4f, head_pos.pitch - ballSearchPitchCenter()),
                    sta_yaw, sta_pitch);
        head_pos = {.4f + smoothTriAng(time_step, sta_yaw),
                    smoothTriAng(time_step, sta_pitch) + ballSearchPitchCenter()};
    } else if (cur_focus == HeadFocus::BALL_SEARCH_RIGHT) {
        sta_settings sta_yaw{-3, 0.1f, 30_deg};
        sta_settings sta_pitch{6, 0.1f, ballSearchPitchAmplitude()};
        if (cur_focus != last_focus)
            time_step = smoothTriAngStartTime(
                    YawPitch(head_pos.yaw + .4f, head_pos.pitch - ballSearchPitchCenter()),
                    sta_yaw, sta_pitch);
        head_pos = {-.4f + smoothTriAng(time_step, sta_yaw),
                    smoothTriAng(time_step, sta_pitch) + ballSearchPitchCenter()};
    } else {
        head_pos = {0, 0};
    }
#ifdef ROBOT_MODEL_G1
    if (cur_focus != HeadFocus::LOC) {
        g1_loc_scan_was_active = false;
    }
#endif
    last_focus = cur_focus;
    last_time = time;
    return head_pos;
}

}  // namespace htwk
