#include "motion_connector_g1.h"

#ifdef ROBOT_MODEL_G1

#include <algorithm>
#include <cmath>

#include "gc_state.h"
#include "logging.h"
#include "stl_ext.h"

namespace {

constexpr float kMaxForwardSpeed = 0.45f;
constexpr float kMaxBackwardSpeed = 0.15f;
constexpr float kMaxSideSpeed = 0.25f;
constexpr float kMaxYawSpeed = 0.60f;
constexpr float kMaxForwardAccel = 0.60f;
constexpr float kMaxSideAccel = 0.50f;
constexpr float kMaxYawAccel = 1.00f;

constexpr float kHeadYawMinDeg = -50.0f;
constexpr float kHeadYawMaxDeg = 50.0f;
constexpr float kHeadPitchMinDeg = -20.0f;
constexpr float kHeadPitchMaxDeg = 85.0f;

float radToDeg(float rad) {
    return rad * 180.0f / M_PIf;
}

float slew(float previous, float target, float max_delta) {
    return previous + std::clamp(target - previous, -max_delta, max_delta);
}

}  // namespace

MotionConnector::MotionConnector(PlayerIdx idx) : idx(idx) {
    client.Init();
    client.SetTimeout(2.0f);
    client.BalanceStand();

    head_cmd_publisher = std::make_shared<unitree::robot::ChannelPublisher<
            unitree_go::msg::dds_::MotorCmds_>>("rt/g1_comp_servo/cmd");
    head_cmd_publisher->InitChannel();
    head_cmd_msg.cmds().resize(2);

    LOG_F(INFO, "G1 MotionConnector initialized: high-level locomotion only, JointControl blocked");
}

MotionConnector::~MotionConnector() {
    LOG_F(INFO, "G1 MotionConnector shutdown: StopMove/BalanceStand");
    client.StopMove();
    client.BalanceStand();
    if (head_cmd_publisher) {
        head_cmd_publisher->CloseChannel();
    }
}

bool MotionConnector::movementAllowed(const GCState& state) const {
    return !(state.state == GameState::Initial || state.state == GameState::Finished ||
             state.state == GameState::Standby || state.state == GameState::Set ||
             state.my_team.players[idx].is_penalized);
}

MotionCommand::WalkRequest MotionConnector::limitWalkRequest(
        const MotionCommand::WalkRequest& request) {
    const int64_t now = time_us();
    const float dt = last_motion_us == 0 ? 0.02f : std::max(0.001f, (now - last_motion_us) / 1e6f);
    last_motion_us = now;

    MotionCommand::WalkRequest limited{
            .dx = std::clamp(request.dx, -kMaxBackwardSpeed, kMaxForwardSpeed),
            .dy = std::clamp(request.dy, -kMaxSideSpeed, kMaxSideSpeed),
            .da = std::clamp(request.da, -kMaxYawSpeed, kMaxYawSpeed),
    };

    limited.dx = slew(last_sent_walk.dx, limited.dx, kMaxForwardAccel * dt);
    limited.dy = slew(last_sent_walk.dy, limited.dy, kMaxSideAccel * dt);
    limited.da = slew(last_sent_walk.da, limited.da, kMaxYawAccel * dt);
    return limited;
}

void MotionConnector::sendHead(const YawPitch& head_angles) {
    const float yaw_deg = std::clamp(radToDeg(head_angles.yaw), kHeadYawMinDeg, kHeadYawMaxDeg);
    const float pitch_deg =
            std::clamp(radToDeg(head_angles.pitch), kHeadPitchMinDeg, kHeadPitchMaxDeg);

    if (std::abs(yaw_deg - radToDeg(last_sent_head.yaw)) < 0.5f &&
        std::abs(pitch_deg - radToDeg(last_sent_head.pitch)) < 0.5f) {
        return;
    }

    head_cmd_msg.cmds()[0].mode(1);
    head_cmd_msg.cmds()[0].q(yaw_deg);
    head_cmd_msg.cmds()[1].mode(1);
    head_cmd_msg.cmds()[1].q(pitch_deg);
    head_cmd_publisher->Write(head_cmd_msg);

    last_sent_head = {yaw_deg / 180.0f * M_PIf, pitch_deg / 180.0f * M_PIf};
}

void MotionConnector::sendSafeStop(const YawPitch& head_angles) {
    sendHead(head_angles);
    client.StopMove();
    client.BalanceStand();
    last_sent_walk = {};
    standing = true;
}

void MotionConnector::sendWalk(const MotionCommand::WalkRequest& walk_request,
                               const YawPitch& head_angles) {
    sendHead(head_angles);
    const MotionCommand::WalkRequest limited = limitWalkRequest(walk_request);
    client.Move(limited.dx, limited.dy, limited.da);
    last_sent_walk = limited;
    standing = false;
}

void MotionConnector::proceed() {
    const htwk::FallDownState fall_down_state = fallen_subscriber.latest();
    MotionCommand motion_command = motion_command_subscriber.latest();
    YawPitch head_angles = head_control.proceed(motion_command);
    const GCState gc_state = gc_state_sub.latest();

    if (fall_down_state.type != htwk::FallDownStateType::READY) {
        sendHead(head_angles);
        client.Damp();
        last_sent_walk = {};
        return;
    }

    if (!movementAllowed(gc_state)) {
        sendSafeStop(head_angles);
        return;
    }

    if (motion_command.type == MotionCommand::Type::JOINT_CONTROL) {
        const int64_t now = time_us();
        if (now - last_joint_control_error_us > 1_s) {
            LOG_F(ERROR, "G1 blocked unsafe JointControl command. 23DOF policies are forbidden.");
            last_joint_control_error_us = now;
        }
        sendSafeStop(head_angles);
        return;
    }

    if (motion_command.type == MotionCommand::Type::WALK) {
        sendWalk(motion_command.walk_request, head_angles);
        return;
    }

    sendSafeStop(head_angles);
}

#endif  // ROBOT_MODEL_G1
