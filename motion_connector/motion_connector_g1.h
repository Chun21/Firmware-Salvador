#pragma once

#ifdef ROBOT_MODEL_G1

#include <gc_pub_sub.h>
#include <head_control.h>
#include <motion_pub_sub.h>
#include <robot_time.h>
#include <sensor_pub_sub.h>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

class MotionConnector {
public:
    explicit MotionConnector(PlayerIdx idx);
    ~MotionConnector();

    void proceed();

private:
    htwk::ChannelSubscriber<MotionCommand> motion_command_subscriber =
            motion_command_channel.create_subscriber();
    htwk::ChannelSubscriber<GCState> gc_state_sub = gc_state.create_subscriber();
    htwk::ChannelSubscriber<htwk::FallDownState> fallen_subscriber =
            htwk::fallen_channel.create_subscriber();

    unitree::robot::g1::LocoClient client;
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::MotorCmds_> head_cmd_publisher;
    unitree_go::msg::dds_::MotorCmds_ head_cmd_msg;

    htwk::HeadControl head_control;
    PlayerIdx idx;

    MotionCommand::WalkRequest last_sent_walk{};
    YawPitch last_sent_head{};
    int64_t last_motion_us = 0;
    int64_t last_walk_log_us = 0;
    int64_t last_move_error_us = 0;
    int64_t last_joint_control_error_us = 0;
    bool standing = false;

    void sendSafeStop(const YawPitch& head_angles);
    void sendWalk(const MotionCommand::WalkRequest& walk_request, HeadFocus focus,
                  const YawPitch& head_angles);
    void sendHead(const YawPitch& head_angles);
    MotionCommand::WalkRequest limitWalkRequest(const MotionCommand::WalkRequest& request,
                                                HeadFocus focus);
    bool movementAllowed(const GCState& state) const;
};

#endif  // ROBOT_MODEL_G1
