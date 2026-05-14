#pragma once

#ifdef ROBOT_MODEL_G1

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "DetectionModule.hpp"
#include "g1_ready_location_filter.h"
#include "LocationModule.hpp"

class G1ExternalAdapters {
public:
    G1ExternalAdapters();
    ~G1ExternalAdapters();

    void proceed();

private:
    void handleDetection(const void* msg);
    void handleLocation(const void* msg);
    void handleServoState(const void* msg);

    static bool isBallClass(const std::string& class_name);
    static bool isRobotClass(const std::string& class_name);

    std::unique_ptr<unitree::robot::ChannelSubscriber<DetectionModule::DetectionResults>>
            detection_subscriber;
    std::unique_ptr<unitree::robot::ChannelSubscriber<LocationModule::LocationResult>>
            location_subscriber;
    std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>
            servo_state_subscriber;

    std::atomic<int64_t> last_detection_us{0};
    std::atomic<int64_t> last_location_us{0};
    std::atomic<int64_t> last_servo_state_us{0};
    std::atomic<float> head_yaw_deg{0.0f};
    std::atomic<float> head_pitch_deg{0.0f};
    float ball_min_score = 0.6f;
    int64_t last_timeout_log_us = 0;
    int64_t last_location_filter_log_us = 0;
    int64_t last_low_score_ball_log_us = 0;
    G1ReadyLocationFilter location_filter;
};

#endif  // ROBOT_MODEL_G1
