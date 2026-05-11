#pragma once

#ifdef ROBOT_MODEL_G1

#include <atomic>
#include <memory>
#include <string>

#include <unitree/robot/channel/channel_subscriber.hpp>

#include "DetectionModule.hpp"
#include "LocationModule.hpp"

class G1ExternalAdapters {
public:
    G1ExternalAdapters();
    ~G1ExternalAdapters();

    void proceed();

private:
    void handleDetection(const void* msg);
    void handleLocation(const void* msg);

    static bool isBallClass(const std::string& class_name);
    static bool isRobotClass(const std::string& class_name);

    std::unique_ptr<unitree::robot::ChannelSubscriber<DetectionModule::DetectionResults>>
            detection_subscriber;
    std::unique_ptr<unitree::robot::ChannelSubscriber<LocationModule::LocationResult>>
            location_subscriber;

    std::atomic<int64_t> last_detection_us{0};
    std::atomic<int64_t> last_location_us{0};
    int64_t last_timeout_log_us = 0;
};

#endif  // ROBOT_MODEL_G1

