#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>
#include <unitree/idl/go2/MotorCmds_.hpp>

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/common/thread/thread.hpp>
#include <math.h>

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include "DetectionModule.hpp"
#include "LocationModule.hpp"
#include "pose.h"
#include "Locator.h"
#include "misc.h"

namespace {

constexpr const char* kDefaultMarkerLocationTopic = "rt/locationresults_marker";
constexpr const char* kMarkerLocationTopicEnv = "ROBOCUP_MARKER_LOCATION_TOPIC";

std::string GetEnvOrDefault(const char* key, const char* default_value) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }
    return value;
}

bool GetEnvBool(const char* key, bool default_value) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }
    return std::string(value) == "1" || std::string(value) == "true" || std::string(value) == "TRUE";
}

double GetEnvDouble(const char* key, double default_value) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }
    char* end = nullptr;
    const double parsed = std::strtod(value, &end);
    if (end == value) {
        std::cerr << "Invalid numeric env " << key << "=" << value
                  << ", using default " << default_value << std::endl;
        return default_value;
    }
    return parsed;
}

double ComputeScanYawDeg(double elapsed_sec, double yaw_limit_deg, double yaw_scan_speed_deg_per_sec) {
    if (yaw_limit_deg <= 1e-6 || yaw_scan_speed_deg_per_sec <= 1e-6) {
        return 0.0;
    }
    const double phase = elapsed_sec * yaw_scan_speed_deg_per_sec / yaw_limit_deg;
    return yaw_limit_deg * std::sin(phase);
}

double ComputeScanPitchDeg(double elapsed_sec,
                           double pitch_start_deg,
                           double pitch_end_deg,
                           double pitch_scan_speed_deg_per_sec) {
    if (pitch_scan_speed_deg_per_sec <= 1e-6) {
        return pitch_start_deg;
    }
    if (pitch_end_deg < pitch_start_deg) {
        std::swap(pitch_start_deg, pitch_end_deg);
    }
    const double span = std::max(1e-6, pitch_end_deg - pitch_start_deg);
    double phase = std::fmod(elapsed_sec * pitch_scan_speed_deg_per_sec, 2.0 * span);
    if (phase > span) {
        phase = 2.0 * span - phase;
    }
    return pitch_start_deg + phase;
}

}  // namespace


int main(int argc, char *argv[])
{

    if (argc < 3) {// At least need networkInterface and config_file
        std::cout << "Usage: " << argv[0] << " [networkInterface] [config_file] [is_display]" << std::endl;
        return -1;
    }

    std::string env_cfg_path = "../config.yaml";// Default config file path
    if (argc >= 3) {
        env_cfg_path = argv[2];
    }

    bool is_display = false;// Default: no display
    if (argc == 4 && strcmp(argv[3], "1") == 0) {
        is_display = true;
    }

    unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);// Initialize communication module

    // Create servo Subscriber 舵机订阅器
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> servoState;
    servoState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/g1_comp_servo/state");
    servoState->msg_.states().resize(2);

    // Create wrist Subscriber 手腕订阅器
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>> lowState;
    lowState = std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>("rt/lowstate");

    // Create Odometry Subscriber  里程计订阅器
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>> odomState;
    odomState = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>("rt/lf/odommodestate");

    // Create Detection Subscriber 环境检测订阅器
    dds::domain::DomainParticipant participant(0);
    dds::topic::Topic<DetectionModule::DetectionResults> topic(participant, "detectionresults");
    dds::sub::Subscriber subscriber(participant);
    dds::sub::DataReader<DetectionModule::DetectionResults> reader(subscriber, topic);

    // Create marker absolute-location publisher.
    // This node no longer publishes final rt/locationresults directly.
    // The fusion node subscribes this topic and publishes the only final rt/locationresults.
    const std::string marker_location_topic_name =
        GetEnvOrDefault(kMarkerLocationTopicEnv, kDefaultMarkerLocationTopic);
    std::cout << "Marker absolute location topic: " << marker_location_topic_name << std::endl;
    std::unique_ptr<unitree::robot::RealTimePublisher<LocationModule::LocationResult>> posePub;
    posePub = std::make_unique<unitree::robot::RealTimePublisher<LocationModule::LocationResult>>(marker_location_topic_name);

    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>> servoCmd;
    servoCmd = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>("rt/g1_comp_servo/cmd");
    servoCmd->msg_.cmds().resize(2);

    // Load config file 加载config文件
    YamlParser config;
    try {
        config.setup(env_cfg_path.c_str());
    }
    catch(const std::exception& e)//找不到的报错
    {
        std::cout << "[ERROR] Cannot find config file: " << env_cfg_path << std::endl;
        return 0;
    }

    Locator locator; // Create locator object 创建定位器对象
    locator.init(config); // Initialize locator 初始化定位器


    double odometry_factor = config.ReadFloatFromYaml("odometry", "scale_factor");//里程计缩放因子
    double servo_pitch_compensation = config.ReadFloatFromYaml("servo", "pitch_compensation");//舵机pitch补偿
    double servo_yaw_compensation = config.ReadFloatFromYaml("servo", "yaw_compensation");//舵机yaw补偿，目前未使用？
    double servo_height = config.ReadFloatFromYaml("servo", "height");//相机离地面的距离
    double localization_pitch_target_deg = config.ReadFloatFromYaml("localization_servo", "pitch_target_deg");
    double localization_yaw_scan_limit_deg = config.ReadFloatFromYaml("localization_servo", "yaw_scan_limit_deg");
    double localization_yaw_scan_speed_deg_per_sec =
        config.ReadFloatFromYaml("localization_servo", "yaw_scan_speed_deg_per_sec");
    const bool marker_controls_servo = GetEnvBool("ROBOCUP_MARKER_SERVO_SCAN", false);
    const double scan_pitch_start_deg = GetEnvDouble("ROBOCUP_MARKER_SCAN_PITCH_START_DEG", 25.0);
    const double scan_pitch_end_deg = GetEnvDouble("ROBOCUP_MARKER_SCAN_PITCH_END_DEG", 60.0);
    const double scan_pitch_speed_deg_per_sec =
        GetEnvDouble("ROBOCUP_MARKER_SCAN_PITCH_SPEED_DEG_PER_SEC", 3.0);
    std::cout << "Marker locator servo scan control: "
              << (marker_controls_servo ? "enabled" : "disabled")
              << " (set ROBOCUP_MARKER_SERVO_SCAN=1 to enable), "
              << "pitch_scan_raw_deg=[" << scan_pitch_start_deg << ", " << scan_pitch_end_deg
              << "] speed=" << scan_pitch_speed_deg_per_sec << " deg/s" << std::endl;

    const auto start_time = std::chrono::steady_clock::now();
    auto last_scan_status_time = start_time;

    while (true) {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed_sec = std::chrono::duration<double>(now - start_time).count();

        bool marker_pose_updated = false;

        if (marker_controls_servo && !locator.odomCalibrated) {
            const double yaw_target_deg = ComputeScanYawDeg(
                elapsed_sec, localization_yaw_scan_limit_deg, localization_yaw_scan_speed_deg_per_sec);
            const double raw_pitch_target_deg = ComputeScanPitchDeg(
                elapsed_sec, scan_pitch_start_deg, scan_pitch_end_deg, scan_pitch_speed_deg_per_sec);

            servoCmd->msg_.cmds()[0].mode() = 1;
            servoCmd->msg_.cmds()[0].q(static_cast<float>(yaw_target_deg));
            servoCmd->msg_.cmds()[1].mode() = 1;
            servoCmd->msg_.cmds()[1].q(static_cast<float>(raw_pitch_target_deg));
            servoCmd->unlockAndPublish();

            if (std::chrono::duration<double>(now - last_scan_status_time).count() >= 1.0) {
                std::cout << "[marker_locator] scanning_for_marker "
                          << "yaw_cmd_deg=" << yaw_target_deg << " "
                          << "pitch_cmd_deg=" << raw_pitch_target_deg << " "
                          << "field_size=kid" << std::endl;
                last_scan_status_time = now;
            }
        }

        // Read odometry data 读取里程计数据
        locator.robotPoseToOdom.x = odomState->msg_.position()[0] * odometry_factor; //获取X坐标准确位置
        locator.robotPoseToOdom.y = odomState->msg_.position()[1] * odometry_factor; //获取Y坐标准确位置
        locator.robotPoseToOdom.theta = odomState->msg_.imu_state().rpy()[2];//获取航向角准确位置

        std::cout << "Odometer information: ("
                    << locator.robotPoseToOdom.x << ", "
                    << locator.robotPoseToOdom.y << ", "
                    << locator.robotPoseToOdom.theta << ")" << std::endl;//输出信息

        // Read servo data 读取舵机数据
        double wrist_yaw_angle = rad2deg(lowState->msg_.motor_state()[JointIndex::kWaistYaw].q());//手腕yaw角度
        double servo_yaw_angle = servoState -> msg_.states()[0].q();//舵机yaw角度
        double servo_pitch_angle = servoState -> msg_.states()[1].q() + servo_pitch_compensation;//舵机pitch角度加补偿
	    Pose p_eye2base = Pose(0, -servo_height, 0, deg2rad(servo_pitch_angle), -deg2rad(wrist_yaw_angle) - deg2rad(servo_yaw_angle), 0) ;//

        std::cout << "Servo information: "
                 << "wrist_yaw_angle(" << wrist_yaw_angle << "), "
                 << "servo_yaw_angle(" << servo_yaw_angle << "), "
                 << "servo_pitch_angle(" << servo_pitch_angle << ")" << std::endl;//输出信息

        // Read detection data 读取检测数据
        auto samples = reader.take();//获取检测结果
        for (const auto &sample : samples) {
            if (sample.info().valid()) {
                const auto& det_results = sample.data().results();
                std::cout << "===== Received " << det_results.size() << " detection results =====\n";
                if (det_results.size() > 0) {
                    locator.processDetections(det_results, p_eye2base);
                    marker_pose_updated = locator.selfLocate() || marker_pose_updated;
                }
            }
        }//

        // Publish only fresh marker-based absolute corrections.
        // The final fused pose is published by location_fusion.
        //
        // For display, however, keep the robot icon live even when no new marker
        // correction was accepted in this cycle: apply the latest odom->field
        // transform to current odometry every frame.
        if (locator.odomCalibrated) {

            transCoord(
                locator.robotPoseToOdom.x, locator.robotPoseToOdom.y, locator.robotPoseToOdom.theta,
                locator.odomToField.x, locator.odomToField.y, locator.odomToField.theta,
                locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);

            if (marker_pose_updated) {
                std::cout << "== Final RobotToFiled: ("
                                << locator.robotPoseToField.x << ", "
                                << locator.robotPoseToField.y << ", "
                                << locator.robotPoseToField.theta << ")" << std::endl;

                posePub -> msg_.robot2field_x() = locator.robotPoseToField.x;
                posePub -> msg_.robot2field_y() = locator.robotPoseToField.y;
                posePub -> msg_.robot2field_theta() = locator.robotPoseToField.theta;
                posePub -> unlockAndPublish();
            }

            if (is_display) {
                locator.display_board -> displayRobotPose(locator.robotPoseToField.x, locator.robotPoseToField.y, locator.robotPoseToField.theta);
            }

        }

        if (is_display) {
            locator.display_board->pumpEvents(1);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));// Loop delay 50ms
    }
    return 0;
}
