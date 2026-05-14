#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <dds/dds.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

#include "dds/Subscription.h"
#include "DetectionModule.hpp"
#include "LocationModule.hpp"

namespace {

constexpr const char* kRgbdTopicDefault = "rt/locationresults_rgbd";
constexpr const char* kMarkerTopicDefault = "rt/locationresults_marker";
constexpr const char* kFusedTopicDefault = "rt/locationresults";
constexpr const char* kDetectionTopicDefault = "detectionresults";
constexpr const char* kServoStateTopic = "rt/g1_comp_servo/state";
constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kTorsoImuTopic = "rt/secondary_imu";
constexpr const char* kTeamBallMulticastGroup = "239.255.42.99";
constexpr int kTeamBallUdpPort = 38383;
constexpr std::size_t kHeadYawIndex = 0;
constexpr std::size_t kHeadPitchIndex = 1;
constexpr std::size_t kWaistYawIndex = 12;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Mat3 {
    std::array<std::array<double, 3>, 3> m{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}},
    }};
};

struct FieldObject {
    std::string label;
    double score = 0.0;
    Vec2 root_xy{};
    Vec2 field_xy{};
    bool has_field = false;
};

struct TeamBallUdpSender {
    int fd = -1;
    sockaddr_in dest{};

    ~TeamBallUdpSender() {
        if (fd >= 0) {
            close(fd);
        }
    }

    bool ok() const {
        return fd >= 0;
    }

    void send(const std::string& payload) const {
        if (fd < 0) {
            return;
        }
        sendto(fd, payload.data(), payload.size(), 0,
               reinterpret_cast<const sockaddr*>(&dest), sizeof(dest));
    }
};

std::string EnvOrDefault(const char* key, const char* default_value) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }
    return value;
}

double EnvDoubleOrDefault(const char* key, double default_value) {
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

bool EnvBoolOrDefault(const char* key, bool default_value) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return default_value;
    }
    const std::string parsed(value);
    return parsed == "1" || parsed == "true" || parsed == "TRUE" ||
           parsed == "yes" || parsed == "YES";
}

bool GetInterfaceIpv4(const std::string& iface, in_addr* out_addr) {
    if (iface.empty() || out_addr == nullptr) {
        return false;
    }
    ifaddrs* ifaddr = nullptr;
    if (getifaddrs(&ifaddr) != 0) {
        return false;
    }
    bool found = false;
    for (ifaddrs* it = ifaddr; it != nullptr; it = it->ifa_next) {
        if (it->ifa_addr == nullptr || it->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        if (iface != it->ifa_name) {
            continue;
        }
        *out_addr = reinterpret_cast<sockaddr_in*>(it->ifa_addr)->sin_addr;
        found = true;
        break;
    }
    freeifaddrs(ifaddr);
    return found;
}

TeamBallUdpSender OpenTeamBallSender(const std::string& iface) {
    TeamBallUdpSender sender;
    in_addr iface_addr{};
    if (!GetInterfaceIpv4(iface, &iface_addr)) {
        return sender;
    }

    sender.fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sender.fd < 0) {
        return sender;
    }

    unsigned char ttl = 1;
    setsockopt(sender.fd, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl));
    setsockopt(sender.fd, IPPROTO_IP, IP_MULTICAST_IF, &iface_addr, sizeof(iface_addr));

    sender.dest.sin_family = AF_INET;
    sender.dest.sin_addr.s_addr = inet_addr(kTeamBallMulticastGroup);
    sender.dest.sin_port = htons(kTeamBallUdpPort);
    return sender;
}

std::string Lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool IsMarkerClass(const std::string& name) {
    return name == "L" || name == "T" || name == "X" || name == "PenaltyPoint";
}

bool IsOpponentClass(const std::string& name) {
    const std::string lower = Lower(name);
    return lower == "opponent" || lower == "person" || lower == "robot" ||
           lower == "human" || lower == "humanoid" || lower == "player";
}

double WrapRad(double angle) {
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double DegToRad(double deg) {
    return deg * M_PI / 180.0;
}

double RadToDeg(double rad) {
    return rad * 180.0 / M_PI;
}

double PoseX(const LocationModule::LocationResult& pose) {
    return static_cast<double>(pose.robot2field_x());
}

double PoseY(const LocationModule::LocationResult& pose) {
    return static_cast<double>(pose.robot2field_y());
}

double PoseTheta(const LocationModule::LocationResult& pose) {
    return static_cast<double>(pose.robot2field_theta());
}

void SetPose(LocationModule::LocationResult& pose, double x, double y, double theta) {
    pose.robot2field_x(static_cast<float>(x));
    pose.robot2field_y(static_cast<float>(y));
    pose.robot2field_theta(static_cast<float>(WrapRad(theta)));
}

Vec2 RootToField(const LocationModule::LocationResult& robot_pose, const Vec2& root_xy) {
    const double theta = PoseTheta(robot_pose);
    return {
        PoseX(robot_pose) + std::cos(theta) * root_xy.x - std::sin(theta) * root_xy.y,
        PoseY(robot_pose) + std::sin(theta) * root_xy.x + std::cos(theta) * root_xy.y,
    };
}

Mat3 RotX(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Mat3 out;
    out.m = {{{{1.0, 0.0, 0.0}}, {{0.0, c, -s}}, {{0.0, s, c}}}};
    return out;
}

Mat3 RotY(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Mat3 out;
    out.m = {{{{c, 0.0, s}}, {{0.0, 1.0, 0.0}}, {{-s, 0.0, c}}}};
    return out;
}

Mat3 RotZ(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    Mat3 out;
    out.m = {{{{c, -s, 0.0}}, {{s, c, 0.0}}, {{0.0, 0.0, 1.0}}}};
    return out;
}

Mat3 Mul(const Mat3& lhs, const Mat3& rhs) {
    Mat3 out;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            out.m[row][col] = 0.0;
            for (int k = 0; k < 3; ++k) {
                out.m[row][col] += lhs.m[row][k] * rhs.m[k][col];
            }
        }
    }
    return out;
}

Vec3 Mul(const Mat3& mat, const Vec3& vec) {
    return {
        mat.m[0][0] * vec.x + mat.m[0][1] * vec.y + mat.m[0][2] * vec.z,
        mat.m[1][0] * vec.x + mat.m[1][1] * vec.y + mat.m[1][2] * vec.z,
        mat.m[2][0] * vec.x + mat.m[2][1] * vec.y + mat.m[2][2] * vec.z,
    };
}

Vec3 TransformPoint(const Mat3& rotation, const Vec3& translation, const Vec3& point) {
    const Vec3 rotated = Mul(rotation, point);
    return {rotated.x + translation.x, rotated.y + translation.y, rotated.z + translation.z};
}

Mat3 RpyRotation(double roll_rad, double pitch_rad, double yaw_rad) {
    return Mul(Mul(RotZ(yaw_rad), RotY(pitch_rad)), RotX(roll_rad));
}

void QuaternionToRollPitch(const std::array<double, 4>& quat_xyzw, double* roll_rad, double* pitch_rad) {
    const double x = quat_xyzw[0];
    const double y = quat_xyzw[1];
    const double z = quat_xyzw[2];
    const double w = quat_xyzw[3];

    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    *roll_rad = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    sinp = std::clamp(sinp, -1.0, 1.0);
    *pitch_rad = std::asin(sinp);
}

double CurrentHeadYawDeg(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>& servo_state) {
    if (servo_state == nullptr || servo_state->msg_.states().size() <= kHeadYawIndex) {
        return 0.0;
    }
    return static_cast<double>(servo_state->msg_.states()[kHeadYawIndex].q());
}

double CurrentHeadPitchDeg(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>& servo_state) {
    if (servo_state == nullptr || servo_state->msg_.states().size() <= kHeadPitchIndex) {
        return 0.0;
    }
    return static_cast<double>(servo_state->msg_.states()[kHeadPitchIndex].q());
}

double CurrentWaistYawRad(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>& low_state) {
    if (low_state == nullptr || low_state->msg_.motor_state().size() <= kWaistYawIndex) {
        return 0.0;
    }
    return static_cast<double>(low_state->msg_.motor_state()[kWaistYawIndex].q());
}

std::array<double, 4> CurrentTorsoQuatXyzw(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::IMUState_>>& torso_imu) {
    if (torso_imu == nullptr || torso_imu->msg_.quaternion().size() < 4) {
        return {0.0, 0.0, 0.0, 1.0};
    }
    return {
        static_cast<double>(torso_imu->msg_.quaternion()[0]),
        static_cast<double>(torso_imu->msg_.quaternion()[1]),
        static_cast<double>(torso_imu->msg_.quaternion()[2]),
        static_cast<double>(torso_imu->msg_.quaternion()[3]),
    };
}

Vec2 DetectionToRoot(
    const DetectionModule::DetectionResult& detection,
    double head_yaw_deg,
    double head_pitch_deg,
    double waist_yaw_rad,
    const std::array<double, 4>& torso_quat_xyzw) {
    const auto& xyz = detection.xyz();
    Vec3 point{static_cast<double>(xyz[0]), static_cast<double>(xyz[1]), static_cast<double>(xyz[2])};
    if (std::abs(point.z) <= 1e-6) {
        return {};
    }

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    QuaternionToRollPitch(torso_quat_xyzw, &roll_rad, &pitch_rad);

    const double servo0_q = head_yaw_deg * M_PI / 180.0;
    const double servo1_q = -head_pitch_deg * M_PI / 180.0;

    point = TransformPoint(Mul(Mul(RotY(0.6981), RotY(1.5707)), RotZ(-1.5707)), {0.04061, 0.01000, -0.02207}, point);
    point = TransformPoint(RotY(servo1_q), {0.0295, 0.0, 0.013}, point);
    point = TransformPoint(Mul(RotY(0.039968), RotZ(servo0_q)), {0.030518, 0.0, 0.52486}, point);
    point = TransformPoint(Mat3{}, {0.0039635, 0.0, -0.047}, point);
    point = TransformPoint(RotZ(waist_yaw_rad), {-0.0039635, 0.0, 0.044}, point);
    point = TransformPoint(RpyRotation(roll_rad, pitch_rad, 0.0), {0.0, 0.0, 0.0}, point);

    const double range_3d = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range_3d < 0.15 || point.z >= -0.25) {
        // Fallback: camera convention x=right, z=forward. Robot/root convention x=forward, y=left.
        return {static_cast<double>(xyz[2]), static_cast<double>(xyz[0])};
    }
    return {point.x, point.y};
}

std::string FormatPose(const LocationModule::LocationResult& pose) {
    std::ostringstream oss;
    oss << "(x=" << PoseX(pose)
        << ",y=" << PoseY(pose)
        << ",theta_deg=" << PoseTheta(pose) * 180.0 / M_PI << ")";
    return oss.str();
}

std::string FormatObjectList(const std::vector<FieldObject>& objects, std::size_t max_items = 4) {
    if (objects.empty()) {
        return "none";
    }
    std::ostringstream oss;
    const std::size_t n = std::min(max_items, objects.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (i > 0) {
            oss << ";";
        }
        const auto& obj = objects[i];
        oss << obj.label << "#" << i << "[score=" << obj.score
            << ",root=(" << obj.root_xy.x << "," << obj.root_xy.y << ")";
        if (obj.has_field) {
            oss << ",field=(" << obj.field_xy.x << "," << obj.field_xy.y << ")";
        } else {
            oss << ",field=unavailable";
        }
        oss << "]";
    }
    if (objects.size() > n) {
        oss << ";...+" << (objects.size() - n);
    }
    return oss.str();
}

void AddSource(std::string& source, const std::string& item) {
    if (source.empty()) {
        source = item;
    } else if (source.find(item) == std::string::npos) {
        source += "+" + item;
    }
}

bool PoseChanged(
    const LocationModule::LocationResult& prev_pose,
    const LocationModule::LocationResult& curr_pose,
    double min_translation_m,
    double min_rotation_rad) {
    return std::hypot(PoseX(curr_pose) - PoseX(prev_pose), PoseY(curr_pose) - PoseY(prev_pose)) >= min_translation_m ||
           std::abs(WrapRad(PoseTheta(curr_pose) - PoseTheta(prev_pose))) >= min_rotation_rad;
}

void ApplyOdometryIncrement(
    const LocationModule::LocationResult& prev_rgbd,
    const LocationModule::LocationResult& curr_rgbd,
    LocationModule::LocationResult& fused) {
    const double dx_world = PoseX(curr_rgbd) - PoseX(prev_rgbd);
    const double dy_world = PoseY(curr_rgbd) - PoseY(prev_rgbd);
    const double dtheta = WrapRad(PoseTheta(curr_rgbd) - PoseTheta(prev_rgbd));

    // Convert RGB-D absolute-frame delta to robot-local delta, then apply it
    // to the current fused global pose. This uses RGB-D as high-rate odometry.
    const double prev_theta = PoseTheta(prev_rgbd);
    const double local_dx = std::cos(prev_theta) * dx_world + std::sin(prev_theta) * dy_world;
    const double local_dy = -std::sin(prev_theta) * dx_world + std::cos(prev_theta) * dy_world;

    const double fused_theta = PoseTheta(fused);
    const double fused_dx = std::cos(fused_theta) * local_dx - std::sin(fused_theta) * local_dy;
    const double fused_dy = std::sin(fused_theta) * local_dx + std::cos(fused_theta) * local_dy;

    SetPose(
        fused,
        PoseX(fused) + fused_dx,
        PoseY(fused) + fused_dy,
        fused_theta + dtheta);
}

LocationModule::LocationResult ReadRobotOdomPose(
    const std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>& odom_state,
    double odom_scale_factor) {
    LocationModule::LocationResult pose;
    SetPose(
        pose,
        static_cast<double>(odom_state->msg_.position()[0]) * odom_scale_factor,
        static_cast<double>(odom_state->msg_.position()[1]) * odom_scale_factor,
        static_cast<double>(odom_state->msg_.imu_state().rpy()[2]));
    return pose;
}

void SmoothTowardMarker(
    const LocationModule::LocationResult& marker,
    LocationModule::LocationResult& fused,
    double alpha) {
    alpha = std::clamp(alpha, 0.0, 1.0);
    const double x = PoseX(fused) + alpha * (PoseX(marker) - PoseX(fused));
    const double y = PoseY(fused) + alpha * (PoseY(marker) - PoseY(fused));
    const double theta = PoseTheta(fused) + alpha * WrapRad(PoseTheta(marker) - PoseTheta(fused));
    SetPose(fused, x, y, theta);
}

bool MarkerCorrectionAllowed(
    const LocationModule::LocationResult& marker,
    const LocationModule::LocationResult& fused,
    double max_translation_m,
    double max_rotation_rad) {
    if (max_translation_m < 0.0 && max_rotation_rad < 0.0) {
        return true;
    }
    const double dx = PoseX(marker) - PoseX(fused);
    const double dy = PoseY(marker) - PoseY(fused);
    const double translation = std::hypot(dx, dy);
    const double rotation = std::fabs(WrapRad(PoseTheta(marker) - PoseTheta(fused)));
    const bool translation_ok = max_translation_m < 0.0 || translation <= max_translation_m;
    const bool rotation_ok = max_rotation_rad < 0.0 || rotation <= max_rotation_rad;
    return translation_ok && rotation_ok;
}

}  // namespace

int main(int argc, char** argv) {
    const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    const std::string rgbd_topic_name =
        EnvOrDefault("ROBOCUP_RGBD_LOCATION_TOPIC", kRgbdTopicDefault);
    const std::string marker_topic_name =
        EnvOrDefault("ROBOCUP_MARKER_LOCATION_TOPIC", kMarkerTopicDefault);
    const std::string fused_topic_name =
        EnvOrDefault("ROBOCUP_FUSED_LOCATION_TOPIC", kFusedTopicDefault);
    const std::string detection_topic_name =
        EnvOrDefault("ROBOCUP_DETECTION_TOPIC", kDetectionTopicDefault);
    const std::string team_id = EnvOrDefault("ROBOCUP_TEAM_ID", "red");
    const std::string robot_id = EnvOrDefault("ROBOCUP_ROBOT_ID", "unknown_robot");
    const std::string team_iface = EnvOrDefault("ROBOCUP_TEAM_IFACE", "");
    TeamBallUdpSender team_ball_sender = OpenTeamBallSender(team_iface);

    const double publish_period_ms = EnvDoubleOrDefault("ROBOCUP_FUSION_PERIOD_MS", 20.0);
    const double marker_alpha = EnvDoubleOrDefault("ROBOCUP_FUSION_MARKER_ALPHA", 0.05);
    const double marker_timeout_sec = EnvDoubleOrDefault("ROBOCUP_FUSION_MARKER_TIMEOUT_SEC", 1.0);
    const double marker_max_correction_m =
        EnvDoubleOrDefault("ROBOCUP_FUSION_MARKER_MAX_CORRECTION_M", 0.7);
    const double marker_max_correction_rad =
        DegToRad(EnvDoubleOrDefault("ROBOCUP_FUSION_MARKER_MAX_CORRECTION_DEG", 40.0));
    const bool use_rgbd = EnvBoolOrDefault("ROBOCUP_FUSION_USE_RGBD", true);
    const bool init_from_rgbd = EnvBoolOrDefault("ROBOCUP_FUSION_INIT_FROM_RGBD", false);
    const bool use_robot_odom = EnvBoolOrDefault("ROBOCUP_FUSION_USE_ROBOT_ODOM", true);
    const double odom_scale_factor = EnvDoubleOrDefault("ROBOCUP_ODOM_SCALE_FACTOR", 1.4);
    const double odom_min_translation_m = EnvDoubleOrDefault("ROBOCUP_FUSION_ODOM_MIN_TRANSLATION_M", 0.001);
    const double odom_min_rotation_rad = EnvDoubleOrDefault("ROBOCUP_FUSION_ODOM_MIN_ROTATION_RAD", 0.001);
    const double field_length_m = EnvDoubleOrDefault("FIELD_LENGTH", EnvDoubleOrDefault("ROBOCUP_FIELD_LENGTH", 9.0));
    const double attack_goal_sign =
        EnvDoubleOrDefault("ATTACK_GOAL_X_SIGN", 1.0) >= 0.0 ? 1.0 : -1.0;

    std::cout << "Location fusion topics:"
              << " rgbd=" << rgbd_topic_name
              << " marker=" << marker_topic_name
              << " fused=" << fused_topic_name
              << " detection=" << detection_topic_name
              << " team_ball_udp=" << (team_ball_sender.ok() ? "1" : "0")
              << " team_iface=" << (team_iface.empty() ? "unset" : team_iface)
              << " team_id=" << team_id
              << " robot_id=" << robot_id
              << " marker_alpha=" << marker_alpha
              << " marker_max_correction_m=" << marker_max_correction_m
              << " marker_max_correction_deg=" << RadToDeg(marker_max_correction_rad)
              << " use_rgbd=" << (use_rgbd ? "1" : "0")
              << " init_from_rgbd=" << (init_from_rgbd ? "1" : "0")
              << " use_robot_odom=" << (use_robot_odom ? "1" : "0")
              << " odom_scale_factor=" << odom_scale_factor
              << std::endl;

    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>> odom_state;
    if (use_robot_odom) {
        odom_state =
            std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::SportModeState_>>("rt/lf/odommodestate");
    }
    auto servo_state =
        std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>(kServoStateTopic);
    servo_state->msg_.states().resize(2);
    auto low_state =
        std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::LowState_>>(kLowStateTopic);
    auto torso_imu =
        std::make_shared<unitree::robot::SubscriptionBase<unitree_hg::msg::dds_::IMUState_>>(kTorsoImuTopic);

    dds::domain::DomainParticipant participant(0);
    dds::sub::Subscriber subscriber(participant);
    dds::pub::Publisher publisher(participant);

    dds::topic::Topic<LocationModule::LocationResult> rgbd_topic(participant, rgbd_topic_name);
    dds::topic::Topic<LocationModule::LocationResult> marker_topic(participant, marker_topic_name);
    dds::topic::Topic<LocationModule::LocationResult> fused_topic(participant, fused_topic_name);
    dds::topic::Topic<DetectionModule::DetectionResults> detection_topic(participant, detection_topic_name);

    dds::sub::DataReader<LocationModule::LocationResult> rgbd_reader(subscriber, rgbd_topic);
    dds::sub::DataReader<LocationModule::LocationResult> marker_reader(subscriber, marker_topic);
    dds::sub::DataReader<DetectionModule::DetectionResults> detection_reader(subscriber, detection_topic);
    dds::pub::DataWriter<LocationModule::LocationResult> fused_writer(publisher, fused_topic);

    LocationModule::LocationResult prev_rgbd;
    LocationModule::LocationResult prev_odom;
    LocationModule::LocationResult fused_pose;
    LocationModule::LocationResult latest_marker;

    bool has_prev_rgbd = false;
    bool has_prev_odom = false;
    bool has_fused = false;
    bool has_marker = false;
    auto latest_marker_time = std::chrono::steady_clock::time_point{};
    auto last_status_time = std::chrono::steady_clock::now();
    std::uint64_t rgbd_count = 0;
    std::uint64_t odom_count = 0;
    std::uint64_t marker_count = 0;
    std::uint64_t detection_count = 0;
    std::uint64_t fused_count = 0;
    std::uint32_t team_ball_seq = 0;
    std::string last_source = "WAIT_ABSOLUTE_MARKER";
    std::vector<FieldObject> latest_balls;
    std::vector<FieldObject> latest_opponents;
    std::vector<FieldObject> latest_detected_markers;
    std::vector<FieldObject> latest_goalposts;

    while (true) {
        const auto now = std::chrono::steady_clock::now();

        std::string cycle_source;
        bool applied_rgbd_prediction = false;
        bool fresh_marker = false;

        auto rgbd_samples = rgbd_reader.take();
        for (const auto& sample : rgbd_samples) {
            if (!sample.info().valid()) {
                continue;
            }
            const auto curr_rgbd = sample.data();
            // RGB-D is treated only as an odometry/increment source here.
            // It is deliberately not allowed to initialize the field-frame pose.
            if (use_rgbd) {
                if (!has_fused && init_from_rgbd) {
                    fused_pose = curr_rgbd;
                    has_fused = true;
                    AddSource(cycle_source, "RGBD_INIT");
                } else if (has_fused && has_prev_rgbd) {
                    ApplyOdometryIncrement(prev_rgbd, curr_rgbd, fused_pose);
                    applied_rgbd_prediction = true;
                    AddSource(cycle_source, "RGBD_ODOM");
                }
            }
            prev_rgbd = curr_rgbd;
            has_prev_rgbd = true;
            ++rgbd_count;
        }

        if (use_robot_odom && odom_state != nullptr) {
            const auto curr_odom = ReadRobotOdomPose(odom_state, odom_scale_factor);
            if (has_prev_odom && has_fused && !applied_rgbd_prediction &&
                PoseChanged(prev_odom, curr_odom, odom_min_translation_m, odom_min_rotation_rad)) {
                ApplyOdometryIncrement(prev_odom, curr_odom, fused_pose);
                AddSource(cycle_source, "ROBOT_ODOM");
                ++odom_count;
            }
            prev_odom = curr_odom;
            has_prev_odom = true;
        }

        auto marker_samples = marker_reader.take();
        for (const auto& sample : marker_samples) {
            if (!sample.info().valid()) {
                continue;
            }
            latest_marker = sample.data();
            latest_marker_time = now;
            has_marker = true;
            fresh_marker = true;
            ++marker_count;
            if (!has_fused) {
                fused_pose = latest_marker;
                has_fused = true;
                AddSource(cycle_source, "MARKER_INIT");
            }
        }

        if (has_fused && has_marker &&
            std::chrono::duration<double>(now - latest_marker_time).count() <= marker_timeout_sec) {
            if (MarkerCorrectionAllowed(
                    latest_marker,
                    fused_pose,
                    marker_max_correction_m,
                    marker_max_correction_rad)) {
                SmoothTowardMarker(latest_marker, fused_pose, marker_alpha);
                AddSource(cycle_source, fresh_marker ? "MARKER_CORRECT" : "MARKER_SMOOTH");
            } else {
                AddSource(cycle_source, fresh_marker ? "MARKER_REJECT" : "MARKER_REJECT_HOLD");
            }
        }

        auto detection_samples = detection_reader.take();
        for (const auto& sample : detection_samples) {
            if (!sample.info().valid()) {
                continue;
            }
            ++detection_count;
            latest_balls.clear();
            latest_opponents.clear();
            latest_detected_markers.clear();
            latest_goalposts.clear();

            const double head_yaw_deg = CurrentHeadYawDeg(servo_state);
            const double head_pitch_deg = CurrentHeadPitchDeg(servo_state);
            const double waist_yaw_rad = CurrentWaistYawRad(low_state);
            const auto torso_quat = CurrentTorsoQuatXyzw(torso_imu);

            for (const auto& result : sample.data().results()) {
                const std::string label = result.class_name();
                FieldObject object;
                object.label = label.empty() ? ("class_" + result.class_id()) : label;
                object.score = static_cast<double>(result.score());
                object.root_xy = DetectionToRoot(result, head_yaw_deg, head_pitch_deg, waist_yaw_rad, torso_quat);
                if (has_fused) {
                    object.field_xy = RootToField(fused_pose, object.root_xy);
                    object.has_field = true;
                }

                if (label == "Ball") {
                    latest_balls.push_back(object);
                } else if (IsOpponentClass(label)) {
                    latest_opponents.push_back(object);
                } else if (IsMarkerClass(label)) {
                    latest_detected_markers.push_back(object);
                } else if (label == "Goalpost") {
                    latest_goalposts.push_back(object);
                }
            }

            if (has_fused && team_ball_sender.ok() && !latest_balls.empty()) {
                const FieldObject* best_ball = nullptr;
                for (const auto& ball : latest_balls) {
                    if (!ball.has_field ||
                        !std::isfinite(ball.field_xy.x) ||
                        !std::isfinite(ball.field_xy.y)) {
                        continue;
                    }
                    if (best_ball == nullptr || ball.score > best_ball->score) {
                        best_ball = &ball;
                    }
                }
                if (best_ball != nullptr) {
                    std::ostringstream payload;
                    payload << "RBTEAM1 "
                            << team_id << " "
                            << robot_id << " "
                            << best_ball->field_xy.x << " "
                            << best_ball->field_xy.y << " "
                            << best_ball->score << " "
                            << PoseX(fused_pose) << " "
                            << PoseY(fused_pose) << " "
                            << PoseTheta(fused_pose) << " "
                            << team_ball_seq++;
                    team_ball_sender.send(payload.str());
                }
            }
        }

        if (has_fused) {
            fused_writer.write(fused_pose);
            ++fused_count;
            if (!cycle_source.empty()) {
                last_source = cycle_source;
            } else {
                last_source = "HOLD_LAST";
            }
        }

        if (std::chrono::duration<double>(now - last_status_time).count() >= 1.0) {
            std::cout << "[location_fusion] "
                      << "source=" << (has_fused ? last_source : "WAIT_ABSOLUTE_MARKER") << " "
                      << "rgbd_count=" << rgbd_count << " "
                      << "robot_odom_delta_count=" << odom_count << " "
                      << "marker_count=" << marker_count << " "
                      << "fused_count=" << fused_count << " "
                      << "has_fused=" << (has_fused ? "1" : "0");
            if (has_fused) {
                std::cout << " field_pose=("
                          << "x=" << PoseX(fused_pose) << ","
                          << "y=" << PoseY(fused_pose) << ","
                          << "theta_rad=" << PoseTheta(fused_pose) << ","
                          << "theta_deg=" << PoseTheta(fused_pose) * 180.0 / M_PI << ")";
            } else {
                std::cout << " waiting_for=marker_absolute_on_" << marker_topic_name
                          << " rgbd_cached=" << (has_prev_rgbd ? "1" : "0")
                          << " robot_odom_cached=" << (has_prev_odom ? "1" : "0");
            }
            std::cout << std::endl;

            const Vec2 attack_goal{attack_goal_sign * field_length_m * 0.5, 0.0};
            const Vec2 own_goal{-attack_goal_sign * field_length_m * 0.5, 0.0};
            std::cout << "[field_debug] "
                      << "robot_fused=" << (has_fused ? FormatPose(fused_pose) : "uninitialized") << " "
                      << "marker_robot_pose=" << (has_marker ? FormatPose(latest_marker) : "none") << " "
                      << "ball_field=" << FormatObjectList(latest_balls, 3) << " "
                      << "markers_field=" << FormatObjectList(latest_detected_markers, 6) << " "
                      << "goal_fixed={attack=(" << attack_goal.x << "," << attack_goal.y << "),own=("
                      << own_goal.x << "," << own_goal.y << ")} "
                      << "goalpost_detected=" << FormatObjectList(latest_goalposts, 4) << " "
                      << "opponents_field=" << FormatObjectList(latest_opponents, 4) << " "
                      << "detection_count=" << detection_count
                      << std::endl;
            last_status_time = now;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(std::max(1.0, publish_period_ms))));
    }

    return 0;
}
