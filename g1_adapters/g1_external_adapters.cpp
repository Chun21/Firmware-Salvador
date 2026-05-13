#include "g1_external_adapters.h"

#ifdef ROBOT_MODEL_G1

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <optional>
#include <vector>

#include "localization_pub_sub.h"
#include "logging.h"
#include "multi_target_tracker_pub_sub.h"
#include "near_obstacle_tracker_pub_sub.h"
#include "near_obstacle_tracker_result.h"
#include "object_hypothesis.h"
#include "raster.h"
#include "robot_detection.h"
#include "robot_time.h"
#include "stl_ext.h"
#include "vision_pub_sub.h"

namespace {

constexpr const char* kDetectionTopic = "detectionresults";
constexpr const char* kLocationTopic = "rt/locationresults";
constexpr int64_t kDetectionTimeoutUs = 500_ms;
constexpr int64_t kLocationTimeoutUs = 500_ms;

std::string toLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::optional<point_2d> cameraXyzToRobotRelative(const std::array<float, 3>& xyz) {
    // Vendored RoboCup runtime convention:
    // camera z(depth) -> robot forward x, camera x(right) -> negative robot left y.
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[2]) || std::abs(xyz[2]) < 1e-6f) {
        return std::nullopt;
    }
    return point_2d{xyz[2], -xyz[0]};
}

bool envBoolOrDefault(const char* key, bool default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) {
        return default_value;
    }
    const std::string value = toLower(raw);
    if (value == "1" || value == "true" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "0" || value == "false" || value == "no" || value == "off") {
        return false;
    }
    return default_value;
}

float envFloatOrDefault(const char* key, float default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) {
        return default_value;
    }
    char* end = nullptr;
    const float value = std::strtof(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        return default_value;
    }
    return value;
}

int64_t envSecondsUsOrDefault(const char* key, float default_seconds) {
    return static_cast<int64_t>(envFloatOrDefault(key, default_seconds) * 1'000'000.0f);
}

float degToRad(float deg) {
    return deg * static_cast<float>(M_PI) / 180.0f;
}

htwk::ObjectType mapPointFeature(const std::string& class_name) {
    const std::string name = toLower(class_name);
    if (name == "goalpost")
        return htwk::ObjectType::GOAL_POST;
    if (name == "l")
        return htwk::ObjectType::L_SPOT;
    if (name == "t")
        return htwk::ObjectType::T_SPOT;
    if (name == "x")
        return htwk::ObjectType::X_SPOT;
    if (name == "penaltypoint" || name == "penalty_point" || name == "penalty")
        return htwk::ObjectType::PENALTY_SPOT;
    return htwk::ObjectType::NONE;
}

htwk::ObjectHypothesis makeHypothesis(const DetectionModule::DetectionResult& result,
                                      htwk::ObjectType type) {
    const auto& box = result.box();
    const int x = static_cast<int>(box[0] + box[2] * 0.5f);
    const int y = static_cast<int>(box[1] + box[3] * 0.5f);
    const int r = static_cast<int>(std::max(box[2], box[3]) * 0.5f);
    htwk::ObjectHypothesis hypothesis(x, y, r, static_cast<int>(result.score() * 1000.f));
    hypothesis.type = type;
    hypothesis.prob = result.score();
    return hypothesis;
}

std::shared_ptr<htwk::NearObstacleTrackerResult> makeEmptyObstacleResult() {
    return std::make_shared<htwk::NearObstacleTrackerResult>(
            Raster<float>(40, 40, 0.0f), htwk::Position(0.0f, 0.0f, 0.0f), 0.1f);
}

}  // namespace

G1ExternalAdapters::G1ExternalAdapters() {
    location_filter.configure(
            G1ReadyLocationFilter::Config{.enabled = envBoolOrDefault(
                                                  "ROBOCUP_G1_READY_STABLE_LOC", true),
                                          .stable_required_us = envSecondsUsOrDefault(
                                                  "ROBOCUP_G1_LOC_STABLE_SEC", 1.0f),
                                          .stable_translation_m = envFloatOrDefault(
                                                  "ROBOCUP_G1_LOC_STABLE_TRANSLATION_M", 0.25f),
                                          .stable_rotation_rad = degToRad(envFloatOrDefault(
                                                  "ROBOCUP_G1_LOC_STABLE_ROTATION_DEG", 15.0f)),
                                          .jump_translation_m = envFloatOrDefault(
                                                  "ROBOCUP_G1_LOC_JUMP_TRANSLATION_M", 0.50f),
                                          .jump_rotation_rad = degToRad(envFloatOrDefault(
                                                  "ROBOCUP_G1_LOC_JUMP_ROTATION_DEG", 30.0f)),
                                          .stable_quality = 0.9f,
                                          .unstable_quality = 0.0f});

    detection_subscriber =
            std::make_unique<unitree::robot::ChannelSubscriber<DetectionModule::DetectionResults>>(
                    kDetectionTopic);
    detection_subscriber->InitChannel(
            [this](const void* msg) { this->handleDetection(msg); });

    location_subscriber =
            std::make_unique<unitree::robot::ChannelSubscriber<LocationModule::LocationResult>>(
                    kLocationTopic);
    location_subscriber->InitChannel(
            [this](const void* msg) { this->handleLocation(msg); });

    near_obstacles_tracker_result_channel.publish(makeEmptyObstacleResult());
    LOG_F(INFO,
          "G1 external perception/localization adapters initialized; ready stable loc filter=%s",
          envBoolOrDefault("ROBOCUP_G1_READY_STABLE_LOC", true) ? "on" : "off");
}

G1ExternalAdapters::~G1ExternalAdapters() {
    if (detection_subscriber) {
        detection_subscriber->CloseChannel();
    }
    if (location_subscriber) {
        location_subscriber->CloseChannel();
    }
}

void G1ExternalAdapters::proceed() {
    const int64_t now = time_us();
    near_obstacles_tracker_result_channel.publish(makeEmptyObstacleResult());

    if (last_detection_us.load() != 0 && now - last_detection_us.load() > kDetectionTimeoutUs) {
        rel_ball_channel.publish(std::nullopt);
    }

    if (last_location_us.load() != 0 && now - last_location_us.load() > kLocationTimeoutUs) {
        loc_position_channel.publish(LocPosition{.position = htwk::Position(), .quality = 0.0f});
    }

    if (now - last_timeout_log_us > 2_s) {
        if (last_detection_us.load() == 0 || now - last_detection_us.load() > kDetectionTimeoutUs) {
            LOG_F(WARNING, "G1 detection topic '%s' has no fresh data", kDetectionTopic);
        }
        if (last_location_us.load() == 0 || now - last_location_us.load() > kLocationTimeoutUs) {
            LOG_F(WARNING, "G1 location topic '%s' has no fresh data", kLocationTopic);
        }
        last_timeout_log_us = now;
    }
}

void G1ExternalAdapters::handleDetection(const void* msg) {
    const auto* detection_msg = static_cast<const DetectionModule::DetectionResults*>(msg);
    last_detection_us.store(time_us());

    std::optional<RelBall> best_ball;
    float best_ball_score = -1.0f;
    std::vector<RobotDetection> robots;
    std::vector<htwk::ObjectHypothesis> point_features;
    std::vector<htwk::ObjectHypothesis> opponent_features;

    for (const auto& result : detection_msg->results()) {
        const std::string& class_name = result.class_name();
        if (isBallClass(class_name)) {
            ball_hypothesis_channel.publish(makeHypothesis(result, htwk::ObjectType::BALL));
            if (auto rel = cameraXyzToRobotRelative(result.xyz());
                rel && result.score() > best_ball_score) {
                best_ball_score = result.score();
                best_ball = RelBall{.pos_rel = *rel,
                                    .ball_age_us = 0,
                                    .last_seen_time = time_us(),
                                    .velocity = {0.0f, 0.0f},
                                    .high_risk_velocity = {0.0f, 0.0f},
                                    .medium_risk_velocity = {0.0f, 0.0f},
                                    .is_moving = false};
            }
            continue;
        }

        const htwk::ObjectType point_type = mapPointFeature(class_name);
        if (point_type != htwk::ObjectType::NONE) {
            point_features.push_back(makeHypothesis(result, point_type));
            continue;
        }

        if (isRobotClass(class_name)) {
            opponent_features.push_back(makeHypothesis(result, htwk::ObjectType::ROBOT));
            if (auto rel = cameraXyzToRobotRelative(result.xyz())) {
                robots.push_back(RobotDetection{.pos_rel = *rel, .ownTeamProb = 0.5f});
            }
        }
    }

    rel_ball_channel.publish(best_ball);
    rel_robots_channel.publish(robots);
    point_features_channel.publish(point_features);
    opponent_hypotheses_channel.publish(opponent_features);
}

void G1ExternalAdapters::handleLocation(const void* msg) {
    const auto* location_msg = static_cast<const LocationModule::LocationResult*>(msg);
    last_location_us.store(time_us());

    const htwk::Position raw_position(location_msg->robot2field_x(), location_msg->robot2field_y(),
                                      location_msg->robot2field_theta());
    const auto filtered = location_filter.update(raw_position, last_location_us.load());

    LocPosition loc_position;
    loc_position.position = filtered.position;
    loc_position.quality = filtered.quality;
    loc_position_channel.publish(loc_position);

    const int64_t now = last_location_us.load();
    if ((filtered.rejected_jump || !filtered.stable) && now - last_location_filter_log_us > 1_s) {
        LOG_F(WARNING,
              "G1 location not stable: raw=(%.2f, %.2f, %.1fdeg) used=(%.2f, %.2f, %.1fdeg) "
              "quality=%.1f accepted=%d rejected_jump=%d",
              raw_position.x, raw_position.y, raw_position.a * 180.0f / static_cast<float>(M_PI),
              loc_position.position.x, loc_position.position.y,
              loc_position.position.a * 180.0f / static_cast<float>(M_PI),
              loc_position.quality, filtered.accepted ? 1 : 0, filtered.rejected_jump ? 1 : 0);
        last_location_filter_log_us = now;
    }
}

bool G1ExternalAdapters::isBallClass(const std::string& class_name) {
    const std::string name = toLower(class_name);
    return name == "ball" || name.find("ball") != std::string::npos;
}

bool G1ExternalAdapters::isRobotClass(const std::string& class_name) {
    const std::string name = toLower(class_name);
    return name == "opponent" || name == "person" || name == "robot" || name == "human";
}

#endif  // ROBOT_MODEL_G1
