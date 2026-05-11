#include "RgbdLocalizer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

double read_env_double(const char* key, double default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }

    char* end = nullptr;
    const double parsed = std::strtod(raw, &end);
    if (end == raw) {
        std::cerr << "Invalid numeric env " << key << "=" << raw
                  << ", using default " << default_value << std::endl;
        return default_value;
    }
    return parsed;
}

cv::Mat make_transform(const cv::Matx33d& rotation, const cv::Vec3d& translation) {
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            transform.at<double>(row, col) = rotation(row, col);
        }
        transform.at<double>(row, 3) = translation(row);
    }
    return transform;
}

cv::Mat inverse_rigid_transform(const cv::Mat& transform) {
    cv::Mat inverse = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat rotation = transform(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat translation = transform(cv::Rect(3, 0, 1, 3)).clone();

    cv::Mat rotation_t = rotation.t();
    rotation_t.copyTo(inverse(cv::Rect(0, 0, 3, 3)));
    cv::Mat inverse_translation = -rotation_t * translation;
    inverse_translation.copyTo(inverse(cv::Rect(3, 0, 1, 3)));
    return inverse;
}

cv::Matx33d rot_x(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Matx33d(
        1.0, 0.0, 0.0,
        0.0, c, -s,
        0.0, s, c);
}

cv::Matx33d rot_y(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Matx33d(
        c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c);
}

cv::Matx33d rot_z(double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return cv::Matx33d(
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0);
}

cv::Mat camera_to_body_transform(double head_yaw_deg, double head_pitch_deg, double waist_yaw_rad) {
    const double head_yaw_rad = head_yaw_deg * CV_PI / 180.0;
    const double head_pitch_rad = -head_pitch_deg * CV_PI / 180.0;

    cv::Mat transform = make_transform(
        rot_y(0.6981) * rot_y(1.5707) * rot_z(-1.5707),
        {0.04061, 0.01000, -0.02207});
    transform = make_transform(rot_y(head_pitch_rad), {0.0295, 0.0, 0.013}) * transform;
    transform = make_transform(rot_y(0.039968) * rot_z(head_yaw_rad), {0.030518, 0.0, 0.52486}) * transform;
    transform = make_transform(cv::Matx33d::eye(), {0.0039635, 0.0, -0.047}) * transform;
    transform = make_transform(rot_z(waist_yaw_rad), {-0.0039635, 0.0, 0.044}) * transform;
    return transform;
}

cv::Mat body_to_field_transform(double x_m, double y_m, double theta_rad) {
    return make_transform(rot_z(theta_rad), {x_m, y_m, 0.0});
}

double wrap_angle(double angle_rad) {
    while (angle_rad > CV_PI) {
        angle_rad -= 2.0 * CV_PI;
    }
    while (angle_rad < -CV_PI) {
        angle_rad += 2.0 * CV_PI;
    }
    return angle_rad;
}

double extract_yaw(const cv::Mat& transform) {
    return std::atan2(transform.at<double>(1, 0), transform.at<double>(0, 0));
}

double translation_norm(const cv::Mat& transform) {
    const double tx = transform.at<double>(0, 3);
    const double ty = transform.at<double>(1, 3);
    const double tz = transform.at<double>(2, 3);
    return std::sqrt(tx * tx + ty * ty + tz * tz);
}

}  // namespace

RgbdLocalizer::RgbdLocalizer() {
    init_x_m_ = read_env_double("ROBOCUP_RGBD_INIT_X", 0.0);
    init_y_m_ = read_env_double("ROBOCUP_RGBD_INIT_Y", 0.0);
    init_theta_rad_ = read_env_double("ROBOCUP_RGBD_INIT_THETA_DEG", 0.0) * CV_PI / 180.0;
    max_step_translation_m_ = read_env_double("ROBOCUP_RGBD_MAX_STEP_M", max_step_translation_m_);
    max_step_yaw_rad_ = read_env_double("ROBOCUP_RGBD_MAX_STEP_DEG", 30.0) * CV_PI / 180.0;
}

bool RgbdLocalizer::init_odometry_if_needed(const rs2_intrinsics& intrinsics) {
    if (odometry_ready_) {
        return true;
    }

    if (intrinsics.fx <= 1e-6f || intrinsics.fy <= 1e-6f) {
        std::cerr << "RGB-D localizer: invalid intrinsics, skip odometry init" << std::endl;
        return false;
    }

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        intrinsics.fx, 0.0, intrinsics.ppx,
        0.0, intrinsics.fy, intrinsics.ppy,
        0.0, 0.0, 1.0);

    odometry_ = cv::rgbd::RgbdICPOdometry::create(
        camera_matrix,
        0.15f,
        6.0f,
        0.08f,
        0.08f,
        std::vector<int>{10, 6, 4},
        std::vector<float>{12.0f, 5.0f, 3.0f},
        cv::rgbd::Odometry::RIGID_BODY_MOTION);
    odometry_->setMaxTranslation(max_step_translation_m_);
    odometry_->setMaxRotation(max_step_yaw_rad_);
    odometry_ready_ = true;
    return true;
}

void RgbdLocalizer::reset_world_pose(double head_yaw_deg, double head_pitch_deg, double waist_yaw_rad) {
    const cv::Mat world_T_body = body_to_field_transform(init_x_m_, init_y_m_, init_theta_rad_);
    const cv::Mat body_T_camera = inverse_rigid_transform(
        camera_to_body_transform(head_yaw_deg, head_pitch_deg, waist_yaw_rad));
    world_T_camera_ = world_T_body * body_T_camera;

    pose_.robot2field_x(static_cast<float>(init_x_m_));
    pose_.robot2field_y(static_cast<float>(init_y_m_));
    pose_.robot2field_theta(static_cast<float>(init_theta_rad_));
    has_pose_ = true;
}

bool RgbdLocalizer::update(
    const cv::Mat& color_bgr,
    const cv::Mat& depth_z16,
    const rs2_intrinsics& intrinsics,
    float depth_scale_m,
    double head_yaw_deg,
    double head_pitch_deg,
    double waist_yaw_rad) {
    if (color_bgr.empty() || depth_z16.empty()) {
        return has_pose_;
    }

    if (!init_odometry_if_needed(intrinsics)) {
        return has_pose_;
    }

    cv::Mat gray;
    cv::cvtColor(color_bgr, gray, cv::COLOR_BGR2GRAY);

    cv::Mat depth_m;
    depth_z16.convertTo(depth_m, CV_32F, std::max(1e-6f, depth_scale_m));

    if (!has_pose_) {
        reset_world_pose(head_yaw_deg, head_pitch_deg, waist_yaw_rad);
    }

    if (has_reference_frame_) {
        cv::Mat frame_delta;
        const bool ok = odometry_->compute(prev_gray_, prev_depth_m_, cv::Mat(), gray, depth_m, cv::Mat(), frame_delta);
        if (ok && frame_delta.rows == 4 && frame_delta.cols == 4 && frame_delta.type() == CV_64F) {
            const double step_translation_m = translation_norm(frame_delta);
            const double step_yaw_rad = std::abs(extract_yaw(frame_delta));

            if (step_translation_m <= max_step_translation_m_ && step_yaw_rad <= max_step_yaw_rad_) {
                world_T_camera_ = world_T_camera_ * inverse_rigid_transform(frame_delta);
            } else {
                std::cout << "RGB-D localizer rejected step: translation=" << step_translation_m
                          << " yaw_deg=" << step_yaw_rad * 180.0 / CV_PI << std::endl;
            }
        }
    }

    prev_gray_ = gray.clone();
    prev_depth_m_ = depth_m.clone();
    has_reference_frame_ = true;

    const cv::Mat world_T_body = world_T_camera_ *
        camera_to_body_transform(head_yaw_deg, head_pitch_deg, waist_yaw_rad);
    pose_.robot2field_x(static_cast<float>(world_T_body.at<double>(0, 3)));
    pose_.robot2field_y(static_cast<float>(world_T_body.at<double>(1, 3)));
    pose_.robot2field_theta(static_cast<float>(wrap_angle(extract_yaw(world_T_body))));
    has_pose_ = true;
    return true;
}
