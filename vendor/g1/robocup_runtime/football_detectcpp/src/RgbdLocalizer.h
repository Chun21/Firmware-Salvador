#pragma once

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>

#include "common/LocationModule.hpp"

class RgbdLocalizer {
public:
    RgbdLocalizer();

    bool update(
        const cv::Mat& color_bgr,
        const cv::Mat& depth_z16,
        const rs2_intrinsics& intrinsics,
        float depth_scale_m,
        double head_yaw_deg,
        double head_pitch_deg,
        double waist_yaw_rad);

    bool has_pose() const {
        return has_pose_;
    }

    const LocationModule::LocationResult& pose() const {
        return pose_;
    }

private:
    bool init_odometry_if_needed(const rs2_intrinsics& intrinsics);
    void reset_world_pose(double head_yaw_deg, double head_pitch_deg, double waist_yaw_rad);

    cv::Ptr<cv::rgbd::RgbdICPOdometry> odometry_;
    cv::Mat prev_gray_;
    cv::Mat prev_depth_m_;
    cv::Mat world_T_camera_;

    LocationModule::LocationResult pose_;
    bool odometry_ready_ = false;
    bool has_reference_frame_ = false;
    bool has_pose_ = false;

    double init_x_m_ = 0.0;
    double init_y_m_ = 0.0;
    double init_theta_rad_ = 0.0;
    double max_step_translation_m_ = 0.30;
    double max_step_yaw_rad_ = 30.0 * CV_PI / 180.0;
};
