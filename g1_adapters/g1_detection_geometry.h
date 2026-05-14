#pragma once

#ifdef ROBOT_MODEL_G1

#include <array>
#include <cmath>
#include <optional>

#include "point_2d.h"

namespace htwk::g1 {

namespace detail {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct Mat3 {
    double m[3][3] = {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
};

inline Mat3 mul(const Mat3& lhs, const Mat3& rhs) {
    Mat3 out{};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out.m[r][c] = 0.0;
            for (int k = 0; k < 3; ++k) {
                out.m[r][c] += lhs.m[r][k] * rhs.m[k][c];
            }
        }
    }
    return out;
}

inline Mat3 rotY(double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return {{{c, 0.0, s}, {0.0, 1.0, 0.0}, {-s, 0.0, c}}};
}

inline Mat3 rotZ(double rad) {
    const double c = std::cos(rad);
    const double s = std::sin(rad);
    return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

inline Vec3 transformPoint(const Mat3& rot, const Vec3& trans, const Vec3& point) {
    return {
            rot.m[0][0] * point.x + rot.m[0][1] * point.y + rot.m[0][2] * point.z + trans.x,
            rot.m[1][0] * point.x + rot.m[1][1] * point.y + rot.m[1][2] * point.z + trans.y,
            rot.m[2][0] * point.x + rot.m[2][1] * point.y + rot.m[2][2] * point.z + trans.z,
    };
}

}  // namespace detail

inline std::optional<point_2d> cameraXyzToRobotRelative(const std::array<float, 3>& xyz,
                                                        float head_yaw_deg,
                                                        float head_pitch_deg) {
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2]) ||
        std::abs(xyz[2]) < 1e-6f) {
        return std::nullopt;
    }

    // Keep this transform aligned with the vendored G1 RGB-D/fusion code:
    // camera optical frame -> G1 head pitch -> G1 head yaw -> robot/root frame.
    detail::Vec3 point{static_cast<double>(xyz[0]), static_cast<double>(xyz[1]),
                       static_cast<double>(xyz[2])};
    const double head_yaw_rad = static_cast<double>(head_yaw_deg) * M_PI / 180.0;
    const double head_pitch_rad = -static_cast<double>(head_pitch_deg) * M_PI / 180.0;

    point = detail::transformPoint(
            detail::mul(detail::mul(detail::rotY(0.6981), detail::rotY(1.5707)),
                        detail::rotZ(-1.5707)),
            {0.04061, 0.01000, -0.02207}, point);
    point = detail::transformPoint(detail::rotY(head_pitch_rad), {0.0295, 0.0, 0.013},
                                   point);
    point = detail::transformPoint(
            detail::mul(detail::rotY(0.039968), detail::rotZ(head_yaw_rad)),
            {0.030518, 0.0, 0.52486}, point);
    point = detail::transformPoint(detail::Mat3{}, {0.0039635, 0.0, -0.047}, point);

    const double range_3d = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
    if (range_3d < 0.15 || point.z >= -0.25) {
        // Conservative fallback preserves the old adapter convention if the full transform looks
        // geometrically implausible.
        return point_2d{xyz[2], -xyz[0]};
    }

    return point_2d{static_cast<float>(point.x), static_cast<float>(point.y)};
}

}  // namespace htwk::g1

#endif  // ROBOT_MODEL_G1
