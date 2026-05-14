#pragma once

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <string>

inline float g1DetectReadEnvFloat(const char* key, float default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr || raw[0] == '\0') {
        return default_value;
    }
    char* end = nullptr;
    const float parsed = std::strtof(raw, &end);
    if (end == raw || !std::isfinite(parsed)) {
        return default_value;
    }
    return parsed;
}

inline std::string g1DetectLower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

inline bool g1DetectIsBallClass(const std::string& class_name) {
    const std::string name = g1DetectLower(class_name);
    return name == "ball" || name.find("ball") != std::string::npos;
}

inline bool g1DetectIsMarkerClass(const std::string& class_name) {
    return class_name == "L" || class_name == "T" || class_name == "X";
}

inline float g1DetectMarkerMinScore(float range_m) {
    const float base = g1DetectReadEnvFloat("ROBOCUP_MARKER_MIN_CONFIDENCE", 40.0f) / 100.0f;
    const float far_start =
            g1DetectReadEnvFloat("ROBOCUP_MARKER_FAR_CONFIDENCE_START_M", 3.0f);
    const float far_slope =
            g1DetectReadEnvFloat("ROBOCUP_MARKER_FAR_CONFIDENCE_SLOPE", 5.0f) / 100.0f;
    const float far_max =
            g1DetectReadEnvFloat("ROBOCUP_MARKER_FAR_CONFIDENCE_MAX", 65.0f) / 100.0f;
    return std::min(far_max, base + std::max(0.0f, range_m - far_start) * far_slope);
}

inline bool g1DetectDisplayAccepts(const std::string& class_name, float score, float range_m) {
    if (g1DetectIsBallClass(class_name)) {
        return score >= g1DetectReadEnvFloat("ROBOCUP_G1_BALL_MIN_SCORE", 0.6f);
    }
    if (g1DetectIsMarkerClass(class_name)) {
        return score >= g1DetectMarkerMinScore(range_m);
    }
    return true;
}
