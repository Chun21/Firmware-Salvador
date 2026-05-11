#pragma once

#include <string>
#include <vector>

const std::vector<std::string> CLASS_NAMES = {
   "Ball", "Goalpost", "X", "L", "T", "PenaltyPoint", "person", "Opponent", "BRMarker", "Robot", "Human"};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0, 114, 189}, {217, 83, 25}, {237, 177, 32}, {126, 47, 142}, {119, 172, 48},
    {77, 190, 238}, {162, 20, 47}, {255, 255, 0}, {0, 255, 255}, {255, 0, 255}, {255, 128, 0} };

inline const std::string& SafeClassName(int class_id) {
    static const std::string kUnknown = "Unknown";
    if (class_id < 0 || class_id >= static_cast<int>(CLASS_NAMES.size())) {
        return kUnknown;
    }
    return CLASS_NAMES[class_id];
}

inline const std::vector<unsigned int>& SafeColor(int class_id) {
    static const std::vector<unsigned int> kDefaultColor{128, 128, 128};
    if (class_id < 0 || class_id >= static_cast<int>(COLORS.size())) {
        return kDefaultColor;
    }
    return COLORS[class_id];
}
