#pragma once

#ifdef ROBOT_MODEL_G1

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <optional>

#include "localization_utils.h"
#include "rel_ball.h"
#include "tc_data.h"

namespace htwk::g1 {

inline int64_t ballFallbackTtlUs() {
    constexpr int64_t kDefaultTtlUs = 5'000'000;
    const char* raw = std::getenv("ROBOCUP_G1_BALL_FALLBACK_TTL_SEC");
    if (raw == nullptr || raw[0] == '\0') {
        return kDefaultTtlUs;
    }
    char* end = nullptr;
    const float value = std::strtof(raw, &end);
    if (end == raw || !std::isfinite(value) || value <= 0.0f) {
        return kDefaultTtlUs;
    }
    return static_cast<int64_t>(value * 1'000'000.0f);
}

inline bool freshTeamBall(const TeamComData& robot, int64_t now_us, int64_t ttl_us) {
    return robot.ball && robot.loc_quality >= 0.7f && !robot.is_fallen &&
           now_us - robot.sent_time_us + robot.ball->ball_age_us <= ttl_us;
}

class BallFallbackMemory {
public:
    std::optional<point_2d> updateAndSelect(const std::optional<RelBall>& rel_ball,
                                            const LocPosition& own_loc,
                                            const std::optional<TeamComData>& teammate,
                                            int64_t now_us, int64_t ttl_us) {
        if (rel_ball) {
            rememberOwnBall(*rel_ball, own_loc, now_us);
            return rel_ball->pos_rel;
        }

        if (teammate && freshTeamBall(*teammate, now_us, ttl_us)) {
            const point_2d abs_ball =
                    LocalizationUtils::relToAbs(teammate->ball->pos_rel, teammate->pos);
            return LocalizationUtils::absToRel(abs_ball, own_loc.position);
        }

        if (last_own_ball_abs && now_us - last_own_ball_time_us <= ttl_us) {
            return LocalizationUtils::absToRel(*last_own_ball_abs, own_loc.position);
        }

        return std::nullopt;
    }

    bool hasFreshFallback(const std::optional<RelBall>& rel_ball,
                          const std::optional<TeamComData>& teammate, int64_t now_us,
                          int64_t ttl_us) const {
        if (rel_ball) {
            return true;
        }
        if (teammate && freshTeamBall(*teammate, now_us, ttl_us)) {
            return true;
        }
        return last_own_ball_abs && now_us - last_own_ball_time_us <= ttl_us;
    }

private:
    void rememberOwnBall(const RelBall& rel_ball, const LocPosition& own_loc, int64_t now_us) {
        if (own_loc.quality < 0.7f) {
            return;
        }
        // Always overwrite with the newest own observation immediately. Use the current update
        // time for TTL bookkeeping so a freshly received ball cannot be treated as stale because
        // of transport or adapter timestamp jitter.
        last_own_ball_abs = LocalizationUtils::relToAbs(rel_ball.pos_rel, own_loc.position);
        last_own_ball_time_us = now_us;
    }

    std::optional<point_2d> last_own_ball_abs;
    int64_t last_own_ball_time_us = 0;
};

}  // namespace htwk::g1

#endif  // ROBOT_MODEL_G1
