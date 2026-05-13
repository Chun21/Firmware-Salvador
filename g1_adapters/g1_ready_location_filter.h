#pragma once

#include <cmath>
#include <cstdint>
#include <optional>

#include "position.h"

class G1ReadyLocationFilter {
public:
    struct Config {
        bool enabled = true;
        int64_t stable_required_us = 1'000'000;
        float stable_translation_m = 0.25f;
        float stable_rotation_rad = 15.0f * static_cast<float>(M_PI) / 180.0f;
        float jump_translation_m = 0.50f;
        float jump_rotation_rad = 30.0f * static_cast<float>(M_PI) / 180.0f;
        float stable_quality = 0.9f;
        float unstable_quality = 0.0f;
    };

    struct Result {
        htwk::Position position;
        float quality = 0.0f;
        bool accepted = false;
        bool stable = false;
        bool rejected_jump = false;
    };

    void configure(Config new_config) {
        config = new_config;
    }

    Result update(const htwk::Position& raw, int64_t now_us) {
        if (!config.enabled) {
            last_observed = raw;
            last_published = raw;
            stable_since_us = now_us;
            jump_candidate.reset();
            return {.position = raw,
                    .quality = raw.isAnyNan() ? 0.0f : config.stable_quality,
                    .accepted = !raw.isAnyNan(),
                    .stable = !raw.isAnyNan(),
                    .rejected_jump = false};
        }

        if (raw.isAnyNan()) {
            return publishRejected(false);
        }

        if (!last_published) {
            last_observed = raw;
            last_published = raw;
            stable_since_us = now_us;
            return {.position = raw,
                    .quality = config.unstable_quality,
                    .accepted = true,
                    .stable = false,
                    .rejected_jump = false};
        }

        if (isJump(raw, *last_published)) {
            return handleJumpCandidate(raw, now_us);
        }

        jump_candidate.reset();

        if (!last_observed || isStableStep(raw, *last_observed)) {
            if (!last_observed) {
                stable_since_us = now_us;
            }
        } else {
            stable_since_us = now_us;
        }

        last_observed = raw;
        last_published = raw;

        const bool stable = now_us - stable_since_us >= config.stable_required_us;
        return {.position = raw,
                .quality = stable ? config.stable_quality : config.unstable_quality,
                .accepted = true,
                .stable = stable,
                .rejected_jump = false};
    }

private:
    struct JumpCandidate {
        htwk::Position position;
        int64_t since_us = 0;
    };

    static float wrappedAngleDiff(float a, float b) {
        float diff = std::fmod(a - b + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
        if (diff < 0.0f) {
            diff += 2.0f * static_cast<float>(M_PI);
        }
        return std::fabs(diff - static_cast<float>(M_PI));
    }

    static float translationDiff(const htwk::Position& a, const htwk::Position& b) {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    bool isStableStep(const htwk::Position& current, const htwk::Position& previous) const {
        return translationDiff(current, previous) <= config.stable_translation_m &&
               wrappedAngleDiff(current.a, previous.a) <= config.stable_rotation_rad;
    }

    bool isJump(const htwk::Position& current, const htwk::Position& published) const {
        return translationDiff(current, published) > config.jump_translation_m ||
               wrappedAngleDiff(current.a, published.a) > config.jump_rotation_rad;
    }

    Result handleJumpCandidate(const htwk::Position& raw, int64_t now_us) {
        if (!jump_candidate || !isStableStep(raw, jump_candidate->position)) {
            jump_candidate = JumpCandidate{.position = raw, .since_us = now_us};
            return publishRejected(true);
        }

        jump_candidate->position = raw;
        if (now_us - jump_candidate->since_us < config.stable_required_us) {
            return publishRejected(true);
        }

        last_observed = raw;
        last_published = raw;
        stable_since_us = now_us - config.stable_required_us;
        jump_candidate.reset();
        return {.position = raw,
                .quality = config.stable_quality,
                .accepted = true,
                .stable = true,
                .rejected_jump = false};
    }

    Result publishRejected(bool rejected_jump) const {
        return {.position = last_published.value_or(htwk::Position()),
                .quality = config.unstable_quality,
                .accepted = false,
                .stable = false,
                .rejected_jump = rejected_jump};
    }

    Config config;
    std::optional<htwk::Position> last_observed;
    std::optional<htwk::Position> last_published;
    std::optional<JumpCandidate> jump_candidate;
    int64_t stable_since_us = 0;
};
