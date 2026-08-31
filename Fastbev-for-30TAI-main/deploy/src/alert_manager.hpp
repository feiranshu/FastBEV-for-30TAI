#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <cstdint>
#include <vector>

#include "types.hpp"

namespace fastbev {

enum class AlertLevel {
    None = 0,
    Caution = 1,
    Danger = 2,
    Emergency = 3
};

struct AlertDecision {
    AlertLevel level = AlertLevel::None;
    int class_id = -1;
    float score = 0.0f;
    float clearance_m = std::numeric_limits<float>::infinity();
    int frame_id = -1;
    std::string direction;
    bool state_changed = false;
};

struct AlertPolicyConfig {
    float confidence_threshold = 0.62f;
    float caution_distance_m = 10.0f;
    float danger_distance_m = 5.0f;
    float emergency_distance_m = 3.0f;
    int caution_confirm_frames = 3;
    int danger_confirm_frames = 2;
    int emergency_confirm_frames = 2;
    int clear_frames = 3;
    float hysteresis_m = 2.0f;
    std::array<bool, 10> enabled_class = {
        true, true, true, true, true, true, true, true, false, false
    };
};

class AlertPolicy {
public:
    explicit AlertPolicy(const AlertPolicyConfig& config = AlertPolicyConfig{});

    AlertDecision evaluate(const std::vector<BoundingBox>& boxes, int frame_id);
    void reset();

    AlertLevel current_level() const { return current_level_; }
    const AlertPolicyConfig& config() const { return config_; }

    static float clearance_m(const BoundingBox& box);
    static std::string direction_for(float x, float y);
    static const char* level_name(AlertLevel level);
    static const char* class_name(int class_id);

private:
    struct Observation {
        bool valid = false;
        const BoundingBox* box = nullptr;
        float clearance_m = std::numeric_limits<float>::infinity();
    };

    bool is_valid_target(const BoundingBox& box) const;
    float threshold_for(AlertLevel level) const;
    int confirm_frames_for(AlertLevel level) const;
    static int index(AlertLevel level) { return static_cast<int>(level); }

    AlertPolicyConfig config_;
    AlertLevel current_level_ = AlertLevel::None;
    std::array<int, 4> confirm_count_{};
    int clear_count_ = 0;
    int last_frame_id_ = -1;
};

struct AlertLedConfig {
    bool enabled = false;
    std::string dev_mem = "/dev/mem";
    std::uint64_t register_base = 0x400C0000ULL;
    std::uint32_t duration_ms = 3000;
    std::uint32_t refresh_ms = 2500;
    std::uint32_t danger_toggle_ms = 500;
    std::uint32_t emergency_toggle_ms = 125;
    bool dry_run = false;
};

class AlertLedController {
public:
    using RegisterObserver = std::function<void(std::uint32_t, std::uint32_t)>;
    using ErrorReporter = std::function<void(const std::string&)>;
    using Clock = std::chrono::steady_clock;
    using NowProvider = std::function<Clock::time_point()>;

    explicit AlertLedController(const AlertLedConfig& config,
                                RegisterObserver observer = {},
                                ErrorReporter error_reporter = {},
                                NowProvider now_provider = {});
    ~AlertLedController();

    AlertLedController(const AlertLedController&) = delete;
    AlertLedController& operator=(const AlertLedController&) = delete;

    void update(const AlertDecision& decision);
    void clear();
    bool available() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

struct AlertAudioConfig {
    bool enabled = false;
    std::string device = "default";
    std::string asset_dir = "./assets/alerts";
    int cooldown_ms = 5000;
    bool dry_run = false;
};

class AlertAudioPlayer {
public:
    using RequestObserver = std::function<void(AlertLevel, const std::string&)>;
    using ErrorReporter = std::function<void(const std::string&)>;

    explicit AlertAudioPlayer(const AlertAudioConfig& config,
                              RequestObserver observer = {},
                              ErrorReporter error_reporter = {});
    ~AlertAudioPlayer();

    AlertAudioPlayer(const AlertAudioPlayer&) = delete;
    AlertAudioPlayer& operator=(const AlertAudioPlayer&) = delete;

    void update(const AlertDecision& decision);
    bool wait_until_idle(std::chrono::milliseconds timeout);
    void stop();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fastbev
