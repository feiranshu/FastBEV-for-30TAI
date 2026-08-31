#include "alert_config.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

namespace fastbev {
namespace {

template <typename T>
T value_or(const YAML::Node& node, const char* key, const T& fallback) {
    return node && node[key] ? node[key].as<T>() : fallback;
}

void require(bool condition, const char* message) {
    if (!condition) throw std::invalid_argument(message);
}

}  // namespace

AlertRuntimeConfig load_alert_runtime_config(const YAML::Node& root) {
    AlertRuntimeConfig result;
    const YAML::Node alert = root["alert"];
    if (!alert || !value_or<bool>(alert, "enabled", false)) return result;

    result.enabled = true;
    result.policy.confidence_threshold =
        value_or<float>(alert, "confidence_threshold", 0.62f);
    require(std::isfinite(result.policy.confidence_threshold) &&
                result.policy.confidence_threshold >= 0.0f &&
                result.policy.confidence_threshold <= 1.0f,
            "alert.confidence_threshold must be in [0, 1]");

    result.policy.enabled_class.fill(false);
    const std::vector<int> default_classes = {0, 1, 2, 3, 4, 5, 6, 7};
    const std::vector<int> classes = alert["class_ids"]
        ? alert["class_ids"].as<std::vector<int>>() : default_classes;
    require(!classes.empty(), "alert.class_ids must not be empty");
    for (int id : classes) {
        require(id >= 0 && id <= 7,
                "alert.class_ids may only contain dynamic classes 0 through 7");
        require(!result.policy.enabled_class[static_cast<std::size_t>(id)],
                "alert.class_ids contains a duplicate class ID");
        result.policy.enabled_class[static_cast<std::size_t>(id)] = true;
    }

    const YAML::Node distance = alert["distance_m"];
    result.policy.caution_distance_m = value_or<float>(distance, "caution", 10.0f);
    result.policy.danger_distance_m = value_or<float>(distance, "danger", 5.0f);
    result.policy.emergency_distance_m = value_or<float>(distance, "emergency", 3.0f);

    const YAML::Node confirm = alert["confirm_frames"];
    result.policy.caution_confirm_frames = value_or<int>(confirm, "caution", 3);
    result.policy.danger_confirm_frames = value_or<int>(confirm, "danger", 2);
    result.policy.emergency_confirm_frames = value_or<int>(confirm, "emergency", 2);
    result.policy.clear_frames = value_or<int>(alert, "clear_frames", 3);
    result.policy.hysteresis_m = value_or<float>(alert, "hysteresis_m", 2.0f);

    // Construct once here to apply the same validation used by runtime policy objects.
    AlertPolicy validated_policy(result.policy);
    (void)validated_policy;

    const YAML::Node direction = alert["direction_degrees"];
    const float front_half_width = value_or<float>(direction, "front_half_width", 30.0f);
    const float diagonal_width = value_or<float>(direction, "diagonal_width", 60.0f);
    require(std::fabs(front_half_width - 30.0f) < 1e-4f &&
                std::fabs(diagonal_width - 60.0f) < 1e-4f,
            "only the verified 30/60 degree direction sectors are supported");

    const YAML::Node led = alert["led"];
    result.led.enabled = value_or<bool>(led, "enabled", true);
    result.led.dev_mem = value_or<std::string>(led, "dev_mem", "/dev/mem");
    result.led.duration_ms = value_or<std::uint32_t>(led, "duration_ms", 3000);
    result.led.refresh_ms = value_or<std::uint32_t>(led, "refresh_ms", 2500);
    result.led.danger_toggle_ms =
        value_or<std::uint32_t>(led, "danger_toggle_ms", 500);
    result.led.emergency_toggle_ms =
        value_or<std::uint32_t>(led, "emergency_toggle_ms", 125);
    AlertLedConfig led_validation = result.led;
    led_validation.enabled = false;
    AlertLedController validated_led(led_validation);
    (void)validated_led;

    const YAML::Node audio = alert["audio"];
    result.audio.enabled = value_or<bool>(audio, "enabled", true);
    result.audio.device = value_or<std::string>(audio, "device", "default");
    result.audio.asset_dir =
        value_or<std::string>(audio, "directory", "./assets/alerts");
    result.audio.cooldown_ms = value_or<int>(audio, "cooldown_ms", 5000);
    const std::string minimum_level =
        value_or<std::string>(audio, "minimum_level", "danger");
    const bool emergency_preempts = value_or<bool>(audio, "emergency_preempts", true);
    require(minimum_level == "danger",
            "alert.audio.minimum_level currently must be danger");
    require(emergency_preempts,
            "alert.audio.emergency_preempts must be true for the verified policy");
    AlertAudioConfig audio_validation = result.audio;
    audio_validation.enabled = false;
    AlertAudioPlayer validated_audio(audio_validation);
    validated_audio.stop();

    return result;
}

}  // namespace fastbev
