#pragma once

#include "alert_manager.hpp"
#include "yaml-cpp/yaml.h"

namespace fastbev {

struct AlertRuntimeConfig {
    bool enabled = false;
    AlertPolicyConfig policy;
    AlertLedConfig led;
    AlertAudioConfig audio;
};

// Missing alert node is intentionally treated as disabled for old-config compatibility.
// Invalid enabled configuration throws std::invalid_argument; the pipeline catches it and
// continues inference with the complete alert subsystem disabled.
AlertRuntimeConfig load_alert_runtime_config(const YAML::Node& root);

}  // namespace fastbev
