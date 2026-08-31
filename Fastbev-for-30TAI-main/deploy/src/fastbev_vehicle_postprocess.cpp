#include "fastbev_vehicle_postprocess.hpp"

#include "nms.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>

namespace fastbev {
namespace vehicle_postprocess {
namespace {

constexpr float kPi = 3.14159265358979323846f;

// Vehicle sandbox training/decode constants. These are intentionally kept
// local to the vehicle postprocess path because NuScenes/CARLA use different
// class counts, anchor sets, grid ranges, and output shapes.
constexpr float kRangeMinX = -72.0f;
constexpr float kRangeMinY = -72.0f;
constexpr float kGridStepX = 1.44f;
constexpr float kGridStepY = 1.44f;

constexpr float kAnchorLength = 5.124f;
constexpr float kAnchorWidth = 1.928f;
constexpr float kAnchorHeight = 2.004f;
constexpr float kAnchorBottomZ = -4.32f;
constexpr float kAnchorRotations[model::kNumAnchors] = {0.0f, 1.57079632679f};

constexpr float kDirectionOffset = 0.7854f;
constexpr float kDirectionLimitOffset = 0.0f;

struct CandidateIndex {
    float score = 0.0f;
    int spatial = 0;
    int anchor = 0;
};

float value_at(const float* data,
               TensorLayout layout,
               int channels,
               int channel,
               int spatial) {
    if (layout == TensorLayout::NCHW) {
        return data[static_cast<std::size_t>(channel) * model::kSpatialSize + spatial];
    }
    return data[static_cast<std::size_t>(spatial) * channels + channel];
}

float limit_period(float value, float offset, float period) {
    return value - std::floor(value / period + offset) * period;
}

float correct_direction(float yaw, int direction_label) {
    const float direction_rot = limit_period(
        yaw - kDirectionOffset, kDirectionLimitOffset, kPi);
    return direction_rot + kDirectionOffset + kPi * direction_label;
}

BoundingBox decode_one(const float* bbox,
                       const float* direction,
                       TensorLayout layout,
                       int spatial,
                       int anchor_index,
                       float score) {
    const int grid_x = spatial % model::kGridWidth;
    const int grid_y = spatial / model::kGridWidth;
    const float anchor_x = kRangeMinX + (static_cast<float>(grid_x) + 0.5f) * kGridStepX;
    const float anchor_y = kRangeMinY + (static_cast<float>(grid_y) + 0.5f) * kGridStepY;
    const float anchor_z = kAnchorBottomZ;
    const float anchor_w = kAnchorLength;
    const float anchor_l = kAnchorWidth;
    const float anchor_h = kAnchorHeight;
    const float anchor_r = kAnchorRotations[anchor_index];

    auto delta = [&](int code) {
        const int channel = anchor_index * model::kBoxCodeSize + code;
        return value_at(bbox, layout, model::kBboxChannels, channel, spatial);
    };

    const float diagonal = std::sqrt(anchor_l * anchor_l + anchor_w * anchor_w);
    const float x = delta(0) * diagonal + anchor_x;
    const float y = delta(1) * diagonal + anchor_y;
    const float z_center = delta(2) * anchor_h + (anchor_z + anchor_h * 0.5f);
    const float length = std::exp(delta(3)) * anchor_w;
    const float width = std::exp(delta(4)) * anchor_l;
    const float height = std::exp(delta(5)) * anchor_h;
    const float z_bottom = z_center - height * 0.5f;
    const float yaw_raw = delta(6) + anchor_r;

    const int dir_channel = anchor_index * model::kNumDirectionBins;
    const float dir0 = value_at(
        direction, layout, model::kDirChannels, dir_channel, spatial);
    const float dir1 = value_at(
        direction, layout, model::kDirChannels, dir_channel + 1, spatial);
    const int direction_label = dir1 > dir0 ? 1 : 0;

    BoundingBox box;
    box.x = x;
    box.y = y;
    box.z = z_bottom;
    box.w = length;
    box.l = width;
    box.h = height;
    box.yaw = correct_direction(yaw_raw, direction_label);
    box.vx = delta(7);
    box.vy = delta(8);
    box.score = score;
    box.id = 0;
    return box;
}

void validate_config(const VehiclePostprocessConfig& config) {
    if (!std::isfinite(config.score_threshold) ||
        config.score_threshold < 0.0f || config.score_threshold > 1.0f) {
        throw std::runtime_error("vehicle score_threshold must be in [0,1]");
    }
    if (config.nms_pre <= 0) {
        throw std::runtime_error("vehicle nms_pre must be positive");
    }
    if (config.max_num <= 0) {
        throw std::runtime_error("vehicle max_num must be positive");
    }
    if (!std::isfinite(config.nms_iou_threshold) ||
        config.nms_iou_threshold < 0.0f || config.nms_iou_threshold > 1.0f) {
        throw std::runtime_error("vehicle nms_iou_threshold must be in [0,1]");
    }
}

}  // namespace

std::vector<BoundingBox> decode_raw_outputs(
    const float* cls,
    const float* bbox,
    const float* direction,
    const VehiclePostprocessConfig& config) {
    if (!cls || !bbox || !direction) {
        throw std::invalid_argument("vehicle raw output pointer is null");
    }
    validate_config(config);

    std::vector<CandidateIndex> selected;
    selected.reserve(1024);
    for (int spatial = 0; spatial < model::kSpatialSize; ++spatial) {
        for (int anchor = 0; anchor < model::kNumAnchors; ++anchor) {
            // cls has only one class per anchor. The score is already sigmoid.
            const int channel = anchor * model::kNumClasses;
            const float score = value_at(
                cls, config.layout, model::kClsChannels, channel, spatial);
            if (std::isfinite(score) && score >= config.score_threshold) {
                selected.push_back({score, spatial, anchor});
            }
        }
    }

    std::sort(selected.begin(), selected.end(),
              [](const CandidateIndex& a, const CandidateIndex& b) {
                  return a.score > b.score;
              });
    if (selected.size() > static_cast<std::size_t>(config.nms_pre)) {
        selected.resize(static_cast<std::size_t>(config.nms_pre));
    }

    std::vector<BoundingBox> boxes;
    boxes.reserve(selected.size());
    for (const CandidateIndex& item : selected) {
        boxes.push_back(decode_one(
            bbox, direction, config.layout, item.spatial, item.anchor, item.score));
    }
    return boxes;
}

std::vector<BoundingBox> run_nms(
    std::vector<BoundingBox> candidates,
    const VehiclePostprocessConfig& config) {
    validate_config(config);
    NMSConfig nms_config;
    nms_config.score_thr = config.score_threshold;
    nms_config.max_num = config.max_num;
    nms_config.nms_thr_list = {config.nms_iou_threshold};
    return nms::run_multi_class_nms(candidates, nms_config);
}

std::vector<BoundingBox> decode_and_nms(
    const float* cls,
    const float* bbox,
    const float* direction,
    const VehiclePostprocessConfig& config) {
    return run_nms(decode_raw_outputs(cls, bbox, direction, config), config);
}

std::vector<BoundingBox> consume_decoded_outputs(
    const float* scores,
    const std::int32_t* direction_labels,
    const float* boxes,
    std::size_t count,
    const VehiclePostprocessConfig& config) {
    if (!scores || !direction_labels || !boxes) {
        throw std::invalid_argument("vehicle decoded output pointer is null");
    }
    validate_config(config);

    std::vector<BoundingBox> candidates;
    candidates.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        const float score = scores[i];
        if (!std::isfinite(score) || score < config.score_threshold) continue;

        const float* box = boxes + i * model::kBoxCodeSize;
        const int direction_label = direction_labels[i] == 0 ? 0 : 1;

        BoundingBox decoded;
        decoded.x = box[0];
        decoded.y = box[1];
        decoded.z = box[2];
        decoded.w = box[3];
        decoded.l = box[4];
        decoded.h = box[5];
        decoded.yaw = correct_direction(box[6], direction_label);
        decoded.vx = box[7];
        decoded.vy = box[8];
        decoded.score = score;
        decoded.id = 0;
        candidates.push_back(decoded);
    }
    return run_nms(std::move(candidates), config);
}

void write_result_txt(const std::string& path, const std::vector<BoundingBox>& boxes) {
    std::ofstream output(path);
    if (!output) {
        throw std::runtime_error("cannot create vehicle result file: " + path);
    }
    output.precision(9);
    for (const BoundingBox& box : boxes) {
        output << box.x << ' ' << box.y << ' ' << box.z << ' '
               << box.w << ' ' << box.l << ' ' << box.h << ' '
               << box.yaw << ' ' << box.id << ' ' << box.score << '\n';
    }
}

}  // namespace vehicle_postprocess
}  // namespace fastbev
