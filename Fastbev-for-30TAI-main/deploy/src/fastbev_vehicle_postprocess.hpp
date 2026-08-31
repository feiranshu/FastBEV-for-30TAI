#pragma once

#include "types.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fastbev {
namespace vehicle_postprocess {

enum class TensorLayout {
    NCHW,
    NHWC,
};

struct VehiclePostprocessConfig {
    float score_threshold = 0.6f;
    int nms_pre = 1000;
    int max_num = 50;
    float nms_iou_threshold = 0.2f;
    TensorLayout layout = TensorLayout::NHWC;
};

namespace model {

constexpr int kGridWidth = 100;
constexpr int kGridHeight = 100;
constexpr int kSpatialSize = kGridWidth * kGridHeight;
constexpr int kNumClasses = 1;
constexpr int kNumAnchors = 2;
constexpr int kBoxCodeSize = 9;
constexpr int kNumDirectionBins = 2;

constexpr int kClsChannels = kNumAnchors * kNumClasses;       // 2
constexpr int kBboxChannels = kNumAnchors * kBoxCodeSize;     // 18
constexpr int kDirChannels = kNumAnchors * kNumDirectionBins; // 4

constexpr std::size_t kClsElements =
    static_cast<std::size_t>(kClsChannels) * kSpatialSize;
constexpr std::size_t kBboxElements =
    static_cast<std::size_t>(kBboxChannels) * kSpatialSize;
constexpr std::size_t kDirElements =
    static_cast<std::size_t>(kDirChannels) * kSpatialSize;

}  // namespace model

// Vehicle FastBEV Part3 raw outputs:
//   cls  [1,2,100,100]  already sigmoid, do not apply sigmoid again.
//   bbox [1,18,100,100] raw anchor deltas.
//   dir  [1,4,100,100]  raw direction logits.
std::vector<BoundingBox> decode_raw_outputs(
    const float* cls,
    const float* bbox,
    const float* direction,
    const VehiclePostprocessConfig& config);

std::vector<BoundingBox> run_nms(
    std::vector<BoundingBox> candidates,
    const VehiclePostprocessConfig& config);

std::vector<BoundingBox> decode_and_nms(
    const float* cls,
    const float* bbox,
    const float* direction,
    const VehiclePostprocessConfig& config);

// Optional entry for a model variant that has already performed anchor decode.
// boxes is [count,9] in x/y/z/length/width/height/yaw/vx/vy order.
std::vector<BoundingBox> consume_decoded_outputs(
    const float* scores,
    const std::int32_t* direction_labels,
    const float* boxes,
    std::size_t count,
    const VehiclePostprocessConfig& config);

void write_result_txt(const std::string& path, const std::vector<BoundingBox>& boxes);

}  // namespace vehicle_postprocess
}  // namespace fastbev
