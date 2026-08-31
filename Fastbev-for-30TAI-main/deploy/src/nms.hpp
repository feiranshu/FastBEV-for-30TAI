#pragma once
#include "types.hpp"

namespace fastbev {
namespace nms {

std::vector<BoundingBox> run_multi_class_nms(
    std::vector<BoundingBox>& candidates, 
    const NMSConfig& config);

} // namespace nms
} // namespace fastbev