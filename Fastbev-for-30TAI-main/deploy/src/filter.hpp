#pragma once
#include "types.hpp"

namespace fastbev {
namespace filter {

// 核心解算函数：将 NPU 输出的 NHWC 裸内存数据，转换为结构化的 3D 候选框
std::vector<BoundingBox> threshold_and_decode(
    const float* cls_ptr,  // 分类得分指针 (NHWC排布)
    const float* bbox_ptr, // 3D框回归参数指针 (NHWC排布)
    const float* dir_ptr,  // 方向分类指针 (NHWC排布)
    float score_thr);      // 过滤阈值

} // namespace filter
} // namespace fastbev