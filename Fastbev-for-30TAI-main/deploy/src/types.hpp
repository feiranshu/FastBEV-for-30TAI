#pragma once
#include <vector>

namespace fastbev {


// ============================================================================
// [结构体] BoundingBox: 下游规控模块直接使用的最终 3D 目标框结构
// ============================================================================
struct BoundingBox {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // 默认原点
    float w = 0.0f, l = 0.0f, h = 0.0f;  // 默认尺寸为 0
    float yaw = 0.0f;                    // 默认无旋转
    float vx = 0.0f, vy = 0.0f;          // 默认静止状态，防止速度乱码导致规控急刹
    float score = 0.0f;                  // 默认无置信度
    int id = -1;                         // 默认 -1，表示这是个“无效类别”或“未分配类别”
};

// ============================================================================
// [结构体] Anchor: 网络在每个网格点预设的先验框基准
// ============================================================================
struct Anchor {
    float w = 0.0f, l = 0.0f, h = 0.0f; 
    float z = 0.0f;         
    float rot = 0.0f;       
};

// ============================================================================
// [结构体] NMSConfig: 非极大值抑制算法的配置参数
// ============================================================================
struct NMSConfig {
    float score_thr = 0.1f;          // 初筛阈值，低于此分数的框直接丢弃
    int max_num = 500;               // 单帧最多输出的框数量，防撑爆下游总线
    std::vector<float> nms_thr_list; // 针对不同类别设置不同的 NMS IoU 阈值
    // Optional CARLA refinements. Zero disables each refinement so existing
    // NuScenes pipelines retain their original NMS behavior.
    float pedestrian_motorcycle_center_distance_m = 0.0f;
    float cross_class_iou_threshold = 0.0f;
};

} // namespace fastbev
