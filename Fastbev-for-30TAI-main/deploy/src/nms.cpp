#include "nms.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace fastbev {
namespace nms {

const float ThresHold = 1e-8;

// 因为脱离了 CUDA 环境，我们需要自己定义一个简单的 float2
struct float2 {
    float x, y;
};

// ===================================================================
// 纯数学几何函数区 (用于计算旋转框的重叠面积)
// ===================================================================

static float cross(const float2 p1, const float2 p2, const float2 p0) {
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

static int check_box2d(const BoundingBox& box, const float2 p) {
    const float MARGIN = 1e-5;
    float center_x = box.x;
    float center_y = box.y;
    float angle_cos = std::cos(-box.yaw);
    float angle_sin = std::sin(-box.yaw);
    float rot_x = (p.x - center_x) * angle_cos + (p.y - center_y) * angle_sin + center_x;
    float rot_y = -(p.x - center_x) * angle_sin + (p.y - center_y) * angle_cos + center_y;

    float half_w = box.w / 2.0f;
    float half_l = box.l / 2.0f;

    return (rot_x > (center_x - half_w - MARGIN) && rot_x < (center_x + half_w + MARGIN) && 
            rot_y > (center_y - half_l - MARGIN) && rot_y < (center_y + half_l + MARGIN));
}

static bool intersection(const float2 p1, const float2 p0, const float2 q1, const float2 q0, float2 &ans) {
    if (( std::min(p0.x, p1.x) <= std::max(q0.x, q1.x) &&
          std::min(q0.x, q1.x) <= std::max(p0.x, p1.x) &&
          std::min(p0.y, p1.y) <= std::max(q0.y, q1.y) &&
          std::min(q0.y, q1.y) <= std::max(p0.y, p1.y) ) == 0)
        return false;

    float s1 = cross(q0, p1, p0);
    float s2 = cross(p1, q1, p0);
    float s3 = cross(p0, q1, q0);
    float s4 = cross(q1, p1, q0);

    if (!(s1 * s2 > 0 && s3 * s4 > 0))
        return false;

    float s5 = cross(q1, p1, p0);
    if (std::fabs(s5 - s1) > ThresHold) {
        ans.x = (s5 * q0.x - s1 * q1.x) / (s5 - s1);
        ans.y = (s5 * q0.y - s1 * q1.y) / (s5 - s1);
    } else {
        float a0 = p0.y - p1.y, b0 = p1.x - p0.x, c0 = p0.x * p1.y - p1.x * p0.y;
        float a1 = q0.y - q1.y, b1 = q1.x - q0.x, c1 = q0.x * q1.y - q1.x * q0.y;
        float D = a0 * b1 - a1 * b0;
        ans.x = (b0 * c1 - b1 * c0) / D;
        ans.y = (a1 * c0 - a0 * c1) / D;
    }
    return true;
}

static void rotate_around_center(const float2 &center, const float angle_cos, const float angle_sin, float2 &p) {
    float new_x = (p.x - center.x) * angle_cos + (p.y - center.y) * angle_sin + center.x;
    float new_y = -(p.x - center.x) * angle_sin + (p.y - center.y) * angle_cos + center.y;
    p = {new_x, new_y};
}

static float box_overlap(const BoundingBox& box_a, const BoundingBox& box_b) {
    float2 center_a = {box_a.x, box_a.y};
    float2 center_b = {box_b.x, box_b.y};

    float2 box_a_corners[5];
    float2 box_b_corners[5];

    float half_wa = box_a.w / 2.0f, half_la = box_a.l / 2.0f;
    box_a_corners[0] = {box_a.x - half_wa, box_a.y - half_la};
    box_a_corners[1] = {box_a.x + half_wa, box_a.y - half_la};
    box_a_corners[2] = {box_a.x + half_wa, box_a.y + half_la};
    box_a_corners[3] = {box_a.x - half_wa, box_a.y + half_la};

    float half_wb = box_b.w / 2.0f, half_lb = box_b.l / 2.0f;
    box_b_corners[0] = {box_b.x - half_wb, box_b.y - half_lb};
    box_b_corners[1] = {box_b.x + half_wb, box_b.y - half_lb};
    box_b_corners[2] = {box_b.x + half_wb, box_b.y + half_lb};
    box_b_corners[3] = {box_b.x - half_wb, box_b.y + half_lb};

    float a_angle_cos = std::cos(box_a.yaw), a_angle_sin = std::sin(box_a.yaw);
    float b_angle_cos = std::cos(box_b.yaw), b_angle_sin = std::sin(box_b.yaw);

    for (int k = 0; k < 4; k++) {
        rotate_around_center(center_a, a_angle_cos, a_angle_sin, box_a_corners[k]);
        rotate_around_center(center_b, b_angle_cos, b_angle_sin, box_b_corners[k]);
    }

    box_a_corners[4] = box_a_corners[0];
    box_b_corners[4] = box_b_corners[0];

    float2 cross_points[16];
    float2 poly_center = {0.0f, 0.0f};
    int cnt = 0;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (intersection(box_a_corners[i + 1], box_a_corners[i],
                             box_b_corners[j + 1], box_b_corners[j], cross_points[cnt])) {
                poly_center.x += cross_points[cnt].x;
                poly_center.y += cross_points[cnt].y;
                cnt++;
            }
        }
    }

    for (int k = 0; k < 4; k++) {
        if (check_box2d(box_a, box_b_corners[k])) {
            poly_center.x += box_b_corners[k].x;
            poly_center.y += box_b_corners[k].y;
            cross_points[cnt++] = box_b_corners[k];
        }
        if (check_box2d(box_b, box_a_corners[k])) {
            poly_center.x += box_a_corners[k].x;
            poly_center.y += box_a_corners[k].y;
            cross_points[cnt++] = box_a_corners[k];
        }
    }

    if (cnt == 0) return 0.0f;

    poly_center.x /= cnt;
    poly_center.y /= cnt;

    // 极角排序
    for (int j = 0; j < cnt - 1; j++) {
        for (int i = 0; i < cnt - j - 1; i++) {
            if (std::atan2(cross_points[i].y - poly_center.y, cross_points[i].x - poly_center.x) >
                std::atan2(cross_points[i+1].y - poly_center.y, cross_points[i+1].x - poly_center.x)) {
                std::swap(cross_points[i], cross_points[i + 1]);
            }
        }
    }

    // Shoelace 公式算面积
    float area = 0.0f;
    for (int k = 0; k < cnt - 1; k++) {
        float2 a = {cross_points[k].x - cross_points[0].x, cross_points[k].y - cross_points[0].y};
        float2 b = {cross_points[k + 1].x - cross_points[0].x, cross_points[k + 1].y - cross_points[0].y};
        area += (a.x * b.y - a.y * b.x);
    }
    return std::fabs(area) / 2.0f;
}

// ===================================================================
// NMS 调度与执行逻辑
// ===================================================================

static std::vector<int> argsort_cpu(const std::vector<BoundingBox>& boxes) {
    std::vector<int> order(boxes.size());
    std::iota(order.begin(), order.end(), 0); 
    std::sort(order.begin(), order.end(),
              [&boxes](int i1, int i2) { return boxes[i1].score > boxes[i2].score; });
    return order;
}

static bool is_cross_class_nms_class(int class_id) {
    // car, truck, trailer, bus, construction_vehicle, motorcycle, pedestrian
    return (class_id >= 0 && class_id <= 4) || class_id == 6 || class_id == 7;
}

static float center_distance_squared(const BoundingBox& box_a,
                                     const BoundingBox& box_b) {
    const float dx = box_a.x - box_b.x;
    const float dy = box_a.y - box_b.y;
    return dx * dx + dy * dy;
}

std::vector<BoundingBox> run_multi_class_nms(std::vector<BoundingBox>& candidates, const NMSConfig& config) {
    std::vector<BoundingBox> result;
    if (candidates.empty()) return result;

    std::vector<int> order = argsort_cpu(candidates);
    std::vector<int> keep(candidates.size(), 0);

    for (size_t i = 0; i < order.size(); i++) {
        if (keep[order[i]] == 1) continue;
        
        BoundingBox& box_a = candidates[order[i]];
        result.push_back(box_a);

        if (result.size() >= static_cast<size_t>(config.max_num)) break;

        for (size_t j = i + 1; j < order.size(); j++) {
            if (keep[order[j]] == 1) continue; 
            
            BoundingBox& box_b = candidates[order[j]];

            const bool same_class = box_a.id == box_b.id;
            const bool cross_class_pair =
                !same_class && config.cross_class_iou_threshold > 0.0f &&
                is_cross_class_nms_class(box_a.id) &&
                is_cross_class_nms_class(box_b.id);

            if (!same_class && !cross_class_pair) continue;

            // Small motorcycle and pedestrian boxes can look duplicated while
            // their BEV IoU stays close to zero. Suppress nearby same-class
            // centers before applying rotated-IoU NMS.
            if (same_class && (box_a.id == 6 || box_a.id == 7) &&
                config.pedestrian_motorcycle_center_distance_m > 0.0f) {
                const float radius_sq =
                    config.pedestrian_motorcycle_center_distance_m *
                    config.pedestrian_motorcycle_center_distance_m;
                if (center_distance_squared(box_a, box_b) <= radius_sq) {
                    keep[order[j]] = 1;
                    continue;
                }
            }

            // 特判类别 9 (交通锥等) 使用 Circle NMS 算中心点距离
            // 注意：这里用 dx * dx 代替了极其耗时的 std::pow
            if (same_class && box_a.id == 9) {
                float dx = box_a.x - box_b.x;
                float dy = box_a.y - box_b.y;
                float dist_sq = dx * dx + dy * dy; 
                
                if (dist_sq <= 1.0f) {
                    keep[order[j]] = 1; 
                }
                continue;
            }

            float sa = box_a.w * box_a.l;
            float sb = box_b.w * box_b.l;
            float s_overlap = box_overlap(box_a, box_b);
            float iou = s_overlap / std::fmax(sa + sb - s_overlap, ThresHold);
            float thresh = config.cross_class_iou_threshold;
            if (same_class) {
                if (box_a.id < 0 ||
                    static_cast<size_t>(box_a.id) >= config.nms_thr_list.size()) {
                    continue; // 非法类别直接丢弃
                }
                thresh = config.nms_thr_list[box_a.id];
            }
            if (iou >= thresh) {
                keep[order[j]] = 1;
            }
        }
    }
    return result;
}

} // namespace nms
} // namespace fastbev
