#include "filter.hpp"
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

namespace fastbev {
namespace filter {

// 生成 8 种基础 Anchor (对应网络配置里的 4种尺寸 * 2种角度)
static const std::vector<Anchor> get_anchors() {
    std::vector<Anchor> anchors;
    float sizes[4][3] = {{0.8660, 2.5981, 1.0}, {0.5774, 1.7321, 1.0}, {1.0, 1.0, 1.0}, {0.4, 0.4, 1.0}};
    float rots[2] = {0.0f, 1.57f}; // 0 和 90度
    float base_z = -1.8f;          // 地面基准高度
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 2; ++j) {
            anchors.push_back({sizes[i][0], sizes[i][1], sizes[i][2], base_z, rots[j]});
        }
    }
    return anchors;
}

// 角度约束函数：将任意角度限制在 [-pi, pi] 范围内
static float limit_period(float val, float offset = 0.5f) {
    return val - std::floor(val / M_PI + offset) * M_PI;
}

std::vector<BoundingBox> threshold_and_decode(
    const float* cls_ptr, const float* bbox_ptr, const float* dir_ptr, float score_thr) 
{
    // --- 空间参数 ---
    const int GRID_W = 100, GRID_H = 100;       // BEV 空间的网格维度
    const int SPATIAL_SIZE = GRID_W * GRID_H;   // 总网格数 (10000)
    const int NUM_ANCHORS = 8, NUM_CLASSES = 10;// 8个Anchor，10个类别
    
    // --- 物理尺度 ---
    const float MIN_X = -50.0f, MIN_Y = -50.0f; // 坐标原点起算点
    const float VOXEL_X = 1.0f, VOXEL_Y = 1.0f; // 每个格子代表 1 米

    std::vector<Anchor> base_anchors = get_anchors();
    std::vector<BoundingBox> global_candidates;

    // ====================================================================
    // [性能优化] OpenMP 多核并行计算
    // 在 ARM A53/A72 等多核架构上，分配不同线程处理不同区域的网格
    // ====================================================================
    #pragma omp parallel
    {
        // 每个线程私有的候选框队列，避免多线程 push_back 时的锁竞争
        std::vector<BoundingBox> local_candidates;
        local_candidates.reserve(100);

        // 动态分配 i (0 ~ 9999) 给各个线程
        #pragma omp for nowait
        for (int i = 0; i < SPATIAL_SIZE; ++i) {
            // 算出当前网格的二维坐标
            int grid_x = i % GRID_W;
            int grid_y = i / GRID_W;
            
            // 算出当前网格在真实世界中的中心点坐标 (米)
            float grid_center_x = MIN_X + grid_x * VOXEL_X + (VOXEL_X / 2.0f);
            float grid_center_y = MIN_Y + grid_y * VOXEL_Y + (VOXEL_Y / 2.0f);

            // 遍历每个格子预设的 8 个 Anchor
            for (int a = 0; a < NUM_ANCHORS; ++a) {
                // 遍历 10 个类别
                for (int c = 0; c < NUM_CLASSES; ++c) {
                    
                    // ========================================================
                    // [NHWC 寻址核心逻辑]
                    // 假设排布为 [H*W, Anchor, Class]
                    // NHWC 的好处是：同一个空间点 (i) 的 80 个类别数据在物理内存上
                    // 是连续的！ARM CPU 读取时会触发 Cache 预取，极大幅度提升访存速度。
                    // ========================================================
                    float score = cls_ptr[i * (NUM_ANCHORS * NUM_CLASSES) + a * NUM_CLASSES + c];
                    
                    // 【关键优化】：阈值拦截。绝大多数点分数为 0，直接跳过后续极其耗时的解码
                    if (score > score_thr) {
                        Anchor& anchor = base_anchors[a];
                        
                        // 定位 9 个 BBox 参数在 NHWC 内存中的起点
                        int bbox_offset = i * (NUM_ANCHORS * 9) + a * 9;
                        
                        // 1. 解码 X, Y (依据对角线长度归一化)
                        float diagonal = std::sqrt(anchor.l * anchor.l + anchor.w * anchor.w);
                        float xg = bbox_ptr[bbox_offset + 0] * diagonal + grid_center_x;
                        float yg = bbox_ptr[bbox_offset + 1] * diagonal + grid_center_y;
                        
                        // 2. 解码 Z (依据高度归一化，从底部中心转换)
                        float zg = bbox_ptr[bbox_offset + 2] * anchor.h + (anchor.z + anchor.h / 2.0f);

                        // 3. 解码宽高长 (网络输出是 log，需进行 exp 还原)
                        // 注意：std::exp 在嵌入式 ARM 上是重度耗时指令，我们仅对幸存框计算！
                        float wg = std::exp(bbox_ptr[bbox_offset + 3]) * anchor.w;
                        float lg = std::exp(bbox_ptr[bbox_offset + 4]) * anchor.l;
                        float hg = std::exp(bbox_ptr[bbox_offset + 5]) * anchor.h;
                        
                        // 4. 解码偏航角
                        float rg = bbox_ptr[bbox_offset + 6] + anchor.rot;
                        zg = zg - hg / 2.0f; // 把 Z 轴重新降回物体底部

                        // 5. 方向类别纠偏 (应对网络对 180 度对称方向预测模糊的问题)
                        int dir_offset = i * (NUM_ANCHORS * 2) + a * 2;
                        // 取得分更高的方向类别作为最终判断
                        int dir_label = (dir_ptr[dir_offset + 1] > dir_ptr[dir_offset + 0]) ? 1 : 0;
                        
                        // 如果网络认为方向反了 (dir_label=1)，就给偏航角加上 pi (180度)
                        rg = limit_period(rg) + dir_label * M_PI;

                        // 组装解码后的真实框并放入线程私有队列
                        local_candidates.push_back({
                            xg, yg, zg, wg, lg, hg, rg, 
                            bbox_ptr[bbox_offset + 7], // 速度 vx
                            bbox_ptr[bbox_offset + 8], // 速度 vy
                            score, c                   //10个类别中的一个
                        });
                    }
                }
            }
        }
        
        // ====================================================================
        // [OpenMP 临界区]：所有线程安全地把自己的局部队列合并到全局队列
        // ====================================================================
        #pragma omp critical
        {
            global_candidates.insert(global_candidates.end(), local_candidates.begin(), local_candidates.end());
        }
    }
    return global_candidates;
}

} // namespace filter
} // namespace fastbev