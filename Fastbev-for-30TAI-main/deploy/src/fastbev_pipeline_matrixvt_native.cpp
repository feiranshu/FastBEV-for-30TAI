// MatrixVT NuScenes native serial pipeline.
//
// Data path:
//   6 camera JPEGs -> FP32 NCHW [6,3,256,704]
//   -> single MatrixVT NPU forward -> [1,70,128,128]
//   -> MatrixVT CenterPoint decode -> native MatrixVT result.txt / PNG.
//
// This program is intentionally independent from FastBEV Part1/Part2/Part3:
// no LUT table, no PL registers, no Part2 bridge and no alert side effects.
// It also avoids the shared visualize executable so MatrixVT-native rendering
// cannot affect FastBEV/NuScenes/CARLA visualizers.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>

#include "fastbev_export.hpp"
#include "fastbev_reader.h"
#include "types.hpp"

#include <icraft-xrt/core/session.h>
#include <icraft-xrt/dev/host_device.h>
#include <icraft-xrt/dev/buyi_device.h>
#include <icraft-xrt/dev/zg330_device.h>
#include <icraft-backends/buyibackend/buyibackend.h>
#include <icraft-backends/zg330backend/zg330backend.h>
#include <icraft-backends/hostbackend/cuda/device.h>
#include <icraft-backends/hostbackend/backend.h>
#include <icraft-backends/hostbackend/utils.h>
#include <icraft-xir/core/network.h>
#include <icraft-xir/core/data.h>

#include <compile_fpai_target.hpp>
#include <et_device.hpp>
#include <icraft_utils.hpp>

using namespace icraft::xir;
using namespace icraft::xrt;
namespace fs = std::filesystem;

namespace {

using Clock = std::chrono::high_resolution_clock;
using fastbev::BoundingBox;

constexpr int kNumCameras = 6;
constexpr int kInputC = 3;
constexpr int kInputH = 256;
constexpr int kInputW = 704;
constexpr int kResizeW = 704;
constexpr int kResizeH = 396;
constexpr int kOutputC = 70;
constexpr int kOutputH = 128;
constexpr int kOutputW = 128;
constexpr int kFeatureStride = 4;
constexpr float kVoxelSizeM = 0.2f;
constexpr float kPcRangeMinX = -51.2f;
constexpr float kPcRangeMinY = -51.2f;
constexpr float kScoreEps = 1e-6f;
constexpr float kPi = 3.14159265358979323846f;

const std::array<int, kNumCameras> kMatrixVTCameraOrder = {
    CAM_FRONT_LEFT,
    CAM_FRONT,
    CAM_FRONT_RIGHT,
    CAM_BACK_LEFT,
    CAM_BACK,
    CAM_BACK_RIGHT,
};

const std::array<const char*, kNumCameras> kFastbevCameraNames = {
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
};

struct PipelineConfig {
    std::string device_url;
    bool mmu_mode = true;
    bool speed_mode = false;
    bool compress_ftmp = false;
    int ocm_option = -1;

    std::string model_dir;
    std::string model_stage = "g";
    std::string model_backend = "zg330";

    std::string dataset_json;
    std::string box_dir;
    std::string png_dir;
    std::string para_dir;
    std::string raw_dir = "./io/output/matrixvt_raw";
    std::string log_dir;
    std::vector<std::string> eval_scenes;

    int source_width = 1600;
    int source_height = 900;
    int crop_x = 0;
    int crop_y = 140;
    int crop_width = 704;
    int crop_height = 256;
    std::array<float, 3> mean = {123.675f, 116.28f, 103.53f};
    std::array<float, 3> std = {58.395f, 57.12f, 57.375f};

    float score_threshold = 0.10f;
    int max_num = 500;
    int post_max_size = 83;
    int max_boxes = 100;
    std::array<float, 6> post_center_range = {-61.2f, -61.2f, -10.0f, 61.2f, 61.2f, 10.0f};
    std::array<float, 6> circle_thresholds = {4.0f, 12.0f, 10.0f, 1.0f, 0.85f, 0.175f};
    bool visualize = true;
    bool raw_only = false;
};

struct TaskSpec {
    int reg = 0;
    int height = 0;
    int dim = 0;
    int rot = 0;
    int vel = 0;
    int heatmap = 0;
    int num_classes = 0;
    std::array<int, 2> class_ids = {-1, -1};
    const char* name = "";
};

const std::array<TaskSpec, 6> kTasks = {{
    {0, 2, 3, 6, 8, 10, 1, {0, -1}, "car"},
    {11, 13, 14, 17, 19, 21, 2, {1, 4}, "truck/construction_vehicle"},
    {23, 25, 26, 29, 31, 33, 2, {3, 2}, "bus/trailer"},
    {35, 37, 38, 41, 43, 45, 1, {9, -1}, "barrier"},
    {46, 48, 49, 52, 54, 56, 2, {6, 5}, "motorcycle/bicycle"},
    {58, 60, 61, 64, 66, 68, 2, {7, 8}, "pedestrian/traffic_cone"},
}};

struct HeatCandidate {
    float score = 0.0f;
    int local_class = 0;
    int x = 0;
    int y = 0;
};

static double elapsed_ms(Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

static float sigmoid(float x) {
    if (x >= 0.0f) {
        const float z = std::exp(-x);
        return 1.0f / (1.0f + z);
    }
    const float z = std::exp(x);
    return z / (1.0f + z);
}

static float normalize_yaw(float yaw) {
    while (yaw > kPi) yaw -= 2.0f * kPi;
    while (yaw <= -kPi) yaw += 2.0f * kPi;
    return yaw;
}

// Icraft returns the MatrixVT FD output as channel-last memory even though the
// logical contract is [1,70,128,128]. Decode from the observed runtime layout.
static size_t matrixvt_output_index(int c, int y, int x) {
    return (static_cast<size_t>(y) * kOutputW + x) * kOutputC + c;
}

static std::array<float, 3> read_array3(const YAML::Node& node, const char* name,
                                        std::array<float, 3> fallback) {
    if (!node || !node[name]) return fallback;
    const auto values = node[name].as<std::vector<float>>();
    if (values.size() != 3) throw std::runtime_error(std::string(name) + " must have 3 values");
    return {values[0], values[1], values[2]};
}

static std::array<float, 6> read_array6(const YAML::Node& node, const char* name,
                                        std::array<float, 6> fallback) {
    if (!node || !node[name]) return fallback;
    const auto values = node[name].as<std::vector<float>>();
    if (values.size() != 6) throw std::runtime_error(std::string(name) + " must have 6 values");
    return {values[0], values[1], values[2], values[3], values[4], values[5]};
}

static std::vector<std::string> read_string_vector(const YAML::Node& node, const char* name) {
    std::vector<std::string> out;
    if (!node || !node[name]) return out;
    const auto values = node[name].as<std::vector<std::string>>();
    out.reserve(values.size());
    for (const auto& value : values) {
        if (!value.empty()) out.push_back(value);
    }
    return out;
}

static PipelineConfig load_config(const std::string& path) {
    PipelineConfig cfg;
    const YAML::Node root = YAML::LoadFile(path);

    const auto device = root["device"];
    if (!device || !device["url"]) throw std::runtime_error("missing device.url");
    cfg.device_url = device["url"].as<std::string>();
    cfg.mmu_mode = device["mmuMode"].as<bool>(cfg.mmu_mode);
    cfg.speed_mode = device["speedMode"].as<bool>(cfg.speed_mode);
    cfg.compress_ftmp = device["compressFtmp"].as<bool>(cfg.compress_ftmp);
    cfg.ocm_option = device["ocm_option"].as<int>(cfg.ocm_option);

    const auto model = root["matrixvt"];
    if (!model || !model["dir"]) throw std::runtime_error("missing matrixvt.dir");
    cfg.model_dir = model["dir"].as<std::string>();
    cfg.model_stage = model["stage"].as<std::string>(cfg.model_stage);
    cfg.model_backend = model["run_backend"].as<std::string>(cfg.model_backend);

    const auto dataset = root["dataset"];
    if (!dataset || !dataset["imageDir"]) throw std::runtime_error("missing dataset.imageDir");
    cfg.dataset_json = dataset["imageDir"].as<std::string>();
    cfg.box_dir = dataset["boxDir"].as<std::string>("./io/output/result");
    cfg.png_dir = dataset["pngDir"].as<std::string>("./io/output/png");
    cfg.para_dir = dataset["paraDir"].as<std::string>("./io/output/parameter");
    cfg.raw_dir = dataset["rawOutputDir"].as<std::string>(cfg.raw_dir);
    cfg.log_dir = dataset["logDir"].as<std::string>("./io/log");
    cfg.eval_scenes = read_string_vector(dataset, "evalScenes");

    const auto pre = root["preprocess"];
    if (pre) {
        cfg.source_width = pre["source_width"].as<int>(cfg.source_width);
        cfg.source_height = pre["source_height"].as<int>(cfg.source_height);
        cfg.crop_x = pre["crop_x"].as<int>(cfg.crop_x);
        cfg.crop_y = pre["crop_y"].as<int>(cfg.crop_y);
        cfg.crop_width = pre["crop_width"].as<int>(cfg.crop_width);
        cfg.crop_height = pre["crop_height"].as<int>(cfg.crop_height);
        cfg.mean = read_array3(pre, "mean", cfg.mean);
        cfg.std = read_array3(pre, "std", cfg.std);
    }
    if (cfg.crop_width != kInputW || cfg.crop_height != kInputH) {
        throw std::runtime_error("MatrixVT input requires crop_width=704 and crop_height=256");
    }

    const auto post = root["postprocess"];
    if (post) {
        cfg.score_threshold = post["score_threshold"].as<float>(cfg.score_threshold);
        cfg.max_num = post["max_num"].as<int>(cfg.max_num);
        cfg.post_max_size = post["post_max_size"].as<int>(cfg.post_max_size);
        cfg.max_boxes = post["max_boxes"].as<int>(cfg.max_boxes);
        cfg.post_center_range = read_array6(post, "post_center_range", cfg.post_center_range);
        cfg.circle_thresholds = read_array6(post, "circle_thresholds", cfg.circle_thresholds);
    }
    const auto vis = root["visualize"];
    if (vis) cfg.visualize = vis["enabled"].as<bool>(cfg.visualize);
    const auto output = root["output"];
    if (output) cfg.raw_only = output["rawOnly"].as<bool>(cfg.raw_only);
    return cfg;
}

static bool should_run_sample(const FastBEVSample* sample, const PipelineConfig& cfg) {
    if (cfg.eval_scenes.empty()) return true;
    if (!sample) return false;
    for (const auto& scene : cfg.eval_scenes) {
        if (std::strcmp(sample->scene_name, scene.c_str()) == 0) return true;
    }
    return false;
}

static bool has_shape(const TensorType& type, std::initializer_list<int64_t> expected) {
    if (type->shape.size() != expected.size()) return false;
    size_t i = 0;
    for (int64_t value : expected) {
        if (type->shape[i++] != value) return false;
    }
    return true;
}

static std::vector<float> tensor_to_vector(const Tensor& tensor) {
    if (!tensor.hasData()) throw std::runtime_error("MatrixVT output tensor has no data");
    if (!tensor.dtype()->element_dtype.isFP32()) {
        throw std::runtime_error("MatrixVT output tensor must be FP32");
    }
    const uint64_t elements = tensor.dtype().numElements();
    std::vector<float> out(elements);
    tensor.read(reinterpret_cast<char*>(out.data()), 0, elements * sizeof(float));
    return out;
}

static void preprocess_sample(const FastBEVSample* sample,
                              const PipelineConfig& cfg,
                              std::vector<float>& input) {
    input.assign(static_cast<size_t>(kNumCameras) * kInputC * kInputH * kInputW, 0.0f);

    for (int out_cam = 0; out_cam < kNumCameras; ++out_cam) {
        const int src_cam = kMatrixVTCameraOrder[out_cam];
        const FastBEVCamera* cam = &sample->cameras[src_cam];
        if (!cam->image_path || !cam->image_path[0]) {
            throw std::runtime_error("missing image path for MatrixVT camera input");
        }

        cv::Mat bgr = cv::imread(cam->image_path, cv::IMREAD_COLOR);
        if (bgr.empty()) {
            throw std::runtime_error(std::string("cannot read image: ") + cam->image_path);
        }
        if (bgr.cols != cfg.source_width || bgr.rows != cfg.source_height) {
            throw std::runtime_error(std::string("unexpected image size for ") + cam->image_path +
                                     ": got " + std::to_string(bgr.cols) + "x" +
                                     std::to_string(bgr.rows));
        }

        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(kResizeW, kResizeH), 0.0, 0.0, cv::INTER_CUBIC);

        const cv::Rect roi(cfg.crop_x, cfg.crop_y, cfg.crop_width, cfg.crop_height);
        if ((roi & cv::Rect(0, 0, resized.cols, resized.rows)) != roi) {
            throw std::runtime_error("MatrixVT crop rectangle is outside resized image");
        }
        const cv::Mat crop = resized(roi);

        float* dst_cam = input.data() + static_cast<size_t>(out_cam) * kInputC * kInputH * kInputW;
        for (int y = 0; y < kInputH; ++y) {
            const cv::Vec3b* row = crop.ptr<cv::Vec3b>(y);
            for (int x = 0; x < kInputW; ++x) {
                for (int c = 0; c < kInputC; ++c) {
                    const size_t off = static_cast<size_t>(c) * kInputH * kInputW +
                                       static_cast<size_t>(y) * kInputW + x;
                    dst_cam[off] = (static_cast<float>(row[x][c]) - cfg.mean[c]) / cfg.std[c];
                }
            }
        }
    }
}

static std::vector<HeatCandidate> topk_task_candidates(const std::vector<float>& pred,
                                                       const TaskSpec& task,
                                                       int max_num) {
    std::vector<HeatCandidate> all;
    all.reserve(static_cast<size_t>(max_num) * task.num_classes);

    for (int local_cls = 0; local_cls < task.num_classes; ++local_cls) {
        std::vector<int> indices(kOutputH * kOutputW);
        std::iota(indices.begin(), indices.end(), 0);
        const int heat_c = task.heatmap + local_cls;
        const int keep = std::min(max_num, static_cast<int>(indices.size()));

        auto score_at = [&](int idx) {
            const int y = idx / kOutputW;
            const int x = idx % kOutputW;
            return sigmoid(pred[matrixvt_output_index(heat_c, y, x)]);
        };
        std::partial_sort(indices.begin(), indices.begin() + keep, indices.end(),
                          [&](int a, int b) {
                              const float sa = score_at(a);
                              const float sb = score_at(b);
                              if (std::fabs(sa - sb) > kScoreEps) return sa > sb;
                              return a < b;
                          });

        for (int i = 0; i < keep; ++i) {
            const int idx = indices[i];
            all.push_back({score_at(idx), local_cls, idx % kOutputW, idx / kOutputW});
        }
    }

    const int keep = std::min(max_num, static_cast<int>(all.size()));
    std::partial_sort(all.begin(), all.begin() + keep, all.end(),
                      [](const HeatCandidate& a, const HeatCandidate& b) {
                          if (std::fabs(a.score - b.score) > kScoreEps) return a.score > b.score;
                          if (a.local_class != b.local_class) return a.local_class < b.local_class;
                          if (a.y != b.y) return a.y < b.y;
                          return a.x < b.x;
                      });
    all.resize(keep);
    return all;
}

static bool inside_post_range(const BoundingBox& box, const std::array<float, 6>& range) {
    return box.x >= range[0] && box.y >= range[1] && box.z >= range[2] &&
           box.x <= range[3] && box.y <= range[4] && box.z <= range[5];
}

static std::vector<BoundingBox> circle_nms_task(std::vector<BoundingBox> boxes,
                                                float min_dist_sq,
                                                int post_max_size) {
    std::sort(boxes.begin(), boxes.end(), [](const BoundingBox& a, const BoundingBox& b) {
        return a.score > b.score;
    });

    std::vector<BoundingBox> kept;
    kept.reserve(std::min(post_max_size, static_cast<int>(boxes.size())));
    for (const auto& box : boxes) {
        bool suppressed = false;
        for (const auto& prev : kept) {
            const float dx = box.x - prev.x;
            const float dy = box.y - prev.y;
            if (dx * dx + dy * dy <= min_dist_sq) {
                suppressed = true;
                break;
            }
        }
        if (!suppressed) {
            kept.push_back(box);
            if (static_cast<int>(kept.size()) >= post_max_size) break;
        }
    }
    return kept;
}

static std::vector<BoundingBox> decode_matrixvt(const std::vector<float>& pred,
                                                const PipelineConfig& cfg) {
    if (pred.size() != static_cast<size_t>(kOutputC) * kOutputH * kOutputW) {
        throw std::runtime_error("MatrixVT output element count mismatch");
    }

    std::vector<BoundingBox> final_boxes;
    for (size_t task_idx = 0; task_idx < kTasks.size(); ++task_idx) {
        const TaskSpec& task = kTasks[task_idx];
        std::vector<BoundingBox> task_boxes;
        auto heat_candidates = topk_task_candidates(pred, task, cfg.max_num);
        task_boxes.reserve(heat_candidates.size());

        for (const HeatCandidate& candidate : heat_candidates) {
            if (candidate.score < cfg.score_threshold) continue;
            const int y = candidate.y;
            const int x = candidate.x;

            BoundingBox box;
            const float reg_x = pred[matrixvt_output_index(task.reg + 0, y, x)];
            const float reg_y = pred[matrixvt_output_index(task.reg + 1, y, x)];
            const float raw_x =
                (static_cast<float>(x) + reg_x) * kFeatureStride * kVoxelSizeM + kPcRangeMinX;
            const float raw_y =
                (static_cast<float>(y) + reg_y) * kFeatureStride * kVoxelSizeM + kPcRangeMinY;
            box.x = raw_x;
            box.y = raw_y;
            box.z = pred[matrixvt_output_index(task.height, y, x)];
            box.w = std::exp(pred[matrixvt_output_index(task.dim + 0, y, x)]);
            box.l = std::exp(pred[matrixvt_output_index(task.dim + 1, y, x)]);
            box.h = std::exp(pred[matrixvt_output_index(task.dim + 2, y, x)]);
            const float rot_sin = pred[matrixvt_output_index(task.rot + 0, y, x)];
            const float rot_cos = pred[matrixvt_output_index(task.rot + 1, y, x)];
            box.yaw = normalize_yaw(std::atan2(rot_sin, rot_cos));
            box.vx = pred[matrixvt_output_index(task.vel + 0, y, x)];
            box.vy = pred[matrixvt_output_index(task.vel + 1, y, x)];
            box.id = task.class_ids[candidate.local_class];
            box.score = candidate.score;

            if (box.id < 0 || !inside_post_range(box, cfg.post_center_range)) continue;
            task_boxes.push_back(box);
        }

        auto kept = circle_nms_task(std::move(task_boxes),
                                    cfg.circle_thresholds[task_idx],
                                    cfg.post_max_size);
        final_boxes.insert(final_boxes.end(), kept.begin(), kept.end());
    }
    std::sort(final_boxes.begin(), final_boxes.end(), [](const BoundingBox& a, const BoundingBox& b) {
        return a.score > b.score;
    });
    if (cfg.max_boxes >= 0 && static_cast<int>(final_boxes.size()) > cfg.max_boxes) {
        final_boxes.resize(static_cast<size_t>(cfg.max_boxes));
    }
    return final_boxes;
}

static const int kCornerSigns[8][3] = {
    { 1,  1, -1}, { 1, -1, -1}, {-1, -1, -1}, {-1,  1, -1},
    { 1,  1,  1}, { 1, -1,  1}, {-1, -1,  1}, {-1,  1,  1},
};

static const int kEdges3d[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
};

static const int kEdgesBev[4][2] = {{0,1},{1,2},{2,3},{3,0}};

static const cv::Scalar kMatrixVTBoxYellow(0, 255, 255);

static void compute_corners(const BoundingBox& d, cv::Vec3f corners[8]) {
    const float cy = std::cos(d.yaw);
    const float sy = std::sin(d.yaw);
    // MatrixVT output z is treated as box center height for this renderer.
    // Compared with bottom-height interpretation, this shifts every box down by h/2.
    const float center_z = d.z;
    for (int i = 0; i < 8; ++i) {
        const float lx = kCornerSigns[i][0] * d.w * 0.5f;
        const float ly = kCornerSigns[i][1] * d.l * 0.5f;
        const float rx = lx * cy - ly * sy;
        const float ry = lx * sy + ly * cy;
        corners[i] = cv::Vec3f(d.x + rx, d.y + ry,
                               center_z + kCornerSigns[i][2] * d.h * 0.5f);
    }
}

static cv::Matx33f quat_to_rotmat(const float q_in[4]) {
    const float norm = std::sqrt(q_in[0] * q_in[0] + q_in[1] * q_in[1] +
                                 q_in[2] * q_in[2] + q_in[3] * q_in[3]);
    const float inv = norm > 1e-12f ? 1.0f / norm : 1.0f;
    const float w = q_in[0] * inv;
    const float x = q_in[1] * inv;
    const float y = q_in[2] * inv;
    const float z = q_in[3] * inv;

    return cv::Matx33f(
        1.0f - 2.0f * y * y - 2.0f * z * z, 2.0f * x * y - 2.0f * z * w,     2.0f * x * z + 2.0f * y * w,
        2.0f * x * y + 2.0f * z * w,     1.0f - 2.0f * x * x - 2.0f * z * z, 2.0f * y * z - 2.0f * x * w,
        2.0f * x * z - 2.0f * y * w,     2.0f * y * z + 2.0f * x * w,     1.0f - 2.0f * x * x - 2.0f * y * y);
}

static bool project_point_cdeploy(const cv::Vec3f& p,
                                  const FastBEVCamera& cam,
                                  const cv::Matx33f& sensor_to_ego_r,
                                  cv::Point2f& out) {
    const float ex = p[0] - cam.extrinsic_t[0];
    const float ey = p[1] - cam.extrinsic_t[1];
    const float ez = p[2] - cam.extrinsic_t[2];

    // c_deploy: camera = R^T * (ego - t), where R/t are camera-to-ego.
    const float cx = ex * sensor_to_ego_r(0, 0) + ey * sensor_to_ego_r(1, 0) + ez * sensor_to_ego_r(2, 0);
    const float cy = ex * sensor_to_ego_r(0, 1) + ey * sensor_to_ego_r(1, 1) + ez * sensor_to_ego_r(2, 1);
    const float cz = ex * sensor_to_ego_r(0, 2) + ey * sensor_to_ego_r(1, 2) + ez * sensor_to_ego_r(2, 2);
    if (!(cz > 0.1f)) return false;

    const float* k = cam.intrinsic;
    const float px = k[0] * cx + k[1] * cy + k[2] * cz;
    const float py = k[3] * cx + k[4] * cy + k[5] * cz;
    const float pw = k[6] * cx + k[7] * cy + k[8] * cz;
    if (!(pw > 1.0e-6f)) return false;
    out.x = px / pw;
    out.y = py / pw;
    return std::isfinite(out.x) && std::isfinite(out.y);
}

static cv::Point2f bev_point_cdeploy(int width, int height, float range, float x, float y) {
    const float scale = static_cast<float>(std::min(width, height)) / (2.0f * range);
    return cv::Point2f(width * 0.5f - y * scale,
                       height * 0.5f - x * scale);
}

static void copy_fit(const cv::Mat& src, cv::Mat& dst, const cv::Rect& roi) {
    if (src.empty() || roi.width <= 0 || roi.height <= 0) return;
    const double src_aspect = static_cast<double>(src.cols) / std::max(1, src.rows);
    const double dst_aspect = static_cast<double>(roi.width) / std::max(1, roi.height);
    int tw = roi.width;
    int th = roi.height;
    if (dst_aspect > src_aspect) {
        th = roi.height;
        tw = std::max(1, static_cast<int>(std::lround(th * src_aspect)));
    } else {
        tw = roi.width;
        th = std::max(1, static_cast<int>(std::lround(tw / src_aspect)));
    }
    cv::Mat resized;
    cv::resize(src, resized, cv::Size(tw, th), 0.0, 0.0, cv::INTER_LINEAR);
    const int x = roi.x + (roi.width - tw) / 2;
    const int y = roi.y + (roi.height - th) / 2;
    resized.copyTo(dst(cv::Rect(x, y, tw, th)));
}

static int render_matrixvt_native_png(const FastBEVSample* sample,
                                      const std::vector<BoundingBox>& boxes,
                                      const std::string& png_path) {
    if (!sample) return -1;

    constexpr int kImgW = 1600;
    constexpr int kImgH = 900;
    constexpr int kDashW = 1600;
    constexpr int kDashH = 900;
    constexpr float kShowRange = 55.0f;

    std::vector<std::array<cv::Vec3f, 8>> all_corners;
    all_corners.reserve(boxes.size());
    for (const auto& box : boxes) {
        std::array<cv::Vec3f, 8> corners{};
        compute_corners(box, corners.data());
        all_corners.push_back(corners);
    }

    std::vector<cv::Mat> rendered;
    rendered.reserve(kNumCameras);
    for (int view_idx = 0; view_idx < kNumCameras; ++view_idx) {
        const int cam_id = kMatrixVTCameraOrder[view_idx];
        const FastBEVCamera& cam = sample->cameras[cam_id];
        cv::Mat img = cv::imread(cam.image_path ? cam.image_path : "", cv::IMREAD_COLOR);
        if (img.empty()) {
            std::fprintf(stderr, "[Warn] cannot read image: %s\n",
                         cam.image_path ? cam.image_path : "(null)");
            img = cv::Mat::zeros(kImgH, kImgW, CV_8UC3);
        }

        const cv::Matx33f sensor_to_ego_r = quat_to_rotmat(cam.extrinsic_r);
        for (size_t bi = 0; bi < all_corners.size(); ++bi) {
            const auto& corners = all_corners[bi];
            cv::Point2f pix[8];
            bool valid[8];
            for (int k = 0; k < 8; ++k) valid[k] = project_point_cdeploy(corners[k], cam, sensor_to_ego_r, pix[k]);
            for (const auto& edge : kEdges3d) {
                const int a = edge[0], b = edge[1];
                if (valid[a] && valid[b]) {
                    cv::line(img,
                             cv::Point(static_cast<int>(pix[a].x), static_cast<int>(pix[a].y)),
                             cv::Point(static_cast<int>(pix[b].x), static_cast<int>(pix[b].y)),
                             kMatrixVTBoxYellow, 3);
                }
            }
            if (valid[0] && valid[1]) {
                cv::line(img,
                         cv::Point(static_cast<int>(pix[0].x), static_cast<int>(pix[0].y)),
                         cv::Point(static_cast<int>(pix[1].x), static_cast<int>(pix[1].y)),
                         kMatrixVTBoxYellow, 3);
            }
        }
        rendered.push_back(std::move(img));
    }

    cv::Mat bev(kDashH, kDashH, CV_8UC3, cv::Scalar(18, 22, 28));
    const int bev_w = bev.cols;
    const int bev_h = bev.rows;
    const cv::Scalar minor(56, 48, 42);
    const cv::Scalar axis(120, 110, 100);
    for (float grid = -kShowRange; grid <= kShowRange + 1e-3f; grid += 10.0f) {
        cv::Point2f a = bev_point_cdeploy(bev_w, bev_h, kShowRange, -kShowRange, grid);
        cv::Point2f b = bev_point_cdeploy(bev_w, bev_h, kShowRange,  kShowRange, grid);
        cv::line(bev, a, b, std::fabs(grid) < 1e-3f ? axis : minor, 1);
        a = bev_point_cdeploy(bev_w, bev_h, kShowRange, grid, -kShowRange);
        b = bev_point_cdeploy(bev_w, bev_h, kShowRange, grid,  kShowRange);
        cv::line(bev, a, b, std::fabs(grid) < 1e-3f ? axis : minor, 1);
    }

    auto draw_ego_edge = [&](float x0, float y0, float x1, float y1) {
        cv::line(bev,
                 bev_point_cdeploy(bev_w, bev_h, kShowRange, x0, y0),
                 bev_point_cdeploy(bev_w, bev_h, kShowRange, x1, y1),
                 cv::Scalar(245, 245, 245), 2);
    };
    draw_ego_edge(-2.3f, -1.0f,  2.3f, -1.0f);
    draw_ego_edge( 2.3f, -1.0f,  2.3f,  1.0f);
    draw_ego_edge( 2.3f,  1.0f, -2.3f,  1.0f);
    draw_ego_edge(-2.3f,  1.0f, -2.3f, -1.0f);

    for (size_t bi = 0; bi < all_corners.size(); ++bi) {
        const auto& corners = all_corners[bi];
        cv::Point bot[4];
        for (int k = 0; k < 4; ++k) {
            cv::Point2f p = bev_point_cdeploy(bev_w, bev_h, kShowRange, corners[k][0], corners[k][1]);
            bot[k] = cv::Point(static_cast<int>(p.x), static_cast<int>(p.y));
        }
        for (const auto& edge : kEdgesBev) cv::line(bev, bot[edge[0]], bot[edge[1]], kMatrixVTBoxYellow, 3);
        cv::line(bev, bot[0], bot[1], kMatrixVTBoxYellow, 3);

        cv::Point2f center = bev_point_cdeploy(bev_w, bev_h, kShowRange, boxes[bi].x, boxes[bi].y);
        cv::Point2f vel = bev_point_cdeploy(bev_w, bev_h, kShowRange,
                                            boxes[bi].x + boxes[bi].vx,
                                            boxes[bi].y + boxes[bi].vy);
        cv::line(bev, center, vel, kMatrixVTBoxYellow, 2);
    }

    cv::Mat out(kDashH, kDashW, CV_8UC3, cv::Scalar(0, 0, 0));
    const int camera_area_width = kDashW * 3 / 4;
    const int cell_w = camera_area_width / 3;
    const int cell_h = kDashH / 2;
    for (int k = 0; k < kNumCameras; ++k) {
        copy_fit(rendered[k], out, cv::Rect((k % 3) * cell_w, (k / 3) * cell_h, cell_w, cell_h));
    }
    copy_fit(bev, out, cv::Rect(camera_area_width, 0, kDashW - camera_area_width, kDashH));
    if (!cv::imwrite(png_path, out)) return -1;
    std::printf("Saved %s  (%dx%d)\n", png_path.c_str(), out.cols, out.rows);
    return 0;
}

static void write_raw_predictions_txt(const std::string& path,
                                      const FastBEVSample* sample,
                                      int original_frame_id,
                                      int eval_frame_id,
                                      const std::vector<float>& predictions) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write MatrixVT raw output: " + path);

    out << "# matrixvt raw predictions\n";
    out << "# logical_shape [1,70,128,128]\n";
    out << "# runtime_flat_layout NHWC: index=(y*128+x)*70+c\n";
    out << "# original_frame_id " << original_frame_id << "\n";
    out << "# eval_frame_id " << eval_frame_id << "\n";
    out << "# sample_token " << (sample ? sample->sample_token : "") << "\n";
    out << "# scene_name " << (sample ? sample->scene_name : "") << "\n";
    out << std::setprecision(9);
    for (int y = 0; y < kOutputH; ++y) {
        for (int x = 0; x < kOutputW; ++x) {
            for (int c = 0; c < kOutputC; ++c) {
                if (c) out << ' ';
                out << predictions[matrixvt_output_index(c, y, x)];
            }
            out << '\n';
        }
    }
}

static void write_result_txt(const std::string& path, const std::vector<BoundingBox>& boxes) {
    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("cannot write result file: " + path);
    ofs << std::fixed << std::setprecision(6);
    for (const auto& b : boxes) {
        ofs << b.x << ' ' << b.y << ' ' << b.z << ' '
            << b.w << ' ' << b.l << ' ' << b.h << ' '
            << b.yaw << ' ' << b.vx << ' ' << b.vy << ' '
            << b.id << ' ' << b.score << '\n';
    }
}

static int export_matrixvt_camera_params(const FastBEVSample* sample, const char* out_path) {
    if (!sample || !out_path) return -1;
    std::ofstream out(out_path);
    if (!out) return -1;

    const cv::Matx33d post_rot(
        0.44, 0.0, 0.0,
        0.0, 0.44, 0.0,
        0.0, 0.0, 1.0);
    const std::array<double, 3> post_tran = {0.0, -70.0, 0.0};

    out << kNumCameras << "\n";
    for (int cam_idx = 0; cam_idx < kNumCameras; ++cam_idx) {
        const FastBEVCamera* cam = &sample->cameras[cam_idx];
        out << kFastbevCameraNames[cam_idx] << "\n";
        out << (cam->image_path ? cam->image_path : "") << "\n";

        cv::Matx44d E;
        if (cam->has_lidar2img) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) E(r, c) = cam->lidar2img[r * 4 + c];
            }
        } else {
            float fallback_e[16];
            _compute_ego2img_E_fallback(
                cam->intrinsic, cam->extrinsic_r, cam->extrinsic_t, fallback_e);
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) E(r, c) = fallback_e[r * 4 + c];
            }
            const cv::Matx33d dataset_aug(
                0.44, 0.0, 0.0,
                0.0, 0.44, -70.0,
                0.0, 0.0, 1.0);
            const cv::Matx44d raw_e = E;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 4; ++c) {
                    E(r, c) = dataset_aug(r, 0) * raw_e(0, c) +
                              dataset_aug(r, 1) * raw_e(1, c) +
                              dataset_aug(r, 2) * raw_e(2, c);
                }
            }
        }

        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (r || c) out << ' ';
                out << E(r, c);
            }
        }
        out << "\n";

        for (int i = 0; i < 9; ++i) {
            if (i) out << ' ';
            out << FASTBEV_IDENTITY_3x3[i];
        }
        out << "\n";

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                if (r || c) out << ' ';
                out << post_rot(r, c);
            }
        }
        out << "\n";
        out << post_tran[0] << ' ' << post_tran[1] << ' ' << post_tran[2] << "\n";
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::fprintf(stderr, "Usage: %s <config/matrixvt_nuscenes_native.yaml>\n", argv[0]);
            return 1;
        }

        PipelineConfig cfg = load_config(argv[1]);
        fs::create_directories(cfg.box_dir);
        fs::create_directories(cfg.png_dir);
        fs::create_directories(cfg.para_dir);
        fs::create_directories(cfg.raw_dir);
        fs::create_directories(cfg.log_dir);

        std::printf("============================================================\n");
        std::printf(" MatrixVT NuScenes Native Serial Pipeline\n");
        std::printf("============================================================\n");
        std::printf("[Config] model=%s stage=%s backend=%s\n",
                    cfg.model_dir.c_str(), cfg.model_stage.c_str(), cfg.model_backend.c_str());
        std::printf("[Config] dataset=%s\n", cfg.dataset_json.c_str());
        std::printf("[Config] preprocess=NCHW FP32 BGR-normalized crop_y=%d\n", cfg.crop_y);
        std::printf("[Config] score_threshold=%.3f max_boxes=%d visualize=%s\n",
                    cfg.score_threshold, cfg.max_boxes, cfg.visualize ? "on" : "off");
        std::printf("[Config] raw_output=%s raw_only=%s eval_scenes=%zu\n",
                    cfg.raw_dir.c_str(), cfg.raw_only ? "true" : "false",
                    cfg.eval_scenes.size());

        auto device = Device::Open(cfg.device_url);
        auto jr_path = getJrPath(cfg.model_backend, cfg.model_dir, cfg.model_stage);
        Network network = loadNetwork(jr_path.first, jr_path.second);
        NetInfo net_info(network);

        auto inputs = network.inputs();
        if (inputs.size() != 1 ||
            !inputs[0].tensorType()->element_dtype.isFP32() ||
            !has_shape(inputs[0].tensorType(), {6, 3, 256, 704})) {
            throw std::runtime_error("MatrixVT public input must be FP32 [6,3,256,704]");
        }

        std::printf("[Init] MatrixVT network ready; using one fresh session per frame\n");

        FastBEVDataset* dataset = fastbev_load(cfg.dataset_json.c_str());
        if (!dataset) throw std::runtime_error("failed to load dataset_info.json");
        std::printf("[Init] Loaded dataset samples=%d\n", dataset->num_samples);

        int eval_frame_id = 0;
        for (int frame_index = 0; frame_index < dataset->num_samples; ++frame_index) {
            const int frame_id = frame_index + 1;
            const FastBEVSample* sample = &dataset->samples[frame_index];
            if (!should_run_sample(sample, cfg)) {
                continue;
            }
            ++eval_frame_id;

            std::printf("\n========== MatrixVT Native Eval Frame %04d original=%04d scene=%s ==========\n",
                        eval_frame_id, frame_id, sample->scene_name);

            const auto frame_begin = Clock::now();
            const auto pre_begin = Clock::now();
            std::vector<float> input;
            input.reserve(static_cast<size_t>(kNumCameras) * kInputC * kInputH * kInputW);
            preprocess_sample(sample, cfg, input);
            const double pre_ms = elapsed_ms(pre_begin, Clock::now());

            double reset_ms = 0.0;
            if (frame_index > 0) {
                // Reusing the MatrixVT session after a reset produces invalid
                // results on the board runtime. A fresh session per frame is the
                // stable validation path for this single large graph.
                const auto reset_begin = Clock::now();
                device.reset(1);
                reset_ms = elapsed_ms(reset_begin, Clock::now());
            }

            const auto session_begin = Clock::now();
            Session frame_session = initSession(cfg.model_backend, network, device,
                                                cfg.ocm_option, net_info.mmu || cfg.mmu_mode,
                                                cfg.speed_mode, cfg.compress_ftmp);
            frame_session.apply();
            const double session_ms = elapsed_ms(session_begin, Clock::now());

            const auto tensor_begin = Clock::now();
            Tensor in_tensor = data2Tensor<float>(input.data(), network.inputs()[0]);
            const double input_ms = elapsed_ms(tensor_begin, Clock::now());

            const auto infer_begin = Clock::now();
            std::vector<Tensor> outputs = frame_session.forward({in_tensor});
            if (outputs.empty()) throw std::runtime_error("MatrixVT forward returned no outputs");
            if (!outputs[0].waitForReady(std::chrono::seconds(10))) {
                throw std::runtime_error("MatrixVT output wait timeout");
            }
            const double infer_ms = elapsed_ms(infer_begin, Clock::now());

            const auto read_begin = Clock::now();
            std::vector<float> predictions = tensor_to_vector(outputs[0]);
            if (predictions.size() != static_cast<size_t>(kOutputC) * kOutputH * kOutputW) {
                throw std::runtime_error("MatrixVT output must contain 70*128*128 FP32 elements");
            }
            const double read_ms = elapsed_ms(read_begin, Clock::now());

            char raw_path[512];
            std::snprintf(raw_path, sizeof(raw_path), "%s/raw_%04d_%s.txt",
                          cfg.raw_dir.c_str(), eval_frame_id, sample->sample_token);
            write_raw_predictions_txt(raw_path, sample, frame_id, eval_frame_id, predictions);

            if (cfg.raw_only) {
                const double total_ms = elapsed_ms(frame_begin, Clock::now());
                std::printf("[Raw] saved %s\n", raw_path);
                std::printf("[Result] eval_frame=%04d original=%04d pre=%.2f reset=%.2f session=%.2f input=%.2f npu=%.2f read=%.2f total=%.2f ms\n",
                            eval_frame_id, frame_id, pre_ms, reset_ms, session_ms, input_ms, infer_ms, read_ms, total_ms);
                continue;
            }

            const auto post_begin = Clock::now();
            std::vector<BoundingBox> boxes = decode_matrixvt(predictions, cfg);
            const double post_ms = elapsed_ms(post_begin, Clock::now());

            char result_path[512];
            char param_path[512];
            char png_path[512];
            std::snprintf(result_path, sizeof(result_path), "%s/result_%04d.txt",
                          cfg.box_dir.c_str(), frame_id);
            std::snprintf(param_path, sizeof(param_path), "%s/camera_params_%04d.txt",
                          cfg.para_dir.c_str(), frame_id);
            std::snprintf(png_path, sizeof(png_path), "%s/output_%04d.png",
                          cfg.png_dir.c_str(), frame_id);

            write_result_txt(result_path, boxes);
            if (export_matrixvt_camera_params(sample, param_path) != 0) {
                throw std::runtime_error("failed to export camera params");
            }
            if (cfg.visualize && render_matrixvt_native_png(sample, boxes, png_path) != 0) {
                std::fprintf(stderr, "[Warn] native MatrixVT render failed: %s\n", png_path);
            }

            const double total_ms = elapsed_ms(frame_begin, Clock::now());
            std::printf("[Result] frame=%04d objects=%zu pre=%.2f reset=%.2f session=%.2f input=%.2f npu=%.2f read=%.2f post=%.2f total=%.2f ms\n",
                        frame_id, boxes.size(), pre_ms, reset_ms, session_ms, input_ms, infer_ms, read_ms, post_ms, total_ms);
        }

        fastbev_free(dataset);
        Device::Close(device);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "MATRIXVT_PIPELINE_ERROR: %s\n", e.what());
        return 1;
    }
}
