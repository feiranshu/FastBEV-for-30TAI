/*
 * fastbev_preprocess_cv.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * 图像预处理模块（OpenCV 版）：对齐 CUDA-FastBEV 作者推理端前处理逻辑，
 * 输出 NHWC 张量，用于板载 NPU（part1）。
 * 仿射参数直接从 FastBEVSample 解包，供 FPGA SA 模块（part2）使用。
 *
 * 对齐作者 CUDA-FastBEV 的预处理步骤：
 *   A. cv::imread(BGR) -> raw BGR FP32; Parser performs SwapOrder internally
 *   B. 等比缩放：1600×900 按 resize_lim=0.44 缩放到 704×396
 *      插值方式使用 cv::INTER_NEAREST，对齐作者 Interpolation::Nearest
 *   C. 中心裁剪：从 704×396 中心裁剪 704×256
 *      crop_x = (704 - 704) / 2 = 0
 *      crop_y = (396 - 256) / 2 = 70
 *   D. Parser Add/Multiply performs (pixel - MEAN) / STD
 *   E. 输出 NHWC：
 *      单图 float[256][704][3]
 *      6路  float[6][256][704][3]
 *
 * 依赖：OpenCV 4.x、fastbev_reader.h
 * 编译：g++ -O2 main.cpp cJSON.c $(pkg-config --cflags --libs opencv4) -o fastbev_inference
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef FASTBEV_PREPROCESS_CV_HPP
#define FASTBEV_PREPROCESS_CV_HPP

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <vector>

#include "fastbev_reader.h"   /* FastBEVCamera, FastBEVSample 等结构体 */

/* ── 超参数：对齐 CUDA-FastBEV 作者推理端 ───────────────────────────── */

#define FASTBEV_SRC_W       1600
#define FASTBEV_SRC_H       900

#define FASTBEV_TARGET_W    704
#define FASTBEV_TARGET_H    256
#define FASTBEV_CHANNELS    3
#define FASTBEV_NUM_CAMS    6

#define FASTBEV_RESIZE_LIM  0.44f

/*
 * FASTBEV_ONE_CAM_SIZE = H × W × C = 256 × 704 × 3 = 542,208 个 float
 * FASTBEV_TENSOR_SIZE  = 6 × 542,208 = 3,253,248 个 float，约 12.4 MB
 */
#define FASTBEV_ONE_CAM_SIZE  (FASTBEV_TARGET_H * FASTBEV_TARGET_W * FASTBEV_CHANNELS)
#define FASTBEV_TENSOR_SIZE   (FASTBEV_NUM_CAMS  * FASTBEV_ONE_CAM_SIZE)
#define FASTBEV_DISPLAY_W     400
#define FASTBEV_DISPLAY_H     225

struct FastBEVPreprocessTiming {
    double imread_ms = 0.0;
    double resize_ms = 0.0;
    double bgr_pack_nhwc_ms = 0.0;
    double preview_encode_ms = 0.0;
    double affine_ms = 0.0;
    double total_ms = 0.0;
    double camera_total_ms[FASTBEV_NUM_CAMS] = {};
};

struct FastBEVDisplayImages {
    std::vector<unsigned char> jpeg[FASTBEV_NUM_CAMS];
};

using FastBEVPreprocessClock = std::chrono::steady_clock;

static double fastbev_preprocess_elapsed_ms(FastBEVPreprocessClock::time_point begin,
                                            FastBEVPreprocessClock::time_point end)
{
    return std::chrono::duration<double, std::milli>(end - begin).count();
}


/* ═══════════════════════════════════════════════════════════════════════════
 * 核心 API 1 — fastbev_preprocess_one_image()
 *
 * 输入：
 *   img_path   图像路径（由 fastbev_load 在运行时拼接，无硬编码绝对路径）
 *   out_hwc    调用方已分配的 float[256 * 704 * 3]
 *
 * 输出布局：
 *   NHWC 单图 [256, 704, 3]
 *   线性索引 row * W * C + col * C + ch
 *
 * 返回值：
 *   0  = 成功
 *  -1  = 失败
 * ═══════════════════════════════════════════════════════════════════════════ */
static int fastbev_preprocess_one_image(const char *img_path,
                                        float *out_hwc,
                                        FastBEVPreprocessTiming *timing = nullptr,
                                        std::vector<unsigned char> *display_jpeg = nullptr)
{
    const auto total_begin = FastBEVPreprocessClock::now();
    if (img_path == nullptr || out_hwc == nullptr) {
        fprintf(stderr, "[preprocess] img_path 或 out_hwc 为空\n");
        return -1;
    }

    /* A. 读取图像。OpenCV 默认输出 BGR uint8 HWC */
    const auto imread_begin = FastBEVPreprocessClock::now();
    cv::Mat img_bgr = cv::imread(img_path, cv::IMREAD_COLOR);
    if (timing) timing->imread_ms += fastbev_preprocess_elapsed_ms(imread_begin, FastBEVPreprocessClock::now());
    if (img_bgr.empty()) {
        fprintf(stderr, "[preprocess] 无法读取图像: %s\n", img_path);
        return -1;
    }

    if (display_jpeg) {
        const auto preview_begin = FastBEVPreprocessClock::now();
        cv::Mat preview;
        cv::resize(img_bgr, preview, cv::Size(FASTBEV_DISPLAY_W, FASTBEV_DISPLAY_H),
                   0, 0, cv::INTER_AREA);
        const std::vector<int> jpeg_params = {
            cv::IMWRITE_JPEG_QUALITY, 85,
        };
        if (!cv::imencode(".jpg", preview, *display_jpeg, jpeg_params)) {
            display_jpeg->clear();
        }
        if (timing) {
            timing->preview_encode_ms += fastbev_preprocess_elapsed_ms(
                preview_begin, FastBEVPreprocessClock::now());
        }
    }

    const int ori_h = img_bgr.rows;
    const int ori_w = img_bgr.cols;

    if (ori_w != FASTBEV_SRC_W || ori_h != FASTBEV_SRC_H) {
        fprintf(stderr,
                "[preprocess] 警告: 输入图像尺寸为 %dx%d，不是作者推理端默认的 %dx%d: %s\n",
                ori_w, ori_h, FASTBEV_SRC_W, FASTBEV_SRC_H, img_path);
    }

    /* B. Keep BGR order; Parser executes channel swap and normalization. */
    /*
     * C. 等比缩放。
     * 对 1600×900，resize_lim=0.44：
     *   resized_width  = int(1600 * 0.44) = 704
     *   resized_height = int(900  * 0.44) = 396
     * 插值方式：INTER_NEAREST，对齐作者训练端。
     */
    const float resize_ratio = FASTBEV_RESIZE_LIM;
    const int resized_w = static_cast<int>((float)ori_w * resize_ratio);
    const int resized_h = static_cast<int>((float)ori_h * resize_ratio);

    if (resized_w < FASTBEV_TARGET_W || resized_h < FASTBEV_TARGET_H) {
        fprintf(stderr,
                "[preprocess] 缩放后尺寸 %dx%d 小于目标尺寸 %dx%d，跳过: %s\n",
                resized_w, resized_h, FASTBEV_TARGET_W, FASTBEV_TARGET_H, img_path);
        return -1;
    }

    const auto resize_begin = FastBEVPreprocessClock::now();
    cv::Mat resized;
    cv::resize(img_bgr,
               resized,
               cv::Size(resized_w, resized_h),
               0, 0,
               cv::INTER_NEAREST);
    if (timing) timing->resize_ms += fastbev_preprocess_elapsed_ms(resize_begin, FastBEVPreprocessClock::now());

    /*
     * D. 中心裁剪。
     * 对 704×396 → 704×256：
     *   crop_x = 0
     *   crop_y = 70
     */
    const int crop_x = (resized_w - FASTBEV_TARGET_W) / 2;
    const int crop_y = (resized_h - FASTBEV_TARGET_H) / 2;

    if (crop_x < 0 || crop_y < 0 ||
        crop_x + FASTBEV_TARGET_W > resized.cols ||
        crop_y + FASTBEV_TARGET_H > resized.rows) {
        fprintf(stderr,
                "[preprocess] crop 区域非法: resized=%dx%d, crop=(%d,%d,%d,%d), path=%s\n",
                resized.cols, resized.rows,
                crop_x, crop_y, FASTBEV_TARGET_W, FASTBEV_TARGET_H,
                img_path);
        return -1;
    }

    cv::Mat cropped = resized(cv::Rect(crop_x, crop_y,
                                       FASTBEV_TARGET_W, FASTBEV_TARGET_H));

    /*
     * E. Pack raw BGR pixels into FP32 NHWC.
     */
    // Parser SwapOrder/Add/Multiply handles RGB conversion and normalization.
    const int W = FASTBEV_TARGET_W;
    const int C = FASTBEV_CHANNELS;

    const auto pack_begin = FastBEVPreprocessClock::now();
    for (int row = 0; row < FASTBEV_TARGET_H; row++) {
        const uint8_t *src_row = cropped.ptr<uint8_t>(row);
        float *dst_row = out_hwc + (size_t)row * W * C;
        for (int col = 0; col < W; col++) {
            const uint8_t *px = src_row + (size_t)col * C;
            float *dst = dst_row + (size_t)col * C;
            dst[0] = static_cast<float>(px[0]);
            dst[1] = static_cast<float>(px[1]);
            dst[2] = static_cast<float>(px[2]);
        }
    }
    if (timing) {
        timing->bgr_pack_nhwc_ms += fastbev_preprocess_elapsed_ms(pack_begin, FastBEVPreprocessClock::now());
        timing->total_ms += fastbev_preprocess_elapsed_ms(total_begin, FastBEVPreprocessClock::now());
    }

    return 0;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * 核心 API 2 — fastbev_preprocess_cameras()
 *
 * 6 路相机依次预处理，写入连续内存，布局：
 *   NHWC [6, 256, 704, 3]
 *
 * 线性索引：
 *   cam * H * W * C + row * W * C + col * C + ch
 *
 * 参数：
 *   cameras     FastBEVCamera[NUM_CAMERAS]
 *   out_tensor  float[6 * 256 * 704 * 3]
 *
 * 返回值：
 *   成功处理的相机数量；失败相机对应区域填 0
 * ═══════════════════════════════════════════════════════════════════════════ */
static int fastbev_preprocess_cameras(
    const FastBEVCamera cameras[NUM_CAMERAS],
    float *out_tensor,
    FastBEVPreprocessTiming *timing = nullptr,
    FastBEVDisplayImages *display_images = nullptr)
{
    if (cameras == nullptr || out_tensor == nullptr) {
        fprintf(stderr, "[preprocess] cameras 或 out_tensor 为空\n");
        return 0;
    }

    int ok = 0;

    for (int cam = 0; cam < NUM_CAMERAS; cam++) {
        float *buf = out_tensor + (size_t)cam * FASTBEV_ONE_CAM_SIZE;
        std::vector<unsigned char> *display_jpeg = display_images
            ? &display_images->jpeg[cam]
            : nullptr;
        if (display_jpeg) display_jpeg->clear();

        if (cameras[cam].image_path[0] == '\0') {
            memset(buf, 0, (size_t)FASTBEV_ONE_CAM_SIZE * sizeof(float));
            fprintf(stderr, "[preprocess] cam[%d] 路径为空，填零\n", cam);
            continue;
        }

        const auto camera_begin = FastBEVPreprocessClock::now();
        if (fastbev_preprocess_one_image(cameras[cam].image_path, buf, timing, display_jpeg) == 0) {
            ok++;
        } else {
            memset(buf, 0, (size_t)FASTBEV_ONE_CAM_SIZE * sizeof(float));
            fprintf(stderr, "[preprocess] cam[%d] 预处理失败，填零: %s\n",
                    cam, cameras[cam].image_path);
        }
        if (timing) {
            timing->camera_total_ms[cam] = fastbev_preprocess_elapsed_ms(
                camera_begin, FastBEVPreprocessClock::now());
        }
    }

    return ok;
}


/* ═══════════════════════════════════════════════════════════════════════════
 * 主数据结构 — FastBEVFrameInput
 *
 * 只包含：
 *   current_tensor   当前帧 T 的 6 路图像张量，NHWC [6,256,704,3]
 *   affine[3][6]     3 组仿射参数，供 FPGA SA 模块（part2）使用
 *
 * 单帧内存：6 × 256 × 704 × 3 × 4B ≈ 12.4 MB
 * ═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float current_tensor[FASTBEV_TENSOR_SIZE];          /* NHWC [6,256,704,3] */
    float affine[NUM_HIST_FRAMES][AFFINE_PARAMS];        /* [3][6]             */
} FastBEVFrameInput;


/* ═══════════════════════════════════════════════════════════════════════════
 * 高层 API — fastbev_prepare_frame_input()
 *
 * 一次调用完成：
 *   1. 当前帧 6 路图像预处理      → inp->current_tensor
 *   2. 3 组仿射参数拷贝           → inp->affine[0..2]
 *
 * 返回值：失败的相机路数（0 = 全部成功）
 * ═══════════════════════════════════════════════════════════════════════════ */
static int fastbev_prepare_frame_input(const FastBEVSample *sample,
                                       FastBEVFrameInput   *inp,
                                       FastBEVPreprocessTiming *timing = nullptr,
                                       FastBEVDisplayImages *display_images = nullptr)
{
    if (sample == nullptr || inp == nullptr) {
        fprintf(stderr, "[preprocess] sample 或 inp 为空\n");
        return NUM_CAMERAS;
    }

    if (timing) *timing = {};

    /* 当前帧 6 路图像预处理 */
    int ok = fastbev_preprocess_cameras(sample->cameras, inp->current_tensor, timing, display_images);
    int fail = NUM_CAMERAS - ok;

    /* 3 组仿射参数：直接从 FastBEVSample 拷贝 */
    const auto affine_begin = FastBEVPreprocessClock::now();
    for (int t = 0; t < NUM_HIST_FRAMES; t++) {
        memcpy(inp->affine[t],
               sample->temporal[t].affine_params,
               sizeof(float) * AFFINE_PARAMS);
    }
    if (timing) {
        timing->affine_ms = fastbev_preprocess_elapsed_ms(affine_begin, FastBEVPreprocessClock::now());
    }

    return fail;
}


/* ── 调试工具：打印单路相机张量统计 ──────────────────────────────────── */
static void fastbev_tensor_stats(const float *tensor, int cam, const char *label)
{
    if (tensor == nullptr) {
        fprintf(stderr, "[stats] tensor 为空\n");
        return;
    }

    if (cam < 0 || cam >= NUM_CAMERAS) {
        fprintf(stderr, "[stats] cam index 非法: %d\n", cam);
        return;
    }

    const char *name = (label != nullptr) ? label : "tensor";
    const float *p = tensor + (size_t)cam * FASTBEV_ONE_CAM_SIZE;

    float vmin = p[0];
    float vmax = p[0];
    double vsum = 0.0;

    for (int i = 0; i < FASTBEV_ONE_CAM_SIZE; i++) {
        if (p[i] < vmin) vmin = p[i];
        if (p[i] > vmax) vmax = p[i];
        vsum += (double)p[i];
    }

    printf("[stats] %-10s cam[%d]  NHWC  min=%7.4f  max=%7.4f  mean=%7.4f\n",
           name, cam, vmin, vmax,
           (float)(vsum / (double)FASTBEV_ONE_CAM_SIZE));
}

#endif /* FASTBEV_PREPROCESS_CV_HPP */
