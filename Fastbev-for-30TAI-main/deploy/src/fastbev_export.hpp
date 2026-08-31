/*
 * fastbev_export.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * 从 dataset_info.json（已由 fastbev_reader.h 解析）导出 camera_params.txt，
 * 供 visualize 可视化程序读取。
 *
 * camera_params.txt 格式（每路相机 6 行）：
 *
 *   <num_cameras>                    = 6
 *   <camera_name>                    e.g. CAM_FRONT
 *   <image_path>                     SD 卡上原图绝对路径
 *   e00 e01 ... e33                  16 floats：extrinsic 4×4 行主序
 *   i00 i01 ... i22                   9 floats：intrinsic 3×3（固定为 identity）
 *   p00 p01 ... p22                   9 floats：post_rot  3×3（固定 diag(0.44)）
 *   pt0 pt1 pt2                       3 floats：post_tran（固定 [0,-70,0]）
 *
 * extrinsic 4×4 的来源：
 *
 *   【推荐】直接使用 JSON 中预计算的 lidar2img 字段（cam->lidar2img）。
 *   该矩阵由 Python 端从 nuScenes 数据库 / .pth 文件预计算，包含完整的
 *   投影链：post_aug × K × cam_from_ego × ego_pose_correction × ego_from_lidar。
 *
 *   若 JSON 中无 lidar2img 字段，fallback 到从 calibration 近似计算：
 *     E = post_aug × K × [R_e2c | t_e2c]
 *   但此近似缺少 lidar-to-ego 变换和时间戳校正，会产生 30~60 像素级别的偏差。
 *
 *   visualize.cpp 的投影步骤：
 *     cx,cy,cz = E × P_homo           → 增强图像空间
 *     nx,ny    = cx/cz, cy/cz
 *     ix,iy    = K_stored × [nx,ny,1]  （K_stored=identity，无操作）
 *     ox,oy    = inv(post_aug) × [ix,iy,1]  （回到原图坐标）
 *
 * post_rot / post_tran（对所有相机、所有帧固定不变）：
 *   nuScenes 原图 1600×900，resize_lim=0.44 → 704×396，再中心裁到 704×256
 *   post_rot  = diag(0.44, 0.44, 1)
 *   post_tran = [0, -70, 0]   (crop_y = (396-256)/2 = 70，取负)
 *
 * 依赖：fastbev_reader.h、<cmath>、<cstdio>
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef FASTBEV_EXPORT_HPP
#define FASTBEV_EXPORT_HPP

#include <cmath>
#include <cstdio>
#include <cstring>

#include "fastbev_reader.h"

/* ── 固定的图像增强参数（对齐 CUDA-FastBEV 作者推理端）─────────────────── */

/* post_rot：resize_lim=0.44，对角缩放矩阵，行主序 9 个 float */
static const float FASTBEV_POST_ROT[9] = {
    0.44f, 0.0f,  0.0f,
    0.0f,  0.44f, 0.0f,
    0.0f,  0.0f,  1.0f,
};

/* post_tran：中心裁剪偏移，crop_y = (396-256)/2 = 70，取负值 */
static const float FASTBEV_POST_TRAN[3] = {0.0f, -70.0f, 0.0f};

/* intrinsic 写入 identity（K 已 bake 进 extrinsic）*/
static const float FASTBEV_IDENTITY_3x3[9] = {
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 1.0f,
};

/* ── 内部工具：四元数 [w,x,y,z] → 3×3 旋转矩阵（行主序）────────────── */

static void _quat_to_rotmat(const float q[4], float r[9])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float n = sqrtf(w*w + x*x + y*y + z*z);
    if (n > 1e-8f) { w /= n; x /= n; y /= n; z /= n; }

    r[0] = 1 - 2*(y*y + z*z);  r[1] = 2*(x*y - w*z);      r[2] = 2*(x*z + w*y);
    r[3] = 2*(x*y + w*z);      r[4] = 1 - 2*(x*x + z*z);  r[5] = 2*(y*z - w*x);
    r[6] = 2*(x*z - w*y);      r[7] = 2*(y*z + w*x);      r[8] = 1 - 2*(x*x + y*y);
}

/* ── Fallback：从 calibration 近似计算 E ──────────────────────────────
 *
 * 仅当 JSON 中无 lidar2img 字段时使用。
 * 计算 E = post_aug × K × [R_e2c | t_e2c]（ego→image，缺少 lidar→ego）。
 *
 * ⚠ 此近似有 30~60 像素级误差，请尽量在 JSON 中提供 lidar2img 字段。
 * ──────────────────────────────────────────────────────────────────── */
static void _compute_ego2img_E_fallback(
    const float K[9],
    const float rot_c2e[4],
    const float t_c2e[3],
    float       E[16])
{
    float Rc2e[9];
    _quat_to_rotmat(rot_c2e, Rc2e);

    float Re2c[9];
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            Re2c[r*3+c] = Rc2e[c*3+r];

    float te2c[3] = {0,0,0};
    for (int r = 0; r < 3; r++)
        for (int c = 0; c < 3; c++)
            te2c[r] -= Re2c[r*3+c] * t_c2e[c];

    /* E_raw = K × [Re2c | te2c] */
    float E_raw[12];
    memset(E_raw, 0, 12 * sizeof(float));
    for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++)
            for (int k = 0; k < 3; k++)
                E_raw[row*4 + col] += K[row*3+k] * Re2c[k*3+col];
        for (int k = 0; k < 3; k++)
            E_raw[row*4 + 3] += K[row*3+k] * te2c[k];
    }

    /* E = post_aug × E_raw */
    const float pa[3][3] = {
        { FASTBEV_POST_ROT[0], FASTBEV_POST_ROT[1], FASTBEV_POST_TRAN[0] },
        { FASTBEV_POST_ROT[3], FASTBEV_POST_ROT[4], FASTBEV_POST_TRAN[1] },
        { FASTBEV_POST_ROT[6], FASTBEV_POST_ROT[7], 1.0f                 },
    };

    memset(E, 0, 16 * sizeof(float));
    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 4; col++)
            for (int k = 0; k < 3; k++)
                E[row*4 + col] += pa[row][k] * E_raw[k*4 + col];
    E[3*4 + 3] = 1.0f;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 公开 API — fastbev_export_camera_params()
 *
 * 从 FastBEVSample 的当前帧数据生成 camera_params.txt。
 *
 * 参数：
 *   sample      当前帧（只用 sample->cameras[6]，不用历史帧）
 *   output_path camera_params.txt 的写出路径
 *
 * extrinsic 来源优先级：
 *   1. cam->lidar2img（JSON 预计算，精确）
 *   2. fallback 从 calibration 计算（近似，有误差，会打印警告）
 *
 * 返回值：0=成功，-1=失败
 * ═══════════════════════════════════════════════════════════════════════════ */
static int fastbev_export_camera_params(
    const FastBEVSample *sample,
    const char          *output_path)
{
    static const char *CAM_NAMES[NUM_CAMERAS] = {
        "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
        "CAM_BACK",  "CAM_BACK_LEFT",   "CAM_BACK_RIGHT"
    };

    FILE *fp = fopen(output_path, "w");
    if (!fp) {
        fprintf(stderr, "[export] 无法创建: %s\n", output_path);
        return -1;
    }

    fprintf(fp, "%d\n", NUM_CAMERAS);

    int fallback_count = 0;

    for (int i = 0; i < NUM_CAMERAS; i++) {
        const FastBEVCamera *cam = &sample->cameras[i];

        /* ── 第1行：相机名 ── */
        fprintf(fp, "%s\n", CAM_NAMES[i]);

        /* ── 第2行：图像路径 ── */
        fprintf(fp, "%s\n", cam->image_path);

        /* ── 第3行：extrinsic 4×4 ── */
        float E[16];

        if (cam->has_lidar2img) {
            /* 优先使用 JSON 中预计算的 lidar2img（精确） */
            memcpy(E, cam->lidar2img, 16 * sizeof(float));
        } else {
            /* Fallback：从 calibration 近似计算（有误差） */
            _compute_ego2img_E_fallback(
                cam->intrinsic, cam->extrinsic_r, cam->extrinsic_t, E);
            fallback_count++;
        }

        for (int k = 0; k < 16; k++)
            fprintf(fp, "%.10g%c", E[k], k < 15 ? ' ' : '\n');

        /* ── 第4行：intrinsic 3×3（固定 identity） ── */
        for (int k = 0; k < 9; k++)
            fprintf(fp, "%.10g%c", FASTBEV_IDENTITY_3x3[k], k < 8 ? ' ' : '\n');

        /* ── 第5行：post_rot 3×3 ── */
        for (int k = 0; k < 9; k++)
            fprintf(fp, "%.10g%c", FASTBEV_POST_ROT[k], k < 8 ? ' ' : '\n');

        /* ── 第6行：post_tran 3 ── */
        for (int k = 0; k < 3; k++)
            fprintf(fp, "%.10g%c", FASTBEV_POST_TRAN[k], k < 2 ? ' ' : '\n');
    }

    fclose(fp);

    if (fallback_count > 0) {
        fprintf(stderr,
            "[export] 警告: %d/%d 个相机缺少 lidar2img 字段，"
            "使用 calibration fallback（投影有 30~60 像素偏差）。\n"
            "         请在 JSON 的每个相机下添加 \"lidar2img\" 字段以消除误差。\n",
            fallback_count, NUM_CAMERAS);
    }

    return 0;
}

#endif /* FASTBEV_EXPORT_HPP */
