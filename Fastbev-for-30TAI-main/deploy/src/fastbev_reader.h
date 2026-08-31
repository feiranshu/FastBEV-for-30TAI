/*
 * fastbev_reader.h
 * ─────────────────────────────────────────────────────────────────────────────
 * 板端读取 dataset_info.json 的辅助结构与接口
 *
 * 依赖：
 *   - cJSON  (轻量级 JSON 解析库，MIT 协议)
 *     下载：https://github.com/DaveGamble/cJSON
 *     在项目中只需 cJSON.c + cJSON.h 两个文件
 *   - libjpeg 或 stb_image.h 用于图像解码
 *
 * 典型用法（板端推理循环）：
 *
 *   FastBEVDataset *ds = fastbev_load("/sdcard/fastbev_data/dataset_info.json");
 *   if (!ds) { fprintf(stderr, "加载失败\n"); return -1; }
 *
 *   for (int i = 0; i < ds->num_samples; i++) {
 *       FastBEVSample *s = &ds->samples[i];
 *
 *       // 1. 读取当前帧 6 张图像
 *       for (int c = 0; c < NUM_CAMERAS; c++) {
 *           load_image(s->cameras[c].image_path);
 *       }
 *
 *       // 2. 历史帧图像 + 仿射参数
 *       for (int t = 0; t < NUM_HIST_FRAMES; t++) {
 *           FastBEVTemporalFrame *f = &s->temporal[t];
 *           float *params = f->affine_params;   // [a00,a01,a02,a10,a11,a12]
 *           for (int c = 0; c < NUM_CAMERAS; c++) {
 *               load_image(f->cameras[c].image_path);
 *           }
 *           // 送入神经网络...
 *       }
 *   }
 *
 *   fastbev_free(ds);
 * ─────────────────────────────────────────────────────────────────────────────
 */

#ifndef FASTBEV_READER_H
#define FASTBEV_READER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "cJSON.h"

/* ── 编译期常量 ──────────────────────────────────────────────────────────── */

#define NUM_CAMERAS      6
#define NUM_HIST_FRAMES  3
#define AFFINE_PARAMS    6    /* [a00, a01, a02, a10, a11, a12] */

/* 相机通道顺序（与 Python 端 CAMERAS 列表一致） */
#define CAM_FRONT        0
#define CAM_FRONT_RIGHT  1
#define CAM_FRONT_LEFT   2
#define CAM_BACK         3
#define CAM_BACK_LEFT    4
#define CAM_BACK_RIGHT   5

/* ── 数据结构 ────────────────────────────────────────────────────────────── */

typedef struct {
    char     image_path[256];      /* 相对于 dataset_info.json 目录的路径 */
    char     sample_data_token[33];
    int64_t  timestamp;            /* 微秒 */

    /* 外参（相机到车体） */
    float    extrinsic_t[3];       /* translation [x, y, z] */
    float    extrinsic_r[4];       /* rotation [w, x, y, z] */

    /* 内参 3×3（行主序） */
    float    intrinsic[9];         /* [fx, 0, cx, 0, fy, cy, 0, 0, 1] */

    /* ── 预计算的 lidar2img 4×4 矩阵（行主序）──────────────────────
     *
     * 完整的 lidar→image 投影矩阵，已包含：
     *   post_aug × K × cam_from_ego × ego_pose_correction × ego_from_lidar
     *
     * 由 Python 端从 nuScenes 数据库 / .pth 文件预计算后写入 JSON。
     * 这是 visualize.cpp 投影所需的唯一矩阵。
     *
     * 若 JSON 中不含此字段，has_lidar2img 为 0，
     * fastbev_export.hpp 会 fallback 到从 calibration 近似计算（有误差）。
     * ──────────────────────────────────────────────────────────── */
    float    lidar2img[16];
    int      has_lidar2img;        /* 1 = JSON 中存在 lidar2img 字段 */
} FastBEVCamera;

typedef struct {
    int         frame_index;                   /* 0=T-1, 1=T-2, 2=T-3 */
    int64_t     timestamp;
    int         is_key_frame;
    float       ego_translation[3];            /* 全局坐标系 [x, y, z] */
    float       ego_rotation[4];              /* [w, x, y, z] */
    float       affine_params[AFFINE_PARAMS]; /* BEV 对齐仿射参数 */
    FastBEVCamera cameras[NUM_CAMERAS];
} FastBEVTemporalFrame;

typedef struct {
    int         index;
    char        sample_token[33];
    char        scene_token[33];
    char        scene_name[64];
    int64_t     timestamp;
    int         is_first_in_scene;
    float       ego_translation[3];
    float       ego_rotation[4];
    FastBEVCamera        cameras[NUM_CAMERAS];
    FastBEVTemporalFrame temporal[NUM_HIST_FRAMES];
} FastBEVSample;

typedef struct {
    char   version[16];
    int    num_samples;

    /* BEV 参数 */
    float  bev_x_min, bev_x_max;   /* 通常 -51.2, 51.2 */
    float  bev_y_min, bev_y_max;
    int    bev_width, bev_height;   /* 通常 200, 200 */
    float  bev_resolution;          /* 通常 0.512 m/px */

    /* 索引文件所在目录（用于拼接图像路径） */
    char   base_dir[512];

    FastBEVSample *samples;         /* 长度 = num_samples */
} FastBEVDataset;

/* ── 内部工具宏 ──────────────────────────────────────────────────────────── */

#define FASTBEV_SAFE_STR(dst, src, n)                       \
    do {                                                     \
        if ((src) && cJSON_IsString(src)) {                  \
            strncpy((dst), (src)->valuestring, (n) - 1);    \
            (dst)[(n) - 1] = '\0';                          \
        }                                                    \
    } while (0)

#define FASTBEV_SAFE_FLOAT(dst, src)                        \
    do {                                                     \
        if ((src) && cJSON_IsNumber(src))                    \
            (dst) = (float)(src)->valuedouble;               \
    } while (0)

#define FASTBEV_SAFE_INT64(dst, src)                        \
    do {                                                     \
        if ((src) && cJSON_IsNumber(src))                    \
            (dst) = (int64_t)(src)->valuedouble;             \
    } while (0)

/* ── 解析辅助函数（内部使用） ────────────────────────────────────────────── */

static void _parse_float_array(float *dst, int n, const cJSON *arr) {
    if (!arr || !cJSON_IsArray(arr)) return;
    int i = 0;
    const cJSON *elem;
    cJSON_ArrayForEach(elem, arr) {
        if (i >= n) break;
        if (cJSON_IsNumber(elem))
            dst[i] = (float)elem->valuedouble;
        i++;
    }
}

static void _parse_intrinsic(float *intrinsic, const cJSON *j_intr) {
    /* camera_intrinsic 是 3×3 嵌套数组 */
    if (!j_intr || !cJSON_IsArray(j_intr)) return;
    int r = 0;
    const cJSON *row;
    cJSON_ArrayForEach(row, j_intr) {
        int c = 0;
        const cJSON *col;
        cJSON_ArrayForEach(col, row) {
            if (cJSON_IsNumber(col))
                intrinsic[r * 3 + c] = (float)col->valuedouble;
            c++;
        }
        r++;
    }
}

/* 解析 lidar2img：4×4 嵌套数组 → 16 个 float（行主序）*/
static int _parse_lidar2img(float *dst, const cJSON *j_l2i) {
    if (!j_l2i || !cJSON_IsArray(j_l2i)) return 0;
    int r = 0;
    const cJSON *row;
    cJSON_ArrayForEach(row, j_l2i) {
        if (r >= 4) break;
        int c = 0;
        const cJSON *col;
        cJSON_ArrayForEach(col, row) {
            if (c >= 4) break;
            if (cJSON_IsNumber(col))
                dst[r * 4 + c] = (float)col->valuedouble;
            c++;
        }
        r++;
    }
    return (r == 4) ? 1 : 0;  /* 成功返回 1 */
}

/* 相机通道名 → 索引 */
static int _cam_channel_index(const char *name) {
    static const char *ch_names[NUM_CAMERAS] = {
        "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
        "CAM_BACK",  "CAM_BACK_LEFT",   "CAM_BACK_RIGHT"
    };
    for (int i = 0; i < NUM_CAMERAS; i++)
        if (strcmp(ch_names[i], name) == 0) return i;
    return -1;
}

static void _parse_camera_block(FastBEVCamera cams[NUM_CAMERAS],
                                 const cJSON *j_cams,
                                 const char *base_dir)
{
    if (!j_cams) return;
    const cJSON *entry;
    cJSON_ArrayForEach(entry, j_cams) {
        int idx = _cam_channel_index(entry->string);
        if (idx < 0) continue;
        FastBEVCamera *c = &cams[idx];

        /* 图像路径：拼接 base_dir */
        const cJSON *j_path = cJSON_GetObjectItemCaseSensitive(entry, "image_path");
        if (j_path && cJSON_IsString(j_path)) {
            snprintf(c->image_path, sizeof(c->image_path),
                     "%s/%s", base_dir, j_path->valuestring);
        }

        FASTBEV_SAFE_STR(c->sample_data_token,
            cJSON_GetObjectItemCaseSensitive(entry, "sample_data_token"), 33);
        FASTBEV_SAFE_INT64(c->timestamp,
            cJSON_GetObjectItemCaseSensitive(entry, "timestamp"));

        const cJSON *cal = cJSON_GetObjectItemCaseSensitive(entry, "calibration");
        if (cal) {
            _parse_float_array(c->extrinsic_t, 3,
                cJSON_GetObjectItemCaseSensitive(cal, "translation"));
            _parse_float_array(c->extrinsic_r, 4,
                cJSON_GetObjectItemCaseSensitive(cal, "rotation"));
            _parse_intrinsic(c->intrinsic,
                cJSON_GetObjectItemCaseSensitive(cal, "camera_intrinsic"));
        }

        /* ── 解析 lidar2img（可选字段）── */
        const cJSON *j_l2i = cJSON_GetObjectItemCaseSensitive(entry, "lidar2img");
        c->has_lidar2img = _parse_lidar2img(c->lidar2img, j_l2i);
    }
}

/* ── 公开 API ────────────────────────────────────────────────────────────── */

/**
 * fastbev_load - 从 JSON 索引文件加载完整数据集
 *
 * @param json_path  dataset_info.json 的完整路径
 * @return           成功返回已分配的 FastBEVDataset*，失败返回 NULL
 *
 * 注意：内部一次性读取整个 JSON 文件到内存。
 *       nuScenes mini（404 帧）约 20~30 MB，板端内存应足够。
 *       如内存紧张，可改为按需读取（见 fastbev_get_sample）。
 */
static FastBEVDataset *fastbev_load(const char *json_path) {
    /* ── 读文件 ── */
    FILE *fp = fopen(json_path, "r");
    if (!fp) {
        fprintf(stderr, "[fastbev] 无法打开文件: %s\n", json_path);
        return NULL;
    }
    fseek(fp, 0, SEEK_END);
    long fsize = ftell(fp);
    rewind(fp);
    char *buf = (char *)malloc(fsize + 1);
    if (!buf) { fclose(fp); return NULL; }
    fread(buf, 1, fsize, fp);
    buf[fsize] = '\0';
    fclose(fp);

    /* ── 解析 JSON ── */
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        fprintf(stderr, "[fastbev] JSON 解析失败: %s\n", cJSON_GetErrorPtr());
        return NULL;
    }

    FastBEVDataset *ds = (FastBEVDataset *)calloc(1, sizeof(FastBEVDataset));
    if (!ds) { cJSON_Delete(root); return NULL; }

    /* base_dir = json_path 的目录部分 */
    strncpy(ds->base_dir, json_path, sizeof(ds->base_dir) - 1);
    char *last_sep = strrchr(ds->base_dir, '/');
    if (!last_sep) last_sep = strrchr(ds->base_dir, '\\');
    if (last_sep) *last_sep = '\0'; else strcpy(ds->base_dir, ".");

    /* 顶层字段 */
    FASTBEV_SAFE_STR(ds->version, cJSON_GetObjectItemCaseSensitive(root, "version"), 16);

    const cJSON *j_n = cJSON_GetObjectItemCaseSensitive(root, "num_samples");
    ds->num_samples = j_n ? j_n->valueint : 0;

    const cJSON *bev = cJSON_GetObjectItemCaseSensitive(root, "bev_params");
    if (bev) {
        const cJSON *xr = cJSON_GetObjectItemCaseSensitive(bev, "x_range");
        const cJSON *yr = cJSON_GetObjectItemCaseSensitive(bev, "y_range");
        if (xr) {
            ds->bev_x_min = (float)cJSON_GetArrayItem(xr, 0)->valuedouble;
            ds->bev_x_max = (float)cJSON_GetArrayItem(xr, 1)->valuedouble;
        }
        if (yr) {
            ds->bev_y_min = (float)cJSON_GetArrayItem(yr, 0)->valuedouble;
            ds->bev_y_max = (float)cJSON_GetArrayItem(yr, 1)->valuedouble;
        }
        FASTBEV_SAFE_FLOAT(ds->bev_resolution,
            cJSON_GetObjectItemCaseSensitive(bev, "resolution_m_per_pixel"));
        const cJSON *jw = cJSON_GetObjectItemCaseSensitive(bev, "width");
        const cJSON *jh = cJSON_GetObjectItemCaseSensitive(bev, "height");
        if (jw) ds->bev_width  = jw->valueint;
        if (jh) ds->bev_height = jh->valueint;
    }

    /* samples 数组 */
    const cJSON *j_samples = cJSON_GetObjectItemCaseSensitive(root, "samples");
    if (!j_samples || !cJSON_IsArray(j_samples)) {
        cJSON_Delete(root);
        free(ds);
        return NULL;
    }

    ds->samples = (FastBEVSample *)calloc(ds->num_samples, sizeof(FastBEVSample));
    if (!ds->samples) {
        cJSON_Delete(root); free(ds); return NULL;
    }

    int si = 0;
    const cJSON *j_s;
    cJSON_ArrayForEach(j_s, j_samples) {
        if (si >= ds->num_samples) break;
        FastBEVSample *s = &ds->samples[si++];

        s->index = si - 1;
        FASTBEV_SAFE_STR(s->sample_token,
            cJSON_GetObjectItemCaseSensitive(j_s, "sample_token"), 33);
        FASTBEV_SAFE_STR(s->scene_token,
            cJSON_GetObjectItemCaseSensitive(j_s, "scene_token"), 33);
        FASTBEV_SAFE_STR(s->scene_name,
            cJSON_GetObjectItemCaseSensitive(j_s, "scene_name"), 64);
        FASTBEV_SAFE_INT64(s->timestamp,
            cJSON_GetObjectItemCaseSensitive(j_s, "timestamp"));

        const cJSON *j_first = cJSON_GetObjectItemCaseSensitive(j_s, "is_first_in_scene");
        s->is_first_in_scene = (j_first && cJSON_IsTrue(j_first)) ? 1 : 0;

        const cJSON *j_ego = cJSON_GetObjectItemCaseSensitive(j_s, "ego_pose");
        if (j_ego) {
            _parse_float_array(s->ego_translation, 3,
                cJSON_GetObjectItemCaseSensitive(j_ego, "translation"));
            _parse_float_array(s->ego_rotation, 4,
                cJSON_GetObjectItemCaseSensitive(j_ego, "rotation"));
        }

        /* 当前帧相机 */
        _parse_camera_block(s->cameras,
            cJSON_GetObjectItemCaseSensitive(j_s, "cameras"), ds->base_dir);

        /* 历史帧 */
        const cJSON *j_temporal = cJSON_GetObjectItemCaseSensitive(j_s, "temporal_frames");
        if (j_temporal && cJSON_IsArray(j_temporal)) {
            int ti = 0;
            const cJSON *j_f;
            cJSON_ArrayForEach(j_f, j_temporal) {
                if (ti >= NUM_HIST_FRAMES) break;
                FastBEVTemporalFrame *f = &s->temporal[ti++];

                const cJSON *j_fi = cJSON_GetObjectItemCaseSensitive(j_f, "frame_index");
                f->frame_index = j_fi ? j_fi->valueint : ti - 1;

                FASTBEV_SAFE_INT64(f->timestamp,
                    cJSON_GetObjectItemCaseSensitive(j_f, "timestamp"));

                const cJSON *j_kf = cJSON_GetObjectItemCaseSensitive(j_f, "is_key_frame");
                f->is_key_frame = (j_kf && cJSON_IsTrue(j_kf)) ? 1 : 0;

                const cJSON *j_fego = cJSON_GetObjectItemCaseSensitive(j_f, "ego_pose");
                if (j_fego) {
                    _parse_float_array(f->ego_translation, 3,
                        cJSON_GetObjectItemCaseSensitive(j_fego, "translation"));
                    _parse_float_array(f->ego_rotation, 4,
                        cJSON_GetObjectItemCaseSensitive(j_fego, "rotation"));
                }

                /* 仿射参数 [a00,a01,a02,a10,a11,a12] */
                _parse_float_array(f->affine_params, AFFINE_PARAMS,
                    cJSON_GetObjectItemCaseSensitive(j_f, "affine_params"));

                /* 历史帧相机 */
                _parse_camera_block(f->cameras,
                    cJSON_GetObjectItemCaseSensitive(j_f, "cameras"), ds->base_dir);
            }
        }
    }

    cJSON_Delete(root);
    fprintf(stderr, "[fastbev] 加载完成：%d 帧\n", ds->num_samples);
    return ds;
}

/**
 * fastbev_free - 释放 fastbev_load 分配的内存
 */
static void fastbev_free(FastBEVDataset *ds) {
    if (!ds) return;
    free(ds->samples);
    free(ds);
}

/**
 * fastbev_affine_transform - 用预计算的 6 参数做 BEV 坐标变换（调试用）
 *
 * x' = p[0]*x + p[1]*y + p[2]
 * y' = p[3]*x + p[4]*y + p[5]
 */
static inline void fastbev_affine_transform(
    const float params[AFFINE_PARAMS],
    float x, float y,
    float *xp, float *yp)
{
    *xp = params[0]*x + params[1]*y + params[2];
    *yp = params[3]*x + params[4]*y + params[5];
}

/**
 * fastbev_print_sample - 打印一帧的调试信息
 */
static void fastbev_print_sample(const FastBEVSample *s) {
    printf("── Sample [%d] ─────────────────────────────────\n", s->index);
    printf("  token      : %.8s...\n", s->sample_token);
    printf("  scene      : %s\n", s->scene_name);
    printf("  timestamp  : %lld\n", (long long)s->timestamp);
    printf("  ego pos    : (%.2f, %.2f)\n",
           s->ego_translation[0], s->ego_translation[1]);

    static const char *ch_names[NUM_CAMERAS] = {
        "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
        "CAM_BACK",  "CAM_BACK_LEFT",   "CAM_BACK_RIGHT"
    };
    for (int c = 0; c < NUM_CAMERAS; c++) {
        printf("  %-17s: %s", ch_names[c], s->cameras[c].image_path);
        if (s->cameras[c].has_lidar2img)
            printf("  [lidar2img ✓]");
        printf("\n");
    }

    printf("  历史帧:\n");
    for (int t = 0; t < NUM_HIST_FRAMES; t++) {
        const FastBEVTemporalFrame *f = &s->temporal[t];
        printf("    T-%d  affine=[%.5f, %.5f, %.4f, %.5f, %.5f, %.4f]  %s\n",
               t + 1,
               f->affine_params[0], f->affine_params[1], f->affine_params[2],
               f->affine_params[3], f->affine_params[4], f->affine_params[5],
               f->is_key_frame ? "(key)" : "(sweep)");
    }
    printf("\n");
}

#ifdef __cplusplus
}
#endif

#endif /* FASTBEV_READER_H */
