#pragma once

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace fastbev_vehicle {

constexpr int kNumCameras = 6;
constexpr int kInputH = 480;
constexpr int kInputW = 640;
constexpr int kChannels = 3;
constexpr std::size_t kOneCameraElements =
    static_cast<std::size_t>(kInputH) * kInputW * kChannels;
constexpr std::size_t kTensorElements =
    static_cast<std::size_t>(kNumCameras) * kOneCameraElements;

static const std::array<const char*, kNumCameras> kCameraNames = {
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_FRONT_LEFT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_BACK_RIGHT",
};

static const std::array<const char*, kNumCameras> kDefaultImageFiles = {
    "0-FRONT.jpg",
    "1-FRONT_RIGHT.jpg",
    "2-FRONT_LEFT.jpg",
    "3-BACK.jpg",
    "4-BACK_LEFT.jpg",
    "5-BACK_RIGHT.jpg",
};

struct VehiclePreprocessTiming {
    double imread_ms = 0.0;
    double pack_ms = 0.0;
    double total_ms = 0.0;
};

inline double elapsed_ms(std::chrono::steady_clock::time_point begin,
                         std::chrono::steady_clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

inline void zero_camera(float* out_hwc) {
    std::fill(out_hwc, out_hwc + kOneCameraElements, 0.0f);
}

inline int pack_bgr_640x480_to_nhwc(const cv::Mat& image_bgr,
                                    float* out_hwc,
                                    VehiclePreprocessTiming* timing = nullptr) {
    if (out_hwc == nullptr) {
        std::fprintf(stderr, "[vehicle-preprocess] output buffer is null\n");
        return -1;
    }
    if (image_bgr.empty()) {
        std::fprintf(stderr, "[vehicle-preprocess] input image is empty\n");
        zero_camera(out_hwc);
        return -1;
    }
    if (image_bgr.rows != kInputH || image_bgr.cols != kInputW ||
        image_bgr.channels() != kChannels) {
        std::fprintf(stderr,
                     "[vehicle-preprocess] expected BGR %dx%d, got %dx%d c=%d\n",
                     kInputW, kInputH, image_bgr.cols, image_bgr.rows,
                     image_bgr.channels());
        zero_camera(out_hwc);
        return -1;
    }

    const auto begin = std::chrono::steady_clock::now();
    for (int y = 0; y < kInputH; ++y) {
        const unsigned char* src = image_bgr.ptr<unsigned char>(y);
        float* dst = out_hwc + static_cast<std::size_t>(y) * kInputW * kChannels;
        for (int x = 0; x < kInputW; ++x) {
            const unsigned char* px = src + static_cast<std::size_t>(x) * kChannels;
            float* out = dst + static_cast<std::size_t>(x) * kChannels;
            out[0] = static_cast<float>(px[0]);
            out[1] = static_cast<float>(px[1]);
            out[2] = static_cast<float>(px[2]);
        }
    }
    if (timing) {
        timing->pack_ms += elapsed_ms(begin, std::chrono::steady_clock::now());
    }
    return 0;
}

inline int prepare_from_mats(const std::array<cv::Mat, kNumCameras>& images_bgr,
                             float* out_tensor,
                             VehiclePreprocessTiming* timing = nullptr) {
    if (out_tensor == nullptr) {
        std::fprintf(stderr, "[vehicle-preprocess] output tensor is null\n");
        return -1;
    }

    const auto total_begin = std::chrono::steady_clock::now();
    int failures = 0;
    for (int cam = 0; cam < kNumCameras; ++cam) {
        float* dst = out_tensor + static_cast<std::size_t>(cam) * kOneCameraElements;
        if (pack_bgr_640x480_to_nhwc(images_bgr[cam], dst, timing) != 0) {
            std::fprintf(stderr,
                         "[vehicle-preprocess] camera %d (%s) failed; filled zeros\n",
                         cam, kCameraNames[cam]);
            ++failures;
        }
    }
    if (timing) {
        timing->total_ms += elapsed_ms(total_begin, std::chrono::steady_clock::now());
    }
    return failures;
}

inline int prepare_from_paths(const std::array<std::string, kNumCameras>& image_paths,
                              float* out_tensor,
                              std::array<cv::Mat, kNumCameras>* display_images = nullptr,
                              VehiclePreprocessTiming* timing = nullptr) {
    if (out_tensor == nullptr) {
        std::fprintf(stderr, "[vehicle-preprocess] output tensor is null\n");
        return -1;
    }

    const auto total_begin = std::chrono::steady_clock::now();
    int failures = 0;
    for (int cam = 0; cam < kNumCameras; ++cam) {
        float* dst = out_tensor + static_cast<std::size_t>(cam) * kOneCameraElements;
        const auto read_begin = std::chrono::steady_clock::now();
        cv::Mat image_bgr = cv::imread(image_paths[cam], cv::IMREAD_COLOR);
        if (timing) {
            timing->imread_ms += elapsed_ms(read_begin, std::chrono::steady_clock::now());
        }
        if (display_images) {
            (*display_images)[cam] = image_bgr;
        }
        if (pack_bgr_640x480_to_nhwc(image_bgr, dst, timing) != 0) {
            std::fprintf(stderr,
                         "[vehicle-preprocess] camera %d (%s) failed: %s; filled zeros\n",
                         cam, kCameraNames[cam], image_paths[cam].c_str());
            ++failures;
        }
    }
    if (timing) {
        timing->total_ms += elapsed_ms(total_begin, std::chrono::steady_clock::now());
    }
    return failures;
}

inline std::array<std::string, kNumCameras> default_paths_for_directory(
    const std::string& directory) {
    std::array<std::string, kNumCameras> paths;
    for (int cam = 0; cam < kNumCameras; ++cam) {
        paths[cam] = directory;
        if (!paths[cam].empty() && paths[cam].back() != '/' && paths[cam].back() != '\\') {
            paths[cam] += "/";
        }
        paths[cam] += kDefaultImageFiles[cam];
    }
    return paths;
}

}  // namespace fastbev_vehicle
