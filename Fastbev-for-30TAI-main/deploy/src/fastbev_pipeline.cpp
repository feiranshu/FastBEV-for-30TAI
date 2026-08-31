/*
 * FastBEV synchronous AXI pipeline.
 *
 * Active data path:
 *   raw BGR FP32 -> Part1 extractor -> runtime FEAT2D PLDDR
 *   -> Part2 LUT -> decoder view(13) Conv input v245 PLDDR
 *   -> Part3 decoder -> output readback -> threshold/NMS/export.
 *
 * Part2 writes INT8 data directly into the decoder-owned PLDDR region. The
 * CPU never reads, rearranges, or reuploads the Part2 result. The two NPU
 * resets remain explicit session boundaries between Part1 and Part3.
 *
 * FASTBEV_PIPELINE_SA builds a separate Group4 executable from this stable
 * implementation. The default target does not define that macro and retains
 * the original one-frame Part2 protocol.
 * FASTBEV_ENABLE_AUDIO adds post-NMS FPGA LED and non-blocking USB-audio
 * alerts in the dedicated fastbev_pipeline_audio target without changing the
 * inference dataflow.
 */

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstdarg>
#include <string>
#include <new>
#include <vector>
#include <memory>
#include <fstream>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <limits>
#include <tuple>
#include <sstream>
#include <array>
#include <cmath>
#include <cstring>

#ifndef _WIN32
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

/* ── 第三方依赖 ── */
#include "cJSON.h"
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

/* ── FastBEV 前/后处理 ── */
#include "fastbev_reader.h"
#include "fastbev_preprocess_cv.hpp"
#include "fastbev_export.hpp"
#include "types.hpp"
#include "filter.hpp"
#include "nms.hpp"
#ifdef FASTBEV_ENABLE_AUDIO
#include "alert_config.hpp"
#include "alert_manager.hpp"
#endif

/* ── iCraft XRT & NPU ── */
#include <icraft-xrt/core/session.h>
#include <icraft-xrt/dev/host_device.h>
#include <icraft-xrt/dev/buyi_device.h>
#include <icraft-xrt/dev/zg330_device.h>
#include <icraft-backends/buyibackend/buyibackend.h>
#include <icraft-backends/zg330backend/zg330backend.h>
#include <icraft-backends/hostbackend/cuda/device.h>
#include <icraft-backends/hostbackend/backend.h>
#include <icraft-backends/hostbackend/utils.h>
#include "icraft_utils.hpp"
#include "et_device.hpp"
#include "compile_fpai_target.hpp"

using namespace icraft::xrt;
using namespace icraft::xir;
using namespace fastbev;
using namespace fpai;
namespace fs = std::filesystem;
using Clock = std::chrono::high_resolution_clock;

struct PcPushConfig {
    bool enabled = false;
    bool visualize = true;
    std::string host;
    int port = 8092;
};

static std::string json_escape(const std::string& value) {
    std::string out;
    out.reserve(value.size() + 16);
    for (const char c : value) {
        switch (c) {
        case '\\': out += "\\\\"; break;
        case '"': out += "\\\""; break;
        case '\n': out += "\\n"; break;
        case '\r': out += "\\r"; break;
        case '\t': out += "\\t"; break;
        default: out += c; break;
        }
    }
    return out;
}

static std::string base64_encode(const std::vector<unsigned char>& bytes) {
    static constexpr char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string encoded;
    encoded.reserve(((bytes.size() + 2U) / 3U) * 4U);

    size_t i = 0;
    while (i + 2U < bytes.size()) {
        const uint32_t value = (static_cast<uint32_t>(bytes[i]) << 16U) |
                               (static_cast<uint32_t>(bytes[i + 1U]) << 8U) |
                               static_cast<uint32_t>(bytes[i + 2U]);
        encoded.push_back(kAlphabet[(value >> 18U) & 0x3FU]);
        encoded.push_back(kAlphabet[(value >> 12U) & 0x3FU]);
        encoded.push_back(kAlphabet[(value >> 6U) & 0x3FU]);
        encoded.push_back(kAlphabet[value & 0x3FU]);
        i += 3U;
    }
    if (i < bytes.size()) {
        const uint32_t value = static_cast<uint32_t>(bytes[i]) << 16U;
        encoded.push_back(kAlphabet[(value >> 18U) & 0x3FU]);
        if (i + 1U < bytes.size()) {
            const uint32_t with_second = value | (static_cast<uint32_t>(bytes[i + 1U]) << 8U);
            encoded.push_back(kAlphabet[(with_second >> 12U) & 0x3FU]);
            encoded.push_back(kAlphabet[(with_second >> 6U) & 0x3FU]);
            encoded.push_back('=');
        } else {
            encoded.push_back(kAlphabet[(value >> 12U) & 0x3FU]);
            encoded.append("==");
        }
    }
    return encoded;
}

static bool read_text_file(const char* path, std::string& content, size_t max_bytes) {
    std::ifstream file(path, std::ios::binary);
    if (!file.is_open()) return false;
    file.seekg(0, std::ios::end);
    const auto size = file.tellg();
    if (size < 0 || static_cast<uint64_t>(size) > max_bytes) return false;
    file.seekg(0, std::ios::beg);
    content.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
    return file.good() || file.eof();
}

static bool send_real_frame_to_pc(const PcPushConfig& cfg,
                                  int frame_id,
                                  const char* result_path,
                                  const char* param_path,
                                  const FastBEVDisplayImages& display_images,
                                  double& payload_build_ms,
                                  size_t& payload_bytes,
                                  double& push_ms,
                                  std::string& error)
{
    payload_build_ms = 0.0;
    payload_bytes = 0;
    push_ms = 0.0;
    error.clear();
    if (!cfg.enabled) return true;

#ifdef _WIN32
    error = "PC push is only implemented for the Linux board target.";
    return false;
#else
    std::string result_text;
    std::string param_text;
    if (!read_text_file(result_path, result_text, 2U * 1024U * 1024U) ||
        !read_text_file(param_path, param_text, 256U * 1024U)) {
        error = "cannot read result/camera parameter files for PC push";
        return false;
    }

    static constexpr const char* kCameraNames[NUM_CAMERAS] = {
        "CAM_FRONT", "CAM_FRONT_RIGHT", "CAM_FRONT_LEFT",
        "CAM_BACK", "CAM_BACK_LEFT", "CAM_BACK_RIGHT",
    };
    for (int cam = 0; cam < NUM_CAMERAS; ++cam) {
        if (display_images.jpeg[cam].empty()) {
            error = std::string("missing display preview for ") + kCameraNames[cam];
            return false;
        }
    }

    const auto payload_begin = Clock::now();
    std::ostringstream body;
    body << "{"
         << "\"frame_id\":\"";
    body.width(4);
    body.fill('0');
    body << frame_id << "\","
         << "\"source\":\"fastbev_pipeline_real\","
         << "\"result_text\":\"" << json_escape(result_text) << "\","
         << "\"camera_params_text\":\"" << json_escape(param_text) << "\","
         << "\"images\":{";
    for (int cam = 0; cam < NUM_CAMERAS; ++cam) {
        body << (cam == 0 ? "" : ",")
             << "\"" << kCameraNames[cam] << "\":\""
             << base64_encode(display_images.jpeg[cam]) << "\"";
    }
    body << "}}";
    const std::string payload = body.str();
    payload_build_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - payload_begin).count();
    payload_bytes = payload.size();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    addrinfo* addresses = nullptr;
    const std::string port = std::to_string(cfg.port);
    if (getaddrinfo(cfg.host.c_str(), port.c_str(), &hints, &addresses) != 0) {
        error = "getaddrinfo failed for " + cfg.host + ":" + port;
        return false;
    }

    int sock = -1;
    for (addrinfo* addr = addresses; addr != nullptr; addr = addr->ai_next) {
        sock = socket(addr->ai_family, addr->ai_socktype, addr->ai_protocol);
        if (sock < 0) continue;
        if (connect(sock, addr->ai_addr, addr->ai_addrlen) == 0) break;
        close(sock);
        sock = -1;
    }
    freeaddrinfo(addresses);
    if (sock < 0) {
        error = "cannot connect to " + cfg.host + ":" + port;
        return false;
    }

    std::ostringstream request;
    request << "POST /api/push_frame HTTP/1.1\r\n"
            << "Host: " << cfg.host << ":" << cfg.port << "\r\n"
            << "User-Agent: fastbev-pipeline/1.0\r\n"
            << "Content-Type: application/json\r\n"
            << "Content-Length: " << payload.size() << "\r\n"
            << "Connection: close\r\n\r\n"
            << payload;
    const std::string raw_request = request.str();

    const auto start = Clock::now();
    size_t sent = 0;
    while (sent < raw_request.size()) {
        const ssize_t count = send(sock, raw_request.data() + sent, raw_request.size() - sent, 0);
        if (count <= 0) {
            close(sock);
            error = "send failed";
            return false;
        }
        sent += static_cast<size_t>(count);
    }

    std::string response;
    char buf[1024];
    while (true) {
        const ssize_t count = recv(sock, buf, sizeof(buf), 0);
        if (count <= 0) break;
        response.append(buf, static_cast<size_t>(count));
    }
    close(sock);
    push_ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();

    std::istringstream response_stream(response);
    std::string protocol;
    int status = 0;
    response_stream >> protocol >> status;
    if (status < 200 || status >= 300) {
        error = "PC server returned HTTP " + std::to_string(status);
        return false;
    }
    return true;
#endif
}


/* ═══════════════════════════════════════════════════════════════════════════
 *  FPGA 寄存器地址定义（地址空间 0x400C_0000 ~ 0x400F_FFFF）
 * ═══════════════════════════════════════════════════════════════════════════ */
static constexpr uint32_t REG_BASE       = 0x400C0000;
static constexpr uint32_t REG_CTRL_START = REG_BASE + 0x00 * 4;
static constexpr uint32_t REG_LUT_BASE   = REG_BASE + 0x01 * 4;
static constexpr uint32_t REG_LUT_SIZE   = REG_BASE + 0x02 * 4;
static constexpr uint32_t REG_FEAT2D_BASE= REG_BASE + 0x03 * 4;
static constexpr uint32_t REG_FEAT3D_WR  = REG_BASE + 0x04 * 4;
static constexpr uint32_t REG_FEAT3D_SIZE= REG_BASE + 0x05 * 4;
static constexpr uint32_t REG_BEV_PARAMS = REG_BASE + 0x09 * 4;
static constexpr uint32_t REG_IMG_PARAMS = REG_BASE + 0x0A * 4;
static constexpr uint32_t REG_RESET      = REG_BASE + 0x77 * 4;
static constexpr uint32_t REG_COMP_DONE  = REG_BASE + 0x20 * 4;
static constexpr uint32_t REG_VERSION    = REG_BASE + 0x30 * 4;

#ifdef FASTBEV_PIPELINE_SA
// Group4 register map from task/fastbev_part2_group4_driver_example.c.
static constexpr uint32_t SA_REG_FRAME0_ADDR    = REG_BASE + 0x11 * 4;
static constexpr uint32_t SA_REG_FRAME1_ADDR    = REG_BASE + 0x12 * 4;
static constexpr uint32_t SA_REG_FRAME2_ADDR    = REG_BASE + 0x13 * 4;
static constexpr uint32_t SA_REG_FRAME_SIZE     = REG_BASE + 0x15 * 4;
static constexpr uint32_t SA_REG_STATUS         = REG_BASE + 0x21 * 4;
static constexpr uint32_t SA_REG_PERF_CNT_LO    = REG_BASE + 0x22 * 4;
static constexpr uint32_t SA_REG_PERF_CNT_HI    = REG_BASE + 0x23 * 4;
static constexpr uint32_t SA_REG_EXT_MODE       = REG_BASE + 0x24 * 4;
static constexpr uint32_t SA_REG_FEAT2D_FP32    = REG_BASE + 0x25 * 4;
static constexpr uint32_t SA_REG_FEAT2D_INT8    = REG_BASE + 0x26 * 4;
static constexpr uint32_t SA_REG_CONCAT_OUT     = REG_BASE + 0x27 * 4;
static constexpr uint32_t SA_REG_GROUP_STATUS   = REG_BASE + 0x28 * 4;
static constexpr uint32_t SA_REG_ERROR_STATUS   = REG_BASE + 0x2A * 4;
static constexpr uint32_t SA_REG_PERF_STAGE_SEL = REG_BASE + 0x2B * 4;
static constexpr uint32_t SA_REG_XFORM_H0       = REG_BASE + 0x40 * 4;
static constexpr uint32_t SA_REG_XFORM_H1       = REG_BASE + 0x46 * 4;
static constexpr uint32_t SA_REG_XFORM_H2       = REG_BASE + 0x4C * 4;
#endif


/* ═══════════════════════════════════════════════════════════════════════════
 *  工具函数
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  将 NPU 输出 Tensor 提取为 std::vector<float>
 * @note   要求 Tensor 数据类型为 FP32，否则抛出异常
 */
std::vector<float> tensor2Vector(const Tensor& tensor) {
    if (!tensor.hasData())
        throw std::runtime_error("[tensor2Vector] Tensor has no data.");
    if (!tensor.dtype()->element_dtype.isFP32())
        throw std::runtime_error("[tensor2Vector] Tensor dtype is not FP32.");

    uint64_t num_elem = tensor.dtype().numElements();
    std::vector<float> vec(num_elem);
    tensor.read(reinterpret_cast<char*>(vec.data()), 0, num_elem * sizeof(float));
    return vec;
}

/** @brief 计算两个时间点之间的毫秒数 */
inline double elapsed_ms(Clock::time_point t0, Clock::time_point t1) {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

/**
 * @brief  双写日志类：所有输出同时写入 stdout 和日志文件
 */
uint32_t checked_plddr_u32(uint64_t addr, const char* name) {
    if (addr > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error(std::string(name) + " PLDDR address exceeds 32-bit register range.");
    }
    if ((addr & 0x3F) != 0) {
        throw std::runtime_error(std::string(name) + " PLDDR address is not 64-byte aligned.");
    }
    return static_cast<uint32_t>(addr);
}

bool shape_is_decoder_int8_conv_input(const TensorType& dtype) {
    return dtype->shape.size() == 5 &&
           dtype->shape[0] == 1 &&
           dtype->shape[1] == 32 &&
           dtype->shape[2] == 200 &&
           dtype->shape[3] == 200 &&
           dtype->shape[4] == 32;
}

bool shape_is_extractor_fp32_input(const TensorType& dtype) {
    return dtype->shape.size() == 4 &&
           dtype->shape[0] == 6 &&
           dtype->shape[1] == 256 &&
           dtype->shape[2] == 704 &&
           dtype->shape[3] == 3;
}

#ifdef FASTBEV_PIPELINE_SA
struct SaGroupState {
    uint32_t raw = 0;
    uint32_t error = 0;
    uint32_t phase = 0;
    uint32_t history_mask = 0;
    uint32_t output_ready = 0;
};

struct SaPlddrRange {
    const char* name;
    uint32_t base;
    uint32_t bytes;
};

int32_t sa_q16_16(float value) {
    if (!std::isfinite(value)) {
        throw std::runtime_error("SA affine contains NaN or Inf.");
    }
    const double scaled = static_cast<double>(value) * 65536.0;
    if (scaled > static_cast<double>(std::numeric_limits<int32_t>::max()) ||
        scaled < static_cast<double>(std::numeric_limits<int32_t>::min())) {
        throw std::runtime_error("SA affine exceeds signed Q16.16 range.");
    }
    return static_cast<int32_t>(std::llround(scaled));
}

void sa_write_affine(FPAIDevice& dev, uint32_t base, const float affine[6]) {
    for (int i = 0; i < 6; ++i) {
        dev.defaultRegRegion().write(
            base + static_cast<uint32_t>(i) * 4,
            static_cast<uint32_t>(sa_q16_16(affine[i])), false);
    }
}

void sa_reset_group4(FPAIDevice& dev) {
    dev.defaultRegRegion().write(REG_RESET, 1, false);
    usleep(1000);
    dev.defaultRegRegion().write(REG_RESET, 0, false);
    usleep(1000);
}

SaGroupState sa_read_group_state(FPAIDevice& dev) {
    SaGroupState state;
    state.raw = static_cast<uint32_t>(dev.defaultRegRegion().read(SA_REG_GROUP_STATUS, false));
    state.error = static_cast<uint32_t>(dev.defaultRegRegion().read(SA_REG_ERROR_STATUS, false));
    state.phase = (state.raw >> 3) & 0x3U;
    state.history_mask = (state.raw >> 5) & 0x7U;
    state.output_ready = (state.raw >> 2) & 0x1U;
    return state;
}

void sa_validate_group_state(const SaGroupState& state, int frame_index) {
    static constexpr uint32_t kExpectedPhase[4] = {1, 2, 3, 0};
    static constexpr uint32_t kExpectedMask[4] = {1, 3, 7, 0};
    static constexpr uint32_t kExpectedReady[4] = {0, 0, 0, 1};
    if (state.error != 0 || (state.raw & 0x1U) != 0 ||
        state.phase != kExpectedPhase[frame_index] ||
        state.history_mask != kExpectedMask[frame_index] ||
        state.output_ready != kExpectedReady[frame_index]) {
        std::ostringstream oss;
        oss << "Part2 Group4 state mismatch after frame " << frame_index
            << ": raw=0x" << std::hex << state.raw
            << " error=0x" << state.error << std::dec
            << " phase=" << state.phase
            << " mask=" << state.history_mask
            << " ready=" << state.output_ready;
        throw std::runtime_error(oss.str());
    }
}

void sa_validate_plddr_ranges(const std::array<SaPlddrRange, 6>& ranges) {
    for (size_t i = 0; i < ranges.size(); ++i) {
        const uint64_t end = static_cast<uint64_t>(ranges[i].base) + ranges[i].bytes;
        if ((ranges[i].base & 0x3FU) != 0 || end > (1ULL << 32)) {
            throw std::runtime_error(std::string(ranges[i].name) +
                                     " has an invalid Group4 PLDDR range.");
        }
        for (size_t j = i + 1; j < ranges.size(); ++j) {
            const uint64_t i_end = static_cast<uint64_t>(ranges[i].base) + ranges[i].bytes;
            const uint64_t j_end = static_cast<uint64_t>(ranges[j].base) + ranges[j].bytes;
            if (ranges[i].base < j_end && ranges[j].base < i_end) {
                throw std::runtime_error(std::string("Group4 PLDDR overlap: ") +
                                         ranges[i].name + " and " + ranges[j].name);
            }
        }
    }
}
#endif

struct PlddrTensorBinding {
    Value value;
    MemChunk memchunk;
    uint64_t phy_addr = 0;
    uint64_t offset = 0;
    uint64_t bytes = 0;
};

class PipelineLogger {
public:
    FILE* fp = nullptr;

    bool open(const std::string& path) {
        fp = fopen(path.c_str(), "w");
        return fp != nullptr;
    }

    void close() {
        if (fp) { fclose(fp); fp = nullptr; }
    }

    void print(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
        if (fp) {
            va_start(args, fmt);
            vfprintf(fp, fmt, args);
            va_end(args);
            fflush(fp);
        }
    }

    ~PipelineLogger() { close(); }
};

#ifdef FASTBEV_PIPELINE_SA
// Optional SA velocity probes. Keep them disabled during normal runs because
// PORT0 scans the complete 720,000-element bbox output.
struct SaVelocityDebugConfig {
    bool enabled = false;
    int frame_id = 0;  // 0 means every frame.

    bool matches(int current_frame_id) const {
        return enabled && (frame_id == 0 || frame_id == current_frame_id);
    }
};

SaVelocityDebugConfig sa_velocity_debug_from_env() {
    SaVelocityDebugConfig config;
    const char* raw = std::getenv("FASTBEV_SA_VELOCITY_DEBUG");
    if (raw == nullptr || raw[0] == '\0') return config;

    config.enabled = true;
    if (std::strcmp(raw, "all") == 0 || std::strcmp(raw, "ALL") == 0 ||
        std::strcmp(raw, "0") == 0) {
        return config;
    }

    char* end = nullptr;
    const long selected_frame = std::strtol(raw, &end, 10);
    if (end == raw || *end != '\0' || selected_frame <= 0 ||
        selected_frame > std::numeric_limits<int>::max()) {
        throw std::runtime_error(
            "FASTBEV_SA_VELOCITY_DEBUG must be 'all', 0, or a positive frame id.");
    }
    config.frame_id = static_cast<int>(selected_frame);
    return config;
}

struct SaScalarStats {
    size_t finite = 0;
    size_t nan = 0;
    size_t inf = 0;
    size_t zero = 0;
    size_t positive = 0;
    size_t negative = 0;
    float min = std::numeric_limits<float>::infinity();
    float max = -std::numeric_limits<float>::infinity();
    double sum = 0.0;
    double sum_abs = 0.0;

    void add(float value) {
        if (std::isnan(value)) {
            ++nan;
            return;
        }
        if (!std::isfinite(value)) {
            ++inf;
            return;
        }
        ++finite;
        if (value == 0.0f) ++zero;
        else if (value > 0.0f) ++positive;
        else ++negative;
        if (value < min) min = value;
        if (value > max) max = value;
        sum += value;
        sum_abs += std::fabs(static_cast<double>(value));
    }

    double mean() const { return finite == 0 ? 0.0 : sum / finite; }
    double mean_abs() const { return finite == 0 ? 0.0 : sum_abs / finite; }
    float safe_min() const { return finite == 0 ? 0.0f : min; }
    float safe_max() const { return finite == 0 ? 0.0f : max; }
};

void sa_log_scalar_stats(PipelineLogger& log, const char* prefix,
                         const SaScalarStats& stats) {
    log.print("%s finite=%zu nan=%zu inf=%zu zero=%zu pos=%zu neg=%zu "
              "min=%.9g max=%.9g mean=%.9g mean_abs=%.9g\n",
              prefix, stats.finite, stats.nan, stats.inf, stats.zero,
              stats.positive, stats.negative, stats.safe_min(), stats.safe_max(),
              stats.mean(), stats.mean_abs());
}

// PORT0 observes the Decoder bbox tensor before threshold/decode. The expected
// logical order is [100*100][8 anchors][9 codes], with vx/vy at code 7/8.
void sa_debug_bbox_port0(PipelineLogger& log, int frame_id,
                         const std::vector<float>& bbox_data) {
    static constexpr size_t kCells = 100U * 100U;
    static constexpr size_t kAnchors = 8U;
    static constexpr size_t kCodes = 9U;
    static constexpr size_t kExpectedElements = kCells * kAnchors * kCodes;
    static constexpr const char* kCodeNames[kCodes] = {
        "dx", "dy", "dz", "log_w", "log_l", "log_h", "yaw", "vx", "vy"
    };

    const size_t tuple_count = bbox_data.size() / kCodes;
    const size_t trailing = bbox_data.size() % kCodes;
    log.print("  [Debug SA velocity][PORT0 raw_bbox] frame=%04d elements=%zu "
              "expected=%zu tuples=%zu trailing=%zu layout=[10000][8][9]\n",
              frame_id, bbox_data.size(), kExpectedElements, tuple_count, trailing);

    std::array<SaScalarStats, kCodes> code_stats;
    std::array<SaScalarStats, kAnchors> anchor_vx_stats;
    std::array<SaScalarStats, kAnchors> anchor_vy_stats;
    for (size_t tuple = 0; tuple < tuple_count; ++tuple) {
        const size_t base = tuple * kCodes;
        for (size_t code = 0; code < kCodes; ++code) {
            code_stats[code].add(bbox_data[base + code]);
        }
        const size_t anchor = tuple % kAnchors;
        anchor_vx_stats[anchor].add(bbox_data[base + 7]);
        anchor_vy_stats[anchor].add(bbox_data[base + 8]);
    }

    for (size_t code = 0; code < kCodes; ++code) {
        char prefix[128];
        std::snprintf(prefix, sizeof(prefix),
                      "  [Debug SA velocity][PORT0 code%zu:%s]",
                      code, kCodeNames[code]);
        sa_log_scalar_stats(log, prefix, code_stats[code]);
    }
    for (size_t anchor = 0; anchor < kAnchors; ++anchor) {
        const SaScalarStats& vx = anchor_vx_stats[anchor];
        const SaScalarStats& vy = anchor_vy_stats[anchor];
        log.print("  [Debug SA velocity][PORT0 anchor%zu] "
                  "vx_zero=%zu/%zu vx=[%.9g,%.9g] vx_mean_abs=%.9g "
                  "vy_zero=%zu/%zu vy=[%.9g,%.9g] vy_mean_abs=%.9g\n",
                  anchor, vx.zero, vx.finite, vx.safe_min(), vx.safe_max(), vx.mean_abs(),
                  vy.zero, vy.finite, vy.safe_min(), vy.safe_max(), vy.mean_abs());
    }
}

// PORT1/PORT2 observe decoded boxes before and after NMS respectively.
void sa_debug_boxes_port(PipelineLogger& log, int frame_id, const char* port,
                         const std::vector<BoundingBox>& boxes,
                         bool print_samples) {
    SaScalarStats vx_stats;
    SaScalarStats vy_stats;
    SaScalarStats speed_stats;
    size_t both_zero = 0;
    for (const BoundingBox& box : boxes) {
        vx_stats.add(box.vx);
        vy_stats.add(box.vy);
        if (box.vx == 0.0f && box.vy == 0.0f) ++both_zero;
        speed_stats.add(std::hypot(box.vx, box.vy));
    }

    log.print("  [Debug SA velocity][%s] frame=%04d boxes=%zu both_zero=%zu "
              "vx=[%.9g,%.9g] vx_mean_abs=%.9g "
              "vy=[%.9g,%.9g] vy_mean_abs=%.9g "
              "speed_mean=%.9g speed_max=%.9g\n",
              port, frame_id, boxes.size(), both_zero,
              vx_stats.safe_min(), vx_stats.safe_max(), vx_stats.mean_abs(),
              vy_stats.safe_min(), vy_stats.safe_max(), vy_stats.mean_abs(),
              speed_stats.mean(), speed_stats.safe_max());

    if (!print_samples) return;
    const size_t sample_count = boxes.size() < 10 ? boxes.size() : 10;
    for (size_t i = 0; i < sample_count; ++i) {
        const BoundingBox& box = boxes[i];
        log.print("  [Debug SA velocity][%s sample%zu] class=%d score=%.6f "
                  "xy=(%.6f,%.6f) v=(%.9g,%.9g) speed=%.9g\n",
                  port, i, box.id, box.score, box.x, box.y,
                  box.vx, box.vy, std::hypot(box.vx, box.vy));
    }
}
#endif


/* ═══════════════════════════════════════════════════════════════════════════
 *  主程序
 * ═══════════════════════════════════════════════════════════════════════════ */
int main(int argc, char* argv[])
{
    if (argc < 2) {
        fprintf(stderr,
                "Usage: %s <config.yaml> [--push-host <pc-ip>] [--push-port <port>] [--no-visualize]\n",
                argv[0]);
        return 1;
    }

    try {

    PcPushConfig pc_push;
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--push-host") {
            if (++i >= argc) throw std::runtime_error("missing value for --push-host");
            pc_push.enabled = true;
            pc_push.host = argv[i];
        } else if (arg == "--push-port") {
            if (++i >= argc) throw std::runtime_error("missing value for --push-port");
            pc_push.port = std::stoi(argv[i]);
        } else if (arg == "--no-visualize") {
            pc_push.visualize = false;
        } else if (arg == "--help" || arg == "-h") {
            fprintf(stdout,
                    "Usage: %s <config.yaml> [--push-host <pc-ip>] [--push-port <port>] [--no-visualize]\n",
                    argv[0]);
            return 0;
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }
    if (pc_push.enabled && pc_push.host.empty()) {
        throw std::runtime_error("--push-host must not be empty");
    }
    if (pc_push.port <= 0 || pc_push.port > 65535) {
        throw std::runtime_error("--push-port must be in 1..65535");
    }

    /* ==================================================================
     * 1. 解析 YAML 配置
     * ================================================================== */
    YAML::Node config = YAML::LoadFile(argv[1]);

    // -- 设备（NPU 参数统一，extractor/decoder 共用） --
    auto dev_cfg = config["device"];
    std::string device_url  = dev_cfg["url"].as<std::string>();
    bool   dev_mmuMode      = dev_cfg["mmuMode"].as<bool>(true);
    bool   dev_speedMode    = dev_cfg["speedMode"].as<bool>(false);
    bool   dev_compressFtmp = dev_cfg["compressFtmp"].as<bool>(false);
    int    dev_ocmOption    = dev_cfg["ocm_option"].as<int>(-1);

    // -- Extractor (Part1) --
    auto ext_cfg = config["extractor"];
    std::string ext_dir     = ext_cfg["dir"].as<std::string>();
    std::string ext_stage   = ext_cfg["stage"].as<std::string>();
    std::string ext_backend = ext_cfg["run_backend"].as<std::string>();

    // -- Decoder (Part3) --
    auto dec_cfg = config["decoder"];
    std::string dec_dir     = dec_cfg["dir"].as<std::string>();
    std::string dec_stage   = dec_cfg["stage"].as<std::string>();
    std::string dec_backend = dec_cfg["run_backend"].as<std::string>();

    // -- 数据集 & IO 路径 --
    auto ds_cfg = config["dataset"];
    std::string json_path = ds_cfg["imageDir"].as<std::string>();
    std::string lut_path  = ds_cfg["lutDir"].as<std::string>();
    std::string box_dir   = ds_cfg["boxDir"].as<std::string>();
    std::string png_dir   = ds_cfg["pngDir"].as<std::string>();
    std::string para_dir  = ds_cfg["paraDir"].as<std::string>();
    std::string log_dir   = ds_cfg["logDir"].as<std::string>();
    const int CAMERAS     = ds_cfg["camera"].as<int>();
    const int IMG_W       = ds_cfg["imageW"].as<int>();
    const int IMG_H       = ds_cfg["imageH"].as<int>();

    // -- BEV 空间参数 --
    auto bev_cfg = config["bev"];
    const int BEV_X      = bev_cfg["bevx"].as<int>();
    const int BEV_Y      = bev_cfg["bevy"].as<int>();
    const int BEV_Z      = bev_cfg["bevz"].as<int>();
    const int CHANNELS   = bev_cfg["channels"].as<int>();
    const int FP32_BYTES = bev_cfg["fp32Bytes"].as<int>();

    // -- NMS --
    auto nms_cfg = config["nms"];
    float nms_threshold = nms_cfg["threshold"].as<float>();
    std::vector<float> nms_threslist = nms_cfg["threslist"].as<std::vector<float>>();

    // -- 派生常量 --
    const int LUT_COUNT        = BEV_X * BEV_Y * BEV_Z;
    const int FEAT2D_BYTES     = CAMERAS * IMG_H * IMG_W * CHANNELS * FP32_BYTES;
    const int LUT_BYTES        = LUT_COUNT * 8;
    // v245 stores four 64-channel BEV groups as int8: 200*200*4*64*4 bytes.
    // The decoder view exposes the same storage as [1,32,200,200,32].
    const int DECODER_V245_GROUPS = 4;
    const int DECODER_INT8_BYTES = LUT_COUNT * CHANNELS * DECODER_V245_GROUPS;
#ifdef FASTBEV_PIPELINE_SA
    const int SA_FEAT2D_INT8_BYTES = FEAT2D_BYTES / FP32_BYTES;
    const int SA_HISTORY_BYTES = LUT_COUNT * CHANNELS;
#endif

    /* ==================================================================
     * 2. 初始化日志 & 创建输出目录
     * ================================================================== */
    fs::create_directories(box_dir);
    fs::create_directories(png_dir);
    fs::create_directories(para_dir);
    fs::create_directories(log_dir);

    PipelineLogger log;
    std::string log_file = log_dir + "/log.txt";
    if (!log.open(log_file)) {
        fprintf(stderr, "[Warning] Cannot open log file: %s\n", log_file.c_str());
    }

#ifdef FASTBEV_PIPELINE_SA
    const SaVelocityDebugConfig sa_velocity_debug = sa_velocity_debug_from_env();
    if (sa_velocity_debug.enabled) {
        if (sa_velocity_debug.frame_id == 0) {
            log.print("[Debug SA velocity] enabled for every frame via "
                      "FASTBEV_SA_VELOCITY_DEBUG\n");
        } else {
            log.print("[Debug SA velocity] enabled for frame %04d via "
                      "FASTBEV_SA_VELOCITY_DEBUG\n",
                      sa_velocity_debug.frame_id);
        }
    }
#endif

#ifdef FASTBEV_ENABLE_AUDIO
    // Sound and light alerts consume final NMS boxes only. The LED backend
    // writes the FPGA alert registers through /dev/mem; audio playback runs on
    // its own worker thread and never blocks the NPU/FPGA frame loop.
    std::unique_ptr<AlertPolicy> alert_policy;
    std::unique_ptr<AlertLedController> alert_led;
    std::unique_ptr<AlertAudioPlayer> alert_audio;
    try {
        const AlertRuntimeConfig alert_cfg = load_alert_runtime_config(config);
        if (alert_cfg.enabled) {
            alert_policy.reset(new AlertPolicy(alert_cfg.policy));
            if (alert_cfg.led.enabled) {
                // Use the same controller and register protocol validated by
                // the standalone LED bring-up path. Route mapping/capability
                // failures into the pipeline log so a silent LED failure is
                // distinguishable from policy.
                alert_led.reset(new AlertLedController(
                    alert_cfg.led,
                    AlertLedController::RegisterObserver{},
                    [&log](const std::string& message) {
                        log.print("[Alert LED] backend error: %s\n", message.c_str());
                    }));
                log.print("[Alert LED] base=0x%08llX duration=%u ms "
                          "danger_toggle=%u ms emergency_toggle=%u ms\n",
                          static_cast<unsigned long long>(
                              alert_cfg.led.register_base),
                          alert_cfg.led.duration_ms,
                          alert_cfg.led.danger_toggle_ms,
                          alert_cfg.led.emergency_toggle_ms);
            }
            if (alert_cfg.audio.enabled) {
                alert_audio.reset(new AlertAudioPlayer(alert_cfg.audio));
            }
            log.print("[Alert] enabled: LED=%s (available=%s), audio=%s, "
                      "device=%s, cooldown=%d ms\n",
                      alert_cfg.led.enabled ? "on" : "off",
                      alert_led && alert_led->available() ? "yes" : "no",
                      alert_cfg.audio.enabled ? "on" : "off",
                      alert_cfg.audio.device.c_str(), alert_cfg.audio.cooldown_ms);
        } else {
            log.print("[Alert] disabled by configuration\n");
        }
    } catch (const std::exception& e) {
        alert_policy.reset();
        alert_led.reset();
        alert_audio.reset();
        log.print("[Alert] invalid configuration; subsystem disabled: %s\n", e.what());
    }
#endif

    log.print("============================================================\n");
#ifdef FASTBEV_PIPELINE_SA
    log.print("    FastBEV Pipeline SA  (Group4 synchronous validation)\n");
#elif defined(FASTBEV_ENABLE_AUDIO)
    log.print("    FastBEV Pipeline AUDIO+LED  (USB ALSA + FPGA alerts)\n");
#else
    log.print("    FastBEV Pipeline  (AXI Mode, No-FTMP)\n");
#endif
    log.print("============================================================\n");
    log.print("[Config] device_url : %s\n", device_url.c_str());
    log.print("[Config] mmuMode=%d  speedMode=%d  compressFtmp=%d  ocm=%d\n",
              dev_mmuMode, dev_speedMode, dev_compressFtmp, dev_ocmOption);
    log.print("[Config] extractor  : %s (stage=%s, backend=%s)\n",
              ext_dir.c_str(), ext_stage.c_str(), ext_backend.c_str());
    log.print("[Config] decoder    : %s (stage=%s, backend=%s)\n",
              dec_dir.c_str(), dec_stage.c_str(), dec_backend.c_str());
    const std::string pc_push_endpoint = pc_push.enabled
        ? pc_push.host + ":" + std::to_string(pc_push.port)
        : "disabled";
    log.print("[Config] PC push    : %s\n", pc_push_endpoint.c_str());
    log.print("[Config] visualize  : %s\n", pc_push.visualize ? "enabled" : "disabled");
    log.print("[Config] LUT        : %s\n", lut_path.c_str());
    log.print("[Config] BEV space  : %dx%dx%d, %dch\n", BEV_X, BEV_Y, BEV_Z, CHANNELS);
    log.print("[Config] NMS thr    : %.2f\n\n", nms_threshold);

    /* ==================================================================
     * 3. 打开设备（AXI 模式，Part1/2/3 共用同一 Device）
     * ================================================================== */
    log.print("[Init] Opening device...\n");
    auto device = Device::Open(device_url.c_str());
    auto fpai_dev = device.cast<FPAIDevice>();
    log.print("[Init] Device opened successfully\n");

    /* ==================================================================
     * 4. 初始化 Extractor Session (Part1 NPU)
     * ================================================================== */
    log.print("[Init] Loading extractor network...\n");
    auto ext_jr_path = getJrPath(ext_backend, ext_dir, ext_stage);
    Network ext_network = loadNetwork(ext_jr_path.first, ext_jr_path.second);
    NetInfo ext_netinfo = NetInfo(ext_network);
    auto ext_inputs = ext_network.inputs();
    if (ext_inputs.size() != 1 ||
        !shape_is_extractor_fp32_input(ext_inputs[0].tensorType()) ||
        !ext_inputs[0].tensorType()->element_dtype.isFP32()) {
        throw std::runtime_error(
            "extractor must expose one FP32 NHWC input with shape [6,256,704,3] for Parser preprocessing.");
    }

    Session extractor = initSession(
        ext_backend, ext_network, device,
        dev_ocmOption, ext_netinfo.mmu || dev_mmuMode,
        dev_speedMode, dev_compressFtmp);
    extractor.apply();
    log.print("[Init] Extractor Parser preprocessing: BGR -> SwapOrder/Add/Multiply -> normalized RGB\n");
    log.print("[Init] Extractor session ready\n");

    /* ==================================================================
     * 5. 初始化 Decoder Session (Part3 NPU)
     * ================================================================== */
    log.print("[Init] Loading decoder network...\n");
    auto dec_jr_path = getJrPath(dec_backend, dec_dir, dec_stage);
    Network dec_network = loadNetwork(dec_jr_path.first, dec_jr_path.second);
    NetInfo dec_netinfo = NetInfo(dec_network);
    // View removes the decoder prefix so Part2 can write its INT8 result to
    // the first Conv input (v245) without a PS-side layout conversion.
    NetworkView dec_network_view = dec_network.view(13);
    log.print("[Init] Decoder uses runtime view(13), Conv input v245 by SDI-style PLIN path\n");

    Session decoder_sess = initSession(
        dec_backend, dec_network_view, device,
        dev_ocmOption, dec_netinfo.mmu || dev_mmuMode,
        dev_speedMode, dev_compressFtmp);
    decoder_sess.apply();
    log.print("[Init] Decoder session ready\n");

    // Bind the Tensor to the decoder-owned PLDDR segment before any frame runs.
    // Part2 only receives this physical byte address; it must not allocate or own it.
    PlddrTensorBinding dec_conv_input;
    if (dec_backend != "zg330") {
        throw std::runtime_error("decoder.run_backend must be zg330 for the v245 PLDDR path.");
    }
        auto view_input_val = dec_network_view.inputs()[0];
        auto view_input_type = view_input_val.tensorType();
        if (!shape_is_decoder_int8_conv_input(view_input_type)) {
            throw std::runtime_error("decoder view input must be Conv input shape [1,32,200,200,32].");
        }
        if (!view_input_type->element_dtype.getStorageType().isSInt(8)) {
            throw std::runtime_error("decoder view input storage dtype must be sint8.");
        }

        auto forwards = decoder_sess.getForwards();
        if (forwards.empty()) {
            throw std::runtime_error("decoder view session has no forwards.");
        }
        auto backend = std::get<1>(forwards[0]);
        if (!backend.is<FPAIBackend>()) {
            throw std::runtime_error("decoder view first forward is not on ZG330/FPAI backend.");
        }
        auto device_backend = backend.cast<FPAIBackend>();
        auto input_op = std::get<0>(forwards[0]);
        if (input_op->inputs.size() == 0) {
            throw std::runtime_error("decoder view first hardop has no input value.");
        }
        const int64_t vid = input_op->inputs[0]->v_id;
        auto value_info = device_backend->forward_info->value_map.at(vid);
        auto memchunk = device_backend->forward_info->memchunk_map.at(vid)->memChunk;

        dec_conv_input.value = value_info->value;
        dec_conv_input.memchunk = memchunk;
        dec_conv_input.phy_addr = value_info->phy_addr;
        dec_conv_input.offset = value_info->phy_addr - memchunk->begin.addr();
        dec_conv_input.bytes = view_input_type.numElements();
        if (vid != 245) {
            log.print("[Init] WARNING: decoder view first input v_id is %lld, expected 245\n",
                      (long long)vid);
        }
        if (dec_conv_input.bytes != DECODER_INT8_BYTES) {
            throw std::runtime_error("decoder Conv input byte size mismatch.");
        }
        checked_plddr_u32(dec_conv_input.phy_addr, "decoder Conv input");
        log.print("[Init] Decoder Conv input from forwards[0].inputs[0]: v_id=%lld\n",
                  (long long)vid);
    log.print("[Init] Decoder Conv input PLDDR addr=0x%08X, mem=0x%08X, offset=%llu, bytes=%llu\n",
              (uint32_t)dec_conv_input.phy_addr,
              (uint32_t)dec_conv_input.memchunk->begin.addr(),
              (unsigned long long)dec_conv_input.offset,
              (unsigned long long)dec_conv_input.bytes);

    /* ==================================================================
     * 6. FPGA 初始化：复位、PL DDR 分配、LUT 加载
     * ================================================================== */
    log.print("[FPGA] Resetting custom op...\n");
    fpai_dev.defaultRegRegion().write(REG_RESET, 1, false);
    usleep(1000);
    fpai_dev.defaultRegRegion().write(REG_RESET, 0, false);
    usleep(1000);

    uint32_t fpga_ver = (uint32_t)fpai_dev.defaultRegRegion().read(REG_VERSION, false);
    log.print("[FPGA] Version: 0x%08X\n", fpga_ver);

#ifdef FASTBEV_PIPELINE_SA
    log.print("[FPGA] Allocating Group4 PLDDR buffers...\n");
#else
    log.print("[FPGA] Allocating LUT PLDDR...\n");
#endif
    auto lut_mem = fpai_dev.defaultMemRegion().malloc(LUT_BYTES, 0, 64);
#ifdef FASTBEV_PIPELINE_SA
    // Group4 retains three aligned BEV histories and uses one temporary INT8
    // FEAT2D buffer. CONCAT_OUT is the decoder-owned v245 allocation.
    auto sa_feat2d_int8_mem = fpai_dev.defaultMemRegion().malloc(SA_FEAT2D_INT8_BYTES, 0, 64);
    auto sa_history0_mem = fpai_dev.defaultMemRegion().malloc(SA_HISTORY_BYTES, 0, 64);
    auto sa_history1_mem = fpai_dev.defaultMemRegion().malloc(SA_HISTORY_BYTES, 0, 64);
    auto sa_history2_mem = fpai_dev.defaultMemRegion().malloc(SA_HISTORY_BYTES, 0, 64);

    const uint32_t lut_addr = checked_plddr_u32(lut_mem->begin.addr(), "Group4 LUT");
    const uint32_t feat2d_int8_addr = checked_plddr_u32(
        sa_feat2d_int8_mem->begin.addr(), "Group4 FEAT2D INT8");
    const uint32_t history0_addr = checked_plddr_u32(
        sa_history0_mem->begin.addr(), "Group4 history0");
    const uint32_t history1_addr = checked_plddr_u32(
        sa_history1_mem->begin.addr(), "Group4 history1");
    const uint32_t history2_addr = checked_plddr_u32(
        sa_history2_mem->begin.addr(), "Group4 history2");
    const uint32_t concat_out_addr = checked_plddr_u32(
        dec_conv_input.phy_addr, "Group4 CONCAT_OUT/decoder v245");
    sa_validate_plddr_ranges({{{"LUT", lut_addr, static_cast<uint32_t>(LUT_BYTES)},
                               {"FEAT2D_INT8", feat2d_int8_addr, static_cast<uint32_t>(SA_FEAT2D_INT8_BYTES)},
                               {"HISTORY0", history0_addr, static_cast<uint32_t>(SA_HISTORY_BYTES)},
                               {"HISTORY1", history1_addr, static_cast<uint32_t>(SA_HISTORY_BYTES)},
                               {"HISTORY2", history2_addr, static_cast<uint32_t>(SA_HISTORY_BYTES)},
                               {"CONCAT_OUT", concat_out_addr, static_cast<uint32_t>(DECODER_INT8_BYTES)}}});
#endif
    log.print("[FPGA] PL DDR allocated:\n");
    log.print("  LUT:    0x%08X  (%d B)\n", (uint32_t)lut_mem->begin.addr(),  LUT_BYTES);
    log.print("  FEAT2D: extractor Runtime Tensor address at runtime (%d B)\n", FEAT2D_BYTES);
#ifdef FASTBEV_PIPELINE_SA
    log.print("  FEAT2D INT8: 0x%08X  (%d B)\n", feat2d_int8_addr, SA_FEAT2D_INT8_BYTES);
    log.print("  HISTORY0:    0x%08X  (%d B)\n", history0_addr, SA_HISTORY_BYTES);
    log.print("  HISTORY1:    0x%08X  (%d B)\n", history1_addr, SA_HISTORY_BYTES);
    log.print("  HISTORY2:    0x%08X  (%d B)\n", history2_addr, SA_HISTORY_BYTES);
    log.print("  CONCAT_OUT:  0x%08X  (%d B, decoder v245)\n",
              concat_out_addr, DECODER_INT8_BYTES);
#endif

    log.print("[FPGA] Loading LUT table: %s\n", lut_path.c_str());
    {
        std::ifstream lut_ifs(lut_path, std::ios::binary);
        if (!lut_ifs.is_open()) {
            log.print("[Error] Cannot open LUT: %s\n", lut_path.c_str());
            Device::Close(device);
            return -1;
        }
        std::vector<char> lut_buf(LUT_BYTES);
        lut_ifs.read(lut_buf.data(), LUT_BYTES);
        lut_ifs.close();
        lut_mem.write(0, lut_buf.data(), LUT_BYTES);
        log.print("[FPGA] LUT loaded (%d entries, %d B)\n", LUT_COUNT, LUT_BYTES);
    }

    fpai_dev.defaultRegRegion().write(REG_LUT_BASE,   (uint32_t)lut_mem->begin.addr(), false);
    fpai_dev.defaultRegRegion().write(REG_LUT_SIZE,   LUT_COUNT,    false);
    fpai_dev.defaultRegRegion().write(REG_BEV_PARAMS, 0x04C8C840,   false);
    fpai_dev.defaultRegRegion().write(REG_IMG_PARAMS, 0x060400B0,   false);
#ifdef FASTBEV_PIPELINE_SA
    fpai_dev.defaultRegRegion().write(SA_REG_ERROR_STATUS, 0xFFFFFFFFU, false);
    fpai_dev.defaultRegRegion().write(SA_REG_FRAME0_ADDR, history0_addr, false);
    fpai_dev.defaultRegRegion().write(SA_REG_FRAME1_ADDR, history1_addr, false);
    fpai_dev.defaultRegRegion().write(SA_REG_FRAME2_ADDR, history2_addr, false);
    fpai_dev.defaultRegRegion().write(SA_REG_FRAME_SIZE, SA_HISTORY_BYTES, false);
    fpai_dev.defaultRegRegion().write(SA_REG_EXT_MODE, 1, false);
    fpai_dev.defaultRegRegion().write(SA_REG_FEAT2D_INT8, feat2d_int8_addr, false);
    fpai_dev.defaultRegRegion().write(SA_REG_CONCAT_OUT, concat_out_addr, false);
#endif
    log.print("[FPGA] Registers configured\n\n");

    /* ==================================================================
     * 7. 加载数据集 & 分配前处理内存
     * ================================================================== */
    log.print("[Dataset] Loading: %s\n", json_path.c_str());
    FastBEVDataset *ds = fastbev_load(json_path.c_str());
    if (!ds) {
        log.print("[Error] Failed to load dataset\n");
        Device::Close(device);
        return -1;
    }
    const int total_frames = ds->num_samples;
    const int frames_to_process = total_frames;

    FastBEVFrameInput *inp = new (std::nothrow) FastBEVFrameInput;
    if (!inp) {
        log.print("[Error] OOM: FastBEVFrameInput\n");
        fastbev_free(ds);
        Device::Close(device);
        return -1;
    }

    log.print("[Dataset] %d frames, BEV %dx%dx%d, %d cameras (%dx%d)\n",
              total_frames, BEV_X, BEV_Y, BEV_Z, CAMERAS, IMG_W, IMG_H);
    log.print("[Memory] FastBEVFrameInput: %zu MB\n\n",
              sizeof(FastBEVFrameInput) / (1024 * 1024));

    log.print("============================================================\n");
    log.print("    Start Processing\n");
    log.print("============================================================\n\n");

    /* ==================================================================
     * 8. 主循环：逐帧执行全流水线
     * ================================================================== */
    for (int fi = 0; fi < frames_to_process; fi++) {
        const int frame_id = fi + 1;
        FastBEVSample *sample = &ds->samples[fi];

        log.print("══════════ Frame %04d / %d ══════════\n", frame_id, total_frames);
        if (sample->is_first_in_scene)
            log.print("  [Scene] %s (first frame in scene)\n", sample->scene_name);

        auto t_frame_start = Clock::now();
        FastBEVDisplayImages display_images;

#ifdef FASTBEV_PIPELINE_SA
        // Group4 order is temporal[0], temporal[1], temporal[2], current.
        // Part2 consumes each runtime tensor before the next Part1 reuses it.
        double sa_ms_preprocess = 0.0;
        double sa_ms_part1 = 0.0;
        double sa_ms_part2 = 0.0;
        double sa_ms_input_tensor = 0.0;

        fpai_dev.defaultRegRegion().write(SA_REG_ERROR_STATUS, 0xFFFFFFFFU, false);
        const uint32_t affine_bases[3] = {
            SA_REG_XFORM_H0, SA_REG_XFORM_H1, SA_REG_XFORM_H2
        };
        for (int h = 0; h < NUM_HIST_FRAMES; ++h) {
            sa_write_affine(fpai_dev, affine_bases[h], sample->temporal[h].affine_params);
            log.print("  [SA] H%d affine=[%.6f %.6f %.6f %.6f %.6f %.6f] Q16.16=[",
                      h,
                      sample->temporal[h].affine_params[0], sample->temporal[h].affine_params[1],
                      sample->temporal[h].affine_params[2], sample->temporal[h].affine_params[3],
                      sample->temporal[h].affine_params[4], sample->temporal[h].affine_params[5]);
            for (int p = 0; p < AFFINE_PARAMS; ++p) {
                log.print("%s%d", p == 0 ? "" : " ", sa_q16_16(sample->temporal[h].affine_params[p]));
            }
            log.print("]\n");
        }

        for (int group_frame = 0; group_frame < 4; ++group_frame) {
            FastBEVSample history_sample;
            FastBEVSample* model_sample = sample;
            const char* input_name = "current";
            if (group_frame < NUM_HIST_FRAMES) {
                history_sample = *sample;
                std::memcpy(history_sample.cameras,
                            sample->temporal[group_frame].cameras,
                            sizeof(history_sample.cameras));
                model_sample = &history_sample;
                input_name = group_frame == 0 ? "temporal[0]" :
                             group_frame == 1 ? "temporal[1]" : "temporal[2]";
            }

            log.print("  [SA %d/4] Preprocessing %s (%d cameras)...\n",
                      group_frame + 1, input_name, CAMERAS);
            FastBEVPreprocessTiming group_preprocess_timing;
            const auto preprocess_begin = Clock::now();
            const int prep_fail = fastbev_prepare_frame_input(
                model_sample, inp, &group_preprocess_timing,
                (group_frame == 3 && pc_push.enabled) ? &display_images : nullptr);
            sa_ms_preprocess += elapsed_ms(preprocess_begin, Clock::now());
            if (prep_fail > 0) {
                log.print("  [SA %d/4] WARNING: %d camera images failed\n",
                          group_frame + 1, prep_fail);
            }
            log.print("  [SA %d/4] preprocess: imread=%.2f resize+crop=%.2f pack=%.2f ms\n",
                      group_frame + 1,
                      group_preprocess_timing.imread_ms,
                      group_preprocess_timing.resize_ms,
                      group_preprocess_timing.bgr_pack_nhwc_ms);

            const auto input_tensor_begin = Clock::now();
            auto ext_in_val = ext_network.inputs()[0];
            Tensor ext_in_tensor = data2Tensor<float>(inp->current_tensor, ext_in_val);
            const double input_tensor_ms = elapsed_ms(input_tensor_begin, Clock::now());
            sa_ms_input_tensor += input_tensor_ms;

            const auto part1_begin = Clock::now();
            std::vector<Tensor> feat_2d = extractor.forward({ext_in_tensor});
            if (feat_2d.size() != 1) {
                throw std::runtime_error("Extractor must return exactly one FEAT2D tensor.");
            }
            if (!feat_2d[0].waitForReady(std::chrono::seconds(10))) {
                throw std::runtime_error("Extractor output wait timeout in Group4.");
            }
            sa_ms_part1 += elapsed_ms(part1_begin, Clock::now());

            const uint32_t runtime_feat2d_addr = checked_plddr_u32(
                feat_2d[0].data().addr(), "Group4 extractor output");
            log.print("  [SA %d/4] Part1 done: inputTensor=%.2f ms FEAT2D=0x%08X\n",
                      group_frame + 1, input_tensor_ms, runtime_feat2d_addr);

            const auto part2_begin = Clock::now();
            fpai_dev.defaultRegRegion().write(SA_REG_FEAT2D_FP32, runtime_feat2d_addr, false);
            fpai_dev.defaultRegRegion().write(REG_CTRL_START, 1, false);
            usleep(10);

            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            while (true) {
                const uint32_t done = static_cast<uint32_t>(
                    fpai_dev.defaultRegRegion().read(REG_COMP_DONE, false));
                if ((done & 1U) != 0) break;
                if (std::chrono::steady_clock::now() > deadline) {
                    sa_reset_group4(fpai_dev);
                    throw std::runtime_error("Part2 Group4 timeout.");
                }
                usleep(100);
            }

            const SaGroupState group_state = sa_read_group_state(fpai_dev);
            sa_ms_part2 += elapsed_ms(part2_begin, Clock::now());
            log.print("  [SA %d/4] Part2 done: status=0x%08X error=0x%08X "
                      "phase=%u mask=0x%X ready=%u\n",
                      group_frame + 1, group_state.raw, group_state.error,
                      group_state.phase, group_state.history_mask,
                      group_state.output_ready);
            try {
                sa_validate_group_state(group_state, group_frame);
            } catch (...) {
                sa_reset_group4(fpai_dev);
                throw;
            }

            if (ext_backend != "host") {
                log.print("  [SA %d/4] Resetting single-core NPU before %s...\n",
                          group_frame + 1,
                          group_frame == 3 ? "Decoder" : "next Part1");
                device.reset(1);
            }
        }

        log.print("  [SA] Group4 output ready in decoder v245: 0x%08X (%d B)\n",
                  concat_out_addr, DECODER_INT8_BYTES);
        static constexpr const char* kPerfNames[4] = {"total", "quant", "LUT", "SA"};
        for (uint32_t perf = 0; perf < 4; ++perf) {
            fpai_dev.defaultRegRegion().write(SA_REG_PERF_STAGE_SEL, perf, false);
            const uint32_t lo = static_cast<uint32_t>(
                fpai_dev.defaultRegRegion().read(SA_REG_PERF_CNT_LO, false));
            const uint32_t hi = static_cast<uint32_t>(
                fpai_dev.defaultRegRegion().read(SA_REG_PERF_CNT_HI, false));
            const uint64_t cycles = (static_cast<uint64_t>(hi) << 32) | lo;
            log.print("  [SA] PERF %-5s: %llu cycles\n",
                      kPerfNames[perf], static_cast<unsigned long long>(cycles));
        }
#else

        /* ── 8.1 前处理 ── */
        log.print("  [Step 1] Running image preprocessing (%d cameras)...\n", CAMERAS);
        auto t0 = Clock::now();
        FastBEVPreprocessTiming preprocess_timing;
        int prep_fail = fastbev_prepare_frame_input(
            sample, inp, &preprocess_timing, pc_push.enabled ? &display_images : nullptr);
        auto t1 = Clock::now();
        if (prep_fail > 0)
            log.print("  [Step 1] Warning: %d camera images failed, filled with zeros\n", prep_fail);
        else
            log.print("  [Step 1] Preprocessing complete\n");
        log.print("  [Step 1] Timing: imread6=%.2f ms, resize+crop6=%.2f ms, "
                  "FP32 BGR pack=%.2f ms, previewJPEG=%.2f ms, affine=%.3f ms\n",
                  preprocess_timing.imread_ms, preprocess_timing.resize_ms,
                  preprocess_timing.bgr_pack_nhwc_ms,
                  preprocess_timing.preview_encode_ms, preprocess_timing.affine_ms);

        /* ── 8.2 Part1: Extractor NPU 推理 ── */
        log.print("  [Step 2] Building input tensor for extractor...\n");
        const auto input_tensor_begin = Clock::now();
        auto ext_in_val = ext_network.inputs()[0];
        Tensor ext_in_tensor = data2Tensor<float>(inp->current_tensor, ext_in_val);
        const double input_tensor_ms = elapsed_ms(input_tensor_begin, Clock::now());
        log.print("  [Step 2] Extractor input data2Tensor: %.2f ms\n", input_tensor_ms);

        log.print("  [Step 2] Running extractor.forward (Part1 NPU)...\n");
        auto t2 = Clock::now();
        std::vector<Tensor> feat_2d = extractor.forward({ ext_in_tensor });
        for (auto& out_t : feat_2d) {
            if (!out_t.waitForReady(std::chrono::seconds(10)))
                log.print("  [Step 2] ERROR: Extractor output wait timeout!\n");
        }
        auto t3 = Clock::now();
        log.print("  [Step 2] Extractor inference complete\n");

        const uint64_t runtime_feat2d_addr = feat_2d.at(0).data().addr();
        checked_plddr_u32(runtime_feat2d_addr, "Extractor runtime output");
        log.print("  [Step 2] Runtime Tensor PLDDR directly feeds Part2: 0x%08X (%d B)\n",
                  (uint32_t)runtime_feat2d_addr, FEAT2D_BYTES);
        /* ── 8.3 Part2: LUT maps FEAT2D directly into decoder v245 PLDDR ── */
        log.print("  [Step 3] Configuring FPGA LUT engine...\n");
        auto t4 = Clock::now();
        // Part1 owns FEAT2D until its forward has completed; the FPGA consumes
        // the runtime tensor's physical address directly, without Tensor::read().
        fpai_dev.defaultRegRegion().write(
            REG_FEAT2D_BASE, checked_plddr_u32(runtime_feat2d_addr, "FEAT2D Runtime Tensor"), false);
        const uint32_t part2_out_addr = checked_plddr_u32(dec_conv_input.phy_addr, "decoder Conv input");
        fpai_dev.defaultRegRegion().write(
            REG_FEAT3D_WR,   part2_out_addr,  false);
        fpai_dev.defaultRegRegion().write(
            REG_FEAT3D_SIZE, DECODER_INT8_BYTES, false);
        fpai_dev.defaultRegRegion().write(
            REG_CTRL_START,  0x03, false);
        log.print("  [Step 3] Part2 INT8 output directly written to decoder Conv input PLDDR\n");
        log.print("  [Step 3] FEAT3D_WR=0x%08X, FEAT3D_SIZE=%d\n",
                  part2_out_addr, DECODER_INT8_BYTES);

        log.print("  [Step 3] LUT engine started, polling for completion...\n");
        {
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
            while (true) {
                int done = fpai_dev.defaultRegRegion().read(REG_COMP_DONE, false);
                if (done & 1) break;
                if (std::chrono::steady_clock::now() > deadline) {
                    log.print("  [Step 3] ERROR: LUT engine timeout!\n");
                    delete inp;
                    fastbev_free(ds);
                    Device::Close(device);
                    return -1;
                }
                usleep(100);
            }
        }
        auto t5 = Clock::now();
        log.print("  [Step 3] Part2 complete\n");

        if (ext_backend != "host") {
            log.print("  [Step 3] Resetting NPU after Part2 before Decoder session...\n");
            device.reset(1);
        }
#endif

        /* ── 8.4 Part3: Decoder NPU 推理 ── */
        log.print("  [Step 4] Building input tensor for decoder...\n");
        // Reuse the decoder's own input segment. setData describes existing
        // PLDDR storage, so Session::forward does not upload a host tensor.
        Tensor dec_in_tensor(dec_conv_input.value);
        dec_in_tensor.setData(dec_conv_input.memchunk, dec_conv_input.offset);

        log.print("  [Step 4] Running decoder.forward (Part3 NPU)...\n");
        auto t6 = Clock::now();
        std::vector<Tensor> dec_output = decoder_sess.forward({ dec_in_tensor });
        for (auto& out_t : dec_output) {
            if (!out_t.waitForReady(std::chrono::seconds(10)))
                log.print("  [Step 4] ERROR: Decoder output wait timeout!\n");
        }
        auto t7 = Clock::now();
        log.print("  [Step 4] Decoder inference complete\n");

        // 提取三个输出
        log.print("  [Step 4] Extracting decoder outputs (cls/bbox/dir)...\n");
        std::vector<float> cls_data  = tensor2Vector(dec_output[0]);
        std::vector<float> bbox_data = tensor2Vector(dec_output[1]);
        std::vector<float> dir_data  = tensor2Vector(dec_output[2]);
        log.print("  [Step 4] Outputs: cls=%zu, bbox=%zu, dir=%zu elements\n",
                  cls_data.size(), bbox_data.size(), dir_data.size());

#ifdef FASTBEV_PIPELINE_SA
        if (sa_velocity_debug.matches(frame_id)) {
            sa_debug_bbox_port0(log, frame_id, bbox_data);
        }
#endif

        if (dec_backend != "host") {
            device.reset(1);
        }

        /* ── 8.5 后处理：解码 + NMS + 导出 ── */
        log.print("  [Step 5] Running postprocess (threshold + decode)...\n");
        auto t8 = Clock::now();

        std::vector<BoundingBox> candidates = filter::threshold_and_decode(
            cls_data.data(), bbox_data.data(), dir_data.data(), nms_threshold);
        log.print("  [Step 5] Candidates after threshold: %zu\n", candidates.size());
#ifdef FASTBEV_PIPELINE_SA
        if (sa_velocity_debug.matches(frame_id)) {
            sa_debug_boxes_port(log, frame_id, "PORT1 candidates", candidates, false);
        }
#endif

        NMSConfig nms_config;
        nms_config.score_thr     = nms_threshold;
        nms_config.nms_thr_list  = nms_threslist;
        std::vector<BoundingBox> final_boxes =
            nms::run_multi_class_nms(candidates, nms_config);
        log.print("  [Step 5] Final detections after NMS: %zu\n", final_boxes.size());
#ifdef FASTBEV_PIPELINE_SA
        if (sa_velocity_debug.matches(frame_id)) {
            sa_debug_boxes_port(log, frame_id, "PORT2 nms_final", final_boxes, true);
        }
#endif

#ifdef FASTBEV_ENABLE_AUDIO
        if (alert_policy) {
            const AlertDecision alert = alert_policy->evaluate(final_boxes, frame_id);
            if (alert_led) alert_led->update(alert);
            if (alert_audio) alert_audio->update(alert);
            if (alert.level == AlertLevel::None) {
                if (alert.state_changed) {
                    log.print("  [Alert] level=none (changed)\n");
                }
            } else {
                log.print("  [Alert] level=%s class=%s direction=%s "
                          "clearance=%.3f m score=%.3f%s\n",
                          AlertPolicy::level_name(alert.level),
                          AlertPolicy::class_name(alert.class_id),
                          alert.direction.c_str(), alert.clearance_m, alert.score,
                          alert.state_changed ? " (changed)" : "");
            }
        }
#endif

        // 导出 result txt
        char result_path[512];
        snprintf(result_path, sizeof(result_path),
                 "%s/result_%04d.txt", box_dir.c_str(), frame_id);
        {
            std::ofstream ofs(result_path);
            if (ofs.is_open()) {
                for (const auto& b : final_boxes) {
                    ofs << b.x   << " " << b.y   << " " << b.z << " "
                        << b.w   << " " << b.l   << " " << b.h << " "
                        << b.yaw << " ";
#ifdef FASTBEV_PIPELINE_SA
                    // SA downstream consumers use the extended 11-column row:
                    // x y z w l h yaw vx vy class_id score.
                    ofs << b.vx << " " << b.vy << " ";
#endif
                    ofs << b.id << " " << b.score << "\n";
                }
                log.print("  [Step 5] Result saved: %s\n", result_path);
            } else {
                log.print("  [Step 5] ERROR: Cannot write: %s\n", result_path);
            }
        }

        // 导出 camera params txt
        char param_path[512];
        snprintf(param_path, sizeof(param_path),
                 "%s/camera_params_%04d.txt", para_dir.c_str(), frame_id);
        if (fastbev_export_camera_params(sample, param_path) != 0) {
            log.print("  [Step 5] ERROR: Camera params export failed\n");
        } else {
            log.print("  [Step 5] Camera params saved: %s\n", param_path);
        }

        // 导出可视化 png
        char png_path[512];
        snprintf(png_path, sizeof(png_path),
                 "%s/output_%04d.png", png_dir.c_str(), frame_id);
        if (pc_push.visualize) {
            char cmd[2048];
            snprintf(cmd, sizeof(cmd), "./visualize \"%s\" \"%s\" \"%s\"",
                     param_path, result_path, png_path);
            int vis_ret = system(cmd);
            if (vis_ret != 0)
                log.print("  [Step 5] Warning: visualize returned %d\n", vis_ret);
            else
                log.print("  [Step 5] Output image saved: %s\n", png_path);
        }

        if (pc_push.enabled) {
            double payload_build_ms = 0.0;
            size_t payload_bytes = 0;
            double push_ms = 0.0;
            std::string push_error;
            if (send_real_frame_to_pc(pc_push, frame_id, result_path, param_path, display_images,
                                      payload_build_ms, payload_bytes, push_ms, push_error)) {
                log.print("  [Step 5] Real result + 6 board previews pushed: frame=%04d, "
                          "payload=%.1f KB, base64=%.2f ms, HTTP=%.2f ms\n",
                          frame_id, payload_bytes / 1024.0, payload_build_ms, push_ms);
            } else {
                log.print("  [Step 5] WARNING: PC push failed for frame %04d: %s\n",
                          frame_id, push_error.c_str());
            }
        }

        auto t9 = Clock::now();
        auto t_frame_end = Clock::now();

        /* ── 8.6 本帧计时汇总（统一在循环末尾打印） ── */
#ifdef FASTBEV_PIPELINE_SA
        double ms_preprocess = sa_ms_preprocess;
        double ms_part1      = sa_ms_part1;
        double ms_part2      = sa_ms_part2;
#else
        double ms_preprocess = elapsed_ms(t0, t1);
        double ms_part1      = elapsed_ms(t2, t3);
        double ms_part2      = elapsed_ms(t4, t5);
#endif
        double ms_part3      = elapsed_ms(t6, t7);
        double ms_postproc   = elapsed_ms(t8, t9);
        double ms_total      = elapsed_ms(t_frame_start, t_frame_end);

        log.print("  ┌──────────── Timing Summary ────────────┐\n");
        log.print("  │ ① Preprocess      : %9.2f ms       │\n", ms_preprocess);
        log.print("  │ ② Part1 NPU (ext) : %9.2f ms       │\n", ms_part1);
        log.print("  │ ③ Part2 LUT+concat: %9.2f ms       │\n", ms_part2);
#ifdef FASTBEV_PIPELINE_SA
        log.print("  │    Input data2Tensor: %9.2f ms       │\n", sa_ms_input_tensor);
#endif
        log.print("  │ ④ Part3 NPU (dec) : %9.2f ms       │\n", ms_part3);
        log.print("  │ ⑤ Postprocess     : %9.2f ms       │\n", ms_postproc);
        log.print("  │ ⑥ Frame Total     : %9.2f ms       │\n", ms_total);
        log.print("  └─────────────────────────────────────────┘\n\n");
    }

    /* ==================================================================
     * 9. 清理资源
     * ================================================================== */
    log.print("============================================================\n");
    log.print("  Pipeline complete: %d frames processed\n", frames_to_process);
    log.print("  Log saved to: %s\n", log_file.c_str());
    log.print("============================================================\n");

#ifdef FASTBEV_ENABLE_AUDIO
    if (alert_led) alert_led->clear();
    if (alert_audio) alert_audio->stop();
#endif
    delete inp;
    fastbev_free(ds);
    Device::Close(device);
    return 0;

    } catch (const std::exception& e) {
        fprintf(stderr, "[Fatal] %s\n", e.what());
        return -1;
    }
}
