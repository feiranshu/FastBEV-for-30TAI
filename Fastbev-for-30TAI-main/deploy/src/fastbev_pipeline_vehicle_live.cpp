/*
 * Vehicle live Raw BGR FastBEV service.
 *
 *   six corrected 640x480 Raw BGR frames from TCP -> raw BGR NHWC FP32
 *   -> Vehicle Part1 NPU -> FEAT2D PLDDR
 *   -> Vehicle Part2 FPGA LUT + FP16 reorder
 *   -> Decoder view(13) v362 runtime PLDDR -> Vehicle Part3 NPU
 *   -> vehicle postprocess -> optional audio warning -> VEH1 JSON result
 *
 * This executable is intentionally independent from CARLA, BEV1, LED,
 * and the SD-card single-frame vehicle demo.
 */

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <spawn.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <zlib.h>

#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

#include "fastbev_vehicle_preprocess.hpp"
#include "fastbev_vehicle_postprocess.hpp"

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
using namespace fpai;

extern char** environ;

namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

constexpr uint32_t kMaxPacketBytes = 64U * 1024U * 1024U;
constexpr size_t kWireHeaderBytes = 32;
constexpr size_t kCameraEntryBytes = 32;
constexpr uint16_t kProtocolVersion = 1;
constexpr uint16_t kMessageBatch = 1;
constexpr uint16_t kMessageResult = 2;

constexpr int64_t kDecoderViewStart = 13;
constexpr int64_t kDecoderInputValueId = 362;
constexpr uint32_t kExpectedFpgaVersion = 0x20260818;

constexpr int kFeatureCameras = 6;
constexpr int kFeatureHeight = 120;
constexpr int kFeatureWidth = 160;
constexpr int kFeatureChannels = 64;
constexpr uint64_t kFeatureBytes =
    static_cast<uint64_t>(kFeatureCameras) * kFeatureHeight *
    kFeatureWidth * kFeatureChannels * sizeof(float);

constexpr uint64_t kDecoderInputBytes = 20480000ULL;
constexpr uint64_t kDecoderClsElements = 20000ULL;
constexpr uint64_t kDecoderBboxElements = 180000ULL;
constexpr uint64_t kDecoderDirectionElements = 40000ULL;

constexpr uint32_t kRegBase = 0x400C0000;
constexpr uint32_t kRegCtrlStart = kRegBase + 0x00 * 4;
constexpr uint32_t kRegLutBase = kRegBase + 0x01 * 4;
constexpr uint32_t kRegLutSize = kRegBase + 0x02 * 4;
constexpr uint32_t kRegFeat2dBase = kRegBase + 0x03 * 4;
constexpr uint32_t kRegFeat3dWrite = kRegBase + 0x04 * 4;
constexpr uint32_t kRegFeat3dSize = kRegBase + 0x05 * 4;
constexpr uint32_t kRegBevParams = kRegBase + 0x09 * 4;
constexpr uint32_t kRegImgParams = kRegBase + 0x0A * 4;
constexpr uint32_t kRegDone = kRegBase + 0x20 * 4;
constexpr uint32_t kRegVersion = kRegBase + 0x30 * 4;
constexpr uint32_t kRegReset = kRegBase + 0x77 * 4;

constexpr uint32_t kVehicleBevParams = 0x04C8C840; // z=4, y=200, x=200
constexpr uint32_t kVehicleImgParams = 0x060780A0; // cameras=6, h=120, w=160
constexpr float kVehicleCarMinCenterDistanceCm = 5.0f;

struct Arguments {
    std::string config_path;
    std::string host = "0.0.0.0";
    int port = 5200;
    std::string source = "vehicle-real-edge";
};

struct DecoderBinding {
    Value value;
    MemChunk memchunk;
    uint64_t phy_addr = 0;
    uint64_t offset = 0;
    uint64_t bytes = 0;
};

struct VehicleAudioConfig {
    bool enabled = false;
    std::string device = "default";
    std::string directory = "./assets/alerts";
    std::string file = "attention.wav";
    int cooldown_ms = 3000;
};

struct VehicleAlertConfig {
    bool enabled = false;
    float range_cm = 13.0f;
    VehicleAudioConfig audio;
};

struct PipelineConfig {
    std::string device_url;
    bool mmu_mode = true;
    bool speed_mode = false;
    bool compress_ftmp = false;
    int ocm_option = -1;

    std::string extractor_dir;
    std::string extractor_stage = "g";
    std::string extractor_backend = "zg330";
    std::string decoder_dir;
    std::string decoder_stage = "g";
    std::string decoder_backend = "zg330";

    std::string sample_dir;
    std::string camera_params;
    std::string lut_path;
    std::string output_dir;
    bool visualize = true;

    fastbev::vehicle_postprocess::VehiclePostprocessConfig post;
    VehicleAlertConfig alert;
};

class ProtocolError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct RawCameraBatch {
    uint64_t frame_id = 0;
    uint64_t capture_ts_ns = 0;
    std::array<std::vector<uint8_t>, fastbev_vehicle::kNumCameras> raw_bgr;
};

struct FrameTiming {
    double preprocess_ms = 0.0;
    double input_tensor_ms = 0.0;
    double part1_ms = 0.0;
    double part2_ms = 0.0;
    double part3_ms = 0.0;
    double postprocess_ms = 0.0;
    double total_ms = 0.0;
};

double elapsed_ms(Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

uint64_t unix_time_ns() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

Arguments parse_arguments(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error(
            "usage: fastbev_pipeline_vehicle_live <config.yaml> "
            "[--host <address>] [--port <port>] [--source <name>]");
    }
    Arguments args;
    args.config_path = argv[1];
    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--host" && i + 1 < argc) {
            args.host = argv[++i];
        } else if (arg == "--port" && i + 1 < argc) {
            args.port = std::stoi(argv[++i]);
        } else if (arg == "--source" && i + 1 < argc) {
            args.source = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::printf("usage: %s <config.yaml> [--host <address>] "
                        "[--port <port>] [--source <name>]\n", argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error("unknown or incomplete argument: " + arg);
        }
    }
    if (args.host.empty()) throw std::runtime_error("--host must not be empty");
    if (args.port <= 0 || args.port > 65535) {
        throw std::runtime_error("--port must be in 1..65535");
    }
    if (args.source.empty()) throw std::runtime_error("--source must not be empty");
    return args;
}

uint32_t checked_plddr_u32(uint64_t address, const char* name) {
    if (address > std::numeric_limits<uint32_t>::max()) {
        throw std::runtime_error(std::string(name) + " exceeds 32-bit PLDDR register range");
    }
    if ((address & 0x3FU) != 0) {
        throw std::runtime_error(std::string(name) + " is not 64-byte aligned");
    }
    return static_cast<uint32_t>(address);
}

bool extractor_input_shape_ok(const TensorType& type) {
    return type->shape.size() == 4 && type->shape[0] == 6 &&
           type->shape[1] == 480 && type->shape[2] == 640 &&
           type->shape[3] == 3;
}

bool decoder_input_shape_ok(const TensorType& type) {
    return type->shape.size() == 5 && type->shape[0] == 1 &&
           type->shape[1] == 16 && type->shape[2] == 200 &&
           type->shape[3] == 200 && type->shape[4] == 16;
}

std::vector<float> tensor_to_vector(const Tensor& tensor,
                                    uint64_t expected_elements,
                                    const char* output_name) {
    if (!tensor.hasData()) {
        throw std::runtime_error(std::string("decoder ") + output_name +
                                 " output Tensor has no data");
    }
    if (!tensor.dtype()->element_dtype.isFP32()) {
        throw std::runtime_error(std::string("decoder ") + output_name +
                                 " output Tensor is not FP32");
    }
    const uint64_t elements = tensor.dtype().numElements();
    if (elements != expected_elements) {
        std::ostringstream error;
        error << "decoder " << output_name << " output has " << elements
              << " elements, expected " << expected_elements;
        throw std::runtime_error(error.str());
    }
    std::vector<float> values(elements);
    tensor.read(reinterpret_cast<char*>(values.data()), 0, elements * sizeof(float));
    return values;
}

std::string join_path(const std::string& dir, const std::string& file) {
    fs::path path(dir);
    path /= file;
    return path.generic_string();
}

class VehicleAudioAlert {
public:
    explicit VehicleAudioAlert(const VehicleAlertConfig& config)
        : config_(config),
          fallback_wav_path_(join_path(config.audio.directory, config.audio.file)) {}

    ~VehicleAudioAlert() {
        reap_child();
    }

    void update(const std::vector<fastbev::BoundingBox>& boxes, float score_threshold) {
        reap_child();
        if (!config_.enabled || !config_.audio.enabled) return;

        const fastbev::BoundingBox* best_box = nullptr;
        float best_clearance_cm = std::numeric_limits<float>::infinity();
        for (const auto& box : boxes) {
            if (box.score < score_threshold) continue;
            const float center_cm = std::hypot(box.x, box.y);
            const float box_half_diag_cm =
                0.5f * std::hypot(std::max(0.0f, box.w), std::max(0.0f, box.l));
            const float clearance_cm = std::max(0.0f, center_cm - box_half_diag_cm);
            if (clearance_cm <= config_.range_cm &&
                clearance_cm < best_clearance_cm) {
                best_clearance_cm = clearance_cm;
                best_box = &box;
            }
        }
        if (best_box == nullptr) return;

        const auto now = Clock::now();
        if (played_once_) {
            const double since_last_ms = elapsed_ms(last_play_time_, now);
            if (since_last_ms < static_cast<double>(config_.audio.cooldown_ms)) {
                return;
            }
        }
        const std::string direction_wav_path = wav_path_for_box(*best_box);
        start_playback(now, best_clearance_cm, direction_wav_path);
    }

    const std::string& wav_path() const {
        return fallback_wav_path_;
    }

private:
    static const char* class_name(int class_id) {
        switch (class_id) {
            case 0: return "car";
            case 1: return "truck";
            case 2: return "trailer";
            case 3: return "bus";
            case 4: return "construction_vehicle";
            case 5: return "bicycle";
            case 6: return "motorcycle";
            case 7: return "pedestrian";
            case 8: return "traffic_cone";
            case 9: return "barrier";
            default: return nullptr;
        }
    }

    static const char* direction_name(const fastbev::BoundingBox& box) {
        const float abs_x = std::fabs(box.x);
        const float abs_y = std::fabs(box.y);
        if (abs_y <= abs_x * 0.5f) {
            return box.x >= 0.0f ? "front" : "rear";
        }
        if (box.x >= 0.0f) {
            return box.y >= 0.0f ? "front_left" : "front_right";
        }
        return box.y >= 0.0f ? "rear_left" : "rear_right";
    }

    std::string wav_path_for_box(const fastbev::BoundingBox& box) const {
        const char* cls = class_name(box.id);
        if (cls == nullptr) return fallback_wav_path_;
        return join_path(config_.audio.directory,
                         std::string(direction_name(box)) + "_" + cls + ".wav");
    }

    void reap_child() {
        if (active_pid_ <= 0) return;
        int status = 0;
        const pid_t rc = waitpid(active_pid_, &status, WNOHANG);
        if (rc == active_pid_) {
            if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
                std::fprintf(stderr, "[VehicleAudio] aplay exited with status=%d\n", status);
            }
            active_pid_ = -1;
        }
    }

    void start_playback(Clock::time_point now,
                        float clearance_cm,
                        const std::string& preferred_wav_path) {
        if (active_pid_ > 0) return;
        std::string wav_path = preferred_wav_path;
        if (!fs::exists(wav_path)) {
            std::fprintf(stderr,
                         "[VehicleAudio] wav not found: %s; fallback=%s\n",
                         wav_path.c_str(), fallback_wav_path_.c_str());
            wav_path = fallback_wav_path_;
            if (!fs::exists(wav_path)) {
                std::fprintf(stderr, "[VehicleAudio] wav not found: %s\n", wav_path.c_str());
                played_once_ = true;
                last_play_time_ = now;
                return;
            }
        }

        pid_t pid = -1;
        char* const argv[] = {
            const_cast<char*>("aplay"),
            const_cast<char*>("-q"),
            const_cast<char*>("-D"),
            const_cast<char*>(config_.audio.device.c_str()),
            const_cast<char*>(wav_path.c_str()),
            nullptr,
        };
        const int rc = posix_spawnp(&pid, "aplay", nullptr, nullptr, argv, environ);
        if (rc != 0) {
            std::fprintf(stderr, "[VehicleAudio] cannot start aplay: %s\n", std::strerror(rc));
            played_once_ = true;
            last_play_time_ = now;
            return;
        }

        active_pid_ = pid;
        played_once_ = true;
        last_play_time_ = now;
        std::printf("[VehicleAudio] attention clearance=%.1f cm wav=%s\n",
                    static_cast<double>(clearance_cm), wav_path.c_str());
    }

    VehicleAlertConfig config_;
    std::string fallback_wav_path_;
    pid_t active_pid_ = -1;
    bool played_once_ = false;
    Clock::time_point last_play_time_ = Clock::now();
};

PipelineConfig load_config(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    PipelineConfig cfg;

    const auto device = root["device"];
    cfg.device_url = device["url"].as<std::string>();
    cfg.mmu_mode = device["mmuMode"].as<bool>(true);
    cfg.speed_mode = device["speedMode"].as<bool>(false);
    cfg.compress_ftmp = device["compressFtmp"].as<bool>(false);
    cfg.ocm_option = device["ocm_option"].as<int>(-1);

    const auto extractor = root["extractor"];
    cfg.extractor_dir = extractor["dir"].as<std::string>();
    cfg.extractor_stage = extractor["stage"].as<std::string>("g");
    cfg.extractor_backend = extractor["run_backend"].as<std::string>("zg330");

    const auto decoder = root["decoder"];
    cfg.decoder_dir = decoder["dir"].as<std::string>();
    cfg.decoder_stage = decoder["stage"].as<std::string>("g");
    cfg.decoder_backend = decoder["run_backend"].as<std::string>("zg330");

    const auto dataset = root["dataset"];
    if (dataset["sample_dir"]) cfg.sample_dir = dataset["sample_dir"].as<std::string>();
    if (dataset["camera_params"]) cfg.camera_params = dataset["camera_params"].as<std::string>();
    cfg.lut_path = dataset["lut"].as<std::string>();
    if (dataset["output_dir"]) cfg.output_dir = dataset["output_dir"].as<std::string>();

    const auto post = root["postprocess"];
    cfg.post.score_threshold = post["score_threshold"].as<float>(0.6f);
    cfg.post.nms_pre = post["nms_pre"].as<int>(1000);
    cfg.post.max_num = post["max_num"].as<int>(50);
    cfg.post.nms_iou_threshold = post["nms_iou_threshold"].as<float>(0.2f);
    cfg.post.layout = fastbev::vehicle_postprocess::TensorLayout::NHWC;

    const auto visualize = root["visualize"];
    if (visualize) cfg.visualize = visualize["enabled"].as<bool>(true);

    const auto alert = root["alert"];
    if (alert) {
        cfg.alert.enabled = alert["enabled"].as<bool>(false);
        cfg.alert.range_cm = alert["range_cm"].as<float>(13.0f);
        const auto audio = alert["audio"];
        if (audio) {
            cfg.alert.audio.enabled = audio["enabled"].as<bool>(false);
            cfg.alert.audio.device = audio["device"].as<std::string>("default");
            cfg.alert.audio.directory =
                audio["directory"].as<std::string>("./assets/alerts");
            cfg.alert.audio.file = audio["file"].as<std::string>("attention.wav");
            cfg.alert.audio.cooldown_ms = audio["cooldown_ms"].as<int>(3000);
        }
    }

    if (cfg.extractor_backend != "zg330" || cfg.decoder_backend != "zg330") {
        throw std::runtime_error("vehicle pipeline requires ZG330 extractor and decoder backends");
    }
    if (cfg.lut_path.empty()) {
        throw std::runtime_error("vehicle live dataset.lut must not be empty");
    }
    if (cfg.alert.enabled) {
        if (!std::isfinite(cfg.alert.range_cm) || cfg.alert.range_cm <= 0.0f) {
            throw std::runtime_error("vehicle alert.range_cm must be positive");
        }
        if (cfg.alert.audio.enabled) {
            if (cfg.alert.audio.device.empty()) {
                throw std::runtime_error("vehicle alert.audio.device must not be empty");
            }
            if (cfg.alert.audio.directory.empty() || cfg.alert.audio.file.empty()) {
                throw std::runtime_error("vehicle alert audio path must not be empty");
            }
            if (cfg.alert.audio.cooldown_ms < 0) {
                throw std::runtime_error("vehicle alert.audio.cooldown_ms must be >= 0");
            }
        }
    }
    return cfg;
}

std::array<std::string, fastbev_vehicle::kNumCameras> sample_image_paths(
    const std::string& sample_dir) {
    return fastbev_vehicle::default_paths_for_directory(sample_dir);
}

void export_vehicle_camera_params(
    const std::string& template_path,
    const std::array<std::string, fastbev_vehicle::kNumCameras>& image_paths,
    const std::string& output_path) {
    std::ifstream input(template_path);
    if (!input) throw std::runtime_error("cannot open camera params template: " + template_path);
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("cannot create camera params: " + output_path);

    std::string line;
    if (!std::getline(input, line)) {
        throw std::runtime_error("camera params template is empty");
    }
    const int cameras = std::stoi(line);
    if (cameras != fastbev_vehicle::kNumCameras) {
        throw std::runtime_error("camera params template must contain 6 cameras");
    }
    output << line << '\n';

    for (int cam = 0; cam < cameras; ++cam) {
        std::string name;
        if (!std::getline(input, name)) {
            throw std::runtime_error("camera params template is missing camera name");
        }
        output << name << '\n';
        if (!std::getline(input, line)) {
            throw std::runtime_error("camera params template is missing image path");
        }
        output << image_paths[cam] << '\n';
        for (int row = 0; row < 4; ++row) {
            if (!std::getline(input, line)) {
                throw std::runtime_error("camera params template is truncated");
            }
            output << line << '\n';
        }
    }
}

uint16_t read_be16(const uint8_t* data) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) |
                                 static_cast<uint16_t>(data[1]));
}

uint32_t read_be32(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 24U) |
           (static_cast<uint32_t>(data[1]) << 16U) |
           (static_cast<uint32_t>(data[2]) << 8U) |
           static_cast<uint32_t>(data[3]);
}

uint64_t read_be64(const uint8_t* data) {
    uint64_t value = 0;
    for (int i = 0; i < 8; ++i) {
        value = (value << 8U) | static_cast<uint64_t>(data[i]);
    }
    return value;
}

void append_be16(std::vector<uint8_t>& out, uint16_t value) {
    out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
    out.push_back(static_cast<uint8_t>(value & 0xFFU));
}

void append_be32(std::vector<uint8_t>& out, uint32_t value) {
    out.push_back(static_cast<uint8_t>((value >> 24U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((value >> 16U) & 0xFFU));
    out.push_back(static_cast<uint8_t>((value >> 8U) & 0xFFU));
    out.push_back(static_cast<uint8_t>(value & 0xFFU));
}

void append_be64(std::vector<uint8_t>& out, uint64_t value) {
    for (int shift = 56; shift >= 0; shift -= 8) {
        out.push_back(static_cast<uint8_t>((value >> shift) & 0xFFU));
    }
}

bool recv_exact(int fd, void* destination, size_t size, bool allow_clean_eof) {
    auto* out = static_cast<uint8_t*>(destination);
    size_t received = 0;
    while (received < size) {
        const ssize_t count = recv(fd, out + received, size - received, 0);
        if (count == 0) {
            if (received == 0 && allow_clean_eof) return false;
            throw ProtocolError("socket closed in the middle of a VEH1 packet");
        }
        if (count < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("recv failed: ") + std::strerror(errno));
        }
        received += static_cast<size_t>(count);
    }
    return true;
}

void send_all(int fd, const void* source, size_t size) {
    const auto* data = static_cast<const uint8_t*>(source);
    size_t sent = 0;
    while (sent < size) {
        const ssize_t count = send(fd, data + sent, size - sent, MSG_NOSIGNAL);
        if (count < 0) {
            if (errno == EINTR) continue;
            throw std::runtime_error(std::string("send failed: ") + std::strerror(errno));
        }
        if (count == 0) throw std::runtime_error("send returned zero");
        sent += static_cast<size_t>(count);
    }
}

bool recv_packet(int fd, std::vector<uint8_t>& packet) {
    uint8_t length_bytes[4];
    if (!recv_exact(fd, length_bytes, sizeof(length_bytes), true)) return false;
    const uint32_t packet_size = read_be32(length_bytes);
    if (packet_size < kWireHeaderBytes || packet_size > kMaxPacketBytes) {
        throw ProtocolError("invalid VEH1 packet size: " + std::to_string(packet_size));
    }
    packet.resize(packet_size);
    recv_exact(fd, packet.data(), packet.size(), false);
    return true;
}

void send_packet(int fd, const std::vector<uint8_t>& packet) {
    if (packet.size() < kWireHeaderBytes || packet.size() > kMaxPacketBytes) {
        throw ProtocolError("attempted to send invalid VEH1 packet size");
    }
    uint8_t length_bytes[4] = {
        static_cast<uint8_t>((packet.size() >> 24U) & 0xFFU),
        static_cast<uint8_t>((packet.size() >> 16U) & 0xFFU),
        static_cast<uint8_t>((packet.size() >> 8U) & 0xFFU),
        static_cast<uint8_t>(packet.size() & 0xFFU),
    };
    send_all(fd, length_bytes, sizeof(length_bytes));
    send_all(fd, packet.data(), packet.size());
}

std::string camera_name(const uint8_t* bytes) {
    size_t length = 0;
    while (length < 16 && bytes[length] != 0) ++length;
    return std::string(reinterpret_cast<const char*>(bytes), length);
}

uint32_t raw_crc32(const std::vector<uint8_t>& bytes) {
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, reinterpret_cast<const Bytef*>(bytes.data()),
                static_cast<uInt>(bytes.size()));
    return static_cast<uint32_t>(crc);
}

RawCameraBatch parse_raw_batch(const std::vector<uint8_t>& packet) {
    if (packet.size() < kWireHeaderBytes) throw ProtocolError("packet shorter than VEH1 header");
    if (std::memcmp(packet.data(), "VEH1", 4) != 0) throw ProtocolError("bad VEH1 magic");

    const uint16_t version = read_be16(packet.data() + 4);
    const uint16_t message_type = read_be16(packet.data() + 6);
    const uint64_t frame_id = read_be64(packet.data() + 8);
    const uint64_t capture_ts_ns = read_be64(packet.data() + 16);
    const uint32_t item_count = read_be32(packet.data() + 24);
    const uint32_t payload_size = read_be32(packet.data() + 28);

    if (version != kProtocolVersion) {
        throw ProtocolError("unsupported VEH1 version: " + std::to_string(version));
    }
    if (message_type != kMessageBatch) throw ProtocolError("expected VEH1 batch message");
    if (item_count != fastbev_vehicle::kNumCameras) {
        throw ProtocolError("expected exactly six cameras");
    }
    if (payload_size != packet.size() - kWireHeaderBytes) {
        throw ProtocolError("VEH1 payload size mismatch");
    }

    RawCameraBatch batch;
    batch.frame_id = frame_id;
    batch.capture_ts_ns = capture_ts_ns;
    constexpr uint32_t kRawBytes =
        fastbev_vehicle::kInputW * fastbev_vehicle::kInputH * fastbev_vehicle::kChannels;
    size_t offset = kWireHeaderBytes;
    for (size_t i = 0; i < fastbev_vehicle::kCameraNames.size(); ++i) {
        if (offset + kCameraEntryBytes > packet.size()) {
            throw ProtocolError("truncated camera entry");
        }
        const std::string name = camera_name(packet.data() + offset);
        const uint16_t width = read_be16(packet.data() + offset + 16);
        const uint16_t height = read_be16(packet.data() + offset + 18);
        const uint16_t channels = read_be16(packet.data() + offset + 20);
        const uint32_t raw_size = read_be32(packet.data() + offset + 24);
        const uint32_t expected_crc = read_be32(packet.data() + offset + 28);
        offset += kCameraEntryBytes;

        if (name != fastbev_vehicle::kCameraNames[i]) {
            throw ProtocolError("camera order mismatch at index " + std::to_string(i) +
                                ": " + name);
        }
        if (width != fastbev_vehicle::kInputW || height != fastbev_vehicle::kInputH ||
            channels != fastbev_vehicle::kChannels || raw_size != kRawBytes) {
            std::ostringstream error;
            error << name << " has invalid raw shape/bytes: " << width << 'x' << height
                  << " c=" << channels << " bytes=" << raw_size
                  << ", expected 640x480 c=3 bytes=" << kRawBytes;
            throw ProtocolError(error.str());
        }
        if (offset + raw_size > packet.size()) {
            throw ProtocolError("invalid Raw BGR length for " + name);
        }
        auto& raw = batch.raw_bgr[i];
        raw.assign(packet.begin() + static_cast<std::ptrdiff_t>(offset),
                   packet.begin() + static_cast<std::ptrdiff_t>(offset + raw_size));
        offset += raw_size;
        const uint32_t actual_crc = raw_crc32(raw);
        if (actual_crc != expected_crc) {
            std::ostringstream error;
            error << "CRC mismatch for " << name << ": actual=0x" << std::hex
                  << actual_crc << " expected=0x" << expected_crc;
            throw ProtocolError(error.str());
        }
    }
    if (offset != packet.size()) throw ProtocolError("unexpected bytes after sixth camera");
    return batch;
}

std::string json_escape(const std::string& text) {
    std::ostringstream out;
    for (const unsigned char c : text) {
        switch (c) {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\b': out << "\\b"; break;
        case '\f': out << "\\f"; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (c < 0x20) {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(c) << std::dec;
            } else {
                out << static_cast<char>(c);
            }
        }
    }
    return out.str();
}

bool valid_result_box(const fastbev::BoundingBox& box) {
    return std::isfinite(box.x) && std::isfinite(box.y) && std::isfinite(box.z) &&
           std::isfinite(box.w) && std::isfinite(box.l) && std::isfinite(box.h) &&
           std::isfinite(box.yaw) && std::isfinite(box.score) &&
           box.w > 0.0f && box.l > 0.0f && box.h > 0.0f && box.id >= 0;
}

std::vector<fastbev::BoundingBox> sanitize_boxes(
    const std::vector<fastbev::BoundingBox>& boxes) {
    std::vector<fastbev::BoundingBox> valid;
    valid.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (valid_result_box(box)) valid.push_back(box);
    }
    return valid;
}

std::vector<fastbev::BoundingBox> filter_vehicle_origin_cars(
    const std::vector<fastbev::BoundingBox>& boxes) {
    std::vector<fastbev::BoundingBox> filtered;
    filtered.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (box.id == 0 &&
            std::hypot(box.x, box.y) < kVehicleCarMinCenterDistanceCm) {
            continue;
        }
        filtered.push_back(box);
    }
    return filtered;
}

std::vector<uint8_t> build_result_packet(uint64_t frame_id,
                                         uint64_t capture_ts_ns,
                                         uint64_t finished_ts_ns,
                                         double inference_ms,
                                         const std::string& source,
                                         const FrameTiming& timing,
                                         const std::vector<fastbev::BoundingBox>& boxes) {
    std::ostringstream json;
    json.imbue(std::locale::classic());
    json << std::setprecision(9)
         << "{\"frame_id\":" << frame_id
         << ",\"finished_ts_ns\":" << finished_ts_ns
         << ",\"inference_ms\":" << inference_ms
         << ",\"source\":\"" << json_escape(source) << "\""
         << ",\"input_capture_ts_ns\":" << capture_ts_ns
         << ",\"timing\":{\"preprocess_ms\":" << timing.preprocess_ms
         << ",\"input_tensor_ms\":" << timing.input_tensor_ms
         << ",\"part1_ms\":" << timing.part1_ms
         << ",\"part2_ms\":" << timing.part2_ms
         << ",\"part3_ms\":" << timing.part3_ms
         << ",\"postprocess_ms\":" << timing.postprocess_ms
         << ",\"total_ms\":" << timing.total_ms
         << "},\"objects\":[";
    for (size_t i = 0; i < boxes.size(); ++i) {
        const auto& box = boxes[i];
        if (i != 0) json << ',';
        json << "{\"x\":" << box.x
             << ",\"y\":" << box.y
             << ",\"z\":" << box.z
             << ",\"length\":" << box.w
             << ",\"width\":" << box.l
             << ",\"height\":" << box.h
             << ",\"dx\":" << box.w
             << ",\"dy\":" << box.l
             << ",\"dz\":" << box.h
             << ",\"yaw\":" << box.yaw
             << ",\"class_id\":" << box.id
             << ",\"score\":" << box.score << '}';
    }
    json << "]}";
    const std::string payload = json.str();
    if (payload.size() > kMaxPacketBytes - kWireHeaderBytes) {
        throw ProtocolError("result JSON exceeds VEH1 packet limit");
    }

    std::vector<uint8_t> packet;
    packet.reserve(kWireHeaderBytes + payload.size());
    packet.insert(packet.end(), {'V', 'E', 'H', '1'});
    append_be16(packet, kProtocolVersion);
    append_be16(packet, kMessageResult);
    append_be64(packet, frame_id);
    append_be64(packet, finished_ts_ns);
    append_be32(packet, static_cast<uint32_t>(boxes.size()));
    append_be32(packet, static_cast<uint32_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

int create_listener(const std::string& host, int port) {
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE;
    addrinfo* addresses = nullptr;
    const std::string service = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), service.c_str(), &hints, &addresses);
    if (rc != 0) throw std::runtime_error(std::string("getaddrinfo: ") + gai_strerror(rc));

    int listener = -1;
    for (addrinfo* address = addresses; address != nullptr; address = address->ai_next) {
        listener = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (listener < 0) continue;
        int enabled = 1;
        setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
        if (bind(listener, address->ai_addr, address->ai_addrlen) == 0 &&
            listen(listener, 2) == 0) {
            break;
        }
        close(listener);
        listener = -1;
    }
    freeaddrinfo(addresses);
    if (listener < 0) {
        throw std::runtime_error("cannot bind listener on " + host + ':' + service);
    }
    return listener;
}

std::string peer_name(const sockaddr_storage& address, socklen_t length) {
    char host[NI_MAXHOST]{};
    char service[NI_MAXSERV]{};
    if (getnameinfo(reinterpret_cast<const sockaddr*>(&address), length,
                    host, sizeof(host), service, sizeof(service),
                    NI_NUMERICHOST | NI_NUMERICSERV) != 0) {
        return "unknown";
    }
    return std::string(host) + ':' + service;
}

void configure_client_socket(int client) {
    int enabled = 1;
    setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &enabled, sizeof(enabled));
    timeval timeout{};
    timeout.tv_sec = 30;
    setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

DecoderBinding bind_decoder_input(Session& decoder, const NetworkView& decoder_view) {
    const auto decoder_inputs = decoder_view.inputs();
    if (decoder_inputs.size() != 1 ||
        !decoder_input_shape_ok(decoder_inputs[0].tensorType()) ||
        !decoder_inputs[0].tensorType()->element_dtype.getStorageType().isFP16()) {
        throw std::runtime_error(
            "Vehicle decoder view(13) input must be FP16 NCHWc16 [1,16,200,200,16]");
    }

    const auto forwards = decoder.getForwards();
    if (forwards.empty()) throw std::runtime_error("decoder view has no forward operations");
    const auto backend = std::get<1>(forwards[0]);
    if (!backend.is<FPAIBackend>()) {
        throw std::runtime_error("decoder first forward is not on the ZG330 backend");
    }
    const auto fpai_backend = backend.cast<FPAIBackend>();
    const auto first_operation = std::get<0>(forwards[0]);
    if (first_operation->inputs.size() == 0) {
        throw std::runtime_error("decoder first HardOp has no input");
    }
    const int64_t decoder_input_id = first_operation->inputs[0]->v_id;
    if (decoder_input_id != kDecoderInputValueId) {
        std::ostringstream error;
        error << "Vehicle decoder view(" << kDecoderViewStart << ") first input is v"
              << decoder_input_id << ", expected v" << kDecoderInputValueId;
        throw std::runtime_error(error.str());
    }

    DecoderBinding binding;
    const auto value_info = fpai_backend->forward_info->value_map.at(decoder_input_id);
    const auto memchunk =
        fpai_backend->forward_info->memchunk_map.at(decoder_input_id)->memChunk;
    binding.value = value_info->value;
    binding.memchunk = memchunk;
    binding.phy_addr = value_info->phy_addr;
    if (binding.phy_addr < memchunk->begin.addr()) {
        throw std::runtime_error("decoder v362 address is below its MemChunk base");
    }
    binding.offset = binding.phy_addr - memchunk->begin.addr();
    binding.bytes = decoder_inputs[0].tensorType().numElements() * sizeof(uint16_t);
    if (binding.bytes != kDecoderInputBytes ||
        binding.offset + binding.bytes > memchunk->byte_size) {
        throw std::runtime_error("decoder v362 does not fit in its MemChunk");
    }
    return binding;
}

void run_visualize_vehicle(const std::string& camera_params,
                           const std::string& result_path,
                           const std::string& png_path) {
    std::ostringstream cmd;
    cmd << "./visualize_vehicle \"" << camera_params << "\" \""
        << result_path << "\" \"" << png_path << "\"";
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
        std::ostringstream error;
        error << "visualize_vehicle failed with exit code " << rc;
        throw std::runtime_error(error.str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    int listener = -1;
    try {
        const Arguments args = parse_arguments(argc, argv);
        const PipelineConfig cfg = load_config(args.config_path);

        std::printf("============================================================\n");
        std::printf(" FastBEV Vehicle Live Raw BGR Pipeline\n");
        std::printf("============================================================\n");
        std::printf("[Config] listen=%s:%d source=%s\n",
                    args.host.c_str(), args.port, args.source.c_str());
        std::printf("[Config] device=%s LUT=%s\n", cfg.device_url.c_str(), cfg.lut_path.c_str());
        std::printf("[Config] score_threshold=%.3f nms_iou=%.3f max_num=%d\n",
                    static_cast<double>(cfg.post.score_threshold),
                    static_cast<double>(cfg.post.nms_iou_threshold),
                    cfg.post.max_num);
        std::printf("[Config] alert=%s range=%.1f cm audio=%s device=%s wav=%s cooldown=%d ms\n",
                    cfg.alert.enabled ? "on" : "off",
                    static_cast<double>(cfg.alert.range_cm),
                    cfg.alert.audio.enabled ? "on" : "off",
                    cfg.alert.audio.device.c_str(),
                    join_path(cfg.alert.audio.directory, cfg.alert.audio.file).c_str(),
                    cfg.alert.audio.cooldown_ms);

        auto device = Device::Open(cfg.device_url.c_str());
        auto fpai_device = device.cast<FPAIDevice>();

        std::string extractor_stage = cfg.extractor_stage;
        const auto extractor_paths =
            getJrPath(cfg.extractor_backend, cfg.extractor_dir, extractor_stage);
        Network extractor_network =
            loadNetwork(extractor_paths.first, extractor_paths.second);
        NetInfo extractor_info(extractor_network);
        const auto extractor_inputs = extractor_network.inputs();
        if (extractor_inputs.size() != 1 ||
            !extractor_input_shape_ok(extractor_inputs[0].tensorType()) ||
            !extractor_inputs[0].tensorType()->element_dtype.isFP32()) {
            throw std::runtime_error("Vehicle Part1 input must be FP32 NHWC [6,480,640,3]");
        }
        Session extractor = initSession(
            cfg.extractor_backend, extractor_network, device, cfg.ocm_option,
            extractor_info.mmu || cfg.mmu_mode, cfg.speed_mode, cfg.compress_ftmp);
        extractor.apply();
        std::printf("[Init] Vehicle Part1 ready; Parser handles BGR swap and normalization\n");

        std::string decoder_stage = cfg.decoder_stage;
        const auto decoder_paths =
            getJrPath(cfg.decoder_backend, cfg.decoder_dir, decoder_stage);
        Network decoder_network =
            loadNetwork(decoder_paths.first, decoder_paths.second);
        NetInfo decoder_info(decoder_network);
        const auto decoder_outputs_info = decoder_network.outputs();
        if (decoder_outputs_info.size() != 3 ||
            !decoder_outputs_info[0].tensorType()->element_dtype.isFP32() ||
            !decoder_outputs_info[1].tensorType()->element_dtype.isFP32() ||
            !decoder_outputs_info[2].tensorType()->element_dtype.isFP32() ||
            decoder_outputs_info[0].tensorType().numElements() != kDecoderClsElements ||
            decoder_outputs_info[1].tensorType().numElements() != kDecoderBboxElements ||
            decoder_outputs_info[2].tensorType().numElements() != kDecoderDirectionElements) {
            throw std::runtime_error("Vehicle decoder output contract mismatch");
        }

        NetworkView decoder_view = decoder_network.view(kDecoderViewStart);
        Session decoder = initSession(
            cfg.decoder_backend, decoder_view, device, cfg.ocm_option,
            decoder_info.mmu || cfg.mmu_mode, cfg.speed_mode, cfg.compress_ftmp);
        decoder.apply();
        DecoderBinding decoder_input = bind_decoder_input(decoder, decoder_view);
        const uint32_t decoder_input_address =
            checked_plddr_u32(decoder_input.phy_addr, "decoder v362");
        std::printf(
            "[Init] Vehicle Decoder contract: view_start=%lld v_id=%lld "
            "shape=[1,16,200,200,16] FP16 NCHWc16\n",
            static_cast<long long>(kDecoderViewStart),
            static_cast<long long>(kDecoderInputValueId));
        std::printf(
            "[Init] Decoder v362 PLDDR: addr=0x%08X bytes=%llu "
            "memchunk_base=0x%llX offset=%llu memchunk_bytes=%llu\n",
            decoder_input_address,
            static_cast<unsigned long long>(decoder_input.bytes),
            static_cast<unsigned long long>(decoder_input.memchunk->begin.addr()),
            static_cast<unsigned long long>(decoder_input.offset),
            static_cast<unsigned long long>(decoder_input.memchunk->byte_size));

        fpai_device.defaultRegRegion().write(kRegReset, 1, false);
        usleep(1000);
        fpai_device.defaultRegRegion().write(kRegReset, 0, false);
        usleep(1000);
        const uint32_t fpga_version = static_cast<uint32_t>(
            fpai_device.defaultRegRegion().read(kRegVersion, false));

        constexpr int kLutEntries = 200 * 200 * 4;
        constexpr int kLutBytes = kLutEntries * 8;
        auto lut_memory = fpai_device.defaultMemRegion().malloc(kLutBytes, 0, 64);
        std::vector<char> lut_data(kLutBytes);
        std::ifstream lut_file(cfg.lut_path, std::ios::binary);
        if (!lut_file.is_open()) throw std::runtime_error("cannot open LUT: " + cfg.lut_path);
        lut_file.read(lut_data.data(), lut_data.size());
        if (lut_file.gcount() != static_cast<std::streamsize>(lut_data.size())) {
            throw std::runtime_error("LUT file is shorter than expected");
        }
        lut_memory.write(0, lut_data.data(), lut_data.size());
        const uint32_t lut_address = checked_plddr_u32(lut_memory->begin.addr(), "LUT");
        fpai_device.defaultRegRegion().write(kRegLutBase, lut_address, false);
        fpai_device.defaultRegRegion().write(kRegLutSize, kLutEntries, false);
        fpai_device.defaultRegRegion().write(kRegBevParams, kVehicleBevParams, false);
        fpai_device.defaultRegRegion().write(kRegImgParams, kVehicleImgParams, false);
        std::printf(
            "[Init] Vehicle Part2 ready: version=0x%08X expected=0x%08X LUT=0x%08X\n",
            fpga_version, kExpectedFpgaVersion, lut_address);
        if (fpga_version != kExpectedFpgaVersion) {
            std::fprintf(
                stderr,
                "[Init] WARNING: Part2 bitstream version mismatch: actual=0x%08X "
                "expected=0x%08X; continuing\n",
                fpga_version, kExpectedFpgaVersion);
        }
        std::printf("[Init] Part2 output writes directly to Decoder v362: addr=0x%08X bytes=%llu\n",
                    decoder_input_address,
                    static_cast<unsigned long long>(decoder_input.bytes));

        VehicleAudioAlert vehicle_audio(cfg.alert);
        const Value extractor_input = extractor_network.inputs()[0];
        listener = create_listener(args.host, args.port);
        std::printf("[Listen] %s:%d\n", args.host.c_str(), args.port);

        while (true) {
            sockaddr_storage peer_address{};
            socklen_t peer_length = sizeof(peer_address);
            const int client = accept(listener, reinterpret_cast<sockaddr*>(&peer_address),
                                      &peer_length);
            if (client < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error(std::string("accept failed: ") + std::strerror(errno));
            }

            const std::string peer = peer_name(peer_address, peer_length);
            std::printf("[Client] %s connected\n", peer.c_str());
            configure_client_socket(client);

            try {
                std::vector<uint8_t> packet;
                while (recv_packet(client, packet)) {
                    RawCameraBatch batch = parse_raw_batch(packet);
                    const auto frame_begin = Clock::now();

                    std::array<cv::Mat, fastbev_vehicle::kNumCameras> images;
                    for (size_t i = 0; i < images.size(); ++i) {
                        images[i] = cv::Mat(
                            fastbev_vehicle::kInputH,
                            fastbev_vehicle::kInputW,
                            CV_8UC3,
                            batch.raw_bgr[i].data());
                    }

                    std::vector<float> input_tensor(fastbev_vehicle::kTensorElements);
                    fastbev_vehicle::VehiclePreprocessTiming preprocess_timing;
                    const int preprocess_failures =
                        fastbev_vehicle::prepare_from_mats(
                            images, input_tensor.data(), &preprocess_timing);
                    if (preprocess_failures != 0) {
                        throw ProtocolError("vehicle preprocess failed for one or more cameras");
                    }

                    FrameTiming timing;
                    timing.preprocess_ms = preprocess_timing.total_ms;

                    const auto input_begin = Clock::now();
                    Tensor extractor_tensor = data2Tensor<float>(
                        input_tensor.data(), extractor_input);
                    timing.input_tensor_ms = elapsed_ms(input_begin, Clock::now());

                    const auto part1_begin = Clock::now();
                    std::vector<Tensor> feature_tensors = extractor.forward({extractor_tensor});
                    if (feature_tensors.empty()) {
                        throw std::runtime_error("extractor returned no outputs");
                    }
                    for (auto& output : feature_tensors) {
                        if (!output.waitForReady(std::chrono::seconds(10))) {
                            throw std::runtime_error("extractor output timed out");
                        }
                    }
                    if (!feature_tensors[0].dtype()->element_dtype.isFP32() ||
                        feature_tensors[0].dtype().numElements() * sizeof(float) != kFeatureBytes) {
                        throw std::runtime_error(
                            "Vehicle Part1 FEAT2D dtype or byte size mismatch");
                    }
                    timing.part1_ms = elapsed_ms(part1_begin, Clock::now());
                    const uint32_t feature_address =
                        checked_plddr_u32(feature_tensors[0].data().addr(), "extractor FEAT2D");

                    const auto part2_begin = Clock::now();
                    fpai_device.defaultRegRegion().write(kRegFeat2dBase, feature_address, false);
                    fpai_device.defaultRegRegion().write(kRegFeat3dWrite,
                                                         decoder_input_address, false);
                    fpai_device.defaultRegRegion().write(
                        kRegFeat3dSize, static_cast<uint32_t>(decoder_input.bytes), false);
                    fpai_device.defaultRegRegion().write(kRegCtrlStart, 0x03, false);
                    const auto part2_deadline = Clock::now() + std::chrono::seconds(30);
                    while ((static_cast<uint32_t>(
                                fpai_device.defaultRegRegion().read(kRegDone, false)) & 1U) == 0U) {
                        if (Clock::now() > part2_deadline) {
                            throw std::runtime_error("Vehicle Part2 timed out");
                        }
                        usleep(100);
                    }
                    timing.part2_ms = elapsed_ms(part2_begin, Clock::now());
                    device.reset(1);

                    const auto part3_begin = Clock::now();
                    Tensor decoder_tensor(decoder_input.value);
                    decoder_tensor.setData(decoder_input.memchunk, decoder_input.offset);
                    std::vector<Tensor> decoder_outputs = decoder.forward({decoder_tensor});
                    if (decoder_outputs.size() != 3) {
                        throw std::runtime_error("decoder did not return cls/bbox/dir outputs");
                    }
                    for (auto& output : decoder_outputs) {
                        if (!output.waitForReady(std::chrono::seconds(10))) {
                            throw std::runtime_error("decoder output timed out");
                        }
                    }
                    std::vector<float> cls =
                        tensor_to_vector(decoder_outputs[0], kDecoderClsElements, "cls");
                    std::vector<float> bbox =
                        tensor_to_vector(decoder_outputs[1], kDecoderBboxElements, "bbox");
                    std::vector<float> direction =
                        tensor_to_vector(decoder_outputs[2], kDecoderDirectionElements,
                                         "direction");
                    timing.part3_ms = elapsed_ms(part3_begin, Clock::now());
                    device.reset(1);

                    const auto post_begin = Clock::now();
                    std::vector<fastbev::BoundingBox> boxes =
                        fastbev::vehicle_postprocess::decode_and_nms(
                            cls.data(), bbox.data(), direction.data(), cfg.post);
                    boxes = sanitize_boxes(boxes);
                    boxes = filter_vehicle_origin_cars(boxes);
                    vehicle_audio.update(boxes, cfg.post.score_threshold);
                    timing.postprocess_ms = elapsed_ms(post_begin, Clock::now());
                    timing.total_ms = elapsed_ms(frame_begin, Clock::now());

                    const uint64_t finished_ts_ns = unix_time_ns();
                    std::vector<uint8_t> result_packet =
                        build_result_packet(batch.frame_id, batch.capture_ts_ns,
                                            finished_ts_ns, timing.total_ms, args.source,
                                            timing, boxes);
                    send_packet(client, result_packet);

                    std::printf("[Result] frame=%llu objects=%zu pre=%.2f input=%.2f "
                                "p1=%.2f p2=%.2f p3=%.2f post=%.2f total=%.2f ms\n",
                                static_cast<unsigned long long>(batch.frame_id),
                                boxes.size(),
                                timing.preprocess_ms,
                                timing.input_tensor_ms,
                                timing.part1_ms,
                                timing.part2_ms,
                                timing.part3_ms,
                                timing.postprocess_ms,
                                timing.total_ms);
                }
            } catch (const ProtocolError& error) {
                std::fprintf(stderr, "[Client] %s protocol error: %s\n",
                             peer.c_str(), error.what());
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[Client] %s processing error: %s\n",
                             peer.c_str(), error.what());
            }
            close(client);
            std::printf("[Client] %s disconnected\n", peer.c_str());
        }
        return 0;
    } catch (const std::exception& error) {
        if (listener >= 0) close(listener);
        std::fprintf(stderr, "FASTBEV_VEHICLE_LIVE_ERROR: %s\n", error.what());
        return 1;
    }
}
