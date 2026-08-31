/*
 * CARLA/BEV1 service using direct FP16 Part2-to-Decoder PLDDR handoff.
 *
 *   six 1600x900 JPEGs -> FP32 BGR NHWC -> Part1 runtime FEAT2D PLDDR
 *   -> Part2 LUT + FP16 conversion + channel repeat + NCHWc16 reorder
 *   -> Decoder view(12) v244 runtime PLDDR -> FP16 Part3
 *   -> threshold/NMS -> non-blocking USB audio alert -> BEV1 JSON result
 *
 * This executable requires the 0x20260721 Part2 bitstream. The hardware writes
 * 40,960,000 FP16 values (81,920,000 bytes) directly in the Decoder v244
 * NCHWc16 layout. No Part2 output is read through the PS.
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
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <zlib.h>

#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

#include "fastbev_preprocess_cv.hpp"
#include "alert_config.hpp"
#include "alert_manager.hpp"
#include "filter.hpp"
#include "nms.hpp"
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
#include "icraft_utils.hpp"
#include "et_device.hpp"
#include "compile_fpai_target.hpp"

using namespace icraft::xrt;
using namespace icraft::xir;
using namespace fastbev;
using namespace fpai;

namespace {

using Clock = std::chrono::steady_clock;

constexpr uint32_t kMaxPacketBytes = 64U * 1024U * 1024U;
constexpr size_t kWireHeaderBytes = 32;
constexpr size_t kCameraEntryBytes = 24;
constexpr uint16_t kProtocolVersion = 1;
constexpr uint16_t kMessageBatch = 1;
constexpr uint16_t kMessageResult = 2;
constexpr int kBicycleClassId = 5;

constexpr int64_t kDecoderViewStart = 12;
constexpr int64_t kDecoderInputValueId = 244;
constexpr uint32_t kExpectedFpgaVersion = 0x20260721;
constexpr uint64_t kDecoderFp16Elements = 40960000;
constexpr uint64_t kDecoderFp16Bytes = kDecoderFp16Elements * sizeof(uint16_t);
constexpr uint64_t kDecoderClsElements = 800000;
constexpr uint64_t kDecoderBboxElements = 720000;
constexpr uint64_t kDecoderDirectionElements = 160000;

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

constexpr std::array<const char*, FASTBEV_NUM_CAMS> kCameraOrder = {
    "CAM_FRONT",
    "CAM_FRONT_RIGHT",
    "CAM_BACK_RIGHT",
    "CAM_BACK",
    "CAM_BACK_LEFT",
    "CAM_FRONT_LEFT",
};

class ProtocolError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct Arguments {
    std::string config_path;
    std::string host = "0.0.0.0";
    int port = 5200;
    std::string source = "fastbev-real-edge";
};

struct CameraBatch {
    uint64_t frame_id = 0;
    uint64_t capture_ts_ns = 0;
    std::array<std::vector<uint8_t>, FASTBEV_NUM_CAMS> jpeg;
};

struct DecoderBinding {
    Value value;
    MemChunk memchunk;
    uint64_t phy_addr = 0;
    uint64_t offset = 0;
    uint64_t bytes = 0;
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
            throw ProtocolError("socket closed in the middle of a BEV1 packet");
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
        throw ProtocolError("invalid BEV1 packet size: " + std::to_string(packet_size));
    }
    packet.resize(packet_size);
    recv_exact(fd, packet.data(), packet.size(), false);
    return true;
}

void send_packet(int fd, const std::vector<uint8_t>& packet) {
    if (packet.size() < kWireHeaderBytes || packet.size() > kMaxPacketBytes) {
        throw ProtocolError("attempted to send invalid BEV1 packet size");
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

uint32_t jpeg_crc32(const std::vector<uint8_t>& bytes) {
    uLong crc = crc32(0L, Z_NULL, 0);
    crc = crc32(crc, reinterpret_cast<const Bytef*>(bytes.data()),
                static_cast<uInt>(bytes.size()));
    return static_cast<uint32_t>(crc);
}

CameraBatch parse_batch(const std::vector<uint8_t>& packet) {
    if (packet.size() < kWireHeaderBytes) throw ProtocolError("packet shorter than BEV1 header");
    if (std::memcmp(packet.data(), "BEV1", 4) != 0) throw ProtocolError("bad BEV1 magic");

    const uint16_t version = read_be16(packet.data() + 4);
    const uint16_t message_type = read_be16(packet.data() + 6);
    const uint64_t frame_id = read_be64(packet.data() + 8);
    const uint64_t capture_ts_ns = read_be64(packet.data() + 16);
    const uint32_t item_count = read_be32(packet.data() + 24);
    const uint32_t payload_size = read_be32(packet.data() + 28);

    if (version != kProtocolVersion) {
        throw ProtocolError("unsupported BEV1 version: " + std::to_string(version));
    }
    if (message_type != kMessageBatch) {
        throw ProtocolError("expected BEV1 batch message");
    }
    if (item_count != FASTBEV_NUM_CAMS) {
        throw ProtocolError("expected exactly six cameras");
    }
    if (payload_size != packet.size() - kWireHeaderBytes) {
        throw ProtocolError("BEV1 payload size mismatch");
    }

    CameraBatch batch;
    batch.frame_id = frame_id;
    batch.capture_ts_ns = capture_ts_ns;
    size_t offset = kWireHeaderBytes;
    for (size_t i = 0; i < kCameraOrder.size(); ++i) {
        if (offset + kCameraEntryBytes > packet.size()) {
            throw ProtocolError("truncated camera entry");
        }
        const std::string name = camera_name(packet.data() + offset);
        const uint32_t jpeg_size = read_be32(packet.data() + offset + 16);
        const uint32_t expected_crc = read_be32(packet.data() + offset + 20);
        offset += kCameraEntryBytes;
        if (name != kCameraOrder[i]) {
            throw ProtocolError("camera order mismatch at index " + std::to_string(i) +
                                ": " + name);
        }
        if (jpeg_size < 4 || offset + jpeg_size > packet.size()) {
            throw ProtocolError("invalid JPEG length for " + name);
        }
        auto& jpeg = batch.jpeg[i];
        jpeg.assign(packet.begin() + static_cast<std::ptrdiff_t>(offset),
                    packet.begin() + static_cast<std::ptrdiff_t>(offset + jpeg_size));
        offset += jpeg_size;
        if (jpeg.front() != 0xFF || jpeg[1] != 0xD8 ||
            jpeg[jpeg.size() - 2] != 0xFF || jpeg.back() != 0xD9) {
            throw ProtocolError("invalid JPEG markers for " + name);
        }
        const uint32_t actual_crc = jpeg_crc32(jpeg);
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

bool valid_result_box(const BoundingBox& box) {
    return std::isfinite(box.x) && std::isfinite(box.y) && std::isfinite(box.z) &&
           std::isfinite(box.w) && std::isfinite(box.l) && std::isfinite(box.h) &&
           std::isfinite(box.yaw) && std::isfinite(box.score) &&
           box.w > 0.0f && box.l > 0.0f && box.h > 0.0f &&
           box.id >= 0 && box.id <= 9 && box.id != kBicycleClassId;
}

std::vector<BoundingBox> sanitize_boxes(const std::vector<BoundingBox>& boxes) {
    std::vector<BoundingBox> valid;
    valid.reserve(boxes.size());
    for (const auto& box : boxes) {
        if (valid_result_box(box)) valid.push_back(box);
    }
    return valid;
}

std::vector<uint8_t> build_result_packet(uint64_t frame_id,
                                         uint64_t capture_ts_ns,
                                         uint64_t finished_ts_ns,
                                         double inference_ms,
                                         const std::string& source,
                                         const std::vector<BoundingBox>& boxes) {
    std::ostringstream json;
    json.imbue(std::locale::classic());
    json << std::setprecision(9)
         << "{\"frame_id\":" << frame_id
         << ",\"finished_ts_ns\":" << finished_ts_ns
         << ",\"inference_ms\":" << inference_ms
         << ",\"source\":\"" << json_escape(source) << "\""
         << ",\"input_capture_ts_ns\":" << capture_ts_ns
         << ",\"objects\":[";
    for (size_t i = 0; i < boxes.size(); ++i) {
        const auto& box = boxes[i];
        if (i != 0) json << ',';
        json << "{\"x\":" << box.x
             << ",\"y\":" << box.y
             << ",\"z\":" << box.z
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
        throw ProtocolError("result JSON exceeds BEV1 packet limit");
    }

    std::vector<uint8_t> packet;
    packet.reserve(kWireHeaderBytes + payload.size());
    packet.insert(packet.end(), {'B', 'E', 'V', '1'});
    append_be16(packet, kProtocolVersion);
    append_be16(packet, kMessageResult);
    append_be64(packet, frame_id);
    append_be64(packet, finished_ts_ns);
    append_be32(packet, static_cast<uint32_t>(boxes.size()));
    append_be32(packet, static_cast<uint32_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

Arguments parse_arguments(int argc, char** argv) {
    if (argc < 2) {
        throw std::runtime_error(
            "usage: fastbev_pipeline_carla_new <config.yaml> "
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
        throw std::runtime_error(std::string(name) + " exceeds the 32-bit PLDDR register range");
    }
    if ((address & 0x3FU) != 0) {
        throw std::runtime_error(std::string(name) + " is not 64-byte aligned");
    }
    return static_cast<uint32_t>(address);
}

bool extractor_input_shape_ok(const TensorType& type) {
    return type->shape.size() == 4 && type->shape[0] == 6 &&
           type->shape[1] == 256 && type->shape[2] == 704 && type->shape[3] == 3;
}

bool decoder_input_shape_ok(const TensorType& type) {
    return type->shape.size() == 5 && type->shape[0] == 1 &&
           type->shape[1] == 64 && type->shape[2] == 200 &&
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

void preprocess_batch(const CameraBatch& batch,
                      float* output,
                      FastBEVPreprocessTiming& timing) {
    timing = {};
    const auto total_begin = Clock::now();
    for (size_t camera = 0; camera < batch.jpeg.size(); ++camera) {
        const auto camera_begin = Clock::now();
        const auto decode_begin = Clock::now();
        cv::Mat image = cv::imdecode(batch.jpeg[camera], cv::IMREAD_COLOR);
        timing.imread_ms += elapsed_ms(decode_begin, Clock::now());
        if (image.empty()) {
            throw ProtocolError(std::string("OpenCV failed to decode ") + kCameraOrder[camera]);
        }
        if (image.cols != FASTBEV_SRC_W || image.rows != FASTBEV_SRC_H) {
            std::ostringstream error;
            error << kCameraOrder[camera] << " has size " << image.cols << 'x' << image.rows
                  << ", expected " << FASTBEV_SRC_W << 'x' << FASTBEV_SRC_H;
            throw ProtocolError(error.str());
        }

        const auto resize_begin = Clock::now();
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(704, 396), 0, 0, cv::INTER_NEAREST);
        cv::Mat cropped = resized(cv::Rect(0, 70, FASTBEV_TARGET_W, FASTBEV_TARGET_H));
        timing.resize_ms += elapsed_ms(resize_begin, Clock::now());

        const auto pack_begin = Clock::now();
        float* camera_output = output + camera * FASTBEV_ONE_CAM_SIZE;
        for (int row = 0; row < FASTBEV_TARGET_H; ++row) {
            const uint8_t* source = cropped.ptr<uint8_t>(row);
            float* destination = camera_output +
                static_cast<size_t>(row) * FASTBEV_TARGET_W * FASTBEV_CHANNELS;
            for (int column = 0; column < FASTBEV_TARGET_W; ++column) {
                const size_t source_offset = static_cast<size_t>(column) * FASTBEV_CHANNELS;
                destination[source_offset] = static_cast<float>(source[source_offset]);
                destination[source_offset + 1] = static_cast<float>(source[source_offset + 1]);
                destination[source_offset + 2] = static_cast<float>(source[source_offset + 2]);
            }
        }
        timing.bgr_pack_nhwc_ms += elapsed_ms(pack_begin, Clock::now());
        timing.camera_total_ms[camera] = elapsed_ms(camera_begin, Clock::now());
    }
    timing.total_ms = elapsed_ms(total_begin, Clock::now());
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

}  // namespace

int main(int argc, char** argv) {
    int listener = -1;
    try {
        const Arguments args = parse_arguments(argc, argv);
        const YAML::Node config = YAML::LoadFile(args.config_path);

        const auto device_config = config["device"];
        const std::string device_url = device_config["url"].as<std::string>();
        const bool mmu_mode = device_config["mmuMode"].as<bool>(true);
        const bool speed_mode = device_config["speedMode"].as<bool>(false);
        const bool compress_ftmp = device_config["compressFtmp"].as<bool>(false);
        const int ocm_option = device_config["ocm_option"].as<int>(-1);

        const auto extractor_config = config["extractor"];
        const std::string extractor_dir = extractor_config["dir"].as<std::string>();
        std::string extractor_stage = extractor_config["stage"].as<std::string>();
        const std::string extractor_backend = extractor_config["run_backend"].as<std::string>();

        const auto decoder_config = config["decoder"];
        const std::string decoder_dir = decoder_config["dir"].as<std::string>();
        std::string decoder_stage = decoder_config["stage"].as<std::string>();
        const std::string decoder_backend = decoder_config["run_backend"].as<std::string>();

        const auto dataset_config = config["dataset"];
        const std::string lut_path = dataset_config["lutDir"].as<std::string>();
        const int cameras = dataset_config["camera"].as<int>();
        const int feature_width = dataset_config["imageW"].as<int>();
        const int feature_height = dataset_config["imageH"].as<int>();

        const auto bev_config = config["bev"];
        const int bev_x = bev_config["bevx"].as<int>();
        const int bev_y = bev_config["bevy"].as<int>();
        const int bev_z = bev_config["bevz"].as<int>();
        const int channels = bev_config["channels"].as<int>();
        const int fp32_bytes = bev_config["fp32Bytes"].as<int>();

        const auto nms_config_node = config["nms"];
        const float score_threshold = nms_config_node["threshold"].as<float>();
        const std::vector<float> nms_thresholds =
            nms_config_node["threslist"].as<std::vector<float>>();
        if (nms_config_node["pedestrian_center_distance_m"] ||
            nms_config_node["vehicle_group_iou_threshold"]) {
            throw std::runtime_error(
                "legacy CARLA NMS keys are not supported; use "
                "nms.pedestrian_motorcycle_center_distance_m and "
                "nms.cross_class_iou_threshold");
        }
        if (!nms_config_node["pedestrian_motorcycle_center_distance_m"] ||
            !nms_config_node["cross_class_iou_threshold"]) {
            throw std::runtime_error(
                "CARLA NMS requires pedestrian_motorcycle_center_distance_m "
                "and cross_class_iou_threshold");
        }
        const float pedestrian_motorcycle_center_distance_m =
            nms_config_node["pedestrian_motorcycle_center_distance_m"].as<float>();
        const float cross_class_iou_threshold =
            nms_config_node["cross_class_iou_threshold"].as<float>();

        const AlertRuntimeConfig alert_config = load_alert_runtime_config(config);
        if (!alert_config.enabled) {
            throw std::runtime_error(
                "CARLA FP16 audio requires alert.enabled=true");
        }
        if (alert_config.led.enabled) {
            throw std::runtime_error(
                "CARLA FP16 is audio-only; set alert.led.enabled=false");
        }
        if (!alert_config.audio.enabled) {
            throw std::runtime_error(
                "CARLA FP16 audio requires alert.audio.enabled=true");
        }

        if (cameras != FASTBEV_NUM_CAMS || feature_width != 176 || feature_height != 64 ||
            bev_x != 200 || bev_y != 200 || bev_z != 4 || channels != 64 || fp32_bytes != 4) {
            throw std::runtime_error("fastbev_pipeline_carla requires the deployed FastBEV dimensions");
        }
        if (extractor_backend != "zg330" || decoder_backend != "zg330") {
            throw std::runtime_error("CARLA service requires ZG330 extractor and decoder backends");
        }
        if (!std::isfinite(pedestrian_motorcycle_center_distance_m) ||
            pedestrian_motorcycle_center_distance_m < 0.0f) {
            throw std::runtime_error(
                "nms.pedestrian_motorcycle_center_distance_m must be finite and non-negative");
        }
        if (!std::isfinite(cross_class_iou_threshold) ||
            cross_class_iou_threshold < 0.0f || cross_class_iou_threshold > 1.0f) {
            throw std::runtime_error(
                "nms.cross_class_iou_threshold must be in [0, 1]");
        }

        const int lut_count = bev_x * bev_y * bev_z;
        const int lut_bytes = lut_count * 8;
        const int feat2d_bytes = cameras * feature_height * feature_width * channels * fp32_bytes;
        const uint64_t decoder_fp16_bytes =
            static_cast<uint64_t>(lut_count) * channels * 4U * sizeof(uint16_t);
        if (decoder_fp16_bytes != kDecoderFp16Bytes) {
            throw std::runtime_error("calculated Decoder v244 byte size is not 81,920,000");
        }

        std::printf("============================================================\n");
        std::printf(" FastBEV CARLA New FP16 Service (direct Conv PLDDR path)\n");
        std::printf("============================================================\n");
        std::printf("[Config] listen=%s:%d source=%s\n", args.host.c_str(), args.port,
                    args.source.c_str());
        std::printf("[Config] device=%s LUT=%s\n", device_url.c_str(), lut_path.c_str());
        std::printf("[Config] score_threshold=%.4f\n", static_cast<double>(score_threshold));
        std::printf(
            "[Config] NMS pedestrian_motorcycle_center=%.2fm cross_class_iou=%.2f\n",
            static_cast<double>(pedestrian_motorcycle_center_distance_m),
            static_cast<double>(cross_class_iou_threshold));
        std::printf(
            "[Config] audio=on confidence_threshold=%.3f device=%s cooldown=%d ms "
            "LED=off\n",
            static_cast<double>(alert_config.policy.confidence_threshold),
            alert_config.audio.device.c_str(), alert_config.audio.cooldown_ms);

        auto device = Device::Open(device_url.c_str());
        auto fpai_device = device.cast<FPAIDevice>();

        const auto extractor_paths = getJrPath(extractor_backend, extractor_dir, extractor_stage);
        Network extractor_network = loadNetwork(extractor_paths.first, extractor_paths.second);
        NetInfo extractor_info(extractor_network);
        const auto extractor_inputs = extractor_network.inputs();
        if (extractor_inputs.size() != 1 ||
            !extractor_input_shape_ok(extractor_inputs[0].tensorType()) ||
            !extractor_inputs[0].tensorType()->element_dtype.isFP32()) {
            throw std::runtime_error("extractor input must be FP32 NHWC [6,256,704,3]");
        }
        Session extractor = initSession(
            extractor_backend, extractor_network, device, ocm_option,
            extractor_info.mmu || mmu_mode, speed_mode, compress_ftmp);
        extractor.apply();
        std::printf("[Init] Extractor ready; Parser handles BGR swap and normalization\n");

        const auto decoder_paths = getJrPath(decoder_backend, decoder_dir, decoder_stage);
        Network decoder_network = loadNetwork(decoder_paths.first, decoder_paths.second);
        NetInfo decoder_info(decoder_network);
        const auto decoder_values = decoder_network.outputs();
        if (decoder_values.size() != 3 ||
            !decoder_values[0].tensorType()->element_dtype.isFP32() ||
            !decoder_values[1].tensorType()->element_dtype.isFP32() ||
            !decoder_values[2].tensorType()->element_dtype.isFP32() ||
            decoder_values[0].tensorType().numElements() != kDecoderClsElements ||
            decoder_values[1].tensorType().numElements() != kDecoderBboxElements ||
            decoder_values[2].tensorType().numElements() != kDecoderDirectionElements) {
            throw std::runtime_error("CARLA FP16 decoder output contract mismatch");
        }

        NetworkView decoder_view = decoder_network.view(kDecoderViewStart);
        const auto decoder_inputs = decoder_view.inputs();
        if (decoder_inputs.size() != 1 ||
            !decoder_input_shape_ok(decoder_inputs[0].tensorType()) ||
            !decoder_inputs[0].tensorType()->element_dtype.getStorageType().isFP16()) {
            throw std::runtime_error(
                "CARLA FP16 decoder view(12) input must be FP16 "
                "NCHWc16 [1,64,200,200,16]");
        }
        Session decoder = initSession(
            decoder_backend, decoder_view, device, ocm_option,
            decoder_info.mmu || mmu_mode, speed_mode, compress_ftmp);
        decoder.apply();

        DecoderBinding decoder_input;
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
            error << "CARLA FP16 decoder view(" << kDecoderViewStart << ") first input is v"
                  << decoder_input_id << ", expected v" << kDecoderInputValueId;
            throw std::runtime_error(error.str());
        }
        const auto value_info = fpai_backend->forward_info->value_map.at(decoder_input_id);
        const auto memchunk =
            fpai_backend->forward_info->memchunk_map.at(decoder_input_id)->memChunk;
        decoder_input.value = value_info->value;
        decoder_input.memchunk = memchunk;
        decoder_input.phy_addr = value_info->phy_addr;
        if (decoder_input.phy_addr < memchunk->begin.addr()) {
            throw std::runtime_error("decoder v244 address is below its MemChunk base");
        }
        decoder_input.offset = decoder_input.phy_addr - memchunk->begin.addr();
        decoder_input.bytes =
            decoder_inputs[0].tensorType().numElements() * sizeof(uint16_t);
        if (decoder_input.bytes != kDecoderFp16Bytes ||
            decoder_input.offset + decoder_input.bytes > memchunk->byte_size) {
            throw std::runtime_error("decoder v244 does not fit in its MemChunk");
        }
        const uint32_t decoder_input_address =
            checked_plddr_u32(decoder_input.phy_addr, "decoder v244");
        std::printf(
            "[Init] CARLA FP16 Decoder contract: view_start=%lld v_id=%lld "
            "shape=[1,64,200,200,16] FP16 NCHWc16\n",
            static_cast<long long>(kDecoderViewStart),
            static_cast<long long>(decoder_input_id));
        std::printf(
            "[Init] Decoder v244 PLDDR: addr=0x%08X bytes=%llu "
            "memchunk_base=0x%llX offset=%llu memchunk_bytes=%llu\n",
            decoder_input_address,
            static_cast<unsigned long long>(decoder_input.bytes),
            static_cast<unsigned long long>(memchunk->begin.addr()),
            static_cast<unsigned long long>(decoder_input.offset),
            static_cast<unsigned long long>(memchunk->byte_size));

        fpai_device.defaultRegRegion().write(kRegReset, 1, false);
        usleep(1000);
        fpai_device.defaultRegRegion().write(kRegReset, 0, false);
        usleep(1000);
        const uint32_t fpga_version = static_cast<uint32_t>(
            fpai_device.defaultRegRegion().read(kRegVersion, false));

        auto lut_memory = fpai_device.defaultMemRegion().malloc(lut_bytes, 0, 64);
        std::vector<char> lut_data(lut_bytes);
        std::ifstream lut_file(lut_path, std::ios::binary);
        if (!lut_file.is_open()) throw std::runtime_error("cannot open LUT: " + lut_path);
        lut_file.read(lut_data.data(), lut_data.size());
        if (lut_file.gcount() != static_cast<std::streamsize>(lut_data.size())) {
            throw std::runtime_error("LUT file is shorter than expected");
        }
        lut_memory.write(0, lut_data.data(), lut_data.size());
        const uint32_t lut_address = checked_plddr_u32(lut_memory->begin.addr(), "LUT");
        fpai_device.defaultRegRegion().write(kRegLutBase, lut_address, false);
        fpai_device.defaultRegRegion().write(kRegLutSize, lut_count, false);
        fpai_device.defaultRegRegion().write(kRegBevParams, 0x04C8C840, false);
        fpai_device.defaultRegRegion().write(kRegImgParams, 0x060400B0, false);
        std::printf(
            "[Init] Part2 ready: version=0x%08X expected=0x%08X LUT=0x%08X\n",
            fpga_version, kExpectedFpgaVersion, lut_address);
        if (fpga_version != kExpectedFpgaVersion) {
            std::fprintf(
                stderr,
                "[Init] WARNING: Part2 bitstream version mismatch: actual=0x%08X "
                "expected=0x%08X; continuing as requested\n",
                fpga_version, kExpectedFpgaVersion);
        }
        std::printf(
            "[Init] Part2 output writes directly to Decoder v244: "
            "addr=0x%08X bytes=%llu\n",
            decoder_input_address,
            static_cast<unsigned long long>(decoder_input.bytes));

        std::vector<float> input_tensor(FASTBEV_TENSOR_SIZE);
        const Value extractor_input = extractor_network.inputs()[0];
        NMSConfig nms_config;
        nms_config.score_thr = score_threshold;
        nms_config.nms_thr_list = nms_thresholds;
        nms_config.pedestrian_motorcycle_center_distance_m =
            pedestrian_motorcycle_center_distance_m;
        nms_config.cross_class_iou_threshold = cross_class_iou_threshold;

        AlertPolicy alert_policy(alert_config.policy);
        AlertAudioPlayer alert_audio(
            alert_config.audio,
            AlertAudioPlayer::RequestObserver{},
            [](const std::string& message) {
                std::fprintf(stderr, "[AlertAudio] %s\n", message.c_str());
            });

        listener = create_listener(args.host, args.port);
        std::printf("[Listen] %s:%d\n", args.host.c_str(), args.port);
        std::fflush(stdout);

        while (true) {
            sockaddr_storage peer_address{};
            socklen_t peer_length = sizeof(peer_address);
            const int client = accept(listener, reinterpret_cast<sockaddr*>(&peer_address), &peer_length);
            if (client < 0) {
                if (errno == EINTR) continue;
                throw std::runtime_error(std::string("accept failed: ") + std::strerror(errno));
            }
            configure_client_socket(client);
            const std::string peer = peer_name(peer_address, peer_length);
            std::printf("[Client] %s connected\n", peer.c_str());
            std::fflush(stdout);

            // CARLA simulation frame IDs advance faster than inference batches.
            // Use a local contiguous sequence so multi-frame alert confirmation
            // is not reset merely because captured CARLA frame IDs have gaps.
            int alert_frame_sequence = 0;
            alert_policy.reset();
            alert_audio.update(AlertDecision{});

            try {
                std::vector<uint8_t> packet;
                while (recv_packet(client, packet)) {
                    const CameraBatch batch = parse_batch(packet);
                    const auto frame_begin = Clock::now();
                    FrameTiming timing;
                    FastBEVPreprocessTiming preprocess_timing;
                    preprocess_batch(batch, input_tensor.data(), preprocess_timing);
                    timing.preprocess_ms = preprocess_timing.total_ms;

                    const auto input_begin = Clock::now();
                    Tensor extractor_tensor = data2Tensor<float>(input_tensor.data(), extractor_input);
                    timing.input_tensor_ms = elapsed_ms(input_begin, Clock::now());

                    const auto part1_begin = Clock::now();
                    std::vector<Tensor> feature_tensors = extractor.forward({extractor_tensor});
                    if (feature_tensors.empty()) throw std::runtime_error("extractor returned no outputs");
                    for (auto& output : feature_tensors) {
                        if (!output.waitForReady(std::chrono::seconds(10))) {
                            throw std::runtime_error("extractor output timed out");
                        }
                    }
                    if (!feature_tensors[0].dtype()->element_dtype.isFP32() ||
                        feature_tensors[0].dtype().numElements() * sizeof(float) !=
                            static_cast<uint64_t>(feat2d_bytes)) {
                        throw std::runtime_error("extractor FEAT2D dtype or byte size mismatch");
                    }
                    timing.part1_ms = elapsed_ms(part1_begin, Clock::now());
                    const uint64_t feature_address = feature_tensors[0].data().addr();
                    const uint32_t feature_address_32 =
                        checked_plddr_u32(feature_address, "extractor FEAT2D");

                    const auto part2_begin = Clock::now();
                    fpai_device.defaultRegRegion().write(kRegFeat2dBase, feature_address_32, false);
                    fpai_device.defaultRegRegion().write(
                        kRegFeat3dWrite, decoder_input_address, false);
                    fpai_device.defaultRegRegion().write(
                        kRegFeat3dSize, static_cast<uint32_t>(decoder_input.bytes), false);
                    fpai_device.defaultRegRegion().write(kRegCtrlStart, 0x03, false);
                    const auto part2_deadline = Clock::now() + std::chrono::seconds(30);
                    while ((static_cast<uint32_t>(
                                fpai_device.defaultRegRegion().read(kRegDone, false)) & 1U) == 0U) {
                        if (Clock::now() > part2_deadline) {
                            throw std::runtime_error("single-frame Part2 timed out");
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
                    std::vector<float> cls = tensor_to_vector(
                        decoder_outputs[0], kDecoderClsElements, "cls");
                    std::vector<float> bbox = tensor_to_vector(
                        decoder_outputs[1], kDecoderBboxElements, "bbox");
                    std::vector<float> direction = tensor_to_vector(
                        decoder_outputs[2], kDecoderDirectionElements, "direction");
                    timing.part3_ms = elapsed_ms(part3_begin, Clock::now());
                    device.reset(1);

                    const auto postprocess_begin = Clock::now();
                    std::vector<BoundingBox> candidates = filter::threshold_and_decode(
                        cls.data(), bbox.data(), direction.data(), score_threshold);
                    candidates.erase(
                        std::remove_if(candidates.begin(), candidates.end(),
                                       [](const BoundingBox& box) {
                                           return box.id == kBicycleClassId;
                                       }),
                        candidates.end());
                    std::vector<BoundingBox> final_boxes =
                        nms::run_multi_class_nms(candidates, nms_config);
                    std::vector<BoundingBox> valid_boxes = sanitize_boxes(final_boxes);

                    const AlertDecision alert =
                        alert_policy.evaluate(valid_boxes, ++alert_frame_sequence);
                    alert_audio.update(alert);
                    if (alert.state_changed) {
                        std::printf(
                            "[AlertAudio] level=%s class=%s direction=%s "
                            "clearance=%.2fm score=%.3f\n",
                            AlertPolicy::level_name(alert.level),
                            AlertPolicy::class_name(alert.class_id),
                            alert.direction.empty() ? "none" : alert.direction.c_str(),
                            static_cast<double>(alert.clearance_m),
                            static_cast<double>(alert.score));
                    }
                    timing.postprocess_ms = elapsed_ms(postprocess_begin, Clock::now());
                    timing.total_ms = elapsed_ms(frame_begin, Clock::now());

                    const uint64_t finished_ts_ns = unix_time_ns();
                    const std::vector<uint8_t> result_packet = build_result_packet(
                        batch.frame_id, batch.capture_ts_ns, finished_ts_ns,
                        timing.total_ms, args.source, valid_boxes);
                    send_packet(client, result_packet);

                    std::printf(
                        "[Result] frame=%llu candidates=%zu objects=%zu/%zu total=%.2f ms "
                        "decode+pre=%.2f input=%.2f p1=%.2f p2=%.2f "
                        "p3_fp16=%.2f post=%.2f\n",
                        static_cast<unsigned long long>(batch.frame_id), candidates.size(),
                        valid_boxes.size(), final_boxes.size(), timing.total_ms, timing.preprocess_ms,
                        timing.input_tensor_ms, timing.part1_ms, timing.part2_ms,
                        timing.part3_ms, timing.postprocess_ms);
                    std::fflush(stdout);
                }
                std::printf("[Client] %s disconnected\n", peer.c_str());
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[Client] %s closed after error: %s\n",
                             peer.c_str(), error.what());
            }
            close(client);
        }

        Device::Close(device);
        close(listener);
        return 0;
    } catch (const std::exception& error) {
        if (listener >= 0) close(listener);
        std::fprintf(stderr, "[Fatal] %s\n", error.what());
        return 1;
    }
}
