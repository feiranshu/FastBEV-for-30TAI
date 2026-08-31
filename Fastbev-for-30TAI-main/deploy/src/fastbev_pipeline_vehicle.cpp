/*
 * Vehicle single-frame FastBEV pipeline.
 *
 *   six corrected 640x480 JPGs from SD card -> raw BGR NHWC FP32
 *   -> Vehicle Part1 NPU -> FEAT2D PLDDR
 *   -> Vehicle Part2 FPGA LUT + FP16 reorder
 *   -> Decoder view(13) v362 runtime PLDDR -> Vehicle Part3 NPU
 *   -> vehicle postprocess -> visualize_vehicle PNG
 *
 * This executable is intentionally local and single-frame: no BEV1 socket,
 * CARLA gateway, LED, or audio path.
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

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

namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

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

struct Arguments {
    std::string config_path;
};

struct DecoderBinding {
    Value value;
    MemChunk memchunk;
    uint64_t phy_addr = 0;
    uint64_t offset = 0;
    uint64_t bytes = 0;
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
};

double elapsed_ms(Clock::time_point begin, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - begin).count();
}

Arguments parse_arguments(int argc, char** argv) {
    if (argc != 2) {
        throw std::runtime_error("usage: fastbev_pipeline_vehicle <config.yaml>");
    }
    return {argv[1]};
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
    cfg.sample_dir = dataset["sample_dir"].as<std::string>();
    cfg.camera_params = dataset["camera_params"].as<std::string>();
    cfg.lut_path = dataset["lut"].as<std::string>();
    cfg.output_dir = dataset["output_dir"].as<std::string>();

    const auto post = root["postprocess"];
    cfg.post.score_threshold = post["score_threshold"].as<float>(0.6f);
    cfg.post.nms_pre = post["nms_pre"].as<int>(1000);
    cfg.post.max_num = post["max_num"].as<int>(50);
    cfg.post.nms_iou_threshold = post["nms_iou_threshold"].as<float>(0.2f);
    cfg.post.layout = fastbev::vehicle_postprocess::TensorLayout::NHWC;

    const auto visualize = root["visualize"];
    if (visualize) cfg.visualize = visualize["enabled"].as<bool>(true);

    if (cfg.extractor_backend != "zg330" || cfg.decoder_backend != "zg330") {
        throw std::runtime_error("vehicle pipeline requires ZG330 extractor and decoder backends");
    }
    if (cfg.sample_dir.empty() || cfg.camera_params.empty() ||
        cfg.lut_path.empty() || cfg.output_dir.empty()) {
        throw std::runtime_error("vehicle dataset paths must not be empty");
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
    try {
        const Arguments args = parse_arguments(argc, argv);
        const PipelineConfig cfg = load_config(args.config_path);
        fs::create_directories(cfg.output_dir);

        const auto image_paths = sample_image_paths(cfg.sample_dir);
        for (const auto& path : image_paths) {
            cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
            if (image.empty()) throw std::runtime_error("cannot read vehicle image: " + path);
            if (image.cols != fastbev_vehicle::kInputW ||
                image.rows != fastbev_vehicle::kInputH ||
                image.channels() != fastbev_vehicle::kChannels) {
                std::ostringstream error;
                error << "vehicle image has invalid shape: " << path << " got "
                      << image.cols << 'x' << image.rows << " c=" << image.channels()
                      << ", expected 640x480 c=3";
                throw std::runtime_error(error.str());
            }
        }

        std::printf("============================================================\n");
        std::printf(" FastBEV Vehicle Single-frame Pipeline\n");
        std::printf("============================================================\n");
        std::printf("[Config] device=%s LUT=%s\n", cfg.device_url.c_str(), cfg.lut_path.c_str());
        std::printf("[Config] sample_dir=%s\n", cfg.sample_dir.c_str());
        std::printf("[Config] camera_params=%s\n", cfg.camera_params.c_str());
        std::printf("[Config] output_dir=%s visualize=%s\n",
                    cfg.output_dir.c_str(), cfg.visualize ? "on" : "off");
        std::printf("[Config] score_threshold=%.3f nms_iou=%.3f max_num=%d\n",
                    static_cast<double>(cfg.post.score_threshold),
                    static_cast<double>(cfg.post.nms_iou_threshold),
                    cfg.post.max_num);

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

        std::vector<float> input_tensor(fastbev_vehicle::kTensorElements);
        fastbev_vehicle::VehiclePreprocessTiming preprocess_timing;
        const auto frame_begin = Clock::now();
        const int preprocess_failures =
            fastbev_vehicle::prepare_from_paths(
                image_paths, input_tensor.data(), nullptr, &preprocess_timing);
        if (preprocess_failures != 0) {
            throw std::runtime_error("vehicle preprocess failed for one or more cameras");
        }

        const Value extractor_input = extractor_network.inputs()[0];
        const auto input_begin = Clock::now();
        Tensor extractor_tensor = data2Tensor<float>(input_tensor.data(), extractor_input);
        const double input_ms = elapsed_ms(input_begin, Clock::now());

        const auto part1_begin = Clock::now();
        std::vector<Tensor> feature_tensors = extractor.forward({extractor_tensor});
        if (feature_tensors.empty()) throw std::runtime_error("extractor returned no outputs");
        for (auto& output : feature_tensors) {
            if (!output.waitForReady(std::chrono::seconds(10))) {
                throw std::runtime_error("extractor output timed out");
            }
        }
        if (!feature_tensors[0].dtype()->element_dtype.isFP32() ||
            feature_tensors[0].dtype().numElements() * sizeof(float) != kFeatureBytes) {
            throw std::runtime_error("Vehicle Part1 FEAT2D dtype or byte size mismatch");
        }
        const double part1_ms = elapsed_ms(part1_begin, Clock::now());
        const uint32_t feature_address =
            checked_plddr_u32(feature_tensors[0].data().addr(), "extractor FEAT2D");

        const auto part2_begin = Clock::now();
        fpai_device.defaultRegRegion().write(kRegFeat2dBase, feature_address, false);
        fpai_device.defaultRegRegion().write(kRegFeat3dWrite, decoder_input_address, false);
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
        const double part2_ms = elapsed_ms(part2_begin, Clock::now());
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
            tensor_to_vector(decoder_outputs[2], kDecoderDirectionElements, "direction");
        const double part3_ms = elapsed_ms(part3_begin, Clock::now());
        device.reset(1);

        const auto post_begin = Clock::now();
        std::vector<fastbev::BoundingBox> boxes =
            fastbev::vehicle_postprocess::decode_and_nms(
                cls.data(), bbox.data(), direction.data(), cfg.post);

        const std::string result_path = join_path(cfg.output_dir, "result_0001.txt");
        const std::string camera_path = join_path(cfg.output_dir, "camera_params_0001.txt");
        const std::string png_path = join_path(cfg.output_dir, "output_0001.png");
        fastbev::vehicle_postprocess::write_result_txt(result_path, boxes);
        export_vehicle_camera_params(cfg.camera_params, image_paths, camera_path);
        if (cfg.visualize) {
            run_visualize_vehicle(camera_path, result_path, png_path);
        }
        const double post_ms = elapsed_ms(post_begin, Clock::now());
        const double total_ms = elapsed_ms(frame_begin, Clock::now());

        std::printf("[Result] frame=0001 objects=%zu pre=%.2f input=%.2f "
                    "p1=%.2f p2=%.2f p3=%.2f post=%.2f total=%.2f ms\n",
                    boxes.size(),
                    preprocess_timing.total_ms,
                    input_ms,
                    part1_ms,
                    part2_ms,
                    part3_ms,
                    post_ms,
                    total_ms);
        std::printf("[Output] result=%s\n", result_path.c_str());
        std::printf("[Output] camera_params=%s\n", camera_path.c_str());
        if (cfg.visualize) std::printf("[Output] png=%s\n", png_path.c_str());
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "FASTBEV_VEHICLE_ERROR: %s\n", error.what());
        return 1;
    }
}
