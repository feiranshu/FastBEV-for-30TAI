/*
 * pointpillars_icraft_runtime.cpp
 *
 * Self-contained board runtime for the two-stage nuScenes PointPillars model
 * on Icraft XRT / ZG330.
 *
 * Confirmed deployment contract:
 *
 *   Local points_10sweeps_f32.bin
 *       -> CPU pp_preprocess()
 *       -> PFE FP32 [1,11,30000,20]
 *       -> PFE output FP32 [1,30000,1,64] from current ZG public output
 *       -> CPU pp_scatter()
 *       -> BEV FP32 [1,64,512,512]
 *       -> BEV Heads
 *       -> 12 FP32 outputs in EXACT order:
 *            0 h0_cls,  1 h0_box
 *            2 h1_cls,  3 h1_box
 *            4 h2_cls,  5 h2_box
 *            6 h3_cls,  7 h3_box
 *            8 h4_cls,  9 h4_box
 *           10 h5_cls, 11 h5_box
 *       -> CPU pp_postprocess()
 *       -> terminal detections + CSV
 *
 * All model I/O tensors are FP32. The fp16 setting used during compilation
 * applies to network weights/internal parameters and is NOT used to interpret
 * external input/output buffers here.
 *
 * Expected project-side dependency:
 *   icraft_utils.hpp
 *
 * Build this source inside the same board SDK/project environment that was
 * previously used to build the working tcp.cpp runtime.
 */

#include <icraft-xrt/core/session.h>
#include <icraft-xrt/dev/host_device.h>
#include <icraft-xrt/dev/zg330_device.h>
#include <icraft-backends/zg330backend/zg330backend.h>
#include <icraft-backends/hostbackend/backend.h>
#include <icraft-backends/hostbackend/utils.h>

#include "icraft_utils.hpp"

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <sys/stat.h>
#include <sys/types.h>

using namespace icraft::xrt;
using namespace icraft::xir;

/* -------------------------------------------------------------------------- */
/* PointPillars fixed deployment configuration                                */
/* -------------------------------------------------------------------------- */

#define PP_X_MIN (-51.2f)
#define PP_Y_MIN (-51.2f)
#define PP_Z_MIN (-5.0f)
#define PP_X_MAX (51.2f)
#define PP_Y_MAX (51.2f)
#define PP_Z_MAX (3.0f)

#define PP_VOXEL_X (0.2f)
#define PP_VOXEL_Y (0.2f)
#define PP_VOXEL_Z (8.0f)

#define PP_NX 512
#define PP_NY 512
#define PP_MAX_POINTS_PER_PILLAR 20
#define PP_MAX_PILLARS 30000
#define PP_IN_FEATURES 11
#define PP_PFE_CHANNELS 64

#define PP_HEAD_H 128
#define PP_HEAD_W 128
#define PP_NUM_HEADS 6
#define PP_NUM_CLASSES 10
#define PP_BOX_CODE_SIZE 10
#define PP_DIR_BINS 2

#define PP_SCORE_THRESH 0.1f
#define PP_NMS_THRESH 0.2f
#define PP_NMS_PRE_MAX 1000
#define PP_NMS_POST_MAX 83
#define PP_DIR_OFFSET 0.78539f
#define PP_PI 3.14159265358979323846f

/* hova88 fork's ResidualCoder uses atan(sin/(cos+1e-6)), not atan2. */
#define PP_HOVA_ATAN_RATIO 1

typedef struct {
    float x, y, z, intensity, timestamp;
} PPPoint5;

typedef struct {
    int num_pillars;
    int32_t *pillar_xy; /* [PP_MAX_PILLARS][2], x then y */
    float *pfe_input;   /* NCHW memory [1,11,30000,20] */
} PPPillarBatch;

typedef struct {
    const float *cls; /* [N, num_classes] */
    const float *box; /* [N,10] */
    const float *dir; /* [N,2] */
    int n;
    int num_classes;
} PPHeadOutput;

typedef struct {
    int label; /* 0..9 */
    float score;
    float x, y, z, dx, dy, dz, yaw, vx, vy;
} PPDetection;

static_assert(sizeof(PPPoint5) == 5 * sizeof(float),
              "PPPoint5 must contain exactly five float32 values");

namespace {

/* -------------------------------------------------------------------------- */
/* Small runtime helpers                                                      */
/* -------------------------------------------------------------------------- */

using Clock = std::chrono::steady_clock;

constexpr size_t kPfeInputElements =
    static_cast<size_t>(PP_IN_FEATURES) *
    PP_MAX_PILLARS *
    PP_MAX_POINTS_PER_PILLAR;

constexpr size_t kPfeOutputElements =
    static_cast<size_t>(PP_PFE_CHANNELS) *
    PP_MAX_PILLARS;

constexpr size_t kBevInputElements =
    static_cast<size_t>(PP_PFE_CHANNELS) *
    PP_NY *
    PP_NX;

constexpr int kMaxDetections = PP_NUM_CLASSES * PP_NMS_POST_MAX;

double elapsed_ms(const Clock::time_point& a, const Clock::time_point& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
}

struct Arguments {
    std::string points_path =
        "./io/input/pointpillars/points_10sweeps_f32.bin";
    std::string manifest_path;
    std::string pfe_json = "./imodel/pfe/pfe_ZG.json";
    std::string pfe_raw = "./imodel/pfe/pfe_ZG.raw";
    std::string bev_json = "./imodel/bev_heads/bev_heads_ZG.json";
    std::string bev_raw = "./imodel/bev_heads/bev_heads_ZG.raw";
    std::string device_url =
        "axi://zg330aiu?npu=0x40000000&dma=0x80000000";
    std::string csv_path = "./io/output/pointpillars/detections.csv";
    std::string csv_dir = "./io/output/pointpillars/csv";
    std::string token;
};

void usage(const char* program) {
    std::fprintf(
        stderr,
        "Usage:\n"
        "  %s [options]\n\n"
        "Options:\n"
        "  --points PATH     local points_10sweeps_f32.bin\n"
        "  --manifest PATH   batch manifest.csv with sample_token in column 1\n"
        "  --pfe-json PATH   compiled PFE json\n"
        "  --pfe-raw PATH    compiled PFE raw\n"
        "  --bev-json PATH   compiled BEV-heads json\n"
        "  --bev-raw PATH    compiled BEV-heads raw\n"
        "  --device URL      ZG330 device URL\n"
        "  --csv PATH        output CSV path\n"
        "  --csv-dir PATH    output directory for batch per-frame CSV files\n"
        "  --token TOKEN     sample token written to CSV\n"
        "  -h, --help        show this help\n\n"
        "Defaults:\n"
        "  --points   ./io/input/pointpillars/points_10sweeps_f32.bin\n"
        "  --manifest disabled; when enabled points are read from\n"
        "             <manifest_dir>/samples/<sample_token>/points_10sweeps_f32.bin\n"
        "  --pfe-json ./imodel/pfe/pfe_ZG.json\n"
        "  --pfe-raw  ./imodel/pfe/pfe_ZG.raw\n"
        "  --bev-json ./imodel/bev_heads/bev_heads_ZG.json\n"
        "  --bev-raw  ./imodel/bev_heads/bev_heads_ZG.raw\n"
        "  --csv      ./io/output/pointpillars/detections.csv\n",
        program);
}

std::string derive_token(const std::string& path) {
    if (path.empty()) return "sample";

    const size_t slash = path.find_last_of("/\\");
    if (slash != std::string::npos && slash > 0) {
        const size_t prev = path.find_last_of("/\\", slash - 1);
        const size_t begin = (prev == std::string::npos) ? 0 : prev + 1;
        if (slash > begin) {
            return path.substr(begin, slash - begin);
        }
    }

    const size_t begin = (slash == std::string::npos) ? 0 : slash + 1;
    std::string filename = path.substr(begin);
    const size_t dot = filename.find_last_of('.');
    if (dot != std::string::npos) filename.resize(dot);
    return filename.empty() ? "sample" : filename;
}

std::string parent_directory(const std::string& path) {
    const size_t slash = path.find_last_of("/\\");
    if (slash == std::string::npos) return ".";
    if (slash == 0) return path.substr(0, 1);
    return path.substr(0, slash);
}

std::string join_path(const std::string& a, const std::string& b) {
    if (a.empty() || a == ".") return b;
    const char last = a[a.size() - 1];
    if (last == '/' || last == '\\') return a + b;
    return a + "/" + b;
}

std::string strip_cr(std::string s) {
    if (!s.empty() && s[s.size() - 1] == '\r') s.resize(s.size() - 1);
    return s;
}

struct BatchSample {
    std::string token;
    std::string points_path;
    std::string csv_path;
};

std::vector<BatchSample> read_manifest_samples(
    const std::string& manifest_path,
    const std::string& csv_dir
) {
    std::ifstream f(manifest_path);
    if (!f) {
        throw std::runtime_error("could not open manifest: " + manifest_path);
    }

    const std::string manifest_dir = parent_directory(manifest_path);
    std::vector<BatchSample> samples;
    std::string line;
    bool first = true;
    while (std::getline(f, line)) {
        line = strip_cr(line);
        if (line.empty()) continue;
        if (first) {
            first = false;
            if (line.find("sample_token") == 0) continue;
        }

        const size_t comma = line.find(',');
        std::string token = strip_cr(line.substr(0, comma));
        if (token.empty()) continue;

        BatchSample sample;
        sample.token = token;
        sample.points_path = join_path(
            join_path(join_path(manifest_dir, "samples"), token),
            "points_10sweeps_f32.bin");

        char name[512];
        std::snprintf(name, sizeof(name), "detections_%04zu_%s.csv",
                      samples.size() + 1, token.c_str());
        sample.csv_path = join_path(csv_dir, name);
        samples.push_back(std::move(sample));
    }

    if (samples.empty()) {
        throw std::runtime_error("manifest contains no samples: " + manifest_path);
    }
    return samples;
}

Arguments parse_arguments(int argc, char** argv) {
    Arguments args;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        auto need_value = [&](const char* opt) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(
                    std::string("missing value for ") + opt);
            }
            return argv[++i];
        };

        if (arg == "--points") {
            args.points_path = need_value("--points");
        } else if (arg == "--manifest") {
            args.manifest_path = need_value("--manifest");
        } else if (arg == "--pfe-json") {
            args.pfe_json = need_value("--pfe-json");
        } else if (arg == "--pfe-raw") {
            args.pfe_raw = need_value("--pfe-raw");
        } else if (arg == "--bev-json") {
            args.bev_json = need_value("--bev-json");
        } else if (arg == "--bev-raw") {
            args.bev_raw = need_value("--bev-raw");
        } else if (arg == "--device") {
            args.device_url = need_value("--device");
        } else if (arg == "--csv") {
            args.csv_path = need_value("--csv");
        } else if (arg == "--csv-dir") {
            args.csv_dir = need_value("--csv-dir");
        } else if (arg == "--token") {
            args.token = need_value("--token");
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else {
            throw std::runtime_error(
                "unknown argument: " + arg);
        }
    }

    if ((args.manifest_path.empty() && args.points_path.empty()) ||
        args.pfe_json.empty() ||
        args.pfe_raw.empty() ||
        args.bev_json.empty() ||
        args.bev_raw.empty() ||
        args.device_url.empty() ||
        (args.manifest_path.empty() && args.csv_path.empty()) ||
        (!args.manifest_path.empty() && args.csv_dir.empty())) {
        throw std::runtime_error("a required path/URL is empty");
    }

    if (args.manifest_path.empty() && args.token.empty()) {
        args.token = derive_token(args.points_path);
    }

    return args;
}

std::vector<PPPoint5> read_points_file(const std::string& path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        throw std::runtime_error(
            "could not open point file: " + path);
    }

    const std::streamoff end = f.tellg();
    if (end < 0) {
        throw std::runtime_error(
            "could not determine point file size: " + path);
    }

    const size_t bytes = static_cast<size_t>(end);
    if (bytes == 0) {
        throw std::runtime_error("point file is empty: " + path);
    }
    if (bytes % sizeof(PPPoint5) != 0) {
        throw std::runtime_error(
            "point file size is not divisible by 5 float32 values");
    }

    std::vector<PPPoint5> points(bytes / sizeof(PPPoint5));
    f.seekg(0, std::ios::beg);
    f.read(
        reinterpret_cast<char*>(points.data()),
        static_cast<std::streamsize>(bytes));

    if (!f) {
        throw std::runtime_error(
            "failed while reading point file: " + path);
    }

    return points;
}

void ensure_directory(const std::string& dir) {
    if (dir.empty() || dir == ".") return;

    std::string partial;
    partial.reserve(dir.size());

    for (size_t i = 0; i < dir.size(); ++i) {
        const char ch = dir[i];
        partial.push_back(ch);

        const bool is_sep = (ch == '/' || ch == '\\');
        if (!is_sep) continue;

        if (partial == "/" || partial == "./" || partial == "../") continue;
        if (::mkdir(partial.c_str(), 0755) != 0 && errno != EEXIST) {
            throw std::runtime_error(
                "failed to create directory: " + partial);
        }
    }

    if (::mkdir(dir.c_str(), 0755) != 0 && errno != EEXIST) {
        throw std::runtime_error(
            "failed to create directory: " + dir);
    }
}

void ensure_parent_directory(const std::string& file_path) {
    const size_t slash = file_path.find_last_of("/\\");
    if (slash == std::string::npos) return;
    ensure_directory(file_path.substr(0, slash));
}

bool shape_is(
    const TensorType& type,
    std::initializer_list<int64_t> expected
) {
    if (type->shape.size() != expected.size()) return false;

    size_t i = 0;
    for (int64_t dim : expected) {
        if (type->shape[i] != dim) return false;
        ++i;
    }
    return true;
}

void require_fp32_shape(
    const Value& value,
    std::initializer_list<int64_t> expected,
    const std::string& what
) {
    const TensorType type = value.tensorType();

    if (!type->element_dtype.isFP32()) {
        throw std::runtime_error(what + " must be FP32");
    }

    if (!shape_is(type, expected)) {
        throw std::runtime_error(what + " has unexpected shape");
    }
}

void validate_pfe_contract(const Network& network) {
    const auto inputs = network.inputs();
    const auto outputs = network.outputs();

    if (inputs.size() != 1) {
        throw std::runtime_error(
            "PFE contract mismatch: expected exactly 1 input");
    }
    if (outputs.size() != 1) {
        throw std::runtime_error(
            "PFE contract mismatch: expected exactly 1 output");
    }

    require_fp32_shape(
        inputs[0], {1, 11, 30000, 20},
        "PFE input[0]");
    require_fp32_shape(
        outputs[0], {1, 30000, 1, 64},
        "PFE output[0]");
}

bool validate_bev_contract(const Network& network) {
    const auto inputs = network.inputs();
    const auto outputs = network.outputs();

    if (inputs.size() != 1) {
        throw std::runtime_error(
            "BEV contract mismatch: expected exactly 1 input");
    }
    if (outputs.size() != 12 && outputs.size() != 18) {
        throw std::runtime_error(
            "BEV contract mismatch: expected 12 cls/box outputs "
            "or 18 cls/box/dir outputs");
    }

    require_fp32_shape(
        inputs[0], {1, 64, 512, 512},
        "BEV input[0]");

    static const int ncls[PP_NUM_HEADS] = {1, 2, 2, 1, 2, 2};
    const bool has_dir_outputs = (outputs.size() == 18);

    for (int h = 0; h < PP_NUM_HEADS; ++h) {
        const int n =
            ncls[h] * 2 * PP_HEAD_H * PP_HEAD_W;
        const int base = h * (has_dir_outputs ? 3 : 2);

        require_fp32_shape(
            outputs[base + 0],
            {1, n, ncls[h]},
            "BEV h" + std::to_string(h) + "_cls");
        require_fp32_shape(
            outputs[base + 1],
            {1, n, PP_BOX_CODE_SIZE},
            "BEV h" + std::to_string(h) + "_box");
        if (has_dir_outputs) {
            require_fp32_shape(
                outputs[base + 2],
                {1, n, PP_DIR_BINS},
                "BEV h" + std::to_string(h) + "_dir");
        }
    }

    return has_dir_outputs;
}

void copy_output_fp32(
    Tensor& device_tensor,
    size_t expected_elements,
    float* destination,
    const char* name
) {
    if (!device_tensor.waitForReady(std::chrono::seconds(20))) {
        throw std::runtime_error(
            std::string("Icraft output timeout: ") + name);
    }

    /*
     * Runtime guide requirement:
     * move output from PLDDR to host/PS memory before CPU postprocessing.
     */
    Tensor host_tensor =
        device_tensor.to(HostDevice::MemRegion());

    if (!host_tensor.waitForReady(std::chrono::seconds(20))) {
        throw std::runtime_error(
            std::string("Icraft host-copy timeout: ") + name);
    }

    if (!host_tensor.dtype()->element_dtype.isFP32()) {
        throw std::runtime_error(
            std::string("Icraft output is not FP32: ") + name);
    }

    const uint64_t elements =
        host_tensor.dtype().numElements();

    if (elements != expected_elements) {
        throw std::runtime_error(
            std::string("Icraft output element count mismatch: ") + name +
            ", got " + std::to_string(elements) +
            ", expected " + std::to_string(expected_elements));
    }

    host_tensor.read(
        reinterpret_cast<char*>(destination),
        0,
        expected_elements * sizeof(float));
}

void run_pfe_icraft(
    Device& device,
    Session& session,
    const Value& input_value,
    float* input,
    float* output
) {
    bool need_reset = true;

    try {
        Tensor input_tensor =
            data2Tensor<float>(input, input_value);

        std::vector<Tensor> outputs =
            session.forward({input_tensor});

        if (outputs.size() != 1) {
            throw std::runtime_error(
                "PFE forward returned an unexpected output count");
        }

        copy_output_fp32(
            outputs[0],
            kPfeOutputElements,
            output,
            "pfe_output");

        /*
         * The runtime guide recommends quick reset after a completed
         * forward + host copy before the next forward.
         */
        device.reset(1);
        need_reset = false;
    } catch (...) {
        if (need_reset) {
            try {
                device.reset(0);
            } catch (...) {
                // Preserve the original exception.
            }
        }
        throw;
    }
}

struct HeadBuffers {
    std::vector<float> cls;
    std::vector<float> box;
    std::vector<float> dir;
};

void run_bev_icraft(
    Device& device,
    Session& session,
    const Value& input_value,
    float* bev_input,
    HeadBuffers buffers[PP_NUM_HEADS],
    bool has_dir_outputs
) {
    bool need_reset = true;

    try {
        Tensor input_tensor =
            data2Tensor<float>(bev_input, input_value);

        std::vector<Tensor> outputs =
            session.forward({input_tensor});

        const size_t expected_outputs = has_dir_outputs ? 18 : 12;
        if (outputs.size() != expected_outputs) {
            throw std::runtime_error(
                "BEV forward returned an unexpected output count");
        }

        static const int ncls[PP_NUM_HEADS] = {1, 2, 2, 1, 2, 2};

        for (int h = 0; h < PP_NUM_HEADS; ++h) {
            const size_t n =
                static_cast<size_t>(ncls[h]) *
                2 *
                PP_HEAD_H *
                PP_HEAD_W;
            const int base = h * (has_dir_outputs ? 3 : 2);

            copy_output_fp32(
                outputs[base + 0],
                n * static_cast<size_t>(ncls[h]),
                buffers[h].cls.data(),
                ("h" + std::to_string(h) + "_cls").c_str());

            copy_output_fp32(
                outputs[base + 1],
                n * PP_BOX_CODE_SIZE,
                buffers[h].box.data(),
                ("h" + std::to_string(h) + "_box").c_str());

            if (has_dir_outputs) {
                copy_output_fp32(
                    outputs[base + 2],
                    n * PP_DIR_BINS,
                    buffers[h].dir.data(),
                    ("h" + std::to_string(h) + "_dir").c_str());
            }
        }

        device.reset(1);
        need_reset = false;
    } catch (...) {
        if (need_reset) {
            try {
                device.reset(0);
            } catch (...) {
                // Preserve the original exception.
            }
        }
        throw;
    }
}

void allocate_head_buffers(
    HeadBuffers buffers[PP_NUM_HEADS],
    PPHeadOutput heads[PP_NUM_HEADS],
    bool has_dir_outputs
) {
    static const int ncls[PP_NUM_HEADS] = {1, 2, 2, 1, 2, 2};

    for (int h = 0; h < PP_NUM_HEADS; ++h) {
        const int n =
            ncls[h] * 2 * PP_HEAD_H * PP_HEAD_W;

        buffers[h].cls.resize(
            static_cast<size_t>(n) * ncls[h]);
        buffers[h].box.resize(
            static_cast<size_t>(n) * PP_BOX_CODE_SIZE);
        if (has_dir_outputs) {
            buffers[h].dir.resize(
                static_cast<size_t>(n) * PP_DIR_BINS);
        } else {
            buffers[h].dir.clear();
        }

        heads[h].cls = buffers[h].cls.data();
        heads[h].box = buffers[h].box.data();
        heads[h].dir = has_dir_outputs ? buffers[h].dir.data() : nullptr;
        heads[h].n = n;
        heads[h].num_classes = ncls[h];
    }
}

void print_detections(
    const PPDetection* dets,
    int n
);

} // namespace


/* ========================================================================== */
/* CPU core: pp_preprocess.c (from the user-provided deployment package)     */
/* ========================================================================== */

typedef struct {
    int32_t cell_to_pillar[PP_NX * PP_NY];
    float sum_xyz[PP_MAX_PILLARS * 3];
    uint16_t counts[PP_MAX_PILLARS];
} PPWorkspace;

size_t pp_preprocess_workspace_bytes(void) { return sizeof(PPWorkspace); }

static size_t fidx(int c, int p, int k) {
    return ((size_t)c * PP_MAX_PILLARS + (size_t)p) *
           PP_MAX_POINTS_PER_PILLAR + (size_t)k;
}

int pp_preprocess(const PPPoint5 *points, size_t n_points,
                  float *pfe_input, int32_t *pillar_xy,
                  void *workspace, size_t workspace_bytes,
                  PPPillarBatch *out) {
    if (!points || !pfe_input || !pillar_xy || !workspace || !out) return -1;
    if (workspace_bytes < sizeof(PPWorkspace)) return -2;

    PPWorkspace *ws = (PPWorkspace*)workspace;
    for (int i = 0; i < PP_NX * PP_NY; ++i) ws->cell_to_pillar[i] = -1;
    memset(ws->sum_xyz, 0, sizeof(ws->sum_xyz));
    memset(ws->counts, 0, sizeof(ws->counts));
    memset(pfe_input, 0,
           (size_t)PP_IN_FEATURES * PP_MAX_PILLARS *
           PP_MAX_POINTS_PER_PILLAR * sizeof(float));

    int np = 0;
    for (size_t i = 0; i < n_points; ++i) {
        const PPPoint5 *q = &points[i];
        if (!(q->x >= PP_X_MIN && q->x < PP_X_MAX &&
              q->y >= PP_Y_MIN && q->y < PP_Y_MAX &&
              q->z >= PP_Z_MIN && q->z < PP_Z_MAX)) continue;

        int ix = (int)floorf((q->x - PP_X_MIN) / PP_VOXEL_X);
        int iy = (int)floorf((q->y - PP_Y_MIN) / PP_VOXEL_Y);
        if ((unsigned)ix >= PP_NX || (unsigned)iy >= PP_NY) continue;
        int cell = iy * PP_NX + ix;
        int p = ws->cell_to_pillar[cell];

        if (p < 0) {
            if (np >= PP_MAX_PILLARS) continue;
            p = np++;
            ws->cell_to_pillar[cell] = p;
            pillar_xy[p*2 + 0] = ix;
            pillar_xy[p*2 + 1] = iy;
        }

        int k = ws->counts[p];
        if (k >= PP_MAX_POINTS_PER_PILLAR) continue;
        ws->counts[p] = (uint16_t)(k + 1);

        pfe_input[fidx(0,p,k)] = q->x;
        pfe_input[fidx(1,p,k)] = q->y;
        pfe_input[fidx(2,p,k)] = q->z;
        pfe_input[fidx(3,p,k)] = q->intensity;
        pfe_input[fidx(4,p,k)] = q->timestamp;
        ws->sum_xyz[p*3+0] += q->x;
        ws->sum_xyz[p*3+1] += q->y;
        ws->sum_xyz[p*3+2] += q->z;
    }

    const float x_offset = PP_VOXEL_X * 0.5f + PP_X_MIN;
    const float y_offset = PP_VOXEL_Y * 0.5f + PP_Y_MIN;
    const float z_offset = PP_VOXEL_Z * 0.5f + PP_Z_MIN;

    for (int p = 0; p < np; ++p) {
        int cnt = ws->counts[p];
        float mx = ws->sum_xyz[p*3+0] / (float)cnt;
        float my = ws->sum_xyz[p*3+1] / (float)cnt;
        float mz = ws->sum_xyz[p*3+2] / (float)cnt;
        int ix = pillar_xy[p*2+0], iy = pillar_xy[p*2+1];
        float cx = (float)ix * PP_VOXEL_X + x_offset;
        float cy = (float)iy * PP_VOXEL_Y + y_offset;
        float cz = z_offset; /* nz = 1 */

        for (int k = 0; k < cnt; ++k) {
            float x = pfe_input[fidx(0,p,k)];
            float y = pfe_input[fidx(1,p,k)];
            float z = pfe_input[fidx(2,p,k)];
            pfe_input[fidx(5,p,k)]  = x - mx;
            pfe_input[fidx(6,p,k)]  = y - my;
            pfe_input[fidx(7,p,k)]  = z - mz;
            pfe_input[fidx(8,p,k)]  = x - cx;
            pfe_input[fidx(9,p,k)]  = y - cy;
            pfe_input[fidx(10,p,k)] = z - cz;
        }
    }

    out->num_pillars = np;
    out->pillar_xy = pillar_xy;
    out->pfe_input = pfe_input;
    return 0;
}


/* ========================================================================== */
/* CPU core: pp_scatter.c                                                     */
/* ========================================================================== */

void pp_scatter(const float *pfe_out, const PPPillarBatch *pillars, float *bev_out) {
    const size_t plane = (size_t)PP_NX * PP_NY;
    memset(bev_out, 0, (size_t)PP_PFE_CHANNELS * plane * sizeof(float));
    for (int p = 0; p < pillars->num_pillars; ++p) {
        int x = pillars->pillar_xy[p*2+0];
        int y = pillars->pillar_xy[p*2+1];
        size_t cell = (size_t)y * PP_NX + (size_t)x;
        for (int c = 0; c < PP_PFE_CHANNELS; ++c) {
            bev_out[(size_t)c * plane + cell] =
                pfe_out[(size_t)p * PP_PFE_CHANNELS + (size_t)c];
        }
    }
}


/* ========================================================================== */
/* CPU core: pp_postprocess.c                                                 */
/* ========================================================================== */

const char *PP_CLASS_NAMES[PP_NUM_CLASSES] = {
    "car","truck","construction_vehicle","bus","trailer",
    "barrier","motorcycle","bicycle","pedestrian","traffic_cone"
};

typedef struct {
    float dx,dy,dz,bottom;
} AnchorSpec;

static const AnchorSpec A[PP_NUM_CLASSES] = {
    {4.63f,1.97f,1.74f,-0.95f},
    {6.93f,2.51f,2.84f,-0.60f},
    {6.37f,2.85f,3.19f,-0.225f},
    {10.50f,2.94f,3.47f,-0.085f},
    {12.29f,2.90f,3.87f,0.115f},
    {0.50f,2.53f,0.98f,-1.33f},
    {2.11f,0.77f,1.47f,-1.085f},
    {1.70f,0.60f,1.28f,-1.18f},
    {0.73f,0.67f,1.77f,-0.935f},
    {0.41f,0.41f,1.07f,-1.285f}
};

static const int HEAD_CLASSES[PP_NUM_HEADS][2] = {
    {0,-1}, {1,2}, {3,4}, {5,-1}, {6,7}, {8,9}
};
static const int HEAD_NCLS[PP_NUM_HEADS] = {1,2,2,1,2,2};

typedef struct {
    PPDetection d;
} Cand;

static float sigmoidf_safe(float x) {
    if (x >= 0.0f) {
        float z = expf(-x);
        return 1.0f/(1.0f+z);
    } else {
        float z = expf(x);
        return z/(1.0f+z);
    }
}

static float limit_period(float v, float offset, float period) {
    return v - floorf(v / period + offset) * period;
}

static PPDetection decode_one(int h, int idx, int label, float score,
                              const float *box, const float *dir) {
    PPDetection d;
    int ncls = HEAD_NCLS[h];
    int apl = ncls * 2;
    int hw = PP_HEAD_H * PP_HEAD_W;
    int slot = idx / hw;          /* class-anchor slot, then rotation */
    int cell = idx - slot * hw;
    int gy = cell / PP_HEAD_W;
    int gx = cell - gy * PP_HEAD_W;
    int local_anchor_class = slot / 2;
    int anchor_label = HEAD_CLASSES[h][local_anchor_class];
    int rot_idx = slot & 1;

    const AnchorSpec *a = &A[anchor_label];
    float xs = (PP_X_MAX - PP_X_MIN) / (PP_HEAD_W - 1);
    float ys = (PP_Y_MAX - PP_Y_MIN) / (PP_HEAD_H - 1);
    float xa = PP_X_MIN + gx * xs;
    float ya = PP_Y_MIN + gy * ys;
    float za = a->bottom + 0.5f*a->dz;
    float ra = rot_idx ? 1.57f : 0.0f;

    const float *b = box + (size_t)idx * PP_BOX_CODE_SIZE;
    float diag = sqrtf(a->dx*a->dx + a->dy*a->dy);
    d.x = b[0]*diag + xa;
    d.y = b[1]*diag + ya;
    d.z = b[2]*a->dz + za;
    d.dx = expf(b[3])*a->dx;
    d.dy = expf(b[4])*a->dy;
    d.dz = expf(b[5])*a->dz;

    float rg_cos = b[6] + cosf(ra);
    float rg_sin = b[7] + sinf(ra);
#if PP_HOVA_ATAN_RATIO
    d.yaw = atanf(rg_sin / (rg_cos + 1e-6f));
#else
    d.yaw = atan2f(rg_sin, rg_cos);
#endif
    if (dir != nullptr) {
        int dir_label = dir[(size_t)idx*2 + 1] > dir[(size_t)idx*2] ? 1 : 0;
        float period = PP_PI;
        float dir_rot = limit_period(d.yaw - PP_DIR_OFFSET, 0.0f, period);
        d.yaw = dir_rot + PP_DIR_OFFSET + period*(float)dir_label;
    }

    d.vx = b[8]; /* padded anchor velocity = 0 */
    d.vy = b[9];
    d.label = label;
    d.score = score;
    (void)apl;
    return d;
}

/* ---------- min-heap for pre-top1000 ---------- */
static void heap_swap(Cand *a, Cand *b) { Cand t=*a; *a=*b; *b=t; }
static void heap_up(Cand *h, int i) {
    while (i>0) {
        int p=(i-1)/2;
        if (h[p].d.score <= h[i].d.score) break;
        heap_swap(&h[p],&h[i]); i=p;
    }
}
static void heap_down(Cand *h, int n, int i) {
    for (;;) {
        int l=i*2+1, r=l+1, m=i;
        if (l<n && h[l].d.score < h[m].d.score) m=l;
        if (r<n && h[r].d.score < h[m].d.score) m=r;
        if (m==i) break;
        heap_swap(&h[m],&h[i]); i=m;
    }
}
static void topk_push(Cand *heap, int *n, Cand c) {
    if (*n < PP_NMS_PRE_MAX) {
        heap[*n]=c; heap_up(heap,*n); ++(*n);
    } else if (c.d.score > heap[0].d.score) {
        heap[0]=c; heap_down(heap,*n,0);
    }
}
static int cmp_score_desc(const void *pa, const void *pb) {
    float a=((const Cand*)pa)->d.score, b=((const Cand*)pb)->d.score;
    return (a<b) ? 1 : ((a>b) ? -1 : 0);
}

/* ---------- rotated rectangle polygon IoU ---------- */
typedef struct { float x,y; } P2;

static void rect4(const PPDetection *d, P2 p[4]) {
    float hx=0.5f*d->dx, hy=0.5f*d->dy;
    const float lx[4]={-1,1,1,-1}, ly[4]={-1,-1,1,1};
    float c=cosf(d->yaw), s=sinf(d->yaw);
    for (int i=0;i<4;++i) {
        float x=lx[i]*hx, y=ly[i]*hy;
        p[i].x=d->x + c*x - s*y;
        p[i].y=d->y + s*x + c*y;
    }
}
static float cross(P2 a,P2 b,P2 p) {
    return (b.x-a.x)*(p.y-a.y) - (b.y-a.y)*(p.x-a.x);
}
static P2 line_inter(P2 s,P2 e,P2 a,P2 b) {
    float A1=e.y-s.y, B1=s.x-e.x, C1=A1*s.x+B1*s.y;
    float A2=b.y-a.y, B2=a.x-b.x, C2=A2*a.x+B2*a.y;
    float det=A1*B2-A2*B1;
    if (fabsf(det)<1e-8f) return e;
    P2 p={(B2*C1-B1*C2)/det,(A1*C2-A2*C1)/det};
    return p;
}
static int clip_poly(const P2 *in,int nin,P2 a,P2 b,P2 *out) {
    if (nin<=0) return 0;
    int nout=0;
    P2 S=in[nin-1];
    int Sin=cross(a,b,S)>=-1e-6f;
    for (int i=0;i<nin;++i) {
        P2 E=in[i];
        int Ein=cross(a,b,E)>=-1e-6f;
        if (Ein) {
            if (!Sin) out[nout++]=line_inter(S,E,a,b);
            out[nout++]=E;
        } else if (Sin) out[nout++]=line_inter(S,E,a,b);
        S=E; Sin=Ein;
    }
    return nout;
}
static float poly_area(const P2 *p,int n) {
    if (n<3) return 0.0f;
    float s=0.0f;
    for (int i=0;i<n;++i) {
        int j=(i+1)%n;
        s += p[i].x*p[j].y - p[j].x*p[i].y;
    }
    return 0.5f*fabsf(s);
}
static float rotated_iou(const PPDetection *a,const PPDetection *b) {
    P2 ra[4], rb[4], buf1[16], buf2[16];
    rect4(a,ra); rect4(b,rb);
    memcpy(buf1,ra,sizeof(ra));
    int n=4;
    for (int e=0;e<4 && n>0;++e) {
        int en=(e+1)&3;
        n=clip_poly(buf1,n,rb[e],rb[en],buf2);
        memcpy(buf1,buf2,(size_t)n*sizeof(P2));
    }
    float inter=poly_area(buf1,n);
    float aa=a->dx*a->dy, ab=b->dx*b->dy;
    float uni=aa+ab-inter;
    return uni>1e-8f ? inter/uni : 0.0f;
}

int pp_postprocess(const PPHeadOutput heads[PP_NUM_HEADS],
                   PPDetection *out, int max_out) {
    if (!heads || !out || max_out<=0) return -1;
    int out_n=0;
    Cand heap[PP_NMS_PRE_MAX];

    for (int h=0; h<PP_NUM_HEADS; ++h) {
        if (heads[h].num_classes != HEAD_NCLS[h]) return -2;
        int expected = HEAD_NCLS[h]*2*PP_HEAD_H*PP_HEAD_W;
        if (heads[h].n != expected) return -3;

        for (int lc=0; lc<HEAD_NCLS[h]; ++lc) {
            int hn=0;
            int label=HEAD_CLASSES[h][lc];
            for (int i=0;i<heads[h].n;++i) {
                float logit=heads[h].cls[(size_t)i*heads[h].num_classes + lc];
                float score=sigmoidf_safe(logit);
                if (score < PP_SCORE_THRESH) continue;
                if (hn==PP_NMS_PRE_MAX && score <= heap[0].d.score) continue;
                Cand c;
                c.d=decode_one(h,i,label,score,heads[h].box,heads[h].dir);
                topk_push(heap,&hn,c);
            }
            qsort(heap,(size_t)hn,sizeof(heap[0]),cmp_score_desc);

            PPDetection kept[PP_NMS_POST_MAX];
            int kn=0;
            for (int i=0;i<hn && kn<PP_NMS_POST_MAX;++i) {
                int suppress=0;
                for (int j=0;j<kn;++j) {
                    if (rotated_iou(&heap[i].d,&kept[j]) > PP_NMS_THRESH) {
                        suppress=1; break;
                    }
                }
                if (!suppress) kept[kn++]=heap[i].d;
            }
            for (int i=0;i<kn && out_n<max_out;++i) out[out_n++]=kept[i];
        }
    }
    return out_n;
}


/* ========================================================================== */
/* CPU core: pp_io.c                                                          */
/* ========================================================================== */

int pp_write_csv_append(const char *path, const char *token,
                        const PPDetection *d, int n, int write_header) {
    FILE *f=fopen(path, write_header ? "w" : "a");
    if (!f) return -1;
    if (write_header)
        fprintf(f,"sample_token,class,score,x,y,z,dx,dy,dz,yaw,vx,vy\n");
    for (int i=0;i<n;++i) {
        fprintf(f,"%s,%s,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n",
            token,PP_CLASS_NAMES[d[i].label],d[i].score,
            d[i].x,d[i].y,d[i].z,d[i].dx,d[i].dy,d[i].dz,d[i].yaw,d[i].vx,d[i].vy);
    }
    fclose(f); return 0;
}



namespace {

void print_detections(
    const PPDetection* dets,
    int n
) {
    std::printf(
        "\n==================== Detections (%d) ====================\n",
        n);

    if (n <= 0) {
        std::printf("No detections above threshold.\n");
        return;
    }

    std::printf(
        "%-4s %-22s %-8s "
        "%10s %10s %10s "
        "%9s %9s %9s "
        "%10s %9s %9s\n",
        "id", "class", "score",
        "x", "y", "z",
        "dx", "dy", "dz",
        "yaw", "vx", "vy");

    for (int i = 0; i < n; ++i) {
        const PPDetection& d = dets[i];

        const char* cls =
            (d.label >= 0 && d.label < PP_NUM_CLASSES)
            ? PP_CLASS_NAMES[d.label]
            : "unknown";

        std::printf(
            "%-4d %-22s %-8.4f "
            "%10.4f %10.4f %10.4f "
            "%9.4f %9.4f %9.4f "
            "%10.4f %9.4f %9.4f\n",
            i,
            cls,
            static_cast<double>(d.score),
            static_cast<double>(d.x),
            static_cast<double>(d.y),
            static_cast<double>(d.z),
            static_cast<double>(d.dx),
            static_cast<double>(d.dy),
            static_cast<double>(d.dz),
            static_cast<double>(d.yaw),
            static_cast<double>(d.vx),
            static_cast<double>(d.vy));
    }
}

struct RuntimeResources {
    Device device;
    Network pfe_network;
    Network bev_network;
    Value pfe_input_value;
    Value bev_input_value;
    bool bev_has_dir_outputs;

    explicit RuntimeResources(const Arguments& args)
        : device(Device::Open(args.device_url.c_str())),
          pfe_network(Network::CreateFromJsonFile(args.pfe_json)),
          bev_network(Network::CreateFromJsonFile(args.bev_json)),
          pfe_input_value(pfe_network.inputs()[0]),
          bev_input_value(bev_network.inputs()[0]),
          bev_has_dir_outputs(false) {
        pfe_network.lazyLoadParamsFromFile(args.pfe_raw);
        bev_network.lazyLoadParamsFromFile(args.bev_raw);

        validate_pfe_contract(pfe_network);
        bev_has_dir_outputs = validate_bev_contract(bev_network);

        std::printf(
            "[Model] contract check passed:\n"
            "        PFE [1,11,30000,20] -> [1,30000,1,64]\n"
            "        BEV [1,64,512,512] -> %s\n",
            bev_has_dir_outputs ?
                "18 cls/box/dir FP32 outputs" :
                "12 cls/box FP32 outputs, dir correction disabled");
    }
};

struct ReusableBuffers {
    std::vector<float> pfe_input;
    std::vector<float> pfe_output;
    std::vector<int32_t> pillar_xy;
    std::vector<unsigned char> preprocess_workspace;
    std::vector<float> bev_input;

    ReusableBuffers()
        : pfe_input(kPfeInputElements),
          pfe_output(kPfeOutputElements),
          pillar_xy(static_cast<size_t>(PP_MAX_PILLARS) * 2),
          preprocess_workspace(pp_preprocess_workspace_bytes()),
          bev_input(kBevInputElements) {}
};

int run_one_sample(
    RuntimeResources& rt,
    ReusableBuffers& buffers,
    const BatchSample& sample,
    int sample_index,
    int sample_count
) {
    std::printf(
        "\n========== PointPillars Sample %04d / %04d ==========\n",
        sample_index,
        sample_count);
    std::printf("[Sample] token  = %s\n", sample.token.c_str());
    std::printf("[Sample] points = %s\n", sample.points_path.c_str());
    std::printf("[Sample] csv    = %s\n", sample.csv_path.c_str());

    const auto t_all_0 = Clock::now();

    const auto t_read_0 = Clock::now();
    std::vector<PPPoint5> points = read_points_file(sample.points_path);
    const auto t_read_1 = Clock::now();

    std::printf(
        "[Input] loaded %zu points (%.2f MiB)\n",
        points.size(),
        static_cast<double>(points.size() * sizeof(PPPoint5)) /
            (1024.0 * 1024.0));

    PPPillarBatch pillars{};
    const auto t_pre_0 = Clock::now();
    const int pre_rc = pp_preprocess(
        points.data(),
        points.size(),
        buffers.pfe_input.data(),
        buffers.pillar_xy.data(),
        buffers.preprocess_workspace.data(),
        buffers.preprocess_workspace.size(),
        &pillars);
    const auto t_pre_1 = Clock::now();

    if (pre_rc != 0) {
        throw std::runtime_error(
            "pp_preprocess failed, rc=" + std::to_string(pre_rc));
    }
    std::printf("[Pre] num_pillars=%d / %d\n",
                pillars.num_pillars, PP_MAX_PILLARS);

    const auto t_pfe_apply_0 = Clock::now();
    Session pfe_session = initSession(
        "zg330",
        rt.pfe_network,
        rt.device,
        -1,
        true,
        false,
        false);
    pfe_session.apply();
    const auto t_pfe_apply_1 = Clock::now();

    const auto t_pfe_0 = Clock::now();
    run_pfe_icraft(
        rt.device,
        pfe_session,
        rt.pfe_input_value,
        buffers.pfe_input.data(),
        buffers.pfe_output.data());
    const auto t_pfe_1 = Clock::now();

    const auto t_scatter_0 = Clock::now();
    pp_scatter(
        buffers.pfe_output.data(),
        &pillars,
        buffers.bev_input.data());
    const auto t_scatter_1 = Clock::now();

    const auto t_bev_apply_0 = Clock::now();
    Session bev_session = initSession(
        "zg330",
        rt.bev_network,
        rt.device,
        -1,
        true,
        false,
        false);
    bev_session.apply();
    const auto t_bev_apply_1 = Clock::now();

    HeadBuffers head_buffers[PP_NUM_HEADS];
    PPHeadOutput heads[PP_NUM_HEADS]{};
    allocate_head_buffers(
        head_buffers,
        heads,
        rt.bev_has_dir_outputs);

    const auto t_bev_0 = Clock::now();
    run_bev_icraft(
        rt.device,
        bev_session,
        rt.bev_input_value,
        buffers.bev_input.data(),
        head_buffers,
        rt.bev_has_dir_outputs);
    const auto t_bev_1 = Clock::now();

    PPDetection detections[kMaxDetections]{};
    const auto t_post_0 = Clock::now();
    const int det_count = pp_postprocess(
        heads,
        detections,
        kMaxDetections);
    const auto t_post_1 = Clock::now();

    if (det_count < 0) {
        throw std::runtime_error(
            "pp_postprocess failed, rc=" + std::to_string(det_count));
    }

    print_detections(detections, det_count);
    ensure_parent_directory(sample.csv_path);
    if (pp_write_csv_append(
            sample.csv_path.c_str(),
            sample.token.c_str(),
            detections,
            det_count,
            1) != 0) {
        throw std::runtime_error("failed to write CSV: " + sample.csv_path);
    }

    const auto t_all_1 = Clock::now();
    const double apply_ms =
        elapsed_ms(t_pfe_apply_0, t_pfe_apply_1) +
        elapsed_ms(t_bev_apply_0, t_bev_apply_1);

    std::printf(
        "\n[CSV] wrote %d detections to %s\n",
        det_count,
        sample.csv_path.c_str());
    std::printf(
        "[Timing] read=%.3f pre=%.3f apply=%.3f pfe=%.3f scatter=%.3f bev=%.3f post=%.3f total=%.3f ms\n",
        elapsed_ms(t_read_0, t_read_1),
        elapsed_ms(t_pre_0, t_pre_1),
        apply_ms,
        elapsed_ms(t_pfe_0, t_pfe_1),
        elapsed_ms(t_scatter_0, t_scatter_1),
        elapsed_ms(t_bev_0, t_bev_1),
        elapsed_ms(t_post_0, t_post_1),
        elapsed_ms(t_all_0, t_all_1));

    return det_count;
}

int run(const Arguments& args) {
    std::printf(
        "============================================================\n"
        " PointPillars Icraft ZG330 Runtime\n"
        "============================================================\n");

    std::printf("[Config] pfe json = %s\n", args.pfe_json.c_str());
    std::printf("[Config] pfe raw  = %s\n", args.pfe_raw.c_str());
    std::printf("[Config] bev json = %s\n", args.bev_json.c_str());
    std::printf("[Config] bev raw  = %s\n", args.bev_raw.c_str());
    std::printf("[Config] device   = %s\n", args.device_url.c_str());
    if (args.manifest_path.empty()) {
        std::printf("[Config] mode     = single\n");
        std::printf("[Config] points   = %s\n", args.points_path.c_str());
        std::printf("[Config] csv      = %s\n", args.csv_path.c_str());
        std::printf("[Config] token    = %s\n", args.token.c_str());
    } else {
        std::printf("[Config] mode     = manifest batch\n");
        std::printf("[Config] manifest = %s\n", args.manifest_path.c_str());
        std::printf("[Config] csv dir  = %s\n", args.csv_dir.c_str());
    }

    std::vector<BatchSample> samples;
    if (args.manifest_path.empty()) {
        samples.push_back(BatchSample{args.token, args.points_path, args.csv_path});
    } else {
        samples = read_manifest_samples(args.manifest_path, args.csv_dir);
    }
    std::printf("[Config] samples  = %zu\n", samples.size());

    RuntimeResources rt(args);
    bool device_open = true;
    ReusableBuffers buffers;

    try {
        const auto batch_begin = Clock::now();
        int total_detections = 0;
        for (size_t i = 0; i < samples.size(); ++i) {
            total_detections += run_one_sample(
                rt,
                buffers,
                samples[i],
                static_cast<int>(i + 1),
                static_cast<int>(samples.size()));
        }

        const auto batch_end = Clock::now();
        std::printf(
            "\n===================== Batch Summary =====================\n"
            "samples          : %zu\n"
            "total detections : %d\n"
            "total time       : %.3f ms\n",
            samples.size(),
            total_detections,
            elapsed_ms(batch_begin, batch_end));

        Device::Close(rt.device);
        device_open = false;
        return 0;

    } catch (...) {
        if (device_open) {
            try {
                Device::Close(rt.device);
            } catch (...) {
                // Preserve original exception.
            }
        }
        throw;
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Arguments args =
            parse_arguments(argc, argv);
        return run(args);
    } catch (const std::exception& error) {
        std::fprintf(
            stderr,
            "\n[FATAL] %s\n",
            error.what());
        return 1;
    } catch (...) {
        std::fprintf(
            stderr,
            "\n[FATAL] unknown exception\n");
        return 1;
    }
}
