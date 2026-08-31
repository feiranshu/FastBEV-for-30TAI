/*
 * fastbev_part2_int8_driver.cpp
 * ---------------------------------------------------------------------------
 * Reference CPU driver for the FastBEV Part2 INT8 FPGA package.
 *
 * This file mirrors the register API style used by deploy/src/fastbev_pipeline.cpp.
 * The FPGA output is already the Decoder Matmul-pre input layout:
 *   [1, 4, 8, 200, 200, 32] sint8,
 *   layout [N][Z][C/32][X][Y][C%32], 40,960,000 bytes.
 *
 * Important integration note:
 *   The current full decoder model in this repository exposes a public FP32 input
 *   [1,200,200,4,256]. This driver is for the cut/internal INT8 boundary before
 *   the first Matmul. If the full public decoder input is used, keep the old FP32
 *   path or regenerate/cut the decoder so its input uses this blocked INT8 value.
 */

#include <cstdint>
#include <cstdio>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace fastbev_part2_int8 {

static constexpr uint32_t REG_BASE        = 0x400C0000;
static constexpr uint32_t REG_CTRL_START  = REG_BASE + 0x00 * 4;
static constexpr uint32_t REG_LUT_BASE    = REG_BASE + 0x01 * 4;
static constexpr uint32_t REG_LUT_SIZE    = REG_BASE + 0x02 * 4;
static constexpr uint32_t REG_FEAT2D_BASE = REG_BASE + 0x03 * 4;
static constexpr uint32_t REG_FEAT3D_WR   = REG_BASE + 0x04 * 4;
static constexpr uint32_t REG_FEAT3D_SIZE = REG_BASE + 0x05 * 4;
static constexpr uint32_t REG_BEV_PARAMS  = REG_BASE + 0x09 * 4;
static constexpr uint32_t REG_IMG_PARAMS  = REG_BASE + 0x0A * 4;
static constexpr uint32_t REG_COMP_DONE   = REG_BASE + 0x20 * 4;
static constexpr uint32_t REG_VERSION     = REG_BASE + 0x30 * 4;
static constexpr uint32_t REG_RESET       = REG_BASE + 0x77 * 4;

static constexpr int BEV_X = 200;
static constexpr int BEV_Y = 200;
static constexpr int BEV_Z = 4;
static constexpr int SRC_CHANNELS = 64;
static constexpr int REPEAT_TIMES = 4;
static constexpr int DEC_CHANNELS = SRC_CHANNELS * REPEAT_TIMES;
static constexpr int CAMERAS = 6;
static constexpr int IMG_W = 176;
static constexpr int IMG_H = 64;

static constexpr int LUT_COUNT = BEV_X * BEV_Y * BEV_Z;
static constexpr int LUT_BYTES = LUT_COUNT * 8;
static constexpr int FEAT2D_BYTES = CAMERAS * IMG_H * IMG_W * SRC_CHANNELS * 4;
static constexpr int DECODER_INT8_BYTES = LUT_COUNT * DEC_CHANNELS;

struct RegisterRegion {
    void write(uint32_t addr, uint32_t value, bool ordered = false);
    uint32_t read(uint32_t addr, bool ordered = false);
};

struct MemChunkRef {
    uint64_t addr = 0;
    uint64_t bytes = 0;
};

static inline uint32_t pack_bev_params() {
    return (uint32_t(BEV_Z) << 24) |
           (uint32_t(BEV_Y) << 16) |
           (uint32_t(BEV_X) << 8) |
           uint32_t(SRC_CHANNELS);
}

static inline uint32_t pack_img_params() {
    return (uint32_t(CAMERAS) << 24) |
           (uint32_t(IMG_H) << 12) |
           uint32_t(IMG_W);
}

static void reset_part2(RegisterRegion& reg) {
    reg.write(REG_RESET, 1, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    reg.write(REG_RESET, 0, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

static void wait_done(RegisterRegion& reg, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms);
    while (true) {
        if (reg.read(REG_COMP_DONE, false) & 1)
            return;
        if (std::chrono::steady_clock::now() > deadline)
            throw std::runtime_error("FastBEV Part2 INT8 FPGA timeout");
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

static void configure_static_registers(RegisterRegion& reg, uint32_t lut_addr) {
    reg.write(REG_LUT_BASE,   lut_addr, false);
    reg.write(REG_LUT_SIZE,   LUT_COUNT, false);
    reg.write(REG_FEAT3D_SIZE, DECODER_INT8_BYTES, false);
    reg.write(REG_BEV_PARAMS, pack_bev_params(), false);
    reg.write(REG_IMG_PARAMS, pack_img_params(), false);
}

static void run_part2(RegisterRegion& reg,
                      uint32_t feat2d_fp32_addr,
                      uint32_t decoder_int8_addr) {
    reg.write(REG_FEAT2D_BASE, feat2d_fp32_addr, false);
    reg.write(REG_FEAT3D_WR,   decoder_int8_addr, false);
    reg.write(REG_CTRL_START,  0x03, false);
    wait_done(reg, 30000);
}

static void print_contract(uint32_t version,
                           const MemChunkRef& lut_mem,
                           const MemChunkRef& decoder_input_mem) {
    std::printf("FastBEV Part2 INT8 contract\n");
    std::printf("  version              : 0x%08X\n", version);
    std::printf("  LUT                  : 0x%08llX (%d B)\n",
                (unsigned long long)lut_mem.addr, LUT_BYTES);
    std::printf("  Part1 FP32 feature   : runtime tensor PLDDR address (%d B)\n",
                FEAT2D_BYTES);
    std::printf("  Decoder INT8 input   : 0x%08llX (%d B)\n",
                (unsigned long long)decoder_input_mem.addr,
                DECODER_INT8_BYTES);
    std::printf("  output layout        : [1,4,8,200,200,32] N,Z,C/32,X,Y,C%%32\n");
}

} // namespace fastbev_part2_int8

/*
 * Integration sketch:
 *
 *   auto device = Device::Open(device_url.c_str());
 *   auto fpai_dev = device.cast<FPAIDevice>();
 *   auto& reg = fpai_dev.defaultRegRegion();
 *
 *   reset_part2(reg);
 *   uint32_t version = reg.read(REG_VERSION, false);
 *
 *   auto lut_mem = fpai_dev.defaultMemRegion().malloc(LUT_BYTES, 0, 64);
 *
 *   // Preferred: decoder_int8_addr is the PLDDR address of the cut Decoder input
 *   // value whose dtype/layout is sint8 [1,4,8,200,200,32]
 *   // [N][Z][C/32][X][Y][C%32].
 *   MemChunkRef decoder_input = get_decoder_internal_int8_input();
 *
 *   configure_static_registers(reg, static_cast<uint32_t>(lut_mem->begin.addr()));
 *
 *   auto feat_2d = extractor.forward({ext_in_tensor});
 *   feat_2d[0].waitForReady(std::chrono::seconds(10));
 *   uint32_t feat2d_addr = static_cast<uint32_t>(feat_2d[0].data().addr());
 *
 *   run_part2(reg, feat2d_addr, static_cast<uint32_t>(decoder_input.addr));
 *
 *   Tensor dec_in_tensor = bind_decoder_tensor_to_existing_plddr(decoder_input);
 *   auto dec_output = decoder_sess.forward({dec_in_tensor});
 */
