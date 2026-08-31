/*
 * Experimental three-stage FastBEV pipeline.
 *
 * This file intentionally includes the synchronous pipeline implementation as
 * a shared helper source and renames its main(). This target validates coarse
 * pipeline overlap:
 *
 *   preprocess thread -> Part1/Part3 NPU thread -> Part2 FPGA thread -> post thread
 *
 * Part1 and Part3 share the single NPU worker. Part2 owns the custom FPGA
 * registers. The current model exposes one Part1 output and one decoder input
 * PLDDR region, so the NPU worker waits for Part2 before reusing either region.
 */

#define main fastbev_pipeline_sync_main_unused
#include "fastbev_pipeline.cpp"
#undef main

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

namespace {

// Two independent Session/PLDDR slots are the minimum needed for P1/P2/P3 overlap.
static constexpr int BUFFER_COUNT = 2;

template <typename T>
class BoundedQueue {
public:
    explicit BoundedQueue(size_t capacity) : capacity_(capacity) {}

    bool push(T value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [&] { return closed_ || queue_.size() < capacity_; });
        if (closed_) return false;
        queue_.push(std::move(value));
        not_empty_.notify_one();
        return true;
    }

    bool pop(T& value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        not_empty_.wait(lock, [&] { return closed_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        value = std::move(queue_.front());
        queue_.pop();
        not_full_.notify_one();
        return true;
    }

    void close()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
        not_full_.notify_all();
    }

    size_t size() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

private:
    size_t capacity_;
    bool closed_ = false;
    mutable std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::queue<T> queue_;
};

struct PipelineConfig {
    std::string device_url;
    bool dev_mmuMode = true;
    bool dev_speedMode = false;
    bool dev_compressFtmp = false;
    int dev_ocmOption = -1;

    std::string ext_dir;
    std::string ext_stage;
    std::string ext_backend;

    std::string dec_dir;
    std::string dec_stage;
    std::string dec_backend;

    std::string json_path;
    std::string lut_path;
    std::string box_dir;
    std::string png_dir;
    std::string para_dir;
    std::string log_dir;
    int CAMERAS = 0;
    int IMG_W = 0;
    int IMG_H = 0;

    int BEV_X = 0;
    int BEV_Y = 0;
    int BEV_Z = 0;
    int CHANNELS = 0;
    int FP32_BYTES = 4;

    float nms_threshold = 0.6f;
    std::vector<float> nms_threslist;

    int LUT_COUNT = 0;
    int FEAT2D_BYTES = 0;
    int LUT_BYTES = 0;
    int DECODER_INT8_BYTES = 0;
};

struct FrameContext {
    int frame_id = 0;
    int buffer_index = 0;
    FastBEVSample* sample = nullptr;
    std::unique_ptr<FastBEVFrameInput> input;
    int prep_fail = 0;
    bool error = false;
    std::string error_msg;

    std::vector<float> cls_data;
    std::vector<float> bbox_data;
    std::vector<float> dir_data;
    uint64_t part1_feat2d_addr = 0;

    Clock::time_point frame_start;
    Clock::time_point preprocess_begin;
    Clock::time_point preprocess_end;
    Clock::time_point device_pop_time;
    Clock::time_point device_begin;
    Clock::time_point part1_begin;
    Clock::time_point part1_end;
    Clock::time_point part2_begin;
    Clock::time_point part2_end;
    Clock::time_point part3_begin;
    Clock::time_point part3_end;
    Clock::time_point post_pop_time;
    Clock::time_point post_begin;
    Clock::time_point post_end;

    double preprocess_queue_wait_ms = 0.0;
    double device_queue_wait_ms = 0.0;
    double post_queue_wait_ms = 0.0;
};

PipelineConfig parse_config(const char* path)
{
    YAML::Node config = YAML::LoadFile(path);
    PipelineConfig cfg;

    auto dev_cfg = config["device"];
    cfg.device_url = dev_cfg["url"].as<std::string>();
    cfg.dev_mmuMode = dev_cfg["mmuMode"].as<bool>(true);
    cfg.dev_speedMode = dev_cfg["speedMode"].as<bool>(false);
    cfg.dev_compressFtmp = dev_cfg["compressFtmp"].as<bool>(false);
    cfg.dev_ocmOption = dev_cfg["ocm_option"].as<int>(-1);

    auto ext_cfg = config["extractor"];
    cfg.ext_dir = ext_cfg["dir"].as<std::string>();
    cfg.ext_stage = ext_cfg["stage"].as<std::string>();
    cfg.ext_backend = ext_cfg["run_backend"].as<std::string>();

    auto dec_cfg = config["decoder"];
    cfg.dec_dir = dec_cfg["dir"].as<std::string>();
    cfg.dec_stage = dec_cfg["stage"].as<std::string>();
    cfg.dec_backend = dec_cfg["run_backend"].as<std::string>();

    auto ds_cfg = config["dataset"];
    cfg.json_path = ds_cfg["imageDir"].as<std::string>();
    cfg.lut_path = ds_cfg["lutDir"].as<std::string>();
    cfg.box_dir = ds_cfg["boxDir"].as<std::string>();
    cfg.png_dir = ds_cfg["pngDir"].as<std::string>();
    cfg.para_dir = ds_cfg["paraDir"].as<std::string>();
    cfg.log_dir = ds_cfg["logDir"].as<std::string>();
    cfg.CAMERAS = ds_cfg["camera"].as<int>();
    cfg.IMG_W = ds_cfg["imageW"].as<int>();
    cfg.IMG_H = ds_cfg["imageH"].as<int>();

    auto bev_cfg = config["bev"];
    cfg.BEV_X = bev_cfg["bevx"].as<int>();
    cfg.BEV_Y = bev_cfg["bevy"].as<int>();
    cfg.BEV_Z = bev_cfg["bevz"].as<int>();
    cfg.CHANNELS = bev_cfg["channels"].as<int>();
    cfg.FP32_BYTES = bev_cfg["fp32Bytes"].as<int>();

    auto nms_cfg = config["nms"];
    cfg.nms_threshold = nms_cfg["threshold"].as<float>();
    cfg.nms_threslist = nms_cfg["threslist"].as<std::vector<float>>();

    cfg.LUT_COUNT = cfg.BEV_X * cfg.BEV_Y * cfg.BEV_Z;
    cfg.FEAT2D_BYTES = cfg.CAMERAS * cfg.IMG_H * cfg.IMG_W * cfg.CHANNELS * cfg.FP32_BYTES;
    cfg.LUT_BYTES = cfg.LUT_COUNT * 8;
    // v245 stores four 64-channel BEV groups as int8: 200*200*4*64*4 bytes.
    cfg.DECODER_INT8_BYTES = cfg.LUT_COUNT * cfg.CHANNELS * 4;
    return cfg;
}

PlddrTensorBinding get_decoder_conv_binding(Session& session, uint64_t expected_bytes)
{
    auto forwards = session.getForwards();
    if (forwards.empty() || !std::get<1>(forwards[0]).is<FPAIBackend>()) {
        throw std::runtime_error("decoder view first forward is not on ZG330/FPAI backend.");
    }
    auto backend = std::get<1>(forwards[0]).cast<FPAIBackend>();
    auto input_op = std::get<0>(forwards[0]);
    if (input_op->inputs.size() == 0) {
        throw std::runtime_error("decoder view first hardop has no input value.");
    }
    const int64_t vid = input_op->inputs[0]->v_id;
    auto value_info = backend->forward_info->value_map.at(vid);
    auto memchunk = backend->forward_info->memchunk_map.at(vid)->memChunk;
    PlddrTensorBinding binding;
    binding.value = value_info->value;
    binding.memchunk = memchunk;
    binding.phy_addr = value_info->phy_addr;
    binding.offset = value_info->phy_addr - memchunk->begin.addr();
    binding.bytes = expected_bytes;
    checked_plddr_u32(binding.phy_addr, "decoder Conv input");
    return binding;
}

void preprocess_worker(const PipelineConfig& cfg,
                       FastBEVDataset* ds,
                       BoundedQueue<int>& free_slots,
                       BoundedQueue<std::shared_ptr<FrameContext>>& preprocess_to_device,
                       PipelineLogger& log,
                       std::atomic<bool>& failed,
                       std::string& failure_msg,
                       std::mutex& failure_mutex)
{
    try {
        for (int fi = 0; fi < ds->num_samples; ++fi) {
            int buffer_index = -1;
            if (!free_slots.pop(buffer_index)) break;

            auto ctx = std::make_shared<FrameContext>();
            ctx->frame_id = fi + 1;
            ctx->buffer_index = buffer_index;
            ctx->sample = &ds->samples[fi];
            ctx->input.reset(new FastBEVFrameInput);
            ctx->frame_start = Clock::now();
            ctx->preprocess_begin = Clock::now();

            log.print("\n[Async][Pre] Frame %04d/%d start, buffer=%d\n",
                      ctx->frame_id, ds->num_samples, ctx->buffer_index);
            if (ctx->sample->is_first_in_scene) {
                log.print("[Async][Pre] Scene %s first frame\n", ctx->sample->scene_name);
            }
            ctx->prep_fail = fastbev_prepare_frame_input(ctx->sample, ctx->input.get());
            ctx->preprocess_end = Clock::now();
            if (ctx->prep_fail > 0) {
                log.print("[Async][Pre] Frame %04d warning: %d camera images failed\n",
                          ctx->frame_id, ctx->prep_fail);
            }
            auto* ctx_raw = ctx.get();
            auto push_begin = Clock::now();
            if (!preprocess_to_device.push(std::move(ctx))) break;
            auto push_end = Clock::now();
            ctx_raw->preprocess_queue_wait_ms = elapsed_ms(push_begin, push_end);
        }
    } catch (const std::exception& e) {
        {
            std::lock_guard<std::mutex> lock(failure_mutex);
            failure_msg = e.what();
        }
        failed.store(true);
    }
    preprocess_to_device.close();
}

void post_worker(const PipelineConfig& cfg,
                 BoundedQueue<std::shared_ptr<FrameContext>>& device_to_post,
                 PipelineLogger& log,
                 std::atomic<int>& posted_frames,
                 std::atomic<bool>& failed,
                 std::string& failure_msg,
                 std::mutex& failure_mutex)
{
    try {
        std::shared_ptr<FrameContext> ctx;
        while (device_to_post.pop(ctx)) {
            ctx->post_pop_time = Clock::now();
            ctx->post_queue_wait_ms = elapsed_ms(ctx->part3_end, ctx->post_pop_time);
            ctx->post_begin = Clock::now();

            if (ctx->error) {
                log.print("[Async][Post] Frame %04d skipped: %s\n",
                          ctx->frame_id, ctx->error_msg.c_str());
                continue;
            }

            std::vector<BoundingBox> candidates = filter::threshold_and_decode(
                ctx->cls_data.data(), ctx->bbox_data.data(), ctx->dir_data.data(), cfg.nms_threshold);
            NMSConfig nms_config;
            nms_config.score_thr = cfg.nms_threshold;
            nms_config.nms_thr_list = cfg.nms_threslist;
            std::vector<BoundingBox> final_boxes =
                nms::run_multi_class_nms(candidates, nms_config);

            char result_path[512];
            std::snprintf(result_path, sizeof(result_path),
                          "%s/result_%04d.txt", cfg.box_dir.c_str(), ctx->frame_id);
            {
                std::ofstream ofs(result_path);
                if (!ofs.is_open()) {
                    throw std::runtime_error(std::string("Cannot write result: ") + result_path);
                }
                for (const auto& b : final_boxes) {
                    ofs << b.x << " " << b.y << " " << b.z << " "
                        << b.w << " " << b.l << " " << b.h << " "
                        << b.yaw << " " << b.id << " " << b.score << "\n";
                }
            }

            char param_path[512];
            std::snprintf(param_path, sizeof(param_path),
                          "%s/camera_params_%04d.txt", cfg.para_dir.c_str(), ctx->frame_id);
            if (fastbev_export_camera_params(ctx->sample, param_path) != 0) {
                throw std::runtime_error(std::string("Camera params export failed: ") + param_path);
            }

            ctx->post_end = Clock::now();
            const double ms_pre = elapsed_ms(ctx->preprocess_begin, ctx->preprocess_end);
            const double ms_part1 = elapsed_ms(ctx->part1_begin, ctx->part1_end);
            const double ms_part2 = elapsed_ms(ctx->part2_begin, ctx->part2_end);
            const double ms_part3 = elapsed_ms(ctx->part3_begin, ctx->part3_end);
            const double ms_post = elapsed_ms(ctx->post_begin, ctx->post_end);
            const double ms_total = elapsed_ms(ctx->frame_start, ctx->post_end);
            const int done = ++posted_frames;
            const double fps = done > 1 ? 1000.0 * done / elapsed_ms(ctx->frame_start, Clock::now()) : 0.0;

            log.print("[Async][Post] Frame %04d done: candidates=%zu final=%zu result=%s (PNG disabled)\n",
                      ctx->frame_id, candidates.size(), final_boxes.size(), result_path);
            log.print("  [Async Timing] pre=%.2f ms, pre_push_wait=%.2f ms, dev_wait=%.2f ms, p1=%.2f ms, p2=%.2f ms, p3=%.2f ms, post_wait=%.2f ms, post=%.2f ms, latency=%.2f ms, done=%d, approx_fps=%.2f\n",
                      ms_pre,
                      ctx->preprocess_queue_wait_ms,
                      ctx->device_queue_wait_ms,
                      ms_part1,
                      ms_part2,
                      ms_part3,
                      ctx->post_queue_wait_ms,
                      ms_post,
                      ms_total,
                      done,
                      fps);
        }
    } catch (const std::exception& e) {
        {
            std::lock_guard<std::mutex> lock(failure_mutex);
            failure_msg = e.what();
        }
        failed.store(true);
    }
}

int device_stage_main(const PipelineConfig& cfg,
                      FastBEVDataset* ds,
                      BoundedQueue<std::shared_ptr<FrameContext>>& preprocess_to_device,
                      BoundedQueue<std::shared_ptr<FrameContext>>& device_to_post,
                      PipelineLogger& log,
                      std::atomic<bool>& failed)
{
    log.print("[Async][Init] Opening device...\n");
    auto device = Device::Open(cfg.device_url.c_str());
    auto fpai_dev = device.cast<FPAIDevice>();

    log.print("[Async][Init] Loading extractor network...\n");
    std::string ext_stage = cfg.ext_stage;
    auto ext_jr_path = getJrPath(cfg.ext_backend, cfg.ext_dir, ext_stage);
    Network ext_network = loadNetwork(ext_jr_path.first, ext_jr_path.second);
    NetInfo ext_netinfo = NetInfo(ext_network);
    Session extractor = initSession(
        cfg.ext_backend, ext_network, device,
        cfg.dev_ocmOption, ext_netinfo.mmu || cfg.dev_mmuMode,
        cfg.dev_speedMode, cfg.dev_compressFtmp);
    extractor.apply();
    log.print("[Async][Init] Extractor Parser preprocessing: BGR -> SwapOrder/Add/Multiply -> normalized RGB\n");

    log.print("[Async][Init] Loading decoder network...\n");
    std::string dec_stage = cfg.dec_stage;
    auto dec_jr_path = getJrPath(cfg.dec_backend, cfg.dec_dir, dec_stage);
    Network dec_network = loadNetwork(dec_jr_path.first, dec_jr_path.second);
    NetInfo dec_netinfo = NetInfo(dec_network);
    // Part2 writes directly into the first Conv input exposed by this view.
    NetworkView dec_network_view = dec_network.view(13);
    log.print("[Async][Init] Decoder uses runtime view(13), Conv input v245\n");
    Session decoder_sess = initSession(
        cfg.dec_backend, dec_network_view, device,
        cfg.dev_ocmOption, dec_netinfo.mmu || cfg.dev_mmuMode,
        cfg.dev_speedMode, cfg.dev_compressFtmp);
    decoder_sess.apply();

    // Keep this decoder-owned segment alive for all Part2 and Part3 frames.
    PlddrTensorBinding dec_conv_input;
    if (cfg.dec_backend != "zg330") {
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
        if (dec_conv_input.bytes != static_cast<uint64_t>(cfg.DECODER_INT8_BYTES)) {
            throw std::runtime_error("decoder Conv input byte size mismatch.");
        }
        checked_plddr_u32(dec_conv_input.phy_addr, "decoder Conv input");
    log.print("[Async][Init] Decoder Conv input v_id=%lld addr=0x%08X bytes=%llu\n",
              (long long)vid,
              (uint32_t)dec_conv_input.phy_addr,
              (unsigned long long)dec_conv_input.bytes);

    log.print("[Async][FPGA] Resetting custom op...\n");
    fpai_dev.defaultRegRegion().write(REG_RESET, 1, false);
    usleep(1000);
    fpai_dev.defaultRegRegion().write(REG_RESET, 0, false);
    usleep(1000);
    uint32_t fpga_ver = (uint32_t)fpai_dev.defaultRegRegion().read(REG_VERSION, false);
    log.print("[Async][FPGA] Version: 0x%08X\n", fpga_ver);

    // LUT is the only custom allocation; FEAT2D and v245 belong to the NPU runtimes.
    auto lut_mem = fpai_dev.defaultMemRegion().malloc(cfg.LUT_BYTES, 0, 64);
    log.print("[Async][FPGA] PLDDR LUT=0x%08X; FEAT2D and v245 use runtime PLDDR\n",
              (uint32_t)lut_mem->begin.addr());

    {
        std::ifstream lut_ifs(cfg.lut_path, std::ios::binary);
        if (!lut_ifs.is_open()) {
            throw std::runtime_error("Cannot open LUT: " + cfg.lut_path);
        }
        std::vector<char> lut_buf(cfg.LUT_BYTES);
        lut_ifs.read(lut_buf.data(), cfg.LUT_BYTES);
        lut_mem.write(0, lut_buf.data(), cfg.LUT_BYTES);
    }
    fpai_dev.defaultRegRegion().write(REG_LUT_BASE, (uint32_t)lut_mem->begin.addr(), false);
    fpai_dev.defaultRegRegion().write(REG_LUT_SIZE, cfg.LUT_COUNT, false);
    fpai_dev.defaultRegRegion().write(REG_BEV_PARAMS, 0x04C8C840, false);
    fpai_dev.defaultRegRegion().write(REG_IMG_PARAMS, 0x060400B0, false);

    BoundedQueue<std::shared_ptr<FrameContext>> part1_to_part2(BUFFER_COUNT);
    BoundedQueue<std::shared_ptr<FrameContext>> part2_to_part3(BUFFER_COUNT);
    std::exception_ptr part2_error;
    std::thread part2_thread([&] {
        try {
            std::shared_ptr<FrameContext> part2_ctx;
            while (!failed.load() && part1_to_part2.pop(part2_ctx)) {
                part2_ctx->part2_begin = Clock::now();
                const uint32_t part2_out_addr =
                    checked_plddr_u32(dec_conv_input.phy_addr, "decoder Conv input");
                fpai_dev.defaultRegRegion().write(
                    REG_FEAT2D_BASE,
                    checked_plddr_u32(part2_ctx->part1_feat2d_addr, "Async FEAT2D Runtime Tensor"),
                    false);
                fpai_dev.defaultRegRegion().write(REG_FEAT3D_WR, part2_out_addr, false);
                fpai_dev.defaultRegRegion().write(REG_FEAT3D_SIZE, cfg.DECODER_INT8_BYTES, false);
                fpai_dev.defaultRegRegion().write(REG_CTRL_START, 0x03, false);

                auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
                while (true) {
                    int done = fpai_dev.defaultRegRegion().read(REG_COMP_DONE, false);
                    if (done & 1) break;
                    if (std::chrono::steady_clock::now() > deadline) {
                        throw std::runtime_error("LUT engine timeout.");
                    }
                    usleep(100);
                }
                part2_ctx->part2_end = Clock::now();
                log.print("[Async][Part2] Frame %04d done: FEAT2D=0x%08X FEAT3D=0x%08X\n",
                          part2_ctx->frame_id,
                          (uint32_t)part2_ctx->part1_feat2d_addr,
                          part2_out_addr);
                if (!part2_to_part3.push(std::move(part2_ctx))) break;
            }
        } catch (...) {
            part2_error = std::current_exception();
            failed.store(true);
        }
        part2_to_part3.close();
    });

    auto stop_part2_worker = [&] {
        part1_to_part2.close();
        part2_to_part3.close();
        if (part2_thread.joinable()) part2_thread.join();
    };

    try {
    std::shared_ptr<FrameContext> ctx;
    while (!failed.load() && preprocess_to_device.pop(ctx)) {
        ctx->device_pop_time = Clock::now();
        ctx->device_queue_wait_ms = elapsed_ms(ctx->preprocess_end, ctx->device_pop_time);
        ctx->device_begin = Clock::now();
        log.print("[Async][Device] Frame %04d start, pre_queue=%zu, post_queue=%zu\n",
                  ctx->frame_id,
                  preprocess_to_device.size(),
                  device_to_post.size());

        auto ext_in_val = ext_network.inputs()[0];
        Tensor ext_in_tensor = data2Tensor<float>(ctx->input->current_tensor, ext_in_val);
        dmaInit(cfg.ext_backend, ext_netinfo.ImageMake_on, ext_in_tensor, device);

        ctx->part1_begin = Clock::now();
        std::vector<Tensor> feat_2d = extractor.forward({ext_in_tensor});
        for (auto& out_t : feat_2d) {
            if (!out_t.waitForReady(std::chrono::seconds(10))) {
                throw std::runtime_error("Extractor output wait timeout.");
            }
        }
        ctx->part1_end = Clock::now();

        const uint64_t runtime_feat2d_addr = feat_2d.at(0).data().addr();
        checked_plddr_u32(runtime_feat2d_addr, "Async Extractor runtime output");
        ctx->part1_feat2d_addr = runtime_feat2d_addr;
        log.print("[Async][Device] Frame %04d Part1 output directly feeds Part2: 0x%08X (%d B)\n",
                  ctx->frame_id, (uint32_t)runtime_feat2d_addr, cfg.FEAT2D_BYTES);

        if (!part1_to_part2.push(ctx)) break;
        if (!part2_to_part3.pop(ctx)) break;

        if (cfg.ext_backend != "host") {
            device.reset(1);
        }

        // setData reuses the INT8 buffer just written by Part2; no host upload occurs.
        Tensor dec_in_tensor(dec_conv_input.value);
        dec_in_tensor.setData(dec_conv_input.memchunk, dec_conv_input.offset);

        ctx->part3_begin = Clock::now();
        std::vector<Tensor> dec_output = decoder_sess.forward({dec_in_tensor});
        for (auto& out_t : dec_output) {
            if (!out_t.waitForReady(std::chrono::seconds(10))) {
                throw std::runtime_error("Decoder output wait timeout.");
            }
        }
        ctx->cls_data = tensor2Vector(dec_output[0]);
        ctx->bbox_data = tensor2Vector(dec_output[1]);
        ctx->dir_data = tensor2Vector(dec_output[2]);
        ctx->part3_end = Clock::now();
        if (cfg.dec_backend != "host") {
            device.reset(1);
        }

        log.print("[Async][Device] Frame %04d device done: cls=%zu bbox=%zu dir=%zu\n",
                  ctx->frame_id, ctx->cls_data.size(), ctx->bbox_data.size(), ctx->dir_data.size());
        if (!device_to_post.push(std::move(ctx))) break;
    }

    stop_part2_worker();
    if (part2_error) std::rethrow_exception(part2_error);
    } catch (...) {
        stop_part2_worker();
        throw;
    }

    preprocess_to_device.close();
    device_to_post.close();
    Device::Close(device);
    log.print("[Async][Device] Device closed\n");
    (void)ds;
    return 0;
}

int device_stage_multibuffer(const PipelineConfig& cfg,
                             BoundedQueue<std::shared_ptr<FrameContext>>& preprocess_to_part1,
                             BoundedQueue<int>& free_slots,
                             BoundedQueue<std::shared_ptr<FrameContext>>& device_to_post,
                             PipelineLogger& log,
                             std::atomic<bool>& failed)
{
    log.print("[Async][Init] Opening device for %d Session/PLDDR slots...\n", BUFFER_COUNT);
    auto device = Device::Open(cfg.device_url.c_str());
    auto fpai_dev = device.cast<FPAIDevice>();

    std::string ext_stage = cfg.ext_stage;
    auto ext_jr_path = getJrPath(cfg.ext_backend, cfg.ext_dir, ext_stage);
    Network ext_network = loadNetwork(ext_jr_path.first, ext_jr_path.second);
    NetInfo ext_netinfo(ext_network);

    std::string dec_stage = cfg.dec_stage;
    auto dec_jr_path = getJrPath(cfg.dec_backend, cfg.dec_dir, dec_stage);
    Network dec_network = loadNetwork(dec_jr_path.first, dec_jr_path.second);
    NetInfo dec_netinfo(dec_network);
    NetworkView dec_network_view = dec_network.view(13);
    auto view_input_val = dec_network_view.inputs()[0];
    auto view_input_type = view_input_val.tensorType();
    if (!shape_is_decoder_int8_conv_input(view_input_type) ||
        !view_input_type->element_dtype.getStorageType().isSInt(8)) {
        throw std::runtime_error("decoder view(13) input must be sint8 Conv input [1,32,200,200,32].");
    }

    std::vector<Session> extractor_sessions(BUFFER_COUNT);
    std::vector<Session> decoder_sessions(BUFFER_COUNT);
    std::vector<PlddrTensorBinding> decoder_inputs(BUFFER_COUNT);
    std::vector<MemChunk> extractor_output_chunks(BUFFER_COUNT);
    std::vector<MemChunk> decoder_input_chunks(BUFFER_COUNT);
    const int64_t extractor_output_vid = ext_network.outputs()[0]->v_id;
    const int64_t decoder_input_vid = view_input_val->v_id;

    for (int index = 0; index < BUFFER_COUNT; ++index) {
        extractor_sessions[index] = initSession(
            cfg.ext_backend, ext_network, device,
            cfg.dev_ocmOption, ext_netinfo.mmu || cfg.dev_mmuMode,
            cfg.dev_speedMode, cfg.dev_compressFtmp);

        decoder_sessions[index] = initSession(
            cfg.dec_backend, dec_network_view, device,
            cfg.dev_ocmOption, dec_netinfo.mmu || cfg.dev_mmuMode,
            cfg.dev_speedMode, cfg.dev_compressFtmp);
    }

    using ZGSegment = icraft::xrt::zg330::SegmentType;
    auto extractor_backend0 = extractor_sessions[0]->backends[0].cast<FPAIBackend>();
    auto decoder_backend0 = decoder_sessions[0]->backends[0].cast<FPAIBackend>();
    auto extractor_instr_chunk = fpai_dev.defaultMemRegion().malloc(
        extractor_backend0->logic_segment_map.at(ZGSegment::INSTR)->byte_size, true, 4096);
    auto extractor_weight_chunk = fpai_dev.defaultMemRegion().malloc(
        extractor_backend0->logic_segment_map.at(ZGSegment::WEIGHT)->byte_size, true, 4096);
    auto decoder_instr_chunk = fpai_dev.defaultMemRegion().malloc(
        decoder_backend0->logic_segment_map.at(ZGSegment::INSTR)->byte_size, true, 4096);
    auto decoder_weight_chunk = fpai_dev.defaultMemRegion().malloc(
        decoder_backend0->logic_segment_map.at(ZGSegment::WEIGHT)->byte_size, true, 4096);

    for (int index = 0; index < BUFFER_COUNT; ++index) {
        extractor_output_chunks[index] = fpai_dev.defaultMemRegion().malloc(cfg.FEAT2D_BYTES, true, 4096);
        decoder_input_chunks[index] = fpai_dev.defaultMemRegion().malloc(cfg.DECODER_INT8_BYTES, true, 4096);

        auto extractor_backend = extractor_sessions[index]->backends[0].cast<FPAIBackend>();
        auto decoder_backend = decoder_sessions[index]->backends[0].cast<FPAIBackend>();
        extractor_backend.userReuseSegment(extractor_instr_chunk, ZGSegment::INSTR);
        extractor_backend.userReuseSegment(extractor_weight_chunk, ZGSegment::WEIGHT);
        extractor_backend.userConnectNetwork(extractor_output_chunks[index], extractor_output_vid);
        decoder_backend.userReuseSegment(decoder_instr_chunk, ZGSegment::INSTR);
        decoder_backend.userReuseSegment(decoder_weight_chunk, ZGSegment::WEIGHT);
        decoder_backend.userConnectNetwork(decoder_input_chunks[index], decoder_input_vid);

        extractor_sessions[index].apply();
        decoder_sessions[index].apply();

        auto& binding = decoder_inputs[index];
        binding = get_decoder_conv_binding(
            decoder_sessions[index], static_cast<uint64_t>(view_input_type.numElements()));
        if (binding.bytes != static_cast<uint64_t>(cfg.DECODER_INT8_BYTES)) {
            throw std::runtime_error("decoder Conv input byte size mismatch.");
        }
        if (binding.phy_addr != decoder_input_chunks[index]->begin.addr()) {
            throw std::runtime_error("userConnectNetwork did not bind decoder v245 to its requested PLDDR chunk.");
        }
        for (int previous = 0; previous < index; ++previous) {
            if (decoder_inputs[previous].phy_addr == binding.phy_addr) {
                throw std::runtime_error("decoder Session slots unexpectedly share the same Conv input PLDDR.");
            }
        }
        log.print("[Async][Init] Slot %d FEAT2D=%08X decoder v245=%08X (%llu B)\n",
                  index,
                  (uint32_t)extractor_output_chunks[index]->begin.addr(),
                  (uint32_t)binding.phy_addr,
                  (unsigned long long)binding.bytes);
    }

    fpai_dev.defaultRegRegion().write(REG_RESET, 1, false);
    usleep(1000);
    fpai_dev.defaultRegRegion().write(REG_RESET, 0, false);
    usleep(1000);
    auto lut_mem = fpai_dev.defaultMemRegion().malloc(cfg.LUT_BYTES, 0, 64);
    {
        std::ifstream lut_ifs(cfg.lut_path, std::ios::binary);
        if (!lut_ifs.is_open()) throw std::runtime_error("Cannot open LUT: " + cfg.lut_path);
        std::vector<char> lut_buf(cfg.LUT_BYTES);
        lut_ifs.read(lut_buf.data(), cfg.LUT_BYTES);
        lut_mem.write(0, lut_buf.data(), cfg.LUT_BYTES);
    }
    fpai_dev.defaultRegRegion().write(REG_LUT_BASE, (uint32_t)lut_mem->begin.addr(), false);
    fpai_dev.defaultRegRegion().write(REG_LUT_SIZE, cfg.LUT_COUNT, false);
    fpai_dev.defaultRegRegion().write(REG_BEV_PARAMS, 0x04C8C840, false);
    fpai_dev.defaultRegRegion().write(REG_IMG_PARAMS, 0x060400B0, false);
    log.print("[Async][FPGA] LUT=0x%08X; FEAT2D/v245 use official per-slot connected PLDDR\n",
              (uint32_t)lut_mem->begin.addr());
    log.print("[Async][NPU] Session slots are applied once; reset follows each completed NPU forward\n");

    BoundedQueue<std::shared_ptr<FrameContext>> part1_to_part2(BUFFER_COUNT);
    BoundedQueue<std::shared_ptr<FrameContext>> part2_to_part3(BUFFER_COUNT);
    std::mutex npu_mutex;
    std::mutex error_mutex;
    std::exception_ptr stage_error;
    auto fail_stage = [&](std::exception_ptr error) {
        {
            std::lock_guard<std::mutex> lock(error_mutex);
            if (!stage_error) stage_error = error;
        }
        failed.store(true);
        preprocess_to_part1.close();
        part1_to_part2.close();
        part2_to_part3.close();
        device_to_post.close();
        free_slots.close();
    };

    std::thread part1_thread([&] {
        try {
            std::shared_ptr<FrameContext> ctx;
            std::vector<uint64_t> extractor_output_addrs(BUFFER_COUNT, 0);
            while (!failed.load() && preprocess_to_part1.pop(ctx)) {
                ctx->device_pop_time = Clock::now();
                ctx->device_queue_wait_ms = elapsed_ms(ctx->preprocess_end, ctx->device_pop_time);
                ctx->device_begin = Clock::now();
                {
                    std::lock_guard<std::mutex> npu_lock(npu_mutex);
                    auto ext_in_val = ext_network.inputs()[0];
                    Tensor ext_in_tensor = data2Tensor<float>(ctx->input->current_tensor, ext_in_val);
                    dmaInit(cfg.ext_backend, ext_netinfo.ImageMake_on, ext_in_tensor, device);
                    ctx->part1_begin = Clock::now();
                    auto feat_2d = extractor_sessions[ctx->buffer_index].forward({ext_in_tensor});
                    for (auto& out_t : feat_2d) {
                        if (!out_t.waitForReady(std::chrono::seconds(10))) {
                            throw std::runtime_error("Extractor output wait timeout.");
                        }
                    }
                    ctx->part1_end = Clock::now();
                    ctx->part1_feat2d_addr = feat_2d.at(0).data().addr();
                    checked_plddr_u32(ctx->part1_feat2d_addr, "Extractor runtime output");
                    if (ctx->part1_feat2d_addr != extractor_output_chunks[ctx->buffer_index]->begin.addr()) {
                        throw std::runtime_error("userConnectNetwork did not bind Extractor output to its requested PLDDR chunk.");
                    }
                    for (int other = 0; other < BUFFER_COUNT; ++other) {
                        if (other != ctx->buffer_index &&
                            extractor_output_addrs[other] == ctx->part1_feat2d_addr) {
                            throw std::runtime_error("extractor Session slots share one OUTPUT PLDDR address.");
                        }
                    }
                    extractor_output_addrs[ctx->buffer_index] = ctx->part1_feat2d_addr;
                    // Official runtime guidance resets after each completed forward.
                    // The connected output chunk remains valid for Part2 after reset.
                    device.reset(1);
                }
                log.print("[Async][Part1] Frame %04d slot=%d FEAT2D=0x%08X\n",
                          ctx->frame_id, ctx->buffer_index, (uint32_t)ctx->part1_feat2d_addr);
                if (!part1_to_part2.push(std::move(ctx))) break;
            }
        } catch (...) {
            fail_stage(std::current_exception());
        }
        part1_to_part2.close();
    });

    std::thread part2_thread([&] {
        try {
            std::shared_ptr<FrameContext> ctx;
            while (!failed.load() && part1_to_part2.pop(ctx)) {
                const auto& binding = decoder_inputs[ctx->buffer_index];
                ctx->part2_begin = Clock::now();
                fpai_dev.defaultRegRegion().write(
                    REG_FEAT2D_BASE,
                    checked_plddr_u32(ctx->part1_feat2d_addr, "Part2 FEAT2D"), false);
                fpai_dev.defaultRegRegion().write(
                    REG_FEAT3D_WR,
                    checked_plddr_u32(binding.phy_addr, "Part2 decoder v245"), false);
                fpai_dev.defaultRegRegion().write(REG_FEAT3D_SIZE, cfg.DECODER_INT8_BYTES, false);
                fpai_dev.defaultRegRegion().write(REG_CTRL_START, 0x03, false);
                const auto deadline = Clock::now() + std::chrono::seconds(30);
                while (true) {
                    if (fpai_dev.defaultRegRegion().read(REG_COMP_DONE, false) & 1) break;
                    if (Clock::now() > deadline) throw std::runtime_error("LUT engine timeout.");
                    usleep(100);
                }
                ctx->part2_end = Clock::now();
                log.print("[Async][Part2] Frame %04d slot=%d FEAT2D=0x%08X v245=0x%08X\n",
                          ctx->frame_id, ctx->buffer_index,
                          (uint32_t)ctx->part1_feat2d_addr, (uint32_t)binding.phy_addr);
                if (!part2_to_part3.push(std::move(ctx))) break;
            }
        } catch (...) {
            fail_stage(std::current_exception());
        }
        part2_to_part3.close();
    });

    std::thread part3_thread([&] {
        try {
            std::shared_ptr<FrameContext> ctx;
            while (!failed.load() && part2_to_part3.pop(ctx)) {
                const auto binding = decoder_inputs[ctx->buffer_index];
                {
                    std::lock_guard<std::mutex> npu_lock(npu_mutex);
                    // All slots are applied once at initialization. A device-wide reset
                    // invalidates another slot's applied ZG330 layer state, so no reset
                    // is issued while slots are interleaved on the shared NPU.
                    Tensor dec_in_tensor(binding.value);
                    dec_in_tensor.setData(binding.memchunk, binding.offset);
                    ctx->part3_begin = Clock::now();
                    auto dec_output = decoder_sessions[ctx->buffer_index].forward({dec_in_tensor});
                    for (auto& out_t : dec_output) {
                        if (!out_t.waitForReady(std::chrono::seconds(10))) {
                            throw std::runtime_error("Decoder output wait timeout.");
                        }
                    }
                    ctx->cls_data = tensor2Vector(dec_output[0]);
                    ctx->bbox_data = tensor2Vector(dec_output[1]);
                    ctx->dir_data = tensor2Vector(dec_output[2]);
                    ctx->part3_end = Clock::now();
                    device.reset(1);
                }
                log.print("[Async][Part3] Frame %04d slot=%d cls=%zu bbox=%zu dir=%zu\n",
                          ctx->frame_id, ctx->buffer_index,
                          ctx->cls_data.size(), ctx->bbox_data.size(), ctx->dir_data.size());
                if (!free_slots.push(ctx->buffer_index)) break;
                if (!device_to_post.push(std::move(ctx))) break;
            }
        } catch (...) {
            fail_stage(std::current_exception());
        }
    });

    if (part1_thread.joinable()) part1_thread.join();
    if (part2_thread.joinable()) part2_thread.join();
    if (part3_thread.joinable()) part3_thread.join();
    device_to_post.close();
    Device::Close(device);
    if (stage_error) std::rethrow_exception(stage_error);
    return 0;
}

} // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <config.yaml>\n", argv[0]);
        return 1;
    }

    try {
        PipelineConfig cfg = parse_config(argv[1]);

        fs::create_directories(cfg.box_dir);
        fs::create_directories(cfg.para_dir);
        fs::create_directories(cfg.log_dir);

        PipelineLogger log;
        const std::string log_file = cfg.log_dir + "/log_async.txt";
        if (!log.open(log_file)) {
            std::fprintf(stderr, "[Warning] Cannot open log file: %s\n", log_file.c_str());
        }

        log.print("============================================================\n");
        log.print("    FastBEV Pipeline ASYNC v3  (two-slot P1/P2/P3 pipeline)\n");
        log.print("============================================================\n");
        log.print("[Config] device_url : %s\n", cfg.device_url.c_str());
        log.print("[Config] extractor  : %s (stage=%s, backend=%s)\n",
                  cfg.ext_dir.c_str(), cfg.ext_stage.c_str(), cfg.ext_backend.c_str());
        log.print("[Config] decoder    : %s (stage=%s, backend=%s)\n",
                  cfg.dec_dir.c_str(), cfg.dec_stage.c_str(), cfg.dec_backend.c_str());
        log.print("[Config] buffer_count: %d\n", BUFFER_COUNT);
        log.print("[Config] PNG visualize: disabled\n");

        FastBEVDataset* ds = fastbev_load(cfg.json_path.c_str());
        if (!ds) {
            throw std::runtime_error("Failed to load dataset: " + cfg.json_path);
        }
        const int total_frames = ds->num_samples;
        log.print("[Dataset] %d frames loaded\n", ds->num_samples);

        BoundedQueue<int> free_slots(BUFFER_COUNT);
        for (int index = 0; index < BUFFER_COUNT; ++index) {
            free_slots.push(index);
        }
        BoundedQueue<std::shared_ptr<FrameContext>> preprocess_to_device(BUFFER_COUNT);
        BoundedQueue<std::shared_ptr<FrameContext>> device_to_post(BUFFER_COUNT);
        std::atomic<bool> failed{false};
        std::atomic<int> posted_frames{0};
        std::string failure_msg;
        std::mutex failure_mutex;

        std::thread pre_thread(preprocess_worker,
                               std::cref(cfg),
                               ds,
                               std::ref(free_slots),
                               std::ref(preprocess_to_device),
                               std::ref(log),
                               std::ref(failed),
                               std::ref(failure_msg),
                               std::ref(failure_mutex));

        std::thread post_thread(post_worker,
                                std::cref(cfg),
                                std::ref(device_to_post),
                                std::ref(log),
                                std::ref(posted_frames),
                                std::ref(failed),
                                std::ref(failure_msg),
                                std::ref(failure_mutex));

        int device_ret = 0;
        try {
            device_ret = device_stage_multibuffer(
                cfg, preprocess_to_device, free_slots, device_to_post, log, failed);
        } catch (const std::exception& e) {
            {
                std::lock_guard<std::mutex> lock(failure_mutex);
                failure_msg = e.what();
            }
            failed.store(true);
            free_slots.close();
            preprocess_to_device.close();
            device_to_post.close();
            device_ret = -1;
        }

        if (pre_thread.joinable()) pre_thread.join();
        if (post_thread.joinable()) post_thread.join();

        fastbev_free(ds);

        if (failed.load()) {
            std::lock_guard<std::mutex> lock(failure_mutex);
            log.print("[Async][Fatal] %s\n", failure_msg.c_str());
            return -1;
        }

        log.print("============================================================\n");
        log.print("  Async pipeline complete: %d/%d frames posted\n",
                  posted_frames.load(), total_frames);
        log.print("  Log saved to: %s\n", log_file.c_str());
        log.print("============================================================\n");
        return device_ret;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[Fatal] %s\n", e.what());
        return -1;
    }
}
