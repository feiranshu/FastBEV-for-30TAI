#include "alert_manager.hpp"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>

#ifndef _WIN32
#include <csignal>
#include <fcntl.h>
#include <spawn.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>
extern char** environ;
#endif

namespace fastbev {
namespace {

constexpr float kPi = 3.14159265358979323846f;

bool finite_box(const BoundingBox& box) {
    return std::isfinite(box.x) && std::isfinite(box.y) &&
           std::isfinite(box.l) && std::isfinite(box.w) &&
           std::isfinite(box.yaw) && std::isfinite(box.score);
}

}  // namespace

AlertPolicy::AlertPolicy(const AlertPolicyConfig& config) : config_(config) {
    if (!(std::isfinite(config_.confidence_threshold) &&
          config_.confidence_threshold >= 0.0f && config_.confidence_threshold <= 1.0f &&
          std::isfinite(config_.emergency_distance_m) &&
          std::isfinite(config_.danger_distance_m) &&
          std::isfinite(config_.caution_distance_m) &&
          config_.emergency_distance_m > 0.0f &&
          config_.emergency_distance_m < config_.danger_distance_m &&
          config_.danger_distance_m < config_.caution_distance_m &&
          config_.caution_confirm_frames > 0 &&
          config_.danger_confirm_frames > 0 &&
          config_.emergency_confirm_frames > 0 && config_.clear_frames > 0 &&
          std::isfinite(config_.hysteresis_m) && config_.hysteresis_m >= 0.0f)) {
        throw std::invalid_argument("invalid alert policy configuration");
    }
}

void AlertPolicy::reset() {
    current_level_ = AlertLevel::None;
    confirm_count_.fill(0);
    clear_count_ = 0;
    last_frame_id_ = -1;
}

bool AlertPolicy::is_valid_target(const BoundingBox& box) const {
    return box.id >= 0 && box.id < static_cast<int>(config_.enabled_class.size()) &&
           config_.enabled_class[static_cast<std::size_t>(box.id)] &&
           finite_box(box) && box.l >= 0.0f && box.w >= 0.0f &&
           box.score >= config_.confidence_threshold;
}

float AlertPolicy::clearance_m(const BoundingBox& box) {
    const float c = std::cos(box.yaw);
    const float s = std::sin(box.yaw);
    const float local_x = c * (-box.x) + s * (-box.y);
    const float local_y = -s * (-box.x) + c * (-box.y);
    const float dx = std::max(std::fabs(local_x) - box.l * 0.5f, 0.0f);
    const float dy = std::max(std::fabs(local_y) - box.w * 0.5f, 0.0f);
    return std::sqrt(dx * dx + dy * dy);
}

std::string AlertPolicy::direction_for(float x, float y) {
    // FastBEV output coordinates: +X is right and +Y is forward. Measure the
    // bearing from +Y, with positive angles turning toward the vehicle's left,
    // so the existing front/left/right sectors remain intuitive.
    const float angle = std::atan2(-x, y) * 180.0f / kPi;
    if (angle >= -30.0f && angle < 30.0f) return "front";
    if (angle >= 30.0f && angle < 90.0f) return "front_left";
    if (angle >= 90.0f && angle < 150.0f) return "rear_left";
    if (angle >= 150.0f || angle < -150.0f) return "rear";
    if (angle >= -150.0f && angle < -90.0f) return "rear_right";
    return "front_right";
}

const char* AlertPolicy::level_name(AlertLevel level) {
    switch (level) {
        case AlertLevel::Caution: return "caution";
        case AlertLevel::Danger: return "danger";
        case AlertLevel::Emergency: return "emergency";
        default: return "none";
    }
}

const char* AlertPolicy::class_name(int class_id) {
    static const char* const names[] = {
        "car", "truck", "trailer", "bus", "construction_vehicle",
        "bicycle", "motorcycle", "pedestrian", "traffic_cone", "barrier"
    };
    return class_id >= 0 && class_id < 10 ? names[class_id] : "unknown";
}

float AlertPolicy::threshold_for(AlertLevel level) const {
    switch (level) {
        case AlertLevel::Caution: return config_.caution_distance_m;
        case AlertLevel::Danger: return config_.danger_distance_m;
        case AlertLevel::Emergency: return config_.emergency_distance_m;
        default: return -1.0f;
    }
}

int AlertPolicy::confirm_frames_for(AlertLevel level) const {
    switch (level) {
        case AlertLevel::Caution: return config_.caution_confirm_frames;
        case AlertLevel::Danger: return config_.danger_confirm_frames;
        case AlertLevel::Emergency: return config_.emergency_confirm_frames;
        default: return 1;
    }
}

AlertDecision AlertPolicy::evaluate(const std::vector<BoundingBox>& boxes, int frame_id) {
    if (last_frame_id_ >= 0 && frame_id != last_frame_id_ + 1) {
        confirm_count_.fill(0);
        clear_count_ = 0;
    }
    last_frame_id_ = frame_id;

    std::array<Observation, 4> nearest_for_level{};
    Observation nearest_valid;

    for (const BoundingBox& box : boxes) {
        if (!is_valid_target(box)) continue;
        const float clearance = clearance_m(box);
        if (!std::isfinite(clearance)) continue;

        if (!nearest_valid.valid || clearance < nearest_valid.clearance_m) {
            nearest_valid = {true, &box, clearance};
        }

        for (AlertLevel level : {AlertLevel::Caution, AlertLevel::Danger,
                                 AlertLevel::Emergency}) {
            Observation& selected = nearest_for_level[index(level)];
            if (clearance <= threshold_for(level) &&
                (!selected.valid || clearance < selected.clearance_m)) {
                selected = {true, &box, clearance};
            }
        }
    }

    for (AlertLevel level : {AlertLevel::Caution, AlertLevel::Danger,
                             AlertLevel::Emergency}) {
        int& count = confirm_count_[index(level)];
        if (nearest_for_level[index(level)].valid) {
            count = std::min(count + 1, confirm_frames_for(level));
        } else {
            count = 0;
        }
    }

    AlertLevel confirmed = AlertLevel::None;
    for (AlertLevel level : {AlertLevel::Emergency, AlertLevel::Danger,
                             AlertLevel::Caution}) {
        if (confirm_count_[index(level)] >= confirm_frames_for(level)) {
            confirmed = level;
            break;
        }
    }

    const AlertLevel previous = current_level_;
    if (index(confirmed) > index(current_level_)) {
        current_level_ = confirmed;
        clear_count_ = 0;
    } else if (confirmed == current_level_) {
        clear_count_ = 0;
    } else if (current_level_ != AlertLevel::None) {
        const float release_distance = threshold_for(current_level_) + config_.hysteresis_m;
        if (!nearest_valid.valid || nearest_valid.clearance_m > release_distance) {
            ++clear_count_;
            if (clear_count_ >= config_.clear_frames) {
                current_level_ = confirmed;
                clear_count_ = 0;
            }
        } else {
            clear_count_ = 0;
        }
    }

    AlertDecision decision;
    decision.level = current_level_;
    decision.frame_id = frame_id;
    decision.state_changed = previous != current_level_;

    Observation selected;
    if (current_level_ != AlertLevel::None) {
        selected = nearest_for_level[index(current_level_)];
        if (!selected.valid) selected = nearest_valid;
    }
    if (selected.valid) {
        decision.class_id = selected.box->id;
        decision.score = selected.box->score;
        decision.clearance_m = selected.clearance_m;
        decision.direction = direction_for(selected.box->x, selected.box->y);
    }
    return decision;
}

struct AlertLedController::Impl {
    static constexpr std::uint32_t kAlertCmd = 0x0B;
    static constexpr std::uint32_t kDuration = 0x0C;
    static constexpr std::uint32_t kDangerToggle = 0x0E;
    static constexpr std::uint32_t kEmergencyToggle = 0x0F;
    static constexpr std::uint32_t kAlertCaps = 0x25;
    static constexpr std::uint32_t kExpectedCaps = 0x414C0001;
    static constexpr std::size_t kMapSpan = 0x1000;

    Impl(const AlertLedConfig& cfg, RegisterObserver obs, ErrorReporter reporter,
         NowProvider provider)
        : config(cfg), observer(std::move(obs)), error_reporter(std::move(reporter)),
          now_provider(std::move(provider)) {
        if (config.dev_mem.empty() || config.register_base > 0xFFFFFFFFULL ||
            config.duration_ms == 0 || config.refresh_ms == 0 ||
            config.refresh_ms >= config.duration_ms || config.danger_toggle_ms == 0 ||
            config.danger_toggle_ms >= config.duration_ms ||
            config.emergency_toggle_ms == 0 ||
            config.emergency_toggle_ms >= config.duration_ms) {
            throw std::invalid_argument("invalid alert LED configuration");
        }
        if (!now_provider) now_provider = [] { return Clock::now(); };
        if (!config.enabled) return;
        if (config.dry_run) {
            ready = true;
            return;
        }
#ifdef _WIN32
        report_error("/dev/mem LED backend is only available on the Linux target");
#else
        const long page_size_value = ::sysconf(_SC_PAGESIZE);
        if (page_size_value <= 0) {
            report_error("cannot query system page size");
            return;
        }
        const std::uint64_t page_size = static_cast<std::uint64_t>(page_size_value);
        const std::uint64_t page_base = config.register_base & ~(page_size - 1ULL);
        page_delta = static_cast<std::size_t>(config.register_base - page_base);
        map_length = page_delta + kMapSpan;
        fd = ::open(config.dev_mem.c_str(), O_RDWR | O_SYNC);
        if (fd < 0) {
            report_error("cannot open " + config.dev_mem + ": errno " +
                         std::to_string(errno));
            return;
        }
        map_base = ::mmap(nullptr, map_length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                          static_cast<off_t>(page_base));
        if (map_base == MAP_FAILED) {
            map_base = nullptr;
            report_error("cannot map alert registers: errno " + std::to_string(errno));
            ::close(fd);
            fd = -1;
            return;
        }
        registers = reinterpret_cast<volatile std::uint32_t*>(
            static_cast<unsigned char*>(map_base) + page_delta);
        const std::uint32_t caps = registers[kAlertCaps];
        if (caps != kExpectedCaps) {
            char message[128];
            std::snprintf(message, sizeof(message),
                          "ALERT_CAPS mismatch: expected 0x%08X, got 0x%08X; LED disabled",
                          kExpectedCaps, caps);
            report_error(message);
            release_mapping();
            return;
        }
        ready = true;
#endif
    }

    ~Impl() {
        if (ready && last_level != AlertLevel::None) clear();
#ifndef _WIN32
        release_mapping();
#endif
    }

    void report_error(const std::string& message) {
        if (error_reporter) error_reporter(message);
        else std::fprintf(stderr, "[AlertLED] %s\n", message.c_str());
    }

    void write_reg(std::uint32_t word, std::uint32_t value) {
        if (observer) observer(word * 4U, value);
        if (config.dry_run) return;
#ifndef _WIN32
        if (registers) {
            registers[word] = value;
            std::atomic_thread_fence(std::memory_order_seq_cst);
        }
#endif
    }

    void trigger(AlertLevel level, Clock::time_point now) {
        write_reg(kDuration, config.duration_ms);
        write_reg(kDangerToggle, config.danger_toggle_ms);
        write_reg(kEmergencyToggle, config.emergency_toggle_ms);
        write_reg(kAlertCmd, (1U << 8U) | static_cast<std::uint32_t>(level));
        last_level = level;
        last_trigger = now;
    }

    void update(const AlertDecision& decision) {
        if (!ready) return;
        const auto now = now_provider();
        if (decision.level == AlertLevel::None) {
            if (last_level != AlertLevel::None) clear();
            return;
        }
        if (decision.level != last_level ||
            now - last_trigger >= std::chrono::milliseconds(config.refresh_ms)) {
            trigger(decision.level, now);
        }
    }

    void clear() {
        if (!ready) return;
        write_reg(kAlertCmd, 1U << 9U);
        last_level = AlertLevel::None;
    }

#ifndef _WIN32
    void release_mapping() {
        ready = false;
        registers = nullptr;
        if (map_base) {
            ::munmap(map_base, map_length);
            map_base = nullptr;
        }
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }
#endif

    AlertLedConfig config;
    RegisterObserver observer;
    ErrorReporter error_reporter;
    NowProvider now_provider;
    bool ready = false;
    AlertLevel last_level = AlertLevel::None;
    Clock::time_point last_trigger{};
#ifndef _WIN32
    int fd = -1;
    void* map_base = nullptr;
    std::size_t map_length = 0;
    std::size_t page_delta = 0;
    volatile std::uint32_t* registers = nullptr;
#endif
};

AlertLedController::AlertLedController(const AlertLedConfig& config,
                                       RegisterObserver observer,
                                       ErrorReporter error_reporter,
                                       NowProvider now_provider)
    : impl_(new Impl(config, std::move(observer), std::move(error_reporter),
                     std::move(now_provider))) {}

AlertLedController::~AlertLedController() = default;

void AlertLedController::update(const AlertDecision& decision) { impl_->update(decision); }
void AlertLedController::clear() { impl_->clear(); }
bool AlertLedController::available() const { return impl_->ready; }

struct AlertAudioPlayer::Impl {
    struct Request {
        AlertLevel level = AlertLevel::None;
        std::vector<std::string> paths;
    };

    explicit Impl(const AlertAudioConfig& cfg, RequestObserver obs, ErrorReporter reporter)
        : config(cfg), observer(std::move(obs)), error_reporter(std::move(reporter)) {
        if (config.cooldown_ms < 0 || config.device.empty() || config.asset_dir.empty()) {
            throw std::invalid_argument("invalid alert audio configuration");
        }
        if (config.enabled && !config.dry_run) {
            worker = std::thread(&Impl::worker_loop, this);
        }
    }

    ~Impl() { stop(); }

    void report_error(const std::string& message) {
        if (error_reporter) {
            error_reporter(message);
        } else {
            std::fprintf(stderr, "[AlertAudio] %s\n", message.c_str());
        }
    }

    std::string asset_path(const std::string& filename) const {
        const char last = config.asset_dir.back();
        return config.asset_dir + ((last == '/' || last == '\\') ? "" : "/") + filename;
    }

    std::vector<std::string> paths_for(const AlertDecision& decision) const {
        std::vector<std::string> paths;
        if (decision.level == AlertLevel::Emergency) {
            paths.push_back(asset_path("attention.wav"));
        }
        if ((decision.level == AlertLevel::Danger ||
             decision.level == AlertLevel::Emergency) &&
            decision.class_id >= 0 && !decision.direction.empty()) {
            paths.push_back(asset_path(
                decision.direction + "_" + AlertPolicy::class_name(decision.class_id) +
                ".wav"));
        }
        return paths;
    }

    void update(const AlertDecision& decision) {
        if (!config.enabled) return;
        const bool level_changed = decision.level != last_level;
        last_level = decision.level;

        if (decision.level != AlertLevel::Danger &&
            decision.level != AlertLevel::Emergency) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        if (!level_changed && alert_played_once &&
            now - last_alert_request < std::chrono::milliseconds(config.cooldown_ms)) {
            return;
        }

        std::vector<std::string> paths = paths_for(decision);
        if (paths.empty()) {
            report_error("cannot map alert decision to an audio asset");
            return;
        }
        last_alert_request = now;
        alert_played_once = true;
        enqueue({decision.level, std::move(paths)});
    }

    void enqueue(Request request) {
        if (observer) {
            for (const std::string& path : request.paths) observer(request.level, path);
        }
        if (config.dry_run) return;

#ifndef _WIN32
        pid_t interrupt_pid = -1;
#endif
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopping) return;
            if (pending && pending->level == AlertLevel::Emergency &&
                request.level != AlertLevel::Emergency) {
                return;
            }
            pending = std::move(request);
#ifndef _WIN32
            if (pending->level == AlertLevel::Emergency &&
                active_level == AlertLevel::Danger && active_pid > 0) {
                interrupt_pid = active_pid;
            }
#endif
        }
#ifndef _WIN32
        if (interrupt_pid > 0) ::kill(interrupt_pid, SIGTERM);
#endif
        condition.notify_one();
    }

    void worker_loop() {
        for (;;) {
            Request request;
            {
                std::unique_lock<std::mutex> lock(mutex);
                condition.wait(lock, [this] { return stopping || pending.has_value(); });
                if (stopping && !pending) return;
                request = std::move(*pending);
                pending.reset();
                worker_busy = true;
            }

#ifdef _WIN32
            report_error("aplay backend is only available on the Linux target");
            mark_idle();
#else
            for (const std::string& path : request.paths) {
                std::ifstream asset(path, std::ios::binary);
                if (!asset.good()) {
                    report_error("audio asset not found: " + path);
                    continue;
                }
                asset.close();

                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (stopping) return;
                }
                pid_t pid = -1;
                char* const argv[] = {
                    const_cast<char*>("aplay"), const_cast<char*>("-q"),
                    const_cast<char*>("-D"), const_cast<char*>(config.device.c_str()),
                    const_cast<char*>(path.c_str()), nullptr
                };
                const int spawn_result =
                    posix_spawnp(&pid, "aplay", nullptr, nullptr, argv, environ);
                if (spawn_result != 0) {
                    report_error("cannot start aplay (error " +
                                 std::to_string(spawn_result) + ")");
                    continue;
                }
                bool terminate_immediately = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    active_pid = pid;
                    active_level = request.level;
                    terminate_immediately = stopping;
                }
                if (terminate_immediately) ::kill(pid, SIGTERM);
                int status = 0;
                pid_t wait_result = -1;
                do {
                    wait_result = ::waitpid(pid, &status, 0);
                } while (wait_result < 0 && errno == EINTR);
                bool stopped = false;
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    if (active_pid == pid) {
                        active_pid = -1;
                        active_level = AlertLevel::None;
                    }
                    stopped = stopping;
                }
                if (!stopped &&
                    (wait_result < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)) {
                    report_error("aplay failed for: " + path);
                }
            }
            mark_idle();
#endif
        }
    }

    void mark_idle() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            worker_busy = false;
        }
        condition.notify_all();
    }

    bool wait_until_idle(std::chrono::milliseconds timeout) {
        if (!config.enabled || config.dry_run) return true;
        std::unique_lock<std::mutex> lock(mutex);
        return condition.wait_for(lock, timeout,
                                  [this] { return !pending && !worker_busy; });
    }

    void stop() {
#ifndef _WIN32
        pid_t pid = -1;
#endif
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (stopping) return;
            stopping = true;
            pending.reset();
#ifndef _WIN32
            pid = active_pid;
#endif
        }
#ifndef _WIN32
        if (pid > 0) ::kill(pid, SIGTERM);
#endif
        condition.notify_all();
        if (worker.joinable()) worker.join();
    }

    AlertAudioConfig config;
    RequestObserver observer;
    ErrorReporter error_reporter;
    AlertLevel last_level = AlertLevel::None;
    bool alert_played_once = false;
    std::chrono::steady_clock::time_point last_alert_request{};
    std::mutex mutex;
    std::condition_variable condition;
    std::optional<Request> pending;
    bool stopping = false;
    bool worker_busy = false;
    std::thread worker;
#ifndef _WIN32
    pid_t active_pid = -1;
    AlertLevel active_level = AlertLevel::None;
#endif
};

AlertAudioPlayer::AlertAudioPlayer(const AlertAudioConfig& config,
                                   RequestObserver observer,
                                   ErrorReporter error_reporter)
    : impl_(new Impl(config, std::move(observer), std::move(error_reporter))) {}

AlertAudioPlayer::~AlertAudioPlayer() = default;

void AlertAudioPlayer::update(const AlertDecision& decision) {
    impl_->update(decision);
}

bool AlertAudioPlayer::wait_until_idle(std::chrono::milliseconds timeout) {
    return impl_->wait_until_idle(timeout);
}

void AlertAudioPlayer::stop() {
    if (impl_) impl_->stop();
}

}  // namespace fastbev
