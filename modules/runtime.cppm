module;
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

export module rush.engine.runtime;

// Simulation runtime: owns the fixed-step worker thread and a snapshot
// handshake. Game code supplies the step function; render/input stay on the
// caller thread and only ever observe a stable copy of the published state.
//
// Modes:
//   Inline   — Pump() runs steps on the caller thread (drive-test / WASM).
//   Threaded — Start() spawns a worker; consumers CopyFront() once per frame.
export namespace rush {

// Producer/consumer snapshot handoff.
//
// ⚠ A bare Front() reference is unsafe with a second thread: Publish() flips
// the index so the old front becomes the back slot and the producer overwrites
// it while the consumer is still drawing — the car flickers between its
// previous and current pose (seen clearly at race start). Consumers must
// CopyFront() (mutex-protected value copy) and hold that copy for the frame.
template <typename Snapshot>
class SnapshotDoubleBuffer {
public:
    // Producer-only: mutable staging slot (never read by consumers).
    Snapshot& Back() { return staging_; }

    // Producer: publish staging → published (copy under lock).
    void Publish() {
        std::lock_guard lock(mutex_);
        published_ = staging_;
    }

    // Consumer: value copy of the last published snapshot. Safe to keep for
    // the rest of the frame while the producer publishes again.
    [[nodiscard]] Snapshot CopyFront() const {
        std::lock_guard lock(mutex_);
        return published_;
    }

private:
    mutable std::mutex mutex_;
    Snapshot staging_;    // producer write target
    Snapshot published_;  // consumer source (guarded by mutex_)
};

// Worker that repeatedly calls StepFn(dt) at a fixed rate (or as fast as
// the producer wants via NotifyStep). StepFn must be safe to run off the
// main thread; snapshot publication is StepFn's responsibility via the
// double buffer it owns.
class SimThread {
public:
    SimThread() = default;
    ~SimThread() { Stop(); }

    SimThread(const SimThread&) = delete;
    SimThread& operator=(const SimThread&) = delete;

    // Start a worker that runs `loop` until Stop(). `loop` blocks as it
    // wishes (fixed sleep / condition wait); Stop wakes and joins.
    template <typename LoopFn>
    void Start(LoopFn&& loop) {
        Stop();
        stop_.store(false, std::memory_order_release);
        thread_ = std::thread([this, fn = std::forward<LoopFn>(loop)] { fn(); });
    }

    void Stop() {
        if (!thread_.joinable()) return;
        stop_.store(true, std::memory_order_release);
        cv_.notify_all();
        thread_.join();
    }

    [[nodiscard]] bool Running() const { return thread_.joinable(); }

    // Called from the worker loop to wait until Stop() or a notify.
    // Returns true if woken by Stop.
    bool Wait(std::chrono::milliseconds timeout) {
        std::unique_lock lock(mutex_);
        cv_.wait_for(lock, timeout, [this] { return stop_.load(std::memory_order_acquire); });
        return stop_.load(std::memory_order_acquire);
    }

    void Notify() {
        std::lock_guard lock(mutex_);
        cv_.notify_all();
    }

    [[nodiscard]] bool StopRequested() const { return stop_.load(std::memory_order_acquire); }

private:
    std::thread thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
};

// ── Jitter probe ─────────────────────────────────────────────────────────
// Real-time detector for "car flickers between a recent historical pose and
// the current pose" (seen at race start). Feed every pose that is published
// or drawn; on an A/B/history hit, append one line to a log file.
//
// Interpretation:
//   stage=publish hits  → simulation / snapshot content itself jumps
//   stage=render  hits  → only what is drawn jumps (handoff / dual-source)
//   both dirty          → upstream state, not just the render path
//
// Detection: current sample is far from the previous sample but close to some
// older sample in a short window (returned to history after a jump).
// (Already inside `export namespace rush`.)

class JitterProbe {
public:
    static constexpr int kWindow = 8;
    // Jump between consecutive samples that counts as "moved" [m].
    static constexpr float kJumpMin = 0.12f;
    // How close to an older sample counts as "back at history" [m].
    static constexpr float kReturnMax = 0.05f;

    static JitterProbe& Get() {
        static JitterProbe probe;
        return probe;
    }

    void SetEnabled(bool on) {
        std::lock_guard lock(mutex_);
        enabled_ = on;
        if (on && log_ == nullptr) {
            log_ = std::fopen(logPath_.c_str(), "a");
            if (log_) {
                std::fprintf(log_, "# jitter_probe stage car jump return x z prevX prevZ retSlot tSec\n");
                std::fflush(log_);
            }
        }
        if (!on && log_) {
            std::fclose(log_);
            log_ = nullptr;
        }
    }

    void SetLogPath(std::string path) {
        bool reopen = false;
        {
            std::lock_guard lock(mutex_);
            if (log_) {
                std::fclose(log_);
                log_ = nullptr;
            }
            logPath_ = std::move(path);
            reopen = enabled_;
        }
        if (reopen) SetEnabled(true);
    }

    [[nodiscard]] bool Enabled() const {
        std::lock_guard lock(mutex_);
        return enabled_;
    }

    [[nodiscard]] long Hits() const {
        std::lock_guard lock(mutex_);
        return hits_;
    }

    // One pose sample. Returns true when a history-return is detected.
    // thread-safe; stages must not share car indices (publish vs render have
    // separate histories).
    bool Observe(const char* stage, int car, float x, float z) {
        std::lock_guard lock(mutex_);
        if (!enabled_ || car < 0 || car >= kMaxCars) return false;
        auto& h = hist_[StageIndex(stage)][car];
        const float px = h.xs[h.head];
        const float pz = h.zs[h.head];
        const bool hasPrev = h.count >= 1;
        const float now = NowSec();

        float jump = 0.f;
        float ret = 1e9f;
        int retSlot = -1;
        if (hasPrev) {
            jump = std::sqrt((x - px) * (x - px) + (z - pz) * (z - pz));
            // Scan older slots (skip the just-about-to-overwrite head after push).
            for (int i = 1; i < h.count && i < kWindow; ++i) {
                const int slot = (h.head + kWindow - i) % kWindow;
                const float dx = x - h.xs[slot];
                const float dz = z - h.zs[slot];
                const float d = std::sqrt(dx * dx + dz * dz);
                if (d < ret) {
                    ret = d;
                    retSlot = i;
                }
            }
        }

        // Push current.
        h.head = (h.head + 1) % kWindow;
        h.xs[h.head] = x;
        h.zs[h.head] = z;
        if (h.count < kWindow) ++h.count;

        // retSlot>=1: matched a sample that is not the immediate previous
        // (i=0 would be previous; we start at i=1). Classic A-B-A is retSlot=1.
        const bool hit = hasPrev && jump >= kJumpMin && ret <= kReturnMax && retSlot >= 1;
        if (hit) {
            ++hits_;
            if (log_) {
                std::fprintf(log_, "%s %d %.4f %.4f %.3f %.3f %.3f %.3f %d %.6f\n", stage, car,
                             jump, ret, x, z, px, pz, retSlot, now);
                std::fflush(log_);
            }
        }
        return hit;
    }

    void ResetHistories() {
        std::lock_guard lock(mutex_);
        for (auto& stage : hist_) {
            for (auto& h : stage) {
                h = Hist{};
            }
        }
        hits_ = 0;
    }

private:
    JitterProbe() = default;
    ~JitterProbe() {
        if (log_) std::fclose(log_);
    }

    static int StageIndex(const char* stage) {
        return stage && stage[0] == 'r' ? 1 : 0;  // "render" → 1, else publish
    }

    static float NowSec() {
        using clock = std::chrono::steady_clock;
        static const clock::time_point t0 = clock::now();
        return std::chrono::duration<float>(clock::now() - t0).count();
    }

    struct Hist {
        float xs[kWindow] = {};
        float zs[kWindow] = {};
        int head = 0;
        int count = 0;
    };

    static constexpr int kMaxCars = 128;
    static constexpr int kStages = 2;  // 0=publish, 1=render

    mutable std::mutex mutex_;
    bool enabled_ = false;
    long hits_ = 0;
    std::string logPath_ = "jitter_probe.log";
    std::FILE* log_ = nullptr;
    Hist hist_[kStages][kMaxCars] = {};
};

}  // namespace rush
