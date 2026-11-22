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
// Detects "car flickers between a recent historical pose and the current
// pose" (and related visual pops). Feed publish + render poses; optional
// dense player trace for offline inspection when hits are ambiguous.
//
// stage=publish hits → simulation / snapshot content jumps
// stage=render  hits → only what is drawn jumps (handoff / dual-source)
//
// Detection (any of):
//   1) return-to-history: |p - older| small while |p - prev| large
//   2) reverse step: consecutive deltas oppose (zigzag) above threshold
//   3) angular snap: quaternion far from previous (body spin pop)
// Distances are full 3D (x,y,z) — x/z alone misses suspension pops.
// (Already inside `export namespace rush`.)

class JitterProbe {
public:
    static constexpr int kWindow = 32;  // "a few frames ago" at 60–120 Hz
    static constexpr float kJumpMin = 0.03f;    // 3 cm — sub-metre flicker
    static constexpr float kReturnMax = 0.06f;  // within 6 cm of history
    static constexpr float kReverseMin = 0.025f;
    static constexpr float kQuatDotMin = 0.985f;  // ~10°+ snap between frames

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
                std::fprintf(log_,
                             "# jitter hit|trace stage car kind jump ret reverse "
                             "quatDot x y z prevX prevY prevZ tSec\n");
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

    // Pose sample with optional rotation (pass identity quat to skip ang check).
    bool Observe(const char* stage, int car, float x, float y, float z, float qx = 0.f,
                 float qy = 0.f, float qz = 0.f, float qw = 1.f) {
        std::lock_guard lock(mutex_);
        if (!enabled_ || car < 0 || car >= kMaxCars) return false;
        auto& h = hist_[StageIndex(stage)][car];
        const bool hasPrev = h.count >= 1;
        const float now = NowSec();

        float jump = 0.f;
        float ret = 1e9f;
        int retSlot = -1;
        float reverse = 0.f;
        float quatDot = 1.f;
        const char* kind = nullptr;

        if (hasPrev) {
            const float dx = x - h.xs[h.head];
            const float dy = y - h.ys[h.head];
            const float dz = z - h.zs[h.head];
            jump = std::sqrt(dx * dx + dy * dy + dz * dz);

            for (int i = 1; i < h.count && i < kWindow; ++i) {
                const int slot = (h.head + kWindow - i) % kWindow;
                const float ex = x - h.xs[slot];
                const float ey = y - h.ys[slot];
                const float ez = z - h.zs[slot];
                const float d = std::sqrt(ex * ex + ey * ey + ez * ez);
                if (d < ret) {
                    ret = d;
                    retSlot = i;
                }
            }

            // Reverse / zigzag: current step opposes previous step.
            if (h.count >= 2) {
                const int prevSlot = (h.head + kWindow - 1) % kWindow;
                const float px = h.xs[h.head] - h.xs[prevSlot];
                const float py = h.ys[h.head] - h.ys[prevSlot];
                const float pz = h.zs[h.head] - h.zs[prevSlot];
                const float plen = std::sqrt(px * px + py * py + pz * pz);
                const float clen = jump;
                if (plen >= kReverseMin && clen >= kReverseMin) {
                    const float dot = (dx * px + dy * py + dz * pz) / (plen * clen);
                    reverse = dot;  // <0 = opposing
                }
            }

            // Angular snap (only if caller supplied a real quat path: qw may be 1 default).
            const bool hasQuat = !(qx == 0.f && qy == 0.f && qz == 0.f && qw == 1.f) ||
                                 h.hasQuat;
            if (h.hasQuat && hasQuat) {
                quatDot = std::fabs(qx * h.qx[h.head] + qy * h.qy[h.head] + qz * h.qz[h.head] +
                                    qw * h.qw[h.head]);
            }
        }

        // Classify hit.
        // Strict return: must be much closer to history than to prev —
        // otherwise closed-circuit self-proximity (pass same point 0.5s
        // later) floods the log (observed: 155/159 false positives).
        const bool hitReturn = hasPrev && jump >= kJumpMin && ret <= kReturnMax &&
                               ret <= 0.45f * jump && retSlot >= 1;
        const bool hitReverse =
            hasPrev && reverse < -0.5f && jump >= kReverseMin && ret > kReturnMax;
        const bool hitQuat = hasPrev && h.hasQuat && quatDot < kQuatDotMin && jump < kJumpMin;
        if (hitReturn) kind = "return";
        else if (hitReverse) kind = "reverse";
        else if (hitQuat) kind = "quat";
        const bool hit = kind != nullptr;

        // Push current.
        h.head = (h.head + 1) % kWindow;
        h.xs[h.head] = x;
        h.ys[h.head] = y;
        h.zs[h.head] = z;
        h.qx[h.head] = qx;
        h.qy[h.head] = qy;
        h.qz[h.head] = qz;
        h.qw[h.head] = qw;
        h.hasQuat = h.hasQuat || !(qx == 0.f && qy == 0.f && qz == 0.f && qw == 1.f);
        if (h.count < kWindow) ++h.count;

        if (hit) {
            ++hits_;
            if (log_) {
                std::fprintf(log_,
                             "hit %s %d %s %.4f %.4f %.3f %.4f %.3f %.3f %.3f %.3f %.3f %.3f %.6f\n",
                             stage, car, kind, jump, ret, reverse, quatDot, x, y, z,
                             h.xs[(h.head + kWindow - 1) % kWindow],
                             h.ys[(h.head + kWindow - 1) % kWindow],
                             h.zs[(h.head + kWindow - 1) % kWindow], now);
                std::fflush(log_);
            }
        }
        return hit;
    }

    // Dense per-frame line for one subject (player / camera). Always safe to
    // call when enabled; use for offline zoom on a visual flash.
    void Trace(const char* tag, float x, float y, float z) {
        std::lock_guard lock(mutex_);
        if (!enabled_ || !log_) return;
        std::fprintf(log_, "trace %s 0 - 0 0 0 0 %.4f %.4f %.4f 0 0 0 %.6f\n", tag, x, y, z,
                     NowSec());
        // Flush occasionally to keep the file useful if the process dies.
        if ((traceSeq_++ % 30) == 0) std::fflush(log_);
    }

    void ResetHistories() {
        std::lock_guard lock(mutex_);
        for (auto& stage : hist_) {
            for (auto& h : stage) {
                h = Hist{};
            }
        }
        hits_ = 0;
        traceSeq_ = 0;
    }

private:
    JitterProbe() = default;
    ~JitterProbe() {
        if (log_) std::fclose(log_);
    }

    static int StageIndex(const char* stage) {
        return stage && stage[0] == 'r' ? 1 : 0;
    }

    static float NowSec() {
        using clock = std::chrono::steady_clock;
        static const clock::time_point t0 = clock::now();
        return std::chrono::duration<float>(clock::now() - t0).count();
    }

    struct Hist {
        float xs[kWindow] = {};
        float ys[kWindow] = {};
        float zs[kWindow] = {};
        float qx[kWindow] = {};
        float qy[kWindow] = {};
        float qz[kWindow] = {};
        float qw[kWindow] = {};
        int head = 0;
        int count = 0;
        bool hasQuat = false;
    };

    static constexpr int kMaxCars = 128;
    static constexpr int kStages = 2;

    mutable std::mutex mutex_;
    bool enabled_ = false;
    long hits_ = 0;
    long traceSeq_ = 0;
    std::string logPath_ = "jitter_probe.log";
    std::FILE* log_ = nullptr;
    Hist hist_[kStages][kMaxCars] = {};
};

}  // namespace rush
