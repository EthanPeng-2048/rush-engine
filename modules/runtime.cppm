module;
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <utility>

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

}  // namespace rush