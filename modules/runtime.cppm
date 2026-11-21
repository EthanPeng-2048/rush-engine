module;
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>
#include <utility>

export module rush.engine.runtime;

// Simulation runtime: owns the fixed-step worker thread and a double-buffered
// snapshot handshake. Game code supplies the step function; render/input stay
// on the caller thread and only ever observe the published snapshot.
//
// Modes:
//   Inline   — Pump() runs steps on the caller thread (drive-test / WASM).
//   Threaded — Start() spawns a worker; SwapPublished() never blocks the
//              producer longer than a pointer exchange.
export namespace rush {

// Double buffer: producer writes the back slot, then publishes with an
// atomic index flip (release). Consumers read the front slot (acquire).
// One full frame of tearing risk is accepted only on the back slot while
// the consumer is not reading it — the flip is the sync point.
template <typename Snapshot>
class SnapshotDoubleBuffer {
public:
    // Producer: mutable access to the back buffer (not yet published).
    Snapshot& Back() { return slots_[back_]; }

    // Producer: publish back → front. Returns the previous front index so
    // the producer can reclaim it next frame (it becomes the new back).
    int Publish() {
        const int prevFront = front_.exchange(back_, std::memory_order_acq_rel);
        back_ = prevFront;
        return prevFront;
    }

    // Consumer: stable reference to the published snapshot. Valid until the
    // next Publish() from the producer — copy out if you need longer.
    const Snapshot& Front() const {
        return slots_[front_.load(std::memory_order_acquire)];
    }

private:
    Snapshot slots_[2];
    int back_ = 1;
    std::atomic<int> front_{0};
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
