module;
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
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

// Producer/consumer snapshot handoff — triple-buffered shared ownership.
//
// Three slots rotate between producer-exclusive (Back()), published, and
// spare. Publish() is an O(1) pointer swap — the payload is never copied on
// the producer side. CopyFront() hands out a shared_ptr, so the consumer
// pays only a refcount per frame instead of a deep snapshot copy, and may
// hold the result for the whole frame while the producer keeps publishing.
//
// ⚠ A bare reference into a slot is unsafe: the producer reuses slots and
// would overwrite a pose mid-draw (the race-start flicker seen with the
// original Front() API). Always go through CopyFront().
//
// The mutex only serializes pointer swaps and refcount handoff — never a
// payload copy. If the consumer still holds the slot Publish() would recycle
// (a stalled frame), the spare slot takes over; if that is held too, a fresh
// slot is minted and the contested one dies with the consumer's reference.
template <typename Snapshot>
class SnapshotTripleBuffer {
public:
    SnapshotTripleBuffer() {
        staging_ = std::make_shared<Snapshot>();
        published_ = std::make_shared<Snapshot>();
        spare_ = std::make_shared<Snapshot>();
    }

    // Producer-only: mutable staging slot (never read by consumers).
    Snapshot& Back() { return *staging_; }

    // Producer: publish staging → published (pointer swap, no payload copy).
    void Publish() {
        std::lock_guard lock(mutex_);
        std::swap(staging_, published_);
        if (staging_.use_count() > 1) {
            // Consumer still holds the old published slot — never write into
            // a shared buffer. Prefer the spare; mint one only when the
            // consumer is holding that too (two frames behind).
            if (spare_.use_count() == 1) {
                std::swap(staging_, spare_);
            } else {
                staging_ = std::make_shared<Snapshot>();
            }
        }
    }

    // Consumer: shared handle on the last published snapshot. Safe to keep
    // for the rest of the frame (and beyond) while the producer publishes.
    [[nodiscard]] std::shared_ptr<const Snapshot> CopyFront() const {
        std::lock_guard lock(mutex_);
        return published_;
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<Snapshot> staging_;    // producer write target
    std::shared_ptr<Snapshot> published_;  // consumer source
    std::shared_ptr<Snapshot> spare_;      // rotation reserve
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