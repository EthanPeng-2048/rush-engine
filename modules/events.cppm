module;
#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>
#include <vector>

export module rush.engine.events;

// Fully asynchronous event bus: producers Publish and return immediately;
// consumers only observe events through Drain on their own stage tick.
// There is no producer-thread callback API — modules never wait on each other.
export namespace rush {

template <typename E>
class EventBus {
public:
    // Non-blocking publish. Backpressure drops the oldest event so a slow
    // consumer can never stall a producer (physics thread, etc.).
    void Publish(const E& event) {
        std::lock_guard lock(mutex_);
        queue_.push_back(event);
        constexpr std::size_t kMaxQueue = 4096;
        if (queue_.size() > kMaxQueue) queue_.pop_front();
    }

    // Consumer-side drain: hand every pending event to `consumer`, then clear.
    // Only the owning stage may call this — never a producer stack.
    template <typename Fn>
    void Drain(Fn&& consumer) {
        std::deque<E> local;
        {
            std::lock_guard lock(mutex_);
            local.swap(queue_);
        }
        for (E& e : local) consumer(e);
    }

    // Convenience: drain into a vector (cleared first).
    void Drain(std::vector<E>& out) {
        out.clear();
        Drain([&out](E& e) { out.push_back(std::move(e)); });
    }

    [[nodiscard]] std::size_t Pending() const {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

private:
    mutable std::mutex mutex_;
    std::deque<E> queue_;
};

}  // namespace rush
