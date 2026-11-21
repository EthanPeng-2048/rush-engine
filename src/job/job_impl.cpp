module;
#include <cstddef>
#include <functional>
#include <memory>
#include <utility>

#include <exstl/threadpool.hpp>

module rush.engine.job;

namespace rush {

struct JobSystem::Impl {
    std::unique_ptr<exstl::ThreadPool> pool;
    std::size_t activeWorkers = 0;
    bool configured = false;
};

JobSystem::JobSystem() : impl_(new Impl{}) {}

JobSystem& JobSystem::Get() {
    static JobSystem jobs;
    return jobs;
}

void JobSystem::Configure(std::size_t workers) {
    if (workers == impl_->activeWorkers && impl_->configured) return;
    impl_->pool.reset();
    impl_->activeWorkers = workers;
    impl_->configured = true;
    if (workers > 0) impl_->pool = std::make_unique<exstl::ThreadPool>(workers);
}

bool JobSystem::IsParallel() const { return impl_->pool != nullptr; }

std::size_t JobSystem::Workers() const { return impl_->activeWorkers; }

void JobSystem::ParallelFor(std::size_t count, const std::function<void(std::size_t)>& fn) {
    constexpr std::size_t kMinParallel = 4;
    if (!impl_->pool || count < kMinParallel) {
        for (std::size_t i = 0; i < count; ++i) fn(i);
        return;
    }
    impl_->pool->parallel_for_samples(count, [&](std::size_t i) { fn(i); });
}

}  // namespace rush
