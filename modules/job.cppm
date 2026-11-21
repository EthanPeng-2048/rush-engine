module;
#include <cstddef>
#include <functional>

export module rush.engine.job;

// Thin host over exstl::ThreadPool: data-parallel regions for AI / vehicle
// batches. Configure(0) forces the serial inline path (drive-test, WASM,
// deterministic harnesses) — same API, zero thread timing in assertions.
//
// ParallelFor is type-erased (std::function) and defined in job_impl.cpp so
// the ThreadPool lives in one TU — avoids TLS/ODR issues when the template
// body would otherwise be instantiated in every importing TU.
export namespace rush {

class JobSystem {
public:
    static JobSystem& Get();

    // workers == 0 → serial inline (deterministic). workers >= 1 → pool size.
    void Configure(std::size_t workers);

    [[nodiscard]] bool IsParallel() const;
    [[nodiscard]] std::size_t Workers() const;

    // fn(index) for index in [0, count). Serial when parallel is off or
    // count is below the dispatch threshold.
    void ParallelFor(std::size_t count, const std::function<void(std::size_t)>& fn);

private:
    JobSystem();
    struct Impl;
    Impl* impl_ = nullptr;
};

}  // namespace rush
