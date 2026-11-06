#pragma once

// Chunked parallel loop for point batches that several callers may run at once (terrain node builds on the game's
// worker threads): the threads in use across all running batches share one budget, hardware_concurrency() -
// batch_cores_left_free, so a lone caller (the node under the camera) spreads over the free cores while many
// concurrent callers stay near one thread each. Every index is computed by exactly one thread with the same code, so
// the result does not depend on the thread count (bit for bit the serial loop).

#include <algorithm>
#include <atomic>
#include <exception>
#include <thread>
#include <utility>
#include <vector>

namespace starsim::environment::detail {

// Cores left to the rest of the process (simulation tick, render and main threads).
inline constexpr int batch_cores_left_free = 2;
inline constexpr int max_batch_threads = 32;
// [E] Least indices per thread: below this a thread's start-up (~50 us, its thread_local memos) outweighs the work.
inline constexpr int min_batch_chunk = 256;

// Threads of all running batches (each caller counts itself).
inline std::atomic<int>& batch_threads_in_use() {
    static std::atomic<int> in_use{0};
    return in_use;
}

// Threads for a batch of `count` indices given `in_use` threads already running batches and `hardware` cores: what is
// left of the budget (at least 1, the caller), at most one per min_batch_chunk indices and max_batch_threads.
inline int batch_thread_count(int count, int in_use, int hardware) {
    const int free = hardware - batch_cores_left_free - in_use;
    const int by_size = std::max(1, count / min_batch_chunk);
    return std::clamp(free, 1, std::min(by_size, max_batch_threads));
}

// Starts a worker thread running `work` and appends it to `pool` (the default spawner of for_each_index_budgeted; a test
// passes one that fails).
struct ThreadSpawner {
    template <class Work>
    void operator()(std::vector<std::thread>& pool, Work&& work) const {
        pool.emplace_back(std::forward<Work>(work));
    }
};

// Runs fn(k) for k in [0, count): contiguous chunks on batch_thread_count() threads (the caller takes the first). An
// exception in a worker is rethrown on the caller after all have joined. If starting a worker fails (std::system_error
// from std::thread, final review m2), no further workers are started and the caller runs the chunks that have no
// thread: every index is still computed exactly once, the threads started so far are joined and the budget released.
template <class Fn, class Spawn = ThreadSpawner>
void for_each_index_budgeted(int count, Fn fn, Spawn spawn = {}) {
    if (count <= 0) return;
    auto& in_use = batch_threads_in_use();
    const int hardware = static_cast<int>(std::thread::hardware_concurrency());
    const int threads = batch_thread_count(count, in_use.load(std::memory_order_relaxed), hardware);
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(threads));
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(threads - 1));
    in_use.fetch_add(threads, std::memory_order_relaxed);
    struct BudgetRelease {
        std::atomic<int>& in_use;
        int threads;
        ~BudgetRelease() { in_use.fetch_sub(threads, std::memory_order_relaxed); }
    } release{in_use, threads};
    const auto chunk = [&](int t) {
        const int begin = static_cast<int>(static_cast<long long>(count) * t / threads);
        const int end = static_cast<int>(static_cast<long long>(count) * (t + 1) / threads);
        for (int k = begin; k < end; ++k) fn(k);
    };
    const auto guarded = [&](int t) {
        try {
            chunk(t);
        } catch (...) {
            errors[static_cast<std::size_t>(t)] = std::current_exception();
        }
    };
    int started = 1;  // the caller's chunk 0
    try {
        for (int t = 1; t < threads; ++t) {
            spawn(pool, [&guarded, t] { guarded(t); });
            started = t + 1;
        }
    } catch (...) {
        // Thread creation failed: the remaining chunks run on the caller below.
    }
    guarded(0);
    for (int t = started; t < threads; ++t) guarded(t);
    for (auto& th : pool) th.join();
    for (const auto& e : errors) {
        if (e) std::rethrow_exception(e);
    }
}

}  // namespace starsim::environment::detail
