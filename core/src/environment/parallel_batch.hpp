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

// Runs fn(k) for k in [0, count): contiguous chunks on batch_thread_count() threads (the caller takes the first). An
// exception in a worker is rethrown on the caller after all have joined.
template <class Fn>
void for_each_index_budgeted(int count, Fn fn) {
    if (count <= 0) return;
    auto& in_use = batch_threads_in_use();
    const int hardware = static_cast<int>(std::thread::hardware_concurrency());
    const int threads = batch_thread_count(count, in_use.load(std::memory_order_relaxed), hardware);
    in_use.fetch_add(threads, std::memory_order_relaxed);
    const auto chunk = [&](int t) {
        const int begin = static_cast<int>(static_cast<long long>(count) * t / threads);
        const int end = static_cast<int>(static_cast<long long>(count) * (t + 1) / threads);
        for (int k = begin; k < end; ++k) fn(k);
    };
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(threads));
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(threads - 1));
    for (int t = 1; t < threads; ++t) {
        pool.emplace_back([&, t] {
            try {
                chunk(t);
            } catch (...) {
                errors[static_cast<std::size_t>(t)] = std::current_exception();
            }
        });
    }
    try {
        chunk(0);
    } catch (...) {
        errors[0] = std::current_exception();
    }
    for (auto& th : pool) th.join();
    in_use.fetch_sub(threads, std::memory_order_relaxed);
    for (const auto& e : errors) {
        if (e) std::rethrow_exception(e);
    }
}

}  // namespace starsim::environment::detail
