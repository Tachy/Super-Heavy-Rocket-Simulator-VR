#pragma once

// Row-parallel loop of the weather builds (weather snapshots, cloud noise): the game builds them in-game, so a couple
// of cores stay free for the 200 Hz flight simulation thread and the render thread.

#include <algorithm>
#include <exception>
#include <thread>
#include <vector>

namespace starsim::environment::detail {

// Cores left to the rest of the process while a weather build runs.
inline constexpr int build_cores_left_free = 2;
inline constexpr int max_build_threads = 32;

// Runs fn(row) for every row on hardware_concurrency() - build_cores_left_free threads (at least 1, at most 32). An
// exception in a worker stops that worker; the first one is rethrown on the calling thread after all have joined.
template <class Fn>
void for_each_row(int rows, Fn fn) {
    if (rows <= 0) return;
    const int hardware = static_cast<int>(std::thread::hardware_concurrency());
    const int threads = std::clamp(hardware - build_cores_left_free, 1, std::min(rows, max_build_threads));
    if (threads == 1) {
        for (int r = 0; r < rows; ++r) fn(r);
        return;
    }
    std::vector<std::exception_ptr> errors(static_cast<std::size_t>(threads));
    std::vector<std::thread> pool;
    pool.reserve(static_cast<std::size_t>(threads));
    for (int t = 0; t < threads; ++t) {
        pool.emplace_back([&, t] {
            try {
                for (int r = t; r < rows; r += threads) fn(r);
            } catch (...) {
                errors[static_cast<std::size_t>(t)] = std::current_exception();
            }
        });
    }
    for (auto& th : pool) th.join();
    for (const auto& e : errors) {
        if (e) std::rethrow_exception(e);
    }
}

}  // namespace starsim::environment::detail
