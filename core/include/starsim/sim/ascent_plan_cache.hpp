#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include "starsim/sim/ascent_profile_planner.hpp"

namespace starsim::sim {

// A persistent, write-once cache of ascent plans shared by the test processes of one build (ctest runs every test case
// as its own process; many of them plan the same launch). Only active when the environment variable
// STARSIM_PLAN_CACHE_DIR names a directory (the test CMake sets it; the game never does). A plan is stored under the
// hash of its memo key and the contents of the config directory, so a changed config never reuses a plan; the test
// build wipes the directory on every relink, so changed code never does either. Every filesystem error is swallowed:
// the cache is an optimisation only.

// The cache directory from STARSIM_PLAN_CACHE_DIR; nullopt when unset, empty, or disabled.
std::optional<std::filesystem::path> plan_cache_dir();

// Process-wide off switch for tests that must fly their trials.
void disable_plan_cache();

// FNV-1a 64 (hex) over the sorted relative paths and bytes of every regular file under `config_dir`; computed once
// per config directory per process.
std::string config_contents_hash(const std::filesystem::path& config_dir);

// A versioned text format (first line "starsim-plan-cache 1"); doubles as hexfloat, so a round trip is bit-exact.
std::string encode_profile(const AscentProfile& profile);
std::optional<AscentProfile> decode_profile(std::string_view text);  // nullopt on any parse error

// The disk key of a plan: the planner's memo key plus the liftoff epoch (hexfloat). The memo key leaves the epoch out
// (one process reuses its plan at any hour), but a plan made at another epoch differs in the kick's last bits: with
// the epoch a cache hit is bit-identical to the plan the process would have made itself, whichever process ran first.
std::string plan_cache_key(const std::string& memo_key, time::Epoch liftoff);

// `key` is the disk key (plan_cache_key; it holds the config directory path, its contents enter through the hash). (it holds the config directory path; its contents enter through the hash).
std::optional<AscentProfile> load_cached_plan(const std::string& key, const std::filesystem::path& config_dir);
void store_cached_plan(const std::string& key, const std::filesystem::path& config_dir, const AscentProfile& profile);

}  // namespace starsim::sim
