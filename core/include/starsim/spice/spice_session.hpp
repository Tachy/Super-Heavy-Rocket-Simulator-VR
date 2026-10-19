#pragma once

#include <filesystem>
#include <stdexcept>
#include <vector>

namespace starsim::spice {

class SpiceError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

// The kernel set used by the simulator, in load order (later kernels take precedence).
std::vector<std::filesystem::path> default_kernel_files(const std::filesystem::path& kernel_dir);

// Owns the global CSPICE kernel pool. CSPICE is process-global and NOT thread-safe: exactly one session may exist;
// the adapters serialise CSPICE with a process-wide lock, but a SpiceSession must still be opened once, before any
// adapter call.
class SpiceSession {
public:
    explicit SpiceSession(const std::vector<std::filesystem::path>& kernel_files);
    ~SpiceSession();

    SpiceSession(const SpiceSession&) = delete;
    SpiceSession& operator=(const SpiceSession&) = delete;

    static SpiceSession open_default(const std::filesystem::path& kernel_dir) {
        return SpiceSession(default_kernel_files(kernel_dir));
    }

private:
    std::vector<std::filesystem::path> loaded_;
};

}  // namespace starsim::spice
