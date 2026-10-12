#include "starsim/spice/spice_session.hpp"

#include <atomic>

#include "spice_check.hpp"

namespace starsim::spice {

namespace {
std::atomic<bool> g_session_active{false};
}

std::vector<std::filesystem::path> default_kernel_files(const std::filesystem::path& kernel_dir) {
    return {
        kernel_dir / "naif0012.tls",                         // leap seconds
        kernel_dir / "pck00011.tpc",                         // radii, orientation constants
        kernel_dir / "gm_de440.tpc",                         // GM consistent with DE440
        kernel_dir / "de440s.bsp",                           // Sun, Moon, planets 1849-2150
        kernel_dir / "earth_1962_260806_2126_combined.bpc",  // ITRF93 history + long-term prediction
        kernel_dir / "earth_latest_high_prec.bpc",           // ITRF93 high precision (overrides)
        kernel_dir / "moon_de440_250416.tf",                 // MOON_PA / MOON_ME frame definitions
        kernel_dir / "moon_pa_de440_200625.bpc",             // lunar libration from DE440
    };
}

SpiceSession::SpiceSession(const std::vector<std::filesystem::path>& kernel_files) {
    if (g_session_active.exchange(true)) {
        throw SpiceError("only one SpiceSession may exist at a time");
    }
    // Report errors through failed_c() instead of aborting the process.
    erract_c("SET", 0, const_cast<SpiceChar*>("RETURN"));
    errprt_c("SET", 0, const_cast<SpiceChar*>("NONE"));
    try {
        for (const auto& file : kernel_files) {
            if (!std::filesystem::exists(file)) {
                throw SpiceError("SPICE kernel missing: " + file.string() +
                                 " (run: python tools/data_pipeline/fetch.py)");
            }
            furnsh_c(file.string().c_str());
            detail::throw_if_failed("furnsh_c");
            loaded_.push_back(file);
        }
    } catch (...) {
        kclear_c();
        g_session_active = false;
        throw;
    }
}

SpiceSession::~SpiceSession() {
    kclear_c();
    reset_c();
    g_session_active = false;
}

}  // namespace starsim::spice
