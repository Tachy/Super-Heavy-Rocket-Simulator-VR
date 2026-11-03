#include "webcast_report.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

namespace starsim::test {

void webcast_report(const std::string& case_id, const std::vector<WebcastRow>& rows) {
    const auto dir = std::filesystem::path(STARSIM_OUTPUT_DIR) / "webcast_report";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    std::ofstream out(dir / (case_id + ".md"), std::ios::binary | std::ios::trunc);
    out << "# Webcast report: " << case_id << "\n\n";
    out << "| quantity | ours | reference | tolerance | verdict | source | parameters |\n";
    out << "|---|---|---|---|---|---|---|\n";
    for (const auto& r : rows) {
        const double delta = r.ours - r.reference;
        const bool inside = std::abs(delta) <= r.tolerance;
        char line[512];
        std::snprintf(line, sizeof line, "| %s | %.4g | %.4g | +-%.4g | %s (%+.4g) | %s | %s |\n", r.quantity.c_str(), r.ours,
                      r.reference, r.tolerance, inside ? "inside" : "OUTSIDE", delta, r.reference_source.c_str(),
                      r.parameters.c_str());
        out << line;
        if (!inside) {
            std::printf("WARN webcast [%s] %s: ours %.4g vs %.4g +-%.4g (%+.4g) [%s]; parameters: %s\n", case_id.c_str(),
                        r.quantity.c_str(), r.ours, r.reference, r.tolerance, delta, r.reference_source.c_str(),
                        r.parameters.c_str());
        }
    }
    std::fflush(stdout);
}

}  // namespace starsim::test
