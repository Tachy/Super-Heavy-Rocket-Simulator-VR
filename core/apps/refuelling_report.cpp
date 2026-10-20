// refuelling_report: how many tanker flights the lunar mission needs (refuelling r6). Usage:
//   refuelling_report [--height-km 200] [--interval-h 24] [--phasing-limit-h 48] [--tanker-dry-t 209]
//                     [--reserve-t 12.6] [--initial-t 55.5] [--insulation 1] [--hls-wait-d 0] [--reference-phasing]
// Without flags: the r5 reference flights (config/refuelling/reference_flights.toml). Exit code 0 when the campaign
// closes, 2 when it does not, 1 on an error.
#include <iostream>
#include <string>
#include <vector>

#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/sim/refuelling_campaign_text.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/spice/spice_session.hpp"

int main(int argc, char** argv) {
    using namespace starsim;
    try {
        const auto levers = sim::campaign_levers_from_args(std::vector<std::string>(argv + 1, argv + argc));
        const auto kernels = spice::SpiceSession::open_default(STARSIM_KERNEL_DIR);
        const spice::SpiceEphemeris ephemeris;
        const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
        const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
        const spice::SpiceTimeConverter clock;
        const auto report = sim::plan_refuelling_campaign({ephemeris, earth, moon, clock, STARSIM_CONFIG_DIR}, levers);
        std::cout << sim::campaign_report_text(report, clock);
        return report.result.closes ? 0 : 2;
    } catch (const std::exception& e) {
        std::cerr << "refuelling_report: " << e.what() << "\n";
        return 1;
    }
}
