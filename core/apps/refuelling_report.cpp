// refuelling_report: how many tanker flights the lunar mission needs (refuelling r6). Usage:
//   refuelling_report [--height-km 200] [--interval-h 24] [--phasing-limit-h 48] [--tanker-dry-t 209]
//                     [--reserve-t 12.6] [--initial-t 55.5] [--insulation 1] [--hls-wait-d 0] [--reference-phasing]
//   refuelling_report --fly | --fly-placed
// Without flags: the r5 reference flights (config/refuelling/reference_flights.toml). Exit code 0 when the campaign
// closes, 2 when it does not, 1 on an error. --fly (refuelling f2): the whole campaign flown in one session
// (refuelling_campaign_flown: the depot launched; --fly-placed: placed, two tankers) and compared with the calculator
// - 30-45 min for --fly; one progress line per flight on stderr; exit code 0 when the comparison passed, 2 when it
// failed, 1 when the campaign ended early or on an error.
#include <iostream>
#include <string>
#include <vector>

#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_campaign_flight.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/sim/refuelling_campaign_text.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/spice/spice_session.hpp"

namespace {

using namespace starsim;

int fly(const std::string& mission_id) {
    const auto kernels = spice::SpiceSession::open_default(STARSIM_KERNEL_DIR);
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter clock;
    sim::FlightSession session(ephemeris, earth, STARSIM_CONFIG_DIR, environment::PointMassGravityConfig{}, &moon);
    const auto start = sim::start_mission(session, STARSIM_CONFIG_DIR, mission_id, {}, clock);
    if (!start.refusal.empty()) {
        std::cerr << "refuelling_report: " << start.refusal << "\n";
        return 1;
    }
    const auto* driver = dynamic_cast<const sim::RefuellingCampaignFlight*>(session.supervisor());
    size_t reported = 0;
    const auto done = [&] {
        const auto phase = driver->state().phase;
        return phase == sim::CampaignFlightPhase::Complete || phase == sim::CampaignFlightPhase::Ended;
    };
    while (!done() && session.active()) {
        session.advance_to(session.epoch() + 3600.0);
        const auto& st = driver->state();
        for (; reported < st.rows.size() && st.phase != sim::CampaignFlightPhase::Flying; ++reported) {
            const auto& r = st.rows[reported];
            std::cerr << (r.hls ? std::string("HLS") : "tanker " + std::to_string(r.number)) << ": "
                      << (r.failed ? "failed" : "done") << ", depot " << r.depot_after_kg / 1000.0 << " t\n";
        }
    }
    const auto misses = driver->comparison_misses();
    std::cout << sim::campaign_flight_text(driver->state(), misses, clock);
    if (driver->state().phase != sim::CampaignFlightPhase::Complete) return 1;
    return misses.empty() ? 0 : 2;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::vector<std::string> args(argv + 1, argv + argc);
        if (args.size() == 1 && args.front() == "--fly") return fly("refuelling_campaign_flown");
        if (args.size() == 1 && args.front() == "--fly-placed") return fly("refuelling_campaign_flown_placed");
        for (const auto& a : args) {
            if (a == "--fly" || a == "--fly-placed") throw std::invalid_argument(a + " takes no other flags");
        }
        const auto levers = sim::campaign_levers_from_args(args);
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
