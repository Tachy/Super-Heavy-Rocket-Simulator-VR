#pragma once

#include <filesystem>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/report/report.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::sim {

// Where to land, how long to stay, from when on (lunar sites plan Task 8). `mission_id`: a lunar landing mission of
// config/missions (its vehicle, orbit kind, budget and [windows]); its own site and stay are replaced by these.
struct LunarWindowRequest {
    std::string mission_id;
    config::LunarSiteConfig site;
    double stay_s = 0.0;
    time::Epoch start;  // the earliest mission start
};

// A landing date that flies: the mission started at `mission_start` from the parking orbit is planned for this
// touchdown by the same planners the flight uses (outbound plan, touchdown point, budget, descent plan from the
// predicted LLO).
struct LunarLandingWindow {
    time::Epoch mission_start, touchdown, liftoff;   // liftoff = touchdown + stay
    double sun_elevation_rad = 0.0;                  // at touchdown, above the site's geometric horizon
    double sun_azimuth_rad = 0.0;                    // from north, east positive
    double daylight_after_touchdown_s = 0.0;         // to sunset (or the end of the scanned span at polar sites)
    double propellant_reserve_kg = 0.0;              // the budget's margin above the mission's reserve
    double llo_inclination_rad = 0.0;                // the LLO plane against the MOON_ME equator at touchdown
    mission::LunarOrbitKind orbit = mission::LunarOrbitKind::Llo;
};

// `windows`: by preference (Sun elevation's distance to the [windows] band, ties chronological). `refusal`: set when
// `windows` is empty - lunar.no_window(cause) with the most frequent cause across the candidates
// (lunar.no_daylight_for_stay, lunar.budget_short, lunar.no_site_pass, lunar.nrho_site_unreachable, or what else
// refused them, e.g. a missing site tile) - or lunar.window_search_cancelled (`windows`: those found until then).
struct LunarWindowResult {
    std::vector<LunarLandingWindow> windows;
    std::optional<report::Report> refusal;
};

// The landing windows of `request` within the mission's [windows] horizon_days after request.start:
// 1. daylight intervals at the site (mission::daylight_intervals) over the horizon and the stay after it;
// 2. touchdown candidates every lunar_window_candidate_step_s inside intervals that still hold the stay and
//    lunar_window_ascent_lead_s after it, and no earlier than the transfer and the descent lead after the start;
// 3. ranked by the Sun elevation's distance to [windows] sun_preferred_min/max_deg, ties chronological;
// 4. in that order the real chain per candidate - mission start = touchdown - transfer - descent lead
//    (start_lunar_landing_mission on a session of its own: placement, outbound plan with the LLO plane through the
//    site at that touchdown, touchdown point, budget), then the descent plan from the predicted LLO (TLI and insertion
//    flown impulsively, planned once settled in the LLO as the session does); its touchdown must again lie in the
//    daylight holding the stay - until [windows] count pass. The list spreads over the lunar days first: a candidate
//    whose daylight already holds a window is tried only after every other candidate (dates, not one morning's slots).
// The sessions are built from `prototype`'s sources and fidelity flags (prototype is only read: any thread may run the
// search while prototype flies). Runs on the caller's thread (seconds to minutes); `progress` gets 0..1 and cancels the
// search when it returns false. Results are cached in memory per (mission, site, stay, start to the hour, flags).
LunarWindowResult find_lunar_landing_windows(const FlightSession& prototype, const std::filesystem::path& config_dir,
                                             const LunarWindowRequest& request,
                                             const std::function<bool(double)>& progress);

// [E] spacing of the touchdown candidates (brief: every 6 h; the Sun climbs ~3 deg in 6 h at the equator).
inline constexpr double lunar_window_candidate_step_s = 6.0 * 3600.0;
// [E] daylight kept after the stay: the liftoff, the ascent to insertion (~7 min) and the touchdown's slip from the
// listed pass (braking phase ~10 min) stay in sunlight.
inline constexpr double lunar_window_ascent_lead_s = 3600.0;

}  // namespace starsim::sim
