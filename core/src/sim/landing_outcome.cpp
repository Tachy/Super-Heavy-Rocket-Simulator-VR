#include "starsim/sim/landing_outcome.hpp"

#include <cmath>

#include "starsim/frames/geodetic.hpp"

namespace starsim::sim {

using math::Vec3;

const char* to_string(LandingOutcome outcome) {
    switch (outcome) {
        case LandingOutcome::Flying: return "fliegt";
        case LandingOutcome::Caught: return "gefangen";
        case LandingOutcome::Landed: return "gelandet";
        case LandingOutcome::Splashdown: return "Wasserung";
        case LandingOutcome::Impact: return "Aufprall";
        case LandingOutcome::BrokenUp: return "zerbrochen";
        case LandingOutcome::TippedOver: return "umgekippt";
        case LandingOutcome::SplashedDown: return "gewassert";
    }
    return "";
}

bool over_water(const config::SeaConfig& sea, double east_m, double north_m) {
    return east_m * std::sin(sea.direction_rad) + north_m * std::cos(sea.direction_rad) >= sea.shoreline_distance_m;
}

Vec3 site_enu_m(const LaunchSite& site, const Vec3& itrf_m) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const Vec3 d = itrf_m - frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    return {dot(d, Vec3{-so, co, 0.0}), dot(d, Vec3{-sl * co, -sl * so, cl}), dot(d, Vec3{cl * co, cl * so, sl})};
}

const char* to_string(ReturnResult result) {
    switch (result) {
        case ReturnResult::Flying: return "fliegt";
        case ReturnResult::Caught: return "gefangen";
        case ReturnResult::UprightAbort: return "aufrecht abgebrochen";
        case ReturnResult::Lost: return "verloren";
    }
    return "?";
}

ReturnResult return_result(LandingOutcome outcome, double contact_tilt_rad, double contact_sink_mps) {
    switch (outcome) {
        case LandingOutcome::Flying: return ReturnResult::Flying;
        case LandingOutcome::Caught: return ReturnResult::Caught;
        case LandingOutcome::BrokenUp:
        case LandingOutcome::TippedOver: return ReturnResult::Lost;
        default: break;
    }
    const bool upright = contact_tilt_rad < upright_abort_max_tilt_rad && contact_sink_mps <= upright_abort_max_sink_mps;
    return upright ? ReturnResult::UprightAbort : ReturnResult::Lost;
}

LandingOutcome landing_outcome(const vehicle::FlightBody& body, const LaunchSite& site, const Vec3& position_itrf_m) {
    if (body.caught()) return LandingOutcome::Caught;
    if (!body.breakup_reason().empty()) return LandingOutcome::BrokenUp;
    if (!body.impacted() && !body.resting()) return LandingOutcome::Flying;
    if (body.vehicle().splashdown_recovery()) {  // capsule: judged by its sink at the water (FlightBody), not the site
        return body.resting() ? LandingOutcome::SplashedDown : LandingOutcome::Impact;
    }
    const Vec3 enu = site_enu_m(site, position_itrf_m);
    if (site.deck) {
        contact::LandingSurface ship;
        ship.deck = site.deck;
        if (ship.below(enu.x, enu.y).water) return LandingOutcome::Splashdown;
    } else if (site.sea && over_water(*site.sea, enu.x, enu.y)) {
        return LandingOutcome::Splashdown;
    }
    return body.resting() ? LandingOutcome::Landed : LandingOutcome::Impact;
}

}  // namespace starsim::sim
