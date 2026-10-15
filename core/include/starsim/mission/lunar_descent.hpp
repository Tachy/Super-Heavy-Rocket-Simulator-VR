#pragma once

#include <optional>
#include <string>

#include "starsim/environment/gravity.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

struct LunarDescentParameters {
    double pdi_altitude_above_site_m = 15.0e3;  // [S] Apollo: PDI at ~50,000 ft above the landing site
    double braking_range_m = 450.0e3;           // [E] spec: PDI 400-500 km before the site (ground arc)
    double max_crossrange_m = 20.0e3;           // [E] user 2026-10-15: passes farther off are not taken
    double search_s = 30.0 * 86400.0;           // [E] more than one lunar day (two passes per lunar day)
    double min_lead_s = 1800.0;                 // [E] the first burn at least this after the plan
    double trim_threshold_mps = 0.3;            // [E] smaller plane trims are skipped
    double trim_tolerance_m = 50.0;             // [E] predicted cross-track miss after the trim
    double doi_tolerance_m = 50.0;              // [E] predicted periselene radius after the DOI
    double slope_limit_rad = 8.0 * constants::deg_to_rad;  // [E] NASA ~8-10 deg; the 12 m stance lands >= 9 deg (l2)
    double roughness_limit_m = 0.5;             // [E] terrain off the footprint's plane (leg stroke 0.6 m)
    double touchdown_search_radius_m = 1000.0;  // [E]
    double touchdown_grid_m = 20.0;             // the LOLA tile's pixel
    double glide_slope_rad = 30.0 * constants::deg_to_rad;  // the SCvx glide cone (DescentLimits)
    double glide_check_range_m = 3000.0;        // [E] the approach from the high gate
    double glide_margin_m = 50.0;               // [E] terrain below the cone by at least this
};

// Ground range of the braking phase for a lander at PDI (l4 mass correction: the HLS reaches PDI with 500-570 t, not
// ~1,185 t, and a fixed 450 km over-braked it): v^2 / (2 a) with a = thrust_fraction x thrust_n / mass_kg, times
// range_factor (the Apollo law's thrust falls over the phase), clamped to [min_range_m, max_range_m].
struct BrakingRangeParameters {
    // [E] l4 C2 sweep (fraction 0.35/0.45/0.55 x factor 0.8/1.0/1.2, PDI 566 and 500 t): all 18 land within 0.21 m;
    // shorter ranges cost less (2055-2206 m/s); 0.55 x 1.0 keeps the range mass-dependent above the 150 km floor
    // (566 t: 164 km, 2068 m/s; 500 t: 150 km, 2062 m/s).
    double thrust_fraction = 0.55;  // of the first braking set's maximum
    double range_factor = 1.0;
    double min_range_m = 150.0e3;   // [E]
    double max_range_m = 600.0e3;   // [E]
};
double braking_range_m(double pdi_speed_mps, double mass_kg, double thrust_n, const BrakingRangeParameters& p);

struct TouchdownPoint {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;   // terrain above the sphere
    double slope_rad = 0.0;  // of the footprint's plane
    double offset_m = 0.0;   // from the mission's site
};

// Nearest admissible touchdown point to (latitude, longitude) for an approach towards `approach_azimuth_rad`
// (clockwise from north): footprint (the leg circle and its centre) fits a plane within the slope and roughness limits,
// and the terrain uprange stays below the glide cone. Candidates on a grid in the site's east/north plane, nearest
// first; empty when none lies within the search radius.
std::optional<TouchdownPoint> select_touchdown_point(const environment::LunarTerrain& terrain, double latitude_rad,
                                                     double longitude_rad, double approach_azimuth_rad,
                                                     double footprint_radius_m, const LunarDescentParameters& p);

struct LunarDescentPlan {
    bool feasible = false;
    std::string reason;  // German when refused
    std::optional<gnc::BurnNode> plane_trim;
    gnc::BurnNode doi;
    time::Epoch pdi_epoch;        // powered descent initiation (the braking guidance starts)
    time::Epoch site_pass_epoch;  // the unperturbed orbit's closest pass of the touchdown point
    TouchdownPoint touchdown;
    double approach_azimuth_rad = 0.0;  // ground track at the touchdown point, clockwise from north
    double crossrange_before_trim_m = 0.0;
    double crossrange_m = 0.0;            // predicted at the pass after the trim
    double periselene_altitude_m = 0.0;   // predicted after the DOI, above the sphere
    double pdi_range_m = 0.0;             // ground arc PDI -> touchdown point (predicted)
};

// From the LLO: the first pass of the site within max_crossrange_m after min_lead_s + 1.25 revolutions, a plane trim
// 1.25 revolutions before it (the out-of-plane offset of an impulse peaks a quarter revolution later and repeats every
// revolution - the trim must come before the DOI), the DOI half a revolution before the PDI point, PDI braking_range_m
// before the touchdown point. All predictions in the full force model (Earth-centred GCRF states, as l3's planners).
// The plan does not check propellant (the session does it with lunar_budget).
class LunarDescentPlanner {
public:
    LunarDescentPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                        const frames::BodyOrientation& moon_orientation);

    LunarDescentPlan plan(const math::StateVector& state_gcrf, time::Epoch epoch,
                          const environment::LunarTerrain& terrain, double site_latitude_rad,
                          double site_longitude_rad, double footprint_radius_m,
                          const LunarDescentParameters& p) const;

private:
    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& moon_orientation_;
    double gm_moon_;
};

}  // namespace starsim::mission
