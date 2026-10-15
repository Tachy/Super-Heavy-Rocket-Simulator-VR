#pragma once

#include <string>
#include <vector>

#include "starsim/environment/reference_body.hpp"
#include "starsim/gnc/peg_steering.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/time/epoch.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

// Vacuum Isp of the usable engines in `groups` (stage-prefixed group ids; thrust-weighted, full throttle); 0 when none.
double groups_vacuum_isp_s(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups);

// Lunar ascent of a lander (l4). Group ids carry the stage prefix (engine group_id).
struct LunarAscentParameters {
    std::vector<std::string> liftoff_groups{"hls/landing"};
    std::vector<std::string> ascent_groups{"hls/vacuum"};  // user 2026-10-15: fixed RVacs (Isp 380 s), attitude by the RCS
    // A failed engine in the fixed ascent set: the set is cut at once (its off-axis thrust is a torque no RCS holds) and
    // these gimballed engines continue (user 2026-10-15).
    std::vector<std::string> engine_out_groups{"hls/sea_level_1", "hls/sea_level_2", "hls/sea_level_3"};
    // Steering set: the canted landing thrusters turn the hull by differential throttling (~20x the RCS torque); they
    // run from the liftoff (abort: from the abort) until the hull is turned onto PEG's first direction. The RCS alone
    // turned the ~650 t HLS at ~6e-4 rad/s^2: 60-80 deg behind PEG for the whole ascent.
    std::vector<std::string> steering_groups{"hls/landing"};
    double steering_alignment_rad = 2.0 * constants::deg_to_rad;  // [E] aligned: thrust axis this close to PEG ...
    double steering_rate_radps = 0.01;                            // [E] ... turning slower than this
    // [E] while steering, the common throttle stays this far from the minimum and from full throttle: at either limit
    // differential throttling has no authority (l4 C4: a ~340 t HLS at the 1 g limit sat at the minimum and stayed
    // 60-90 deg off PEG; at a 3 g limit it sat at full throttle and crashed), as
    // TerminalDescentParameters::steering_throttle_margin
    double steering_throttle_margin = 0.1;
    double raptor_start_height_m = 30.0;     // [E] the Raptors start this high (spec: "after a few tens of metres")
    double vertical_rise_height_m = 150.0;   // [E] PEG from here (Apollo: ~10 s vertical rise)
    double insertion_altitude_above_site_m = 15.0e3;  // [E] Apollo inserted ~18 km up; >= 10.8 km clears all terrain
    double aposelene_altitude_m = 100.0e3;   // the LLO (spec)
    bool northerly = true;                   // polar plane through the site, launched towards the north
    double abort_min_climb_mps = 5.0;        // [E] abort: climb at least this before PEG takes over ...
    // [E] ... and until a fresh PEG solution steers below this radial share: PEG's time to go comes from the horizontal
    // speed still to gain - from a fast, low state (an abort at the high gate: 1.4 km/s, 14 km below the insertion) it
    // ran to ~25 s, steered at its radial limit (0.9) and cut off sub-orbital.
    double abort_peg_max_radial = 0.85;
    double abort_delta_v_margin = 0.15;      // [E] gravity and steering losses on the abort's propellant check
    double abort_max_tilt_rad = 30.0 * constants::deg_to_rad;  // [E] abort: thrust tilt towards downrange while sinking
    // [D] the ascent set throttles down to this thrust acceleration, never below its minimum throttle (which sets a floor
    // of ~1.4-1.8 g near cutoff at 200-150 t). l4 C4 sweep (user 2026-10-15: "find the optimum"; 340 / 260 t landed):
    // propellant 139.2/109.7 t at 1 g, 138.7/109.7 at 1.5 g, 138.9/109.3 at 1.75 g, 139.7/108.5 at 2 g - flat within
    // 0.15 t over 1.5-2 g; from 2.5 g PEG's time to go degrades (103-105 km orbits, crash at 3 g). 1.5 g: the LLO
    // variant's optimum, farthest from that edge.
    double max_acceleration_mps2 = 1.5 * constants::standard_gravity_mps2;
};

enum class LunarAscentPhase { Liftoff, Vertical, Arrest, Peg, Cutoff };
// German: "Abheben", "Senkrechtaufstieg", "Sinken stoppen", "PEG", "Brennschluss"
const char* to_string(LunarAscentPhase phase);

struct LunarAscentCommand {
    std::vector<std::string> run_groups;
    double throttle = 1.0;
    math::Vec3 axis_gcrf;  // desired body x
    LunarAscentPhase phase = LunarAscentPhase::Liftoff;
    double time_to_go_s = 0.0;  // PEG
    bool cutoff = false;
};

// Ascent from the lunar surface (or an abort from a powered descent) into the (site + insertion) x aposelene ellipse:
// vertical liftoff on the liftoff set, the ascent set lit at raptor_start_height_m, PEG from vertical_rise_height_m.
class LunarAscentGuidance {
public:
    // `moon_gm_m3ps2` from the reference body; `site_height_m` = terrain height above the sphere at the liftoff
    // point (the insertion altitude is above it).
    LunarAscentGuidance(LunarAscentParameters parameters, double moon_gm_m3ps2, double site_height_m);
    // Liftoff from the ground (abort = false) or abort from flight (abort = true: Arrest first if sinking, plane = the
    // current Moon-relative orbital plane). Fixes the target plane.
    void start(const vehicle::FlightBody& body, const environment::BodyFrame& moon, time::Epoch epoch, bool abort);
    LunarAscentCommand update(const vehicle::FlightBody& body, const environment::BodyFrame& moon, time::Epoch epoch);
    LunarAscentPhase phase() const { return phase_; }
    const std::string& status() const { return status_; }  // German: engine-out notice
    const std::vector<std::string>& ascent_groups() const { return ascent_groups_; }
    // Propellant check for an abort from `body`'s state (rocket equation on the ascent set's vacuum Isp, the speed still
    // to gain to the insertion plus the sink to stop, times 1 + abort_delta_v_margin): the delta-v needed.
    double abort_delta_v_mps(const vehicle::FlightBody& body, const environment::BodyFrame& moon) const;
    PegTarget target() const { return target_; }

private:
    // The groups to run with the ascent set: the bridge groups stay on until the ascent set carries the weight.
    std::vector<std::string> with_bridge(const vehicle::Vehicle& vehicle, double weight_n);
    // Throttle of all running groups: the acceleration limit, within their common throttle range; full while bridging.
    double ascent_throttle(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups) const;

    LunarAscentParameters parameters_;
    double gm_;
    double site_height_m_;
    PegTarget target_;
    double liftoff_radius_m_ = 0.0;
    std::vector<std::string> ascent_groups_;
    std::vector<std::string> bridge_groups_;  // running before the ascent set carries the weight (liftoff set, abort)
    LunarAscentPhase phase_ = LunarAscentPhase::Liftoff;
    bool engine_out_ = false;
    bool steering_ = true;  // the steering set runs (until aligned on PEG's direction)
    std::string status_;
    PegSteering peg_;
    bool debug_ = false;  // STARSIM_DEBUG_LUNAR_ASCENT
    time::Epoch next_debug_;
};

}  // namespace starsim::gnc
