#pragma once

#include <string>
#include <vector>

#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

struct LandingSite {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;  // ground (pad) height above WGS84
};

struct BoosterReturnParameters {
    double flight_time_s = 230.0;  // planned time from boostback start to touchdown (Lambert arc shape)
    std::vector<std::string> boostback_groups{"booster/center", "booster/middle"};
    // Landing burn: starts with the inner engines (high deceleration close to the ground, small gravity
    // losses) and continues on the centre engines once these can finish it at the design throttle.
    std::vector<std::string> landing_ignition_groups{"booster/center", "booster/middle"};
    std::vector<std::string> landing_groups{"booster/center"};
    double landing_throttle = 0.8;  // design throttle of the inner-engine segment: the rest is control margin
    double final_throttle = 0.8;    // design throttle of the centre-engine segment (slower, time to settle)
    double gate_speed_mps = 50.0;   // sink rate at the hand-over to the centre engines
    double max_tilt_rad = 20.0 * 3.14159265358979 / 180.0;         // thrust from the vertical, centre engines
    double max_tilt_ignition_rad = 25.0 * 3.14159265358979 / 180.0; // with the inner engines (more gimbal authority)
    double flip_tolerance_rad = 15.0 * 3.14159265358979 / 180.0;
};

enum class ReturnPhase { Flip, Boostback, Coast, LandingBurn, Landed };

const char* to_string(ReturnPhase phase);

struct ReturnCommand {
    ReturnPhase phase = ReturnPhase::Flip;
    std::vector<std::string> run_groups;  // engine groups that should run now
    double throttle = 1.0;
    math::Vec3 axis_gcrf;                 // desired body x-axis
    math::Vec3 rcs_force_gcrf;            // RCS translation, fraction of the maximum force (|f| <= 1)
    double impact_miss_m = 0.0;           // predicted horizontal miss of the landing site (ballistic)
    double time_to_go_s = 0.0;            // landing burn
};

// Return of a first stage to its launch site:
//  Flip      - turn towards the boostback direction with the centre engines at minimum throttle
//  Boostback - burn along v_go of a Lambert arc whose aim point is shifted until the drag-aware
//              prediction of that arc hits the site (shooting on the aim point); cutoff at v_go ~ 0
//  Coast     - engines first (stable for a low CG), RCS holds the attitude and trims the predicted
//              impact point by translation [substitute for grid-fin steering, not modelled yet]
//  Landing   - ZEM/ZEV (zero-effort miss / zero-effort velocity) guidance in the Earth-fixed frame to a
//              slow touchdown on the site, time-to-go from a constant-deceleration profile. Ignition
//              from a one-dimensional simulation of the burn along the flight path (drag, gravity, mass
//              flow, engine start-up, engine-set switch) at the design throttle.
// The tower catch is modelled as a touchdown on the pad (no tower geometry yet).
class BoosterReturnGuidance {
public:
    BoosterReturnGuidance(LandingSite site, BoosterReturnParameters parameters = {});

    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);

    ReturnPhase phase() const { return phase_; }

    // Drag-aware ballistic impact prediction (point mass, tail-first axial drag from the vehicle's table,
    // power off). `velocity_gcrf_mps` overrides the body's velocity (e.g. a planned arc).
    struct Impact {
        math::Vec3 position_itrf_m;
        math::Vec3 velocity_itrf_mps;  // relative to the rotating Earth
        double time_s = 0.0;
        bool valid = false;
    };
    static Impact predict_impact(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                 time::Epoch epoch, double ground_height_m, const math::Vec3* velocity_gcrf_mps = nullptr);

    // One-dimensional landing-burn simulation along the current flight path: height above the ground
    // where the speed reaches zero (negative: below ground) and the burn duration.
    struct BurnPrediction {
        double stop_height_m = 0.0;
        double duration_s = 0.0;
    };
    BurnPrediction predict_landing_burn(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                        time::Epoch epoch) const;

private:
    LandingSite site_;
    BoosterReturnParameters parameters_;
    ReturnPhase phase_ = ReturnPhase::Flip;
    time::Epoch boostback_start_;
    bool started_ = false;
    math::Vec3 aim_offset_itrf_;  // horizontal shift of the Lambert aim point (drag compensation)
    bool final_engines_ = false;  // landing burn continues on the centre engines
    int tick_counter_ = 0;
    double impact_miss_m_ = 0.0;
    math::Vec3 trim_itrf_;  // RCS trim force fraction from the latest impact prediction
};

}  // namespace starsim::gnc
