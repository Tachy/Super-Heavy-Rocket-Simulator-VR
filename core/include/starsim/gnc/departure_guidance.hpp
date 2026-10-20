#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/proximity_guidance.hpp"
#include "starsim/gnc/proximity_transfer.hpp"
#include "starsim/gnc/translation_loop.hpp"

namespace starsim::gnc {

// Departure of a visiting vehicle after the separation springs (ISS i5, spec correction 5): the undock burn opens the
// separation to 0.15 m/s along the port axis, the vehicle backs out on the axis, departure burn 0 puts it on a
// passively safe CW arc out of the keep-out sphere and the approach ellipsoid (the i4 breakout with the ellipsoid
// requirement), then it drifts until it leaves the ellipsoid. The later departure burns 1-3 are not modelled (the
// deorbit planned from the drift orbit replaces them). Same inputs as the proximity guidance.

enum class DeparturePhase { UndockBurn, BackOut, DepartureBurn, Drift, Complete };
const char* to_string(DeparturePhase phase);  // German

struct DepartureParameters {
    double undock_speed_mps = 0.15;  // [S: NASASpaceflight Crew-4 / Demo-1 "undock burn 1 ... 0.15 m/s"]
    double undock_tolerance_mps = 0.005;  // [E] the undock burn ends this close to the speed
    double burn_range_m = 10.0;      // [E] departure burn 0 once the port is this far out (clear of petals and PMA)
    math::Vec3 ellipsoid_semi_axes_m{2000.0, 1000.0, 1000.0};  // [S: NASA, approach ellipsoid 4 x 2 x 2 km]
    double ellipsoid_exit_limit_s = 3600.0;  // [E] Demo-1 left the approach ellipsoid 17 min after undocking
    double burn_tolerance_mps = 0.01;        // [E] departure burn flown to this velocity error
};

struct DepartureEvent {
    time::Epoch epoch;
    DeparturePhase phase;
    std::string status;  // German
};

class DepartureGuidance {
public:
    DepartureGuidance(DepartureParameters departure, ProximityParameters proximity);
    ProximityOutput update(const ProximityInput& in);

    DeparturePhase phase() const { return phase_; }
    const std::string& status() const { return status_; }
    const std::vector<DepartureEvent>& events() const { return events_; }
    const std::optional<Breakout>& departure_burn() const { return burn_; }

private:
    void enter(DeparturePhase phase, std::string status, time::Epoch epoch);
    math::Vec3 axial_velocity_port(const ProximityInput& in) const;  // undock speed out, lateral onto the axis

    DepartureParameters d_;
    ProximityParameters p_;
    DeparturePhase phase_ = DeparturePhase::UndockBurn;
    bool started_ = false;
    std::optional<Breakout> burn_;
    // The state right after an impulsive departure burn (refuelling r5): the burn's target velocity is its free CW drift
    // at the current epoch - a fixed LVLH velocity would fight the Coriolis term (~2 n v), which a heavy chaser's cold gas
    // (the filled HLS: ~2.5 mm/s^2) cannot (it fired for hours).
    orbit::RelativeState burn_start_lvlh_;
    time::Epoch burn_epoch_;
    TranslationLoop loop_;
    std::string status_;
    std::vector<DepartureEvent> events_;
};

}  // namespace starsim::gnc
