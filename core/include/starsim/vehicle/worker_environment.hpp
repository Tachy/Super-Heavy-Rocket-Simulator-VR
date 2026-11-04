#pragma once

#include <memory>
#include <optional>

#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/environment/reference_body.hpp"
#include "starsim/ephemeris/interpolated_ephemeris.hpp"
#include "starsim/frames/interpolated_orientation.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::vehicle {

// The sources a session's environment is built from (they outlive every worker: the CSPICE adapters, serialised by
// their own lock) and how it samples them.
struct EnvironmentSources {
    const ephemeris::Ephemeris* ephemeris = nullptr;
    const frames::BodyOrientation* earth = nullptr;
    const frames::BodyOrientation* moon = nullptr;  // nullptr: no Moon reference body
    environment::PointMassGravityConfig gravity;
    double ephemeris_node_spacing_s = 0.0;    // InterpolatedEphemeris (<= 0: pass through)
    double orientation_node_spacing_s = 0.0;  // InterpolatedOrientation (<= 0: pass through)
    // Immutable, shared with the session (kept alive by the worker while it uses them).
    std::shared_ptr<const aero::RadiativeHeatingTable> radiative_heating;
    std::shared_ptr<const environment::EarthTerrain> earth_terrain;
};

// A FlightEnvironment of its own for one planner thread (Task 11b, planner work off the sim thread): the session's
// interpolated ephemeris and orientations fill their nodes on demand (mutable caches, not thread-safe), so a worker
// may not share them. This one samples the same sources on the same grid: a node is a pure function of its index, so
// every query returns bit for bit what the session's caches return. One thread at a time; a hand-over between threads
// needs a happens-before (the futures of the background planners give it).
class WorkerEnvironment {
public:
    explicit WorkerEnvironment(EnvironmentSources sources);
    WorkerEnvironment(const WorkerEnvironment&) = delete;
    WorkerEnvironment& operator=(const WorkerEnvironment&) = delete;

    // Takes the session's flags (fidelity switches, air density factor) over: call on the sim thread before handing it
    // to a worker.
    void copy_flags(const FlightEnvironment& from);

    const FlightEnvironment& env() const { return *env_; }
    const environment::GravityModel& gravity() const { return gravity_; }
    const frames::BodyOrientation& earth() const { return earth_; }
    const frames::BodyOrientation* moon_orientation() const { return moon_orientation_ ? &*moon_orientation_ : nullptr; }

private:
    EnvironmentSources sources_;
    ephemeris::InterpolatedEphemeris ephemeris_;
    frames::InterpolatedOrientation earth_;
    std::optional<frames::InterpolatedOrientation> moon_orientation_;
    environment::PointMassGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    std::optional<environment::ReferenceBody> moon_;
    std::optional<FlightEnvironment> env_;
};

}  // namespace starsim::vehicle
