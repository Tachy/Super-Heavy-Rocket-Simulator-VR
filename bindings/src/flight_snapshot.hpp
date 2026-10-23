#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/refuelling_campaign_flight.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::bindings {

// Trajectory prediction the main thread asked for (FlightSim::predict_trajectory); index -1: none.
struct PredictionRequest {
    int index = -1;
    double duration_s = 0.0;
    double step_s = 0.0;
    bool operator==(const PredictionRequest&) const = default;
};

// What the commands set on the simulation thread and the snapshot carries to the main thread. Only the worker
// touches it (commands and the observer).
struct SnapshotContext {
    godot::String site_id;
    godot::String mission_display_name;  // [mission] display_name of the started mission ("" for a free launch)
    double mission_start_tdb_s = 0.0;
    PredictionRequest prediction_request;  // copied from the main thread before every build
    std::optional<std::chrono::steady_clock::time_point> prediction_wall;  // when the last prediction was made
    // ISS missions (i6): from start_mission's result
    godot::Array mission_warnings;
    double iss_liftoff_tdb_s = 0.0, iss_separation_tdb_s = 0.0, iss_plane_bias_deg = 0.0;
    // The onboard computer plans a launch on the pad (launch_from_site with a target, engage_ascent_autopilot on a
    // clamped stack, start_mission of a pad launch - also before the session holds a flight): autopilot_status()
    // ["planning"] while the command runs (ascent trajectory and staging, Task 5; final review item 2).
    bool planning = false;
};

// Everything the main thread reads, built on the simulation thread after every command and chunk (immutable once
// published). Godot containers are created on the worker and only read afterwards - GDScript must not modify them.
struct FlightSnapshot {
    bool active = false;
    double epoch_tdb_s = 0.0;
    double mission_time_s = 0.0;
    double mission_start_tdb_s = 0.0;
    godot::String site_id;
    godot::String mission_display_name;
    int controlled_index = -1;
    struct Body {
        godot::Vector3 position_gcrf_m, velocity_gcrf_mps;
        godot::Quaternion attitude_gcrf;
        double cg_station_m = 0.0;
        int64_t serial = 0;  // FlightSession::body_serial: stable identity
        godot::String display_name;  // vehicle::Vehicle::shown_name
        godot::Array stages, engines, flaps, grid_fins, rcs, parachutes;
        godot::Dictionary legs, capsule_state, telemetry, return_state;
    };
    std::vector<Body> bodies;
    godot::Dictionary autopilot_status, tower_state, landing_zone, splashdown_zone;
    godot::Dictionary lunar_state, lunar_site;                       // build_lunar_state / build_lunar_site
    godot::Dictionary iss_state, rendezvous_plan, docking_state, station_geometry, iss_mission;  // ISS i6
    godot::Dictionary transfer_state, depot_state;  // refuelling r6
    godot::Dictionary campaign_flight_state;        // refuelling f2
    godot::Dictionary crew_escape_state;            // crew escape
    godot::Dictionary mission_timeline;             // build_mission_timeline
    std::shared_ptr<const environment::LunarTerrain> moon_terrain;  // immutable: queried on the main thread
    bool has_target_orbit = false;
    godot::Vector3 target_plane_normal_gcrf;
    double target_radius_m = 0.0;
    godot::PackedVector3Array prediction;  // for prediction_request
    PredictionRequest prediction_request;
};

// [E] The worker recomputes the requested trajectory prediction at most this often (wall seconds); the trajectory
// view refreshes on its own REFRESH_S.
inline constexpr double prediction_refresh_s = 1.0;

// The snapshot before any session exists (or before its first build): no flight, zones and tower {present: false}.
std::shared_ptr<const FlightSnapshot> empty_snapshot();

// Lunar parts of the snapshot (flight_snapshot_lunar.cpp; FlightSim::lunar_state / lunar_site document the keys).
godot::Dictionary build_lunar_state(const sim::FlightSession& session);
godot::Dictionary build_lunar_site(const sim::FlightSession& session);

// ISS parts of the snapshot (flight_snapshot_iss.cpp; FlightSim::iss_state etc. document the keys).
godot::Dictionary build_iss_state(const sim::FlightSession& session);
godot::Dictionary build_rendezvous_plan(const sim::FlightSession& session);
godot::Dictionary build_docking_state(const sim::FlightSession& session);
godot::Dictionary build_station_geometry(const sim::FlightSession& session);
godot::Dictionary build_iss_mission(const sim::FlightSession& session, const SnapshotContext& context);

// Refuelling parts (flight_snapshot_refuelling.cpp; FlightSim::transfer_state / depot_state /
// refuelling_campaign document the keys).
godot::Dictionary build_transfer_state(const sim::FlightSession& session);
godot::Dictionary build_depot_state(const sim::FlightSession& session);
godot::Dictionary build_campaign_flight_state(const sim::FlightSession& session);  // f2
// Crew escape (flight_snapshot_crew_escape.cpp; FlightSim::crew_escape_state documents the keys).
godot::Dictionary build_crew_escape_state(const sim::FlightSession& session);
// Mission timeline (flight_snapshot_timeline.cpp; FlightSim::mission_timeline documents the keys).
godot::Dictionary build_mission_timeline(const sim::FlightSession& session, time::Epoch epoch);
godot::Dictionary campaign_dictionary(const sim::CampaignReport& report, const time::TimeScaleConverter& time);

// Builds the snapshot from the session (worker thread). `previous` supplies the prediction when it is not due.
std::shared_ptr<const FlightSnapshot> build_snapshot(const sim::FlightSession& session, time::Epoch epoch,
                                                     SnapshotContext& context, const FlightSnapshot* previous);

}  // namespace starsim::bindings
