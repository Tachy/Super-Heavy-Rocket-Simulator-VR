#pragma once

#include <cstdint>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "flight_snapshot.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/session_runner.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::bindings {

// Godot facade of sim::FlightSession. Requires an open StarsimCore (shared SPICE kernel pool).
// Positions are GCRF metres (float64), attitudes body -> GCRF.
//
// The session runs on its own simulation thread (sim::SessionRunner, l6). Commands are queued and return a command id
// (> 0; 0 when refused before posting, e.g. no setup()); their outcome arrives as result(id) -> {done, ok, message} and
// the signal command_finished(id, name, ok, message), emitted on the main thread from advance_to and wait_idle.
// Getters read the latest snapshot, built on the simulation thread after every command and step chunk: the returned
// containers are shared with it and must be treated as read-only.
class FlightSim : public godot::RefCounted {
    GDCLASS(FlightSim, godot::RefCounted)

public:
    ~FlightSim() override;

    // Joins a previous simulation thread (waits for its running command, drops the queued ones) and starts empty.
    void setup(const godot::String& config_dir);
    int64_t launch_from_pad(const godot::String& vehicle_id, double latitude_deg, double longitude_deg, double height_m,
                            double heading_deg, double tdb_s);
    // Launch from a site of config/launch_sites.toml (with its catch tower, if it has one).
    int64_t launch_from_site(const godot::String& vehicle_id, const godot::String& site_id, double heading_deg,
                             double tdb_s);
    // Mission of config/missions by its [mission] kind: an ascent starts at `tdb_s` (payload, pad, landing zone or drone
    // ship, booster return, ascent autopilot); a capsule return at its own epoch (mission_start_tdb_s()) with the deorbit
    // planned; not ok for an unknown, invalid or ship-return mission.
    int64_t start_mission(const godot::String& mission_id, double tdb_s);
    // Epoch (TDB s) the current flight began at: the launch, or a return mission's own epoch (the world clock jumps there).
    double mission_start_tdb_s() const;
    // Launch site id of the current flight (launch_from_site or the mission), "" before the first.
    godot::String mission_site_id() const;
    // Landing zone of a returning first stage: {present}; present: {origin_itrf_m (its surface: the deck top for a drone
    // ship), east, north, up (ITRF unit vectors), latitude_deg, longitude_deg, pad_radius_m, has_deck}; with a deck also
    // {deck_length_m, deck_width_m, freeboard_m, heading_deg (long axis, clockwise from north), sea_height_m}.
    godot::Dictionary landing_zone() const;
    // Catch tower geometry of a site (no flight needed, synchronous): {present, origin_itrf_m, east, north, up (ITRF
    // units), tower_direction, across, tower_center_m, tower_size_m, hinge_plus_m, hinge_minus_m (site frame), mount_*,
    // arm_* sizes, arm_open_rad}.
    godot::Dictionary site_tower(const godot::String& site_id) const;
    // Tower of the current flight: {present, opening_rad, carriage_offset_m, phase (German)}.
    godot::Dictionary tower_state() const;
    int64_t set_booster_return(bool enabled);
    // Pilot's abort of a returning body's landing or catch: it ditches at sea (return_state "abort"). Not ok when the
    // guidance cannot abort (not returning, no sea, already down or on the arms).
    int64_t abort_landing(int index);
    // Posted only when the value changes (the world sets it every frame); returns the id of the last posting.
    int64_t set_realtime_planning(bool enabled);
    godot::Dictionary return_state(int64_t index) const;
    bool is_active() const;
    // Sets the simulation thread's target epoch, emits command_finished for finished commands and returns the epoch the
    // snapshot has reached (never ahead of the target; `tdb_s` without an active flight).
    double advance_to(double tdb_s);
    double mission_time_s() const;
    // Waits until no command is queued or running and the target is reached; true when idle within the timeout.
    bool wait_idle(double timeout_s);
    // Wall seconds the running command has taken so far (a return plan: seconds); the simulation clock stands meanwhile.
    double planning_s() const;
    // {done, ok, message} of a command id; done == false while it is queued or running.
    godot::Dictionary result(int64_t id) const;

    void set_input(double pitch, double yaw, double roll, double throttle);
    int64_t set_rate_mode(bool enabled);
    int64_t ignite();
    int64_t cutoff();
    int64_t release_clamps();
    int64_t stage();

    // Guidance and manoeuvres.
    int64_t engage_ascent_autopilot(double periapsis_km, double apoapsis_km, double inclination_deg, bool northerly);
    int64_t disengage_autopilot();
    int64_t plan_circularization();
    // Return of the ship from orbit to the site's tower (onboard deorbit plan, burn, entry, catch); the plan or the
    // reason it was refused appears in autopilot_status()["return"].
    int64_t plan_ship_return();
    // Node `seconds_from_now` ahead with orbit-relative components (prograde, normal, radial-out), m/s.
    int64_t plan_burn(double seconds_from_now, double prograde_mps, double normal_mps, double radial_mps);
    // {state, phase, time_to_go_s, direction (GCRF), burn: {time_to_ignition_s, remaining_dv_mps,
    //  burn_duration_s, direction}, return: {feasible, reason, delta_v_mps, propellant_t, burn_in_s, entry_in_s,
    //  crossrange_km}}
    godot::Dictionary autopilot_status() const;
    // The snapshot's prediction for the last requested (index, duration, step); the simulation thread recomputes it at
    // most every prediction_refresh_s, so a changed request shows the previous points until then.
    godot::PackedVector3Array predict_trajectory(int index, double duration_s, double step_s) const;
    // Target orbit of the ascent guidance as a closed curve (GCRF), empty without guidance.
    godot::PackedVector3Array target_orbit_points(int count) const;
    godot::Vector3 body_velocity_gcrf_mps(int index) const;

    int body_count() const;
    int controlled_index() const;
    godot::Vector3 body_position_gcrf_m(int index) const;
    godot::Quaternion body_attitude_gcrf(int index) const;
    double body_cg_station_m(int index) const;
    // [{name, base_station_m, length_m, diameter_m, nose_length_m}] bottom to top
    godot::Array body_stages(int index) const;
    // [{position_m (body, relative to the bottom), firing, throttle, spool}]
    godot::Array body_engines(int index) const;
    // Flaps of a body (stations in its body frame, deflection 0 folded leeward .. 90 broadside) for rendering.
    godot::Array body_flaps(int index) const;
    // Landing legs of a body's lowest stage: {present}; present: {deployment (0 stowed .. 1 locked), touchdown (German,
    // "" while none), base_station_m (bottom of the stage, body frame), hull_radius_m, foot_station_m (deployed foot from
    // the base, negative: below it), footprint_radius_m, azimuths_deg (from body +y towards +z)}. Empty for an invalid
    // index.
    godot::Dictionary body_legs(int index) const;
    // Grid fins: [{station_m (body frame), azimuth_deg, radius_m (centre of pressure), area_m2, deflection_deg}].
    godot::Array body_grid_fins(int index) const;
    // RCS thrusters, one per thruster: [{id, stage, position_m (body), direction (force on the vehicle; the plume points
    // the other way), thrust_n, firing, duty (0..1 allocated this tick), starved (supply empty), blocked (behind a closed
    // nose cone, or a payload still stacked)}].
    godot::Array body_rcs(int index) const;
    // Parachute sets: [{id, role, state (German: verstaut / offen / abgeworfen / Fallschirm gerissen), count,
    // attach_station_m (body), drag_area_m2 (now, all canopies), full_drag_area_m2, inflation (0..1), direction (body unit
    // vector from the riser point towards the canopies: against the point's air velocity; +x below 0.1 m/s),
    // peak_load_n, max_load_n (per canopy)}].
    godot::Array body_parachutes(int index) const;
    // Capsule flying on its own (top stage with a capsule entry profile, not stacked on a launcher): {present}; present:
    // {nose_cone, nose_cone_open, trunk_attached, return_step (German), parachute_phase (German, "" without a sequencer),
    // parachute_torn, splashdown_miss_m (with a zone), outcome (German)}. Empty for an invalid index.
    godot::Dictionary capsule_state(int index) const;
    // Splashdown zone of the flight: {present}; present: {id, name, latitude_deg, longitude_deg, radius_m, origin_itrf_m
    // (sea level), east, north, up (ITRF unit vectors)}.
    godot::Dictionary splashdown_zone() const;
    // Zone of config/splashdown_zones.toml for plan_capsule_return; not ok for an unknown zone.
    int64_t set_splashdown_zone(const godot::String& zone_id);
    // Return of the controlled capsule to the splashdown zone (deorbit, trunk, entry, parachutes); the plan or the reason
    // it was refused appears in autopilot_status()["return"], the step in ["capsule_step"].
    int64_t plan_capsule_return();
    godot::Dictionary telemetry(int index) const;

protected:
    static void _bind_methods();

private:
    struct Outcome {  // written by the command on the worker, read once the runner reports it done
        bool ok = false;
        godot::String message;
    };
    using SessionCommand = std::function<Outcome(sim::FlightSession&)>;

    std::shared_ptr<const FlightSnapshot> snapshot() const;
    // Snapshot body `index`, nullptr (error pushed) when there is none.
    const FlightSnapshot::Body* body(const FlightSnapshot& s, int index) const;
    std::uint64_t post(const char* name, SessionCommand command);
    // Commands that need an active flight: refused with a German message otherwise.
    std::uint64_t post_active(const char* name, std::function<Outcome(sim::FlightSession&)> command);
    void observe(const sim::FlightSession& session, time::Epoch epoch);  // worker: builds and publishes the snapshot
    void emit_finished();                                                // main thread: command_finished signals

    // Members the worker uses are declared before runner_, so the runner (and its thread) is destroyed first.
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    spice::SpiceTimeConverter time_;
    std::filesystem::path config_dir_;
    mutable std::mutex snapshot_mutex_;
    std::shared_ptr<const FlightSnapshot> snapshot_;  // latest published
    SnapshotContext context_;                         // worker-owned (written by commands and the observer)
    mutable std::mutex request_mutex_;
    mutable PredictionRequest prediction_request_;    // main thread writes, worker copies
    // Main thread only: names (for command_finished) and outcomes of posted commands.
    std::map<std::uint64_t, std::pair<godot::String, std::shared_ptr<Outcome>>> commands_;
    std::optional<bool> realtime_planning_;  // last posted value
    std::uint64_t realtime_planning_id_ = 0;
    std::unique_ptr<sim::SessionRunner> runner_;  // owns the session; last member: destroyed (joined) first
};

}  // namespace starsim::bindings
