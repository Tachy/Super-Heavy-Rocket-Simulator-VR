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
// Getters read a snapshot, built on the simulation thread after every command and step chunk: the returned containers
// are shared with it and must be treated as read-only. advance_to pins the latest snapshot for the frame, so every
// getter until the next advance_to reads the same instant (the thread publishes several per frame; mixing them made
// vehicles jump against the pad drawn at the returned epoch); wait_idle releases the pin, later getters read the latest.
class FlightSim : public godot::RefCounted {
    GDCLASS(FlightSim, godot::RefCounted)

public:
    ~FlightSim() override;

    // Joins a previous simulation thread (waits for its running command, drops the queued ones) and starts empty.
    void setup(const godot::String& config_dir);
    int64_t launch_from_pad(const godot::String& vehicle_id, double latitude_deg, double longitude_deg, double height_m,
                            double heading_deg, double tdb_s);
    // Launch from a site of config/launch_sites.toml (with its catch tower, if it has one).
    // countdown_s > 0: the vehicle stands clamped on the pad and liftoff is scheduled at tdb_s + max(countdown_s, the
    // ignition lead); the ascent autopilot lifts off at T-0. Without it the T-clock (mission_timeline) counts down to
    // T-0 and holds there until a liftoff. The mission log (mission_timeline) starts fresh with every launch (also
    // launch_from_pad). `ascent_target` ({periapsis_km, apoapsis_km, inclination_deg, northerly}; empty: the vehicle is
    // only placed): the onboard computer then plans the launch on the simulation thread - autopilot_status()["planning"],
    // the snapshot and the clock hold on the pad meanwhile ("Startvorbereitung") - and engages the ascent autopilot with
    // the booster's return; a refused plan makes the command not ok with the German reason (the vehicle stays on the pad).
    int64_t launch_from_site(const godot::String& vehicle_id, const godot::String& site_id, double heading_deg,
                             double tdb_s, double countdown_s = 0.0, const godot::Dictionary& ascent_target = {});
    // Mission of config/missions by its [mission] kind: an ascent starts at `tdb_s` (payload, pad, landing zone or drone
    // ship, booster return, ascent autopilot); a capsule return at its own epoch (mission_start_tdb_s()) with the deorbit
    // planned; a lunar landing (from LEO) or lunar return (from lunar orbit) at its own epoch with all legs planned; not ok
    // for an unknown, invalid or ship-return mission, or a lunar mission the planner refuses (message: its reason).
    // wait_on_pad: launch missions stand on the pad and lift off at their scheduled T-0 (sim::StartOptions).
    int64_t start_mission(const godot::String& mission_id, double tdb_s, bool wait_on_pad = false);
    // Epoch (TDB s) the current flight began at: the launch, or a return mission's own epoch (the world clock jumps there).
    double mission_start_tdb_s() const;
    // Launch site id of the current flight (launch_from_site or the mission), "" before the first.
    godot::String mission_site_id() const;
    // [mission] display_name of the started mission (German, no trademarks), "" for a free launch or none configured.
    godot::String mission_display_name() const;
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
    // Sets the simulation thread's target epoch, emits command_finished for finished commands, pins the latest snapshot
    // for the getters (see above) and returns its epoch (never ahead of the target; `tdb_s` without an active flight).
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
    // On a clamped stack the onboard computer plans the launch first (as launch_from_site with a target); not ok with
    // the German reason when it refuses.
    int64_t engage_ascent_autopilot(double periapsis_km, double apoapsis_km, double inclination_deg, bool northerly);
    int64_t disengage_autopilot();
    // External view: a free-flying controlled body with the autopilot off goes to the state Orbit
    // (sim::FlightSession::engage_orbit_autopilot); not ok when nothing changed (engaged, clamped, impacted).
    int64_t engage_orbit_autopilot();
    int64_t plan_circularization();
    // Return of the ship from orbit to the site's tower (onboard deorbit plan, burn, entry, catch); the plan or the
    // reason it was refused appears in autopilot_status()["return"].
    int64_t plan_ship_return();
    // Node `seconds_from_now` ahead with orbit-relative components (prograde, normal, radial-out), m/s.
    int64_t plan_burn(double seconds_from_now, double prograde_mps, double normal_mps, double radial_mps);
    // {state, phase, time_to_go_s, direction (GCRF), settling (f3: the push is on), burn_abort_reason (German; e.g.
    //  "Gasansaugung ..." after an unsettled in-orbit ignition), burn: {time_to_ignition_s, remaining_dv_mps,
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
    // [{name, base_station_m, length_m, diameter_m, nose_length_m, settled (f3: 0..1)}] bottom to top
    // (also oxidizer_kg, fuel_kg, oxidizer_capacity_kg, fuel_capacity_kg: the main tanks)
    godot::Array body_stages(int index) const;
    // Stable identity of a body (survives reindexing when bodies separate or are removed), 0 for an invalid index.
    int64_t body_serial(int index) const;
    // Name shown for a body (vehicle::Vehicle::shown_name: display names only, no trademarks), "" for an invalid index.
    godot::String body_display_name(int index) const;
    // [{position_m (body, relative to the bottom), firing, throttle, spool, vacuum, auxiliary (launch escape or landing
    // thrusters: not main propulsion), stage (index into body_stages)}]
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
    // {altitude_m, altitude_terrain_m, latitude_deg, ..., reference_body ("earth" / "moon"), periapsis_altitude_km,
    // apoapsis_altitude_km (INF on an open orbit), inclination_deg}: the orbit values are osculating two-body elements
    // about the reference body (inclination against its equator: the body-relative state rotated into ITRF / MOON_ME
    // axes at the current epoch), altitudes above its equatorial radius / the 1737.4 km sphere. Osculating values differ
    // from the mean orbit (l4: "95 x 100 km at burn end" is the osculating value). propellant_kg / propellant_capacity_kg:
    // oxidizer + fuel of the main and header tanks, now and full (the webcast propellant gauge).
    godot::Dictionary telemetry(int index) const;

    // Lunar flight (l6): {present}; present: {step, status (German), mission_complete, mission_ended, burns: [{step,
    // epoch_tdb_s, delta_v_mps, mass_kg}], descent: {present, feasible, reason, pdi_epoch_tdb_s, touchdown_latitude_deg,
    // touchdown_longitude_deg, touchdown_height_m, slope_deg, offset_m, periselene_altitude_km}, touchdown: {present,
    // outcome (German), offset_m, sink_mps, lateral_mps}, return: {present, feasible, reason, tei_delta_v_mps,
    // braking_delta_v_mps, interface_epoch_tdb_s, interface_speed_mps, interface_angle_deg, replans}, pilot_cue:
    // {present, burn_epoch_tdb_s, remaining_delta_v_gcrf_mps, remaining_mps, attitude_error_deg, window_end_s}}.
    godot::Dictionary lunar_state() const;
    // Lunar landing site of the flight: {present}; present: {id, name, latitude_deg, longitude_deg, height_m (terrain
    // above the 1737.4 km sphere), position_moon_me_m (on the terrain)}.
    godot::Dictionary lunar_site() const;
    // The flight's Moon terrain at a point: {present}; present: {height_m (above the sphere), normal_moon_me, in_tile}.
    // Runs on the calling thread (the terrain is immutable and shared with the session).
    godot::Dictionary moon_terrain(double latitude_deg, double longitude_deg) const;
    // The Earth terrain at a point: {present}; present: {height_m (the surface above the WGS84 ellipsoid; sea: the
    // EGM2008 geoid), water}. Loaded in setup() from data/ (absent: the flat ground); runs on the calling thread (the
    // terrain is immutable and shared with the session).
    godot::Dictionary earth_surface(double latitude_deg, double longitude_deg) const;
    // The crew reports its mid-course correction flown (lunar step PilotCorrection).
    int64_t complete_pilot_correction();
    // Re-plan of the lunar return from the current flight data (ok: the session's result, message: lunar_status()).
    int64_t replan_lunar_return();

    // ISS (i6). iss_state: {present}; present: {index, position_gcrf_m, velocity_gcrf_mps, attitude_gcrf, mass_kg (the
    // compound while docked), docked_mass_kg, ports: [{id, name, position_gcrf_m, axis_gcrf (outward docking axis),
    // up_gcrf, soft_capture}], warnings (German: reboosts in the rendezvous / proximity span)}.
    godot::Dictionary iss_state() const;
    // Phasing plan: {present}; present: {feasible, reason, step, status (German), replans, total_delta_v_mps,
    // propellant_kg, waypoint0_tdb_s, burns_flown, burns: [{kind (German), epoch_tdb_s, delta_v_mps, flown}]}.
    godot::Dictionary rendezvous_plan() const;
    // Proximity / docking / departure: {present}; present: {proximity_phase, docking_phase, departure_phase (German,
    // "" when none), status, position_port_m / velocity_port_mps (chaser port in the station port frame), range_m,
    // range_rate_mps, offset_m, misalignment_deg, clocking_deg (with a rendezvous target), latched, hooks_closed,
    // docked, collision_impulse_ns (0: none), loads: {compression_n, shear_n, bending_nm} (with a mechanism)}.
    godot::Dictionary docking_state() const;
    // Station primitives for drawing, in the station vehicle frame (x from the bottom of stage 0, as
    // body_cg_station_m): {present}; present: {boxes: [{center_m, axis_x, axis_y, axis_z, half_extents_m}],
    // capsules: [{a_m, b_m, radius_m}], ports: [{id, position_m, axis, up}]}.
    godot::Dictionary station_geometry() const;
    // ISS mission sequence: {present}; present: {leg, status (German), port, liftoff_tdb_s, separation_tdb_s,
    // plane_bias_deg (0 without a launch window), undock_tdb_s, deorbit_tdb_s (0 when not set), warnings}.
    godot::Dictionary iss_mission() const;
    // Proximity commands (ok: accepted, message: the German proximity status or refusal) and the undocking
    // (message: the docking status).
    int64_t proximity_hold();
    int64_t proximity_retreat();
    int64_t proximity_abort();
    int64_t proximity_resume();
    int64_t undock();
    // Refuelling f3: the manual settling push (cold gas along +x until switched off; refused while docked or
    // approaching). The snapshot shows it as autopilot_status()["settling"], the progress per stage as
    // body_stages()[i]["settled"].
    int64_t settle(bool enabled);
    // Refuelling (r6). Propellant transfer of the docked pair: {present} (false while idle); present: {phase,
    // status (German), elapsed_s, delivered_oxidizer_kg, delivered_fuel_kg, flow_oxidizer_kgps, flow_fuel_kgps,
    // settling_gas_kg, settling_thrust_n, chilldown_kg, boil_off_kg (both ships since the start)}.
    godot::Dictionary transfer_state() const;
    // The depot (a station with transfer connectors): {present}; present: {main_propellant_kg, capacity_kg,
    // fill_fraction, boil_off_kg_per_day (at the last evaluation), docked}.
    godot::Dictionary depot_state() const;
    // Refuelling f2: the flown campaign (start_mission("refuelling_campaign_flown" / "..._placed")): {present} (false
    // without one); present: {phase, status (German), tankers_flown, tankers_planned, depot_propellant_kg, start_tdb_s,
    // next_launch_tdb_s (absent when none), next_is_hls, rows: [{number (0: HLS), hls, failed, note, launch_tdb_s,
    // arrival_tdb_s, delivered_kg, depot_after_kg, calc_launch_tdb_s, calc_arrival_tdb_s, calc_depot_after_kg (calc_*
    // absent without a calculator row)}], losses: {boil_off_kg, station_keeping_kg, attitude_gas_kg,
    // transfer_vent_kg}, reboosts (count), calc_tanker_flights, calc_duration_s, hls_arrival_tdb_s, hls_propellant_kg,
    // hls_required_propellant_kg, first_return (German), misses (German, once complete; empty: the comparison
    // passed)}.
    godot::Dictionary campaign_flight_state() const;
    // Crew escape (docs/superpowers/specs/2026-10-22-crew-escape-design.md): {present} (false for an uncrewed flight);
    // present: {armed, fired, early, reason (German), give_up_time_s (mission time), members: [{phase, mode (German),
    // fire_time_s, load_g, peak_load_g, min_clearance_m, outcome, reason (German), body_index (-1: not fired / gone),
    // position_gcrf_m, velocity_gcrf_mps, attitude_gcrf, latitude_rad, longitude_rad, height_m (while it has a body),
    // touchdown_enu_m, touchdown_over_water, touchdown_sink_mps (once down)}], ship_impact: {present, enu_m (site frame:
    // east, north, up from the pad centre), speed_mps, tower_distance_m, shore_distance_m (> 0 seaward), over_water}}.
    // Each fired member is also an ordinary body (body_stages etc.). return_state(i) of the ship: phase, guidance,
    // abort, upright_descent, result (sim::ReturnResult, German).
    godot::Dictionary crew_escape_state() const;
    // Mission log: {t0_tdb_s (null: none yet), clock_s (snapshot epoch - T-0, negative = T-; null: none), events:
    // [{kind (snake_case id: "liftoff", "max_q", "meco", ...), label (German), tdb_s, body_serial (0: mission-wide),
    // body_name, predicted}]} recorded and predicted events sorted by tdb_s.
    godot::Dictionary mission_timeline() const;
    // Makes the current flight crewed with config/crew_escape/<id>.toml (empty id: uncrewed). Set it after the ship is
    // placed or the mission started (a new placement makes the flight uncrewed). Not ok for an unknown id.
    int64_t set_crew_escape(const godot::String& id);
    // The tanker campaign (sim::plan_refuelling_campaign; the session is untouched, seconds of window searches on the
    // worker). params (optional; floats unless noted): height_km, interval_h, phasing_limit_h, tanker_dry_t, reserve_t,
    // initial_t, insulation, hls_wait_d, reference_phasing (bool, f1); an unknown key or a wrong type is refused.
    // result(id): ok = the campaign
    // closes, message = its German reason, data = {closes, reason, tanker_flights, duration_d, demand_t,
    // hls_required_t, depot_at_hls_t, tankers: [{launch_utc, arrival_utc, delivered_t, depot_after_t,
    // phasing_periapsis_km, phasing_apoapsis_km}],
    // fill: [[day, t]], losses: {boil_off_t, drag_t, gas_t, vent_t}, lunar_mission, lunar_epoch_utc}.
    int64_t refuelling_campaign(const godot::Dictionary& params);

protected:
    static void _bind_methods();

private:
    struct Outcome {  // written by the command on the worker, read once the runner reports it done
        bool ok = false;
        godot::String message;
        godot::Dictionary data;  // result(id)["data"] (refuelling_campaign)
    };
    using SessionCommand = std::function<Outcome(sim::FlightSession&)>;

    std::shared_ptr<const FlightSnapshot> snapshot() const;         // main thread: the pinned one, else the latest
    std::shared_ptr<const FlightSnapshot> latest_snapshot() const;  // any thread: the latest published
    // Snapshot body `index`, nullptr (error pushed) when there is none.
    const FlightSnapshot::Body* body(const FlightSnapshot& s, int index) const;
    std::uint64_t post(const char* name, SessionCommand command);
    // Commands that need an active flight: refused with a German message otherwise.
    std::uint64_t post_active(const char* name, std::function<Outcome(sim::FlightSession&)> command);
    void observe(const sim::FlightSession& session, time::Epoch epoch);  // worker: builds and publishes the snapshot
    // worker: the ascent autopilot engaged as the onboard computer does (sim::engage_pad_ascent(session, target));
    // on a clamped stack the planning flag is published first (the snapshot on the pad while the plan runs)
    Outcome engage_planned(sim::FlightSession& session, const gnc::AscentTarget& target);
    void emit_finished();                                                // main thread: command_finished signals

    // Members the worker uses are declared before runner_, so the runner (and its thread) is destroyed first.
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_ = spice::SpiceOrientation::moon_mean_earth();  // the Moon as a reference body (l6)
    spice::SpiceTimeConverter time_;
    std::filesystem::path config_dir_;
    std::shared_ptr<const environment::EarthTerrain> earth_terrain_;  // setup(): the session's, nullptr without data/
    mutable std::mutex snapshot_mutex_;
    std::shared_ptr<const FlightSnapshot> snapshot_;  // latest published
    std::shared_ptr<const FlightSnapshot> frame_snapshot_;  // main thread: pinned by advance_to, released by wait_idle
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
