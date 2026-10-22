#include "flight_sim.hpp"

#include <cmath>

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "starsim/config/crew_escape_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/contact/tower_geometry.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"

namespace starsim::bindings {

namespace {

constexpr size_t max_kept_commands = 1000;  // [E] like the runner's results: oldest names/outcomes dropped first

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }
time::Epoch epoch(double tdb_s) { return time::Epoch::from_tdb_seconds_past_j2000(tdb_s); }

}  // namespace

FlightSim::~FlightSim() { runner_.reset(); }

void FlightSim::setup(const godot::String& config_dir) {
    runner_.reset();  // joins the old thread before its session and context go
    config_dir_ = std::filesystem::path(config_dir.utf8().get_data());
    context_ = {};
    commands_.clear();
    realtime_planning_.reset();
    realtime_planning_id_ = 0;
    {
        std::lock_guard lock(snapshot_mutex_);
        snapshot_ = empty_snapshot();
    }
    runner_ = std::make_unique<sim::SessionRunner>(
        std::make_unique<sim::FlightSession>(ephemeris_, earth_, config_dir_, environment::PointMassGravityConfig{}, &moon_),
        [this](const sim::FlightSession& session, time::Epoch now) { observe(session, now); });
}

void FlightSim::observe(const sim::FlightSession& session, time::Epoch now) {
    {
        std::lock_guard lock(request_mutex_);
        context_.prediction_request = prediction_request_;
    }
    const auto previous = snapshot();
    auto next = build_snapshot(session, now, context_, previous.get());
    std::lock_guard lock(snapshot_mutex_);
    snapshot_ = std::move(next);
}

std::shared_ptr<const FlightSnapshot> FlightSim::snapshot() const {
    std::lock_guard lock(snapshot_mutex_);
    return snapshot_ ? snapshot_ : empty_snapshot();
}

std::uint64_t FlightSim::post(const char* name, SessionCommand command) {
    if (!runner_) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.") + name + ": call setup() first");
        return 0;
    }
    auto outcome = std::make_shared<Outcome>();
    const auto id = runner_->post([command = std::move(command), outcome](sim::FlightSession& session) {
        *outcome = command(session);
    });
    commands_[id] = {godot::String(name), outcome};
    while (commands_.size() > max_kept_commands) commands_.erase(commands_.begin());
    return id;
}

std::uint64_t FlightSim::post_active(const char* name, SessionCommand command) {
    return post(name, [command = std::move(command)](sim::FlightSession& session) {
        if (!session.active()) return Outcome{false, godot::String::utf8("Kein aktiver Flug")};
        return command(session);
    });
}

godot::Dictionary FlightSim::result(int64_t id) const {
    godot::Dictionary d;
    d["done"] = false;
    d["ok"] = false;
    d["message"] = godot::String();
    if (!runner_ || id <= 0) {
        d["done"] = true;
        d["message"] = "unknown command id";
        return d;
    }
    const auto r = runner_->result(static_cast<std::uint64_t>(id));
    if (!r.done) return d;
    d["done"] = true;
    const auto it = commands_.find(static_cast<std::uint64_t>(id));
    if (r.threw) {
        d["message"] = godot::String::utf8(r.error.c_str());
    } else if (it != commands_.end()) {
        d["ok"] = it->second.second->ok;
        d["message"] = it->second.second->message;
        d["data"] = it->second.second->data;
    } else {
        d["message"] = "unknown command id";
    }
    return d;
}

void FlightSim::emit_finished() {
    if (!runner_) return;
    // Input, advance and snapshot exceptions have no command id: report each once.
    if (const auto error = runner_->take_error(); !error.empty()) {
        godot::UtilityFunctions::push_error("FlightSim: " + godot::String::utf8(error.c_str()));
    }
    for (const auto& [id, r] : runner_->take_finished()) {
        const auto it = commands_.find(id);
        const godot::String name = it != commands_.end() ? it->second.first : godot::String();
        const auto d = result(static_cast<int64_t>(id));
        if (r.threw) godot::UtilityFunctions::push_error("FlightSim." + name + ": " + godot::String(d["message"]));
        emit_signal("command_finished", static_cast<int64_t>(id), name, d["ok"], d["message"]);
    }
}

bool FlightSim::wait_idle(double timeout_s) {
    const bool idle = !runner_ || runner_->wait_idle(timeout_s);
    emit_finished();
    return idle;
}

double FlightSim::planning_s() const { return runner_ ? runner_->busy_s() : 0.0; }

int64_t FlightSim::launch_from_site(const godot::String& vehicle_id, const godot::String& site_id, double heading_deg,
                                    double tdb_s, double countdown_s) {
    const std::string vehicle = vehicle_id.utf8().get_data(), site = site_id.utf8().get_data();
    return static_cast<int64_t>(post("launch_from_site", [this, vehicle, site, site_id, heading_deg, tdb_s,
                                                          countdown_s](sim::FlightSession& session) {
        const auto config = config::load_launch_site(config_dir_, site);
        session.launch_from_pad(vehicle, sim::launch_site_from_config(config), heading_deg * constants::deg_to_rad,
                                epoch(tdb_s));
        if (countdown_s > 0.0) {
            session.schedule_liftoff(epoch(tdb_s + std::max(countdown_s, session.liftoff_ignition_lead_s())));
        }
        session.reset_timeline();  // a fresh mission log; the clamped vehicle starts on the pad
        context_.site_id = site_id;
        context_.mission_start_tdb_s = tdb_s;
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::start_mission(const godot::String& mission_id, double tdb_s, bool wait_on_pad) {
    const std::string id = mission_id.utf8().get_data();
    return static_cast<int64_t>(post("start_mission", [this, id, tdb_s, wait_on_pad](sim::FlightSession& session) {
        sim::StartOptions options;
        options.wait_on_pad = wait_on_pad;
        const auto s = sim::start_mission(session, config_dir_, id, epoch(tdb_s), time_, options);
        context_.site_id = godot::String::utf8(s.site_id.c_str());
        context_.mission_start_tdb_s = s.epoch.tdb_seconds_past_j2000();
        context_.mission_warnings = godot::Array();
        std::string joined;
        for (const auto& w : s.warnings) {
            context_.mission_warnings.push_back(godot::String::utf8(w.c_str()));
            joined += (joined.empty() ? "" : "; ") + w;
        }
        const bool from_window = s.launch_window && s.launch_window->feasible;
        context_.iss_liftoff_tdb_s = from_window ? s.launch_window->liftoff.tdb_seconds_past_j2000() : 0.0;
        context_.iss_separation_tdb_s = from_window ? s.launch_window->separation.tdb_seconds_past_j2000() : 0.0;
        context_.iss_plane_bias_deg = from_window ? s.launch_window->plane_bias_rad * constants::rad_to_deg : 0.0;
        if (!s.refusal.empty()) return Outcome{false, godot::String::utf8(s.refusal.c_str())};
        return Outcome{true, godot::String::utf8(joined.c_str())};
    }));
}

int64_t FlightSim::launch_from_pad(const godot::String& vehicle_id, double latitude_deg, double longitude_deg,
                                   double height_m, double heading_deg, double tdb_s) {
    const std::string vehicle = vehicle_id.utf8().get_data();
    return static_cast<int64_t>(post("launch_from_pad", [this, vehicle, latitude_deg, longitude_deg, height_m,
                                                         heading_deg, tdb_s](sim::FlightSession& session) {
        const double deg = constants::deg_to_rad;
        session.launch_from_pad(vehicle, {latitude_deg * deg, longitude_deg * deg, height_m}, heading_deg * deg,
                                epoch(tdb_s));
        context_.mission_start_tdb_s = tdb_s;
        return Outcome{true, {}};
    }));
}

double FlightSim::mission_start_tdb_s() const { return snapshot()->mission_start_tdb_s; }
godot::String FlightSim::mission_site_id() const { return snapshot()->site_id; }
godot::Dictionary FlightSim::landing_zone() const { return snapshot()->landing_zone; }
godot::Dictionary FlightSim::tower_state() const { return snapshot()->tower_state; }
godot::Dictionary FlightSim::splashdown_zone() const { return snapshot()->splashdown_zone; }

godot::Dictionary FlightSim::site_tower(const godot::String& site_id) const {
    godot::Dictionary d;
    d["present"] = false;
    try {
        const auto site = config::load_launch_site(config_dir_, site_id.utf8().get_data());
        if (!site.tower) return d;
        const contact::TowerGeometry g(*site.tower, site.latitude_rad, site.longitude_rad, site.height_m);
        const auto& c = *site.tower;
        d["present"] = true;
        d["origin_itrf_m"] = to_godot(g.origin_itrf_m());
        d["east"] = to_godot(math::column(g.site_to_itrf(), 0));
        d["north"] = to_godot(math::column(g.site_to_itrf(), 1));
        d["up"] = to_godot(math::column(g.site_to_itrf(), 2));
        d["tower_direction"] = to_godot(g.tower_direction());
        d["across"] = to_godot(g.across());
        d["tower_center_m"] = to_godot(g.tower_box().center);
        d["tower_size_m"] = godot::Vector3(c.width_m, c.width_m, c.height_m);
        d["mount_height_m"] = c.mount_height_m;
        d["mount_inner_radius_m"] = c.mount_inner_radius_m;
        d["mount_outer_radius_m"] = c.mount_outer_radius_m;
        d["arm_length_m"] = c.arm_length_m;
        d["arm_width_m"] = c.arm_width_m;
        d["arm_thickness_m"] = c.arm_thickness_m;
        d["arm_open_rad"] = c.arm_open_rad;
        d["hinge_plus_m"] = to_godot(g.hinge(1));
        d["hinge_minus_m"] = to_godot(g.hinge(-1));
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("FlightSim.site_tower: ") + e.what());
    }
    return d;
}

int64_t FlightSim::set_booster_return(bool enabled) {
    return static_cast<int64_t>(post("set_booster_return", [enabled](sim::FlightSession& session) {
        session.set_booster_return(enabled);
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::abort_landing(int index) {
    return static_cast<int64_t>(post_active("abort_landing", [index](sim::FlightSession& session) {
        if (index < 0 || static_cast<size_t>(index) >= session.body_count()) {
            return Outcome{false, godot::String::utf8("Ungültiger Körperindex")};
        }
        return Outcome{session.abort_landing(static_cast<size_t>(index)), {}};
    }));
}

int64_t FlightSim::set_realtime_planning(bool enabled) {
    if (realtime_planning_ == enabled) return static_cast<int64_t>(realtime_planning_id_);
    const auto id = post("set_realtime_planning", [enabled](sim::FlightSession& session) {
        session.set_realtime_planning(enabled);
        return Outcome{true, {}};
    });
    if (id > 0) {
        realtime_planning_ = enabled;
        realtime_planning_id_ = id;
    }
    return static_cast<int64_t>(id);
}

godot::Dictionary FlightSim::return_state(int64_t index) const {
    const auto s = snapshot();
    if (index < 0 || static_cast<size_t>(index) >= s->bodies.size()) {
        godot::Dictionary d;
        d["present"] = false;
        return d;
    }
    return s->bodies[static_cast<size_t>(index)].return_state;
}

bool FlightSim::is_active() const { return snapshot()->active; }

double FlightSim::advance_to(double tdb_s) {
    if (runner_) runner_->set_target(epoch(tdb_s));
    emit_finished();
    const auto s = snapshot();
    return s->active ? s->epoch_tdb_s : tdb_s;
}

double FlightSim::mission_time_s() const { return snapshot()->mission_time_s; }

void FlightSim::set_input(double pitch, double yaw, double roll, double throttle) {
    if (!runner_) return;
    runner_->set_latest([input = sim::PilotInput{pitch, yaw, roll, throttle}](sim::FlightSession& session) {
        session.set_input(input);
    });
}

int64_t FlightSim::set_rate_mode(bool enabled) {
    return static_cast<int64_t>(post("set_rate_mode", [enabled](sim::FlightSession& session) {
        session.set_control_mode(enabled ? sim::ControlMode::RateCommand : sim::ControlMode::DirectGimbal);
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::ignite() {
    return static_cast<int64_t>(post_active("ignite", [](sim::FlightSession& session) {
        session.ignite();
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::cutoff() {
    return static_cast<int64_t>(post_active("cutoff", [](sim::FlightSession& session) {
        session.cutoff();
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::release_clamps() {
    return static_cast<int64_t>(post_active("release_clamps", [](sim::FlightSession& session) {
        session.release_clamps();
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::stage() {
    return static_cast<int64_t>(post_active("stage", [](sim::FlightSession& session) {
        session.stage();
        return Outcome{true, {}};
    }));
}

int FlightSim::body_count() const { return static_cast<int>(snapshot()->bodies.size()); }
int FlightSim::controlled_index() const { return snapshot()->controlled_index; }

const FlightSnapshot::Body* FlightSim::body(const FlightSnapshot& s, int index) const {
    if (!s.active || index < 0 || static_cast<size_t>(index) >= s.bodies.size()) {
        godot::UtilityFunctions::push_error("FlightSim: invalid body index");
        return nullptr;
    }
    return &s.bodies[static_cast<size_t>(index)];
}

godot::Vector3 FlightSim::body_position_gcrf_m(int index) const {
    const auto s = snapshot();
    const auto* b = body(*s, index);
    return b ? b->position_gcrf_m : godot::Vector3();
}

godot::Quaternion FlightSim::body_attitude_gcrf(int index) const {
    const auto s = snapshot();
    const auto* b = body(*s, index);
    return b ? b->attitude_gcrf : godot::Quaternion();
}

godot::Vector3 FlightSim::body_velocity_gcrf_mps(int index) const {
    const auto s = snapshot();
    const auto* b = body(*s, index);
    return b ? b->velocity_gcrf_mps : godot::Vector3();
}

double FlightSim::body_cg_station_m(int index) const {
    const auto s = snapshot();
    const auto* b = body(*s, index);
    return b ? b->cg_station_m : 0.0;
}

#define STARSIM_SNAPSHOT_BODY_GETTER(Type, method, field)  \
    Type FlightSim::method(int index) const {               \
        const auto s = snapshot();                          \
        const auto* b = body(*s, index);                    \
        return b ? b->field : Type();                       \
    }

STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_stages, stages)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_engines, engines)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_flaps, flaps)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_grid_fins, grid_fins)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_rcs, rcs)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Array, body_parachutes, parachutes)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Dictionary, body_legs, legs)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Dictionary, capsule_state, capsule_state)
STARSIM_SNAPSHOT_BODY_GETTER(godot::Dictionary, telemetry, telemetry)

#undef STARSIM_SNAPSHOT_BODY_GETTER

int64_t FlightSim::set_splashdown_zone(const godot::String& zone_id) {
    const std::string id = zone_id.utf8().get_data();
    return static_cast<int64_t>(post("set_splashdown_zone", [this, id](sim::FlightSession& session) {
        session.set_splashdown_zone(config::load_splashdown_zone(config_dir_, id));
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::plan_capsule_return() {
    return static_cast<int64_t>(post_active("plan_capsule_return", [](sim::FlightSession& session) {
        return Outcome{session.plan_capsule_return(), {}};
    }));
}

int64_t FlightSim::engage_ascent_autopilot(double periapsis_km, double apoapsis_km, double inclination_deg,
                                           bool northerly) {
    return static_cast<int64_t>(post_active("engage_ascent_autopilot", [=](sim::FlightSession& session) {
        session.engage_ascent_autopilot(
            {periapsis_km * 1000.0, apoapsis_km * 1000.0, inclination_deg * constants::deg_to_rad, northerly});
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::disengage_autopilot() {
    return static_cast<int64_t>(post_active("disengage_autopilot", [](sim::FlightSession& session) {
        session.disengage_autopilot();
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::plan_circularization() {
    return static_cast<int64_t>(post_active("plan_circularization", [](sim::FlightSession& session) {
        return Outcome{session.plan_circularization_at_apoapsis(), {}};
    }));
}

int64_t FlightSim::plan_ship_return() {
    return static_cast<int64_t>(post_active("plan_ship_return", [](sim::FlightSession& session) {
        return Outcome{session.plan_ship_return(), {}};
    }));
}

int64_t FlightSim::plan_burn(double seconds_from_now, double prograde_mps, double normal_mps, double radial_mps) {
    return static_cast<int64_t>(post_active("plan_burn", [=](sim::FlightSession& session) {
        const auto& s = session.body(session.controlled_index()).state();
        const auto at_node = orbit::propagate_two_body({s.position_gcrf_m, s.velocity_gcrf_mps},
                                                       session.gm_earth_m3ps2(), seconds_from_now);
        return Outcome{session.execute_burn(
                           {session.epoch() + seconds_from_now,
                            gnc::delta_v_from_components(at_node, prograde_mps, normal_mps, radial_mps)}),
                       {}};
    }));
}

godot::Dictionary FlightSim::autopilot_status() const { return snapshot()->autopilot_status; }
godot::Dictionary FlightSim::lunar_state() const { return snapshot()->lunar_state; }
godot::Dictionary FlightSim::lunar_site() const { return snapshot()->lunar_site; }

godot::Dictionary FlightSim::moon_terrain(double latitude_deg, double longitude_deg) const {
    godot::Dictionary d;
    d["present"] = false;
    const auto snap = snapshot();
    if (!snap->moon_terrain) return d;
    const double lat = latitude_deg * constants::deg_to_rad, lon = longitude_deg * constants::deg_to_rad;
    const auto& terrain = *snap->moon_terrain;
    d["present"] = true;
    d["height_m"] = terrain.height_m(lat, lon);
    const math::Vec3 up{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
    d["normal_moon_me"] = to_godot(terrain.normal_fixed((terrain.sphere_radius_m() + terrain.height_m(lat, lon)) * up));
    d["in_tile"] = terrain.in_tile(lat, lon);
    return d;
}

int64_t FlightSim::complete_pilot_correction() {
    return static_cast<int64_t>(post_active("complete_pilot_correction", [](sim::FlightSession& session) {
        if (session.lunar_step() != sim::LunarStep::PilotCorrection) {
            return Outcome{false, godot::String::utf8("Keine Korrektur von Hand anstehend")};
        }
        session.complete_pilot_correction();
        return Outcome{true, godot::String::utf8(session.lunar_status().c_str())};
    }));
}

int64_t FlightSim::replan_lunar_return() {
    return static_cast<int64_t>(post_active("replan_lunar_return", [](sim::FlightSession& session) {
        const bool ok = session.replan_lunar_return();
        return Outcome{ok, godot::String::utf8(session.lunar_status().c_str())};
    }));
}

godot::Dictionary FlightSim::iss_state() const { return snapshot()->iss_state; }
godot::Dictionary FlightSim::rendezvous_plan() const { return snapshot()->rendezvous_plan; }
godot::Dictionary FlightSim::docking_state() const { return snapshot()->docking_state; }
godot::Dictionary FlightSim::station_geometry() const { return snapshot()->station_geometry; }
godot::Dictionary FlightSim::iss_mission() const { return snapshot()->iss_mission; }
godot::Dictionary FlightSim::transfer_state() const { return snapshot()->transfer_state; }
godot::Dictionary FlightSim::depot_state() const { return snapshot()->depot_state; }
godot::Dictionary FlightSim::campaign_flight_state() const { return snapshot()->campaign_flight_state; }
godot::Dictionary FlightSim::crew_escape_state() const { return snapshot()->crew_escape_state; }
godot::Dictionary FlightSim::mission_timeline() const { return snapshot()->mission_timeline; }
int64_t FlightSim::body_serial(int index) const {
    const auto snap = snapshot();
    return index >= 0 && static_cast<size_t>(index) < snap->bodies.size() ? snap->bodies[index].serial : int64_t{0};
}

int64_t FlightSim::set_crew_escape(const godot::String& id) {
    const std::string crew_id = id.utf8().get_data();
    return static_cast<int64_t>(post("set_crew_escape", [this, crew_id](sim::FlightSession& session) {
        if (crew_id.empty()) {
            session.set_crew_escape(std::nullopt);
            return Outcome{true, {}};
        }
        try {
            session.set_crew_escape(config::load_crew_escape(config_dir_, crew_id));
        } catch (const std::exception& e) {
            return Outcome{false, godot::String::utf8(e.what())};
        }
        return Outcome{true, {}};
    }));
}

int64_t FlightSim::refuelling_campaign(const godot::Dictionary& params) {
    sim::CampaignLevers levers;
    std::string refusal;
    const godot::Array keys = params.keys();
    for (int64_t k = 0; k < keys.size() && refusal.empty(); ++k) {
        const godot::String key = keys[k];
        const godot::Variant value = params[keys[k]];
        if (key == "reference_phasing") {  // f1: the r5 single phasing orbit
            if (value.get_type() != godot::Variant::BOOL) {
                refusal = "Kampagne: 'reference_phasing' ist kein Wahrheitswert";
                break;
            }
            levers.reference_phasing = value;
            continue;
        }
        if (value.get_type() != godot::Variant::FLOAT && value.get_type() != godot::Variant::INT) {
            refusal = "Kampagne: '" + std::string(key.utf8().get_data()) + "' ist keine Zahl";
            break;
        }
        const double v = value;
        if (key == "height_km") levers.depot_height_m = v * 1e3;
        else if (key == "interval_h") levers.launch_interval_s = v * 3600.0;
        else if (key == "phasing_limit_h") levers.phasing_limit_s = v * 3600.0;
        else if (key == "tanker_dry_t") levers.tanker_dry_mass_kg = v * 1e3;
        else if (key == "reserve_t") levers.tanker_reserve_kg = v * 1e3;
        else if (key == "initial_t") levers.depot_initial_kg = v * 1e3;
        else if (key == "insulation") levers.depot_insulation_scale = v;
        else if (key == "hls_wait_d") levers.hls_wait_s = v * constants::seconds_per_day;
        else refusal = "Kampagne: unbekannter Parameter '" + std::string(key.utf8().get_data()) + "'";
    }
    if (refusal.empty()) refusal = sim::campaign_lever_error(levers);
    return static_cast<int64_t>(post("refuelling_campaign", [this, levers, refusal](sim::FlightSession&) {
        if (!refusal.empty()) return Outcome{false, godot::String::utf8(refusal.c_str())};
        const auto report = sim::plan_refuelling_campaign({ephemeris_, earth_, moon_, time_, config_dir_}, levers);
        return Outcome{report.result.closes, godot::String::utf8(report.result.reason.c_str()),
                       campaign_dictionary(report, time_)};
    }));
}

int64_t FlightSim::proximity_hold() {
    return static_cast<int64_t>(post_active("proximity_hold", [](sim::FlightSession& session) {
        const bool ok = session.proximity_hold();
        return Outcome{ok, godot::String::utf8(session.proximity_status().c_str())};
    }));
}

int64_t FlightSim::proximity_retreat() {
    return static_cast<int64_t>(post_active("proximity_retreat", [](sim::FlightSession& session) {
        const bool ok = session.proximity_retreat();
        return Outcome{ok, godot::String::utf8(session.proximity_status().c_str())};
    }));
}

int64_t FlightSim::proximity_abort() {
    return static_cast<int64_t>(post_active("proximity_abort", [](sim::FlightSession& session) {
        const bool ok = session.proximity_abort();
        return Outcome{ok, godot::String::utf8(session.proximity_status().c_str())};
    }));
}

int64_t FlightSim::proximity_resume() {
    return static_cast<int64_t>(post_active("proximity_resume", [](sim::FlightSession& session) {
        const bool ok = session.proximity_resume();
        return Outcome{ok, godot::String::utf8(session.proximity_status().c_str())};
    }));
}

int64_t FlightSim::undock() {
    return static_cast<int64_t>(post_active("undock", [](sim::FlightSession& session) {
        const bool ok = session.undock();
        return Outcome{ok, godot::String::utf8(session.docking_status().c_str())};
    }));
}

int64_t FlightSim::settle(bool enabled) {
    return static_cast<int64_t>(post_active("settle", [enabled](sim::FlightSession& session) {
        const bool ok = session.settle(enabled);
        return Outcome{ok, godot::String::utf8(ok ? "" : "Settling nicht möglich: angedockt oder im Anflug")};
    }));
}

godot::PackedVector3Array FlightSim::predict_trajectory(int index, double duration_s, double step_s) const {
    const auto s = snapshot();
    if (!body(*s, index)) return {};
    {
        std::lock_guard lock(request_mutex_);
        prediction_request_ = {index, duration_s, step_s};
    }
    return s->prediction;
}

godot::PackedVector3Array FlightSim::target_orbit_points(int count) const {
    godot::PackedVector3Array points;
    const auto s = snapshot();
    if (!s->active || !s->has_target_orbit || count < 3) return points;
    const auto& g = s->target_plane_normal_gcrf;
    const math::Vec3 n{g.x, g.y, g.z};
    const math::Vec3 u = math::normalized(cross(math::Vec3{0.0, 0.0, 1.0}, n));  // ascending node
    const math::Vec3 w = cross(n, u);
    const double r = s->target_radius_m;
    for (int i = 0; i <= count; ++i) {
        const double t = constants::two_pi * i / count;
        points.push_back(to_godot(r * (std::cos(t) * u + std::sin(t) * w)));
    }
    return points;
}

void FlightSim::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    using godot::MethodInfo;
    using godot::PropertyInfo;
    using godot::Variant;
    ClassDB::bind_method(D_METHOD("setup", "config_dir"), &FlightSim::setup);
    ClassDB::bind_method(D_METHOD("launch_from_pad", "vehicle_id", "latitude_deg", "longitude_deg", "height_m",
                                  "heading_deg", "tdb_s"),
                         &FlightSim::launch_from_pad);
    ClassDB::bind_method(D_METHOD("launch_from_site", "vehicle_id", "site_id", "heading_deg", "tdb_s", "countdown_s"),
                         &FlightSim::launch_from_site, DEFVAL(0.0));
    ClassDB::bind_method(D_METHOD("start_mission", "mission_id", "tdb_s", "wait_on_pad"), &FlightSim::start_mission,
                         DEFVAL(false));
    ClassDB::bind_method(D_METHOD("mission_site_id"), &FlightSim::mission_site_id);
    ClassDB::bind_method(D_METHOD("mission_start_tdb_s"), &FlightSim::mission_start_tdb_s);
    ClassDB::bind_method(D_METHOD("wait_idle", "timeout_s"), &FlightSim::wait_idle);
    ClassDB::bind_method(D_METHOD("planning_s"), &FlightSim::planning_s);
    ClassDB::bind_method(D_METHOD("result", "id"), &FlightSim::result);
    ClassDB::bind_method(D_METHOD("body_rcs", "index"), &FlightSim::body_rcs);
    ClassDB::bind_method(D_METHOD("body_parachutes", "index"), &FlightSim::body_parachutes);
    ClassDB::bind_method(D_METHOD("capsule_state", "index"), &FlightSim::capsule_state);
    ClassDB::bind_method(D_METHOD("splashdown_zone"), &FlightSim::splashdown_zone);
    ClassDB::bind_method(D_METHOD("set_splashdown_zone", "zone_id"), &FlightSim::set_splashdown_zone);
    ClassDB::bind_method(D_METHOD("plan_capsule_return"), &FlightSim::plan_capsule_return);
    ClassDB::bind_method(D_METHOD("landing_zone"), &FlightSim::landing_zone);
    ClassDB::bind_method(D_METHOD("body_legs", "index"), &FlightSim::body_legs);
    ClassDB::bind_method(D_METHOD("body_grid_fins", "index"), &FlightSim::body_grid_fins);
    ClassDB::bind_method(D_METHOD("site_tower", "site_id"), &FlightSim::site_tower);
    ClassDB::bind_method(D_METHOD("tower_state"), &FlightSim::tower_state);
    ClassDB::bind_method(D_METHOD("set_booster_return", "enabled"), &FlightSim::set_booster_return);
    ClassDB::bind_method(D_METHOD("set_realtime_planning", "enabled"), &FlightSim::set_realtime_planning);
    ClassDB::bind_method(D_METHOD("abort_landing", "index"), &FlightSim::abort_landing);
    ClassDB::bind_method(D_METHOD("return_state", "index"), &FlightSim::return_state);
    ClassDB::bind_method(D_METHOD("body_flaps", "index"), &FlightSim::body_flaps);
    ClassDB::bind_method(D_METHOD("is_active"), &FlightSim::is_active);
    ClassDB::bind_method(D_METHOD("advance_to", "tdb_s"), &FlightSim::advance_to);
    ClassDB::bind_method(D_METHOD("mission_time_s"), &FlightSim::mission_time_s);
    ClassDB::bind_method(D_METHOD("set_input", "pitch", "yaw", "roll", "throttle"), &FlightSim::set_input);
    ClassDB::bind_method(D_METHOD("set_rate_mode", "enabled"), &FlightSim::set_rate_mode);
    ClassDB::bind_method(D_METHOD("ignite"), &FlightSim::ignite);
    ClassDB::bind_method(D_METHOD("cutoff"), &FlightSim::cutoff);
    ClassDB::bind_method(D_METHOD("release_clamps"), &FlightSim::release_clamps);
    ClassDB::bind_method(D_METHOD("stage"), &FlightSim::stage);
    ClassDB::bind_method(D_METHOD("body_count"), &FlightSim::body_count);
    ClassDB::bind_method(D_METHOD("controlled_index"), &FlightSim::controlled_index);
    ClassDB::bind_method(D_METHOD("body_position_gcrf_m", "index"), &FlightSim::body_position_gcrf_m);
    ClassDB::bind_method(D_METHOD("body_attitude_gcrf", "index"), &FlightSim::body_attitude_gcrf);
    ClassDB::bind_method(D_METHOD("body_cg_station_m", "index"), &FlightSim::body_cg_station_m);
    ClassDB::bind_method(D_METHOD("body_stages", "index"), &FlightSim::body_stages);
    ClassDB::bind_method(D_METHOD("body_engines", "index"), &FlightSim::body_engines);
    ClassDB::bind_method(D_METHOD("telemetry", "index"), &FlightSim::telemetry);
    ClassDB::bind_method(D_METHOD("engage_ascent_autopilot", "periapsis_km", "apoapsis_km", "inclination_deg",
                                  "northerly"),
                         &FlightSim::engage_ascent_autopilot);
    ClassDB::bind_method(D_METHOD("disengage_autopilot"), &FlightSim::disengage_autopilot);
    ClassDB::bind_method(D_METHOD("plan_circularization"), &FlightSim::plan_circularization);
    ClassDB::bind_method(D_METHOD("plan_ship_return"), &FlightSim::plan_ship_return);
    ClassDB::bind_method(D_METHOD("plan_burn", "seconds_from_now", "prograde_mps", "normal_mps", "radial_mps"),
                         &FlightSim::plan_burn);
    ClassDB::bind_method(D_METHOD("autopilot_status"), &FlightSim::autopilot_status);
    ClassDB::bind_method(D_METHOD("lunar_state"), &FlightSim::lunar_state);
    ClassDB::bind_method(D_METHOD("lunar_site"), &FlightSim::lunar_site);
    ClassDB::bind_method(D_METHOD("moon_terrain", "latitude_deg", "longitude_deg"), &FlightSim::moon_terrain);
    ClassDB::bind_method(D_METHOD("complete_pilot_correction"), &FlightSim::complete_pilot_correction);
    ClassDB::bind_method(D_METHOD("replan_lunar_return"), &FlightSim::replan_lunar_return);
    ClassDB::bind_method(D_METHOD("iss_state"), &FlightSim::iss_state);
    ClassDB::bind_method(D_METHOD("rendezvous_plan"), &FlightSim::rendezvous_plan);
    ClassDB::bind_method(D_METHOD("docking_state"), &FlightSim::docking_state);
    ClassDB::bind_method(D_METHOD("station_geometry"), &FlightSim::station_geometry);
    ClassDB::bind_method(D_METHOD("iss_mission"), &FlightSim::iss_mission);
    ClassDB::bind_method(D_METHOD("proximity_hold"), &FlightSim::proximity_hold);
    ClassDB::bind_method(D_METHOD("proximity_retreat"), &FlightSim::proximity_retreat);
    ClassDB::bind_method(D_METHOD("proximity_abort"), &FlightSim::proximity_abort);
    ClassDB::bind_method(D_METHOD("proximity_resume"), &FlightSim::proximity_resume);
    ClassDB::bind_method(D_METHOD("undock"), &FlightSim::undock);
    ClassDB::bind_method(D_METHOD("settle", "enabled"), &FlightSim::settle);
    ClassDB::bind_method(D_METHOD("transfer_state"), &FlightSim::transfer_state);
    ClassDB::bind_method(D_METHOD("depot_state"), &FlightSim::depot_state);
    ClassDB::bind_method(D_METHOD("campaign_flight_state"), &FlightSim::campaign_flight_state);
    ClassDB::bind_method(D_METHOD("crew_escape_state"), &FlightSim::crew_escape_state);
    ClassDB::bind_method(D_METHOD("mission_timeline"), &FlightSim::mission_timeline);
    ClassDB::bind_method(D_METHOD("body_serial", "index"), &FlightSim::body_serial);
    ClassDB::bind_method(D_METHOD("set_crew_escape", "id"), &FlightSim::set_crew_escape);
    ClassDB::bind_method(D_METHOD("refuelling_campaign", "params"), &FlightSim::refuelling_campaign);
    ClassDB::bind_method(D_METHOD("predict_trajectory", "index", "duration_s", "step_s"),
                         &FlightSim::predict_trajectory);
    ClassDB::bind_method(D_METHOD("target_orbit_points", "count"), &FlightSim::target_orbit_points);
    ClassDB::bind_method(D_METHOD("body_velocity_gcrf_mps", "index"), &FlightSim::body_velocity_gcrf_mps);
    ADD_SIGNAL(MethodInfo("command_finished", PropertyInfo(Variant::INT, "id"), PropertyInfo(Variant::STRING, "name"),
                          PropertyInfo(Variant::BOOL, "ok"), PropertyInfo(Variant::STRING, "message")));
}

}  // namespace starsim::bindings
