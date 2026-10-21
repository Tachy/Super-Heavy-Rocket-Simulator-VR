#include "starsim/vehicle/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include "starsim/aero/parachute.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"

namespace starsim::vehicle {

using math::Vec3;

namespace {

// Payload and fairing sit in the upper part of their stage [E].
constexpr double extra_mass_station_fraction = 0.75;
// The RCS allocation is rebuilt when the CG has moved this far (lever change negligible below).
constexpr double rcs_cg_tolerance_m = 0.01;
// Propellant settling [E]: below this specific force the liquid holds its position (orbital drag ~1e-5 m/s^2, RCS
// translation ~0.02 m/s^2 settles it); it follows the apparent gravity with tau = sqrt(2 D / a) - the time to travel
// a tank diameter D from rest - kept between the bounds (quasi-static: no slosh).
constexpr double settling_threshold_mps2 = 0.01;
constexpr double settling_tau_min_s = 0.5;
constexpr double settling_tau_max_s = 60.0;

double approach(double value, double target, double max_step) {
    return value + std::clamp(target - value, -max_step, max_step);
}

const GroupCommand* find_group(const VehicleCommand& command, const std::string& key) {
    for (const auto& g : command.groups) {
        if (g.group == key) return &g;
    }
    return nullptr;
}

}  // namespace

Vec3 Engine::thrust_direction() const {
    if (axis_body.x != 1.0) return axis_body;  // canted: fixed (no gimbal)
    return normalized(Vec3{1.0, std::tan(gimbal_yaw_rad), std::tan(gimbal_pitch_rad)});
}

double Stage::propellant_kg(config::TankContents contents) const {
    double total = 0.0;
    for (const auto& t : tanks) {
        if (t.config.contents == contents) total += t.mass_kg;
    }
    return total;
}

double Stage::propellant_kg(config::TankContents contents, config::TankRole role) const {
    double total = 0.0;
    for (const auto& t : tanks) {
        if (t.config.contents == contents && t.config.role == role) total += t.mass_kg;
    }
    return total;
}

bool Stage::has_header_tanks() const {
    return std::any_of(tanks.begin(), tanks.end(), [](const Tank& t) { return t.config.role == config::TankRole::Header; });
}

Vehicle::Vehicle(const config::VehicleConfig& config, const std::filesystem::path& config_dir)
    : id_(config.id), config_dir_(config_dir) {
    std::map<std::string, std::shared_ptr<const propulsion::EnginePerformance>> performances;
    for (const auto& [engine_id, engine_config] : config.engines) {
        performances[engine_id] = std::make_shared<propulsion::EnginePerformance>(engine_config);
    }
    double station = 0.0;
    for (const auto& stage_config : config.stages) {
        Stage stage;
        stage.config = stage_config;
        stage.base_station_m = station;
        stage.extra_mass_kg = stage_config.payload_mass_kg + stage_config.fairing_mass_kg;
        for (const auto& tank_config : stage_config.tanks) {
            const double diameter = tank_config.diameter_m > 0.0 ? tank_config.diameter_m : stage_config.diameter_m;
            const double area = constants::pi * 0.25 * diameter * diameter;
            const double density = tank_config.contents == config::TankContents::Oxidizer ? config.propellant.oxidizer_density_kg_m3
                                   : tank_config.contents == config::TankContents::Fuel   ? config.propellant.fuel_density_kg_m3
                                                                                          : tank_config.density_kg_m3;
            const bool settles = tank_config.contents != config::TankContents::Rcs && tank_config.capacity_kg > 0.0;
            stage.tanks.push_back({tank_config, density, area, tank_config.mass_kg, 0.5 * diameter,
                                   settles ? tank_config.capacity_kg / (density * area) : 0.0});
            auto& tank = stage.tanks.back();
            if (tank_config.contents == config::TankContents::Oxidizer) {
                tank.boiling_point_k = config.propellant.oxidizer_boiling_point_k;
                tank.latent_heat_j_kg = config.propellant.oxidizer_latent_heat_j_kg;
                tank.surface_tension_n_m = config.propellant.oxidizer_surface_tension_n_m;
            } else if (tank_config.contents == config::TankContents::Fuel) {
                tank.boiling_point_k = config.propellant.fuel_boiling_point_k;
                tank.latent_heat_j_kg = config.propellant.fuel_latent_heat_j_kg;
                tank.surface_tension_n_m = config.propellant.fuel_surface_tension_n_m;
            }
        }
        for (const auto& group : stage_config.engine_groups) {
            for (int k = 0; k < group.count; ++k) {
                const double angle = group.angles_rad.empty() ? constants::two_pi * k / group.count
                                                              : group.angles_rad[static_cast<size_t>(k)];
                Engine engine;
                engine.performance = performances.at(group.engine_id);
                engine.group_id = stage_config.id + "/" + group.id;
                if (group.cant_rad != 0.0 && group.gimbal) {
                    throw std::runtime_error("engine group " + group.id + ": a canted group cannot gimbal");
                }
                engine.gimbal_point_m = {station + group.station_m, group.ring_radius_m * std::cos(angle),
                                         group.ring_radius_m * std::sin(angle)};
                engine.axis_body = {std::cos(group.cant_rad), -std::sin(group.cant_rad) * std::cos(angle),
                                    -std::sin(group.cant_rad) * std::sin(angle)};
                engine.ring_angle_rad = angle;
                engine.gimbaled = group.gimbal;
                engine.relight = group.relight;
                engine.abort_only = group.abort_only;
                engine.landing_only = group.landing_only;
                stage.engines.push_back(engine);
            }
        }
        const auto& fins = stage_config.grid_fins;
        if (!fins.azimuths_rad.empty()) {
            stage.grid_fin_table = std::make_shared<aero::GridFinTable>(config::load_csv_table(config_dir / fins.table));
            for (double azimuth : fins.azimuths_rad) {
                stage.grid_fins.push_back({{station + fins.station_m, azimuth, fins.radius_m, fins.area_m2},
                                           fins.max_deflection_rad, fins.rate_rad_s, 0.0});
            }
        }
        if (!stage_config.flaps.empty()) {
            stage.flap_table =
                std::make_shared<aero::FlapTable>(config::load_csv_table(config_dir / stage_config.aero.flap_table));
            for (const auto& f : stage_config.flaps) {
                stage.flaps.push_back({{station + f.station_fwd_m, station + f.station_aft_m, f.hinge_azimuth_rad,
                                        0.5 * stage_config.diameter_m, f.span_m, f.area_m2,
                                        stage_config.aero.belly_azimuth_rad},
                                       f.max_deflection_rad, f.rate_rad_s,
                                       std::clamp(f.initial_deflection_rad, 0.0, f.max_deflection_rad)});
            }
        }
        for (const auto& p : stage_config.parachutes) {
            Parachute chute;
            chute.config = p;
            chute.attach_station_m = station + p.attach_station_m;
            stage.parachutes.push_back(chute);
        }
        for (const auto& t : stage_config.rcs.thrusters) {
            int tank = -1;
            for (size_t i = 0; i < stage.tanks.size(); ++i) {
                if (stage.tanks[i].config.id == stage_config.rcs.propellant) tank = static_cast<int>(i);
            }
            stage.rcs.push_back({t.id, t.position_m + Vec3{station, 0.0, 0.0}, t.direction, t.thrust_n,
                                 stage_config.rcs.isp_s, tank, 0.0, false});
            stage.rcs.back().behind_nose_cone = t.behind_nose_cone;
        }
        station += stage_config.length_m;
        stages_.push_back(std::move(stage));
    }
    rebuild_aero();
    refresh_rcs_availability();
    if (has_landing_legs() && stages_.front().config.landing_legs.fixed) set_leg_deployment(1.0);  // always out
}

double Vehicle::length_m() const {
    double length = 0.0;
    for (const auto& s : stages_) length += s.config.length_m;
    return length;
}

double Vehicle::diameter_m() const {
    double d = 0.0;
    for (const auto& s : stages_) d = std::max(d, s.config.diameter_m);
    return d;
}

void Vehicle::rebuild_aero() {
    // Drag table of the lowest stage that has one; the ascent table describes the whole stack.
    aero::AxialDragCurve drag;
    for (const auto& s : stages_) {
        if (!s.config.aero.ascent_drag_table.empty()) {
            drag = aero::AxialDragCurve(config::load_csv_table(config_dir_ / s.config.aero.ascent_drag_table));
            break;
        }
    }
    if (drag.empty() && aero_) {
        // [E] Upper stage without its own table keeps the stack table after separation.
        drag = aero_->drag_curve();
    }
    // Tail-first flight: the base shape is that of the lowest stage.
    aero::AxialDragCurve tail_first;
    if (const auto& table = stages_.front().config.aero.descent_drag_table; !table.empty()) {
        tail_first = aero::AxialDragCurve(config::load_csv_table(config_dir_ / table));
        // The lowest stage's grid fins (fixed, deployed) add their axial drag in tail-first flight; the ascent table
        // already describes the whole stack.
        const auto& low = stages_.front();
        if (low.grid_fin_table && !low.grid_fins.empty()) {
            const double reference_area_m2 = constants::pi * 0.25 * diameter_m() * diameter_m();
            double fin_area_m2 = 0.0;
            for (const auto& fin : low.grid_fins) fin_area_m2 += fin.geometry.area_m2;
            std::vector<double> delta;
            for (const double ca : low.grid_fin_table->axial_coefficients()) delta.push_back(ca * fin_area_m2 / reference_area_m2);
            if (!delta.empty()) tail_first = tail_first.plus(low.grid_fin_table->mach(), delta);
        }
    }
    // Identified hull cross-flow of a stage flying alone (the ship's entry); stacks keep the generic value.
    aero::CrossflowCurve crossflow;
    if (stages_.size() == 1 && !stages_.front().config.aero.crossflow_table.empty()) {
        crossflow = aero::CrossflowCurve(config::load_csv_table(config_dir_ / stages_.front().config.aero.crossflow_table));
    }
    aero_ = std::make_unique<aero::SlenderBodyAero>(
        aero::SlenderBodyGeometry{length_m(), diameter_m(), stages_.back().config.aero.nose_length_m}, drag,
        tail_first, crossflow);
    // A capsule flying alone (its heat shield at the bottom of the stack): blunt-body aerodynamics, apex at station 0.
    capsule_aero_.reset();
    if (const auto& low = stages_.front().config.aero; !low.capsule_table.empty()) {
        capsule_aero_ = std::make_unique<aero::CapsuleAero>(
            aero::load_capsule_aero_table(config_dir_ / low.capsule_table), low.reference_diameter_m, 0.0);
    }
    fixed_area_aero_.reset();
    if (const auto& low = stages_.front().config.aero; low.fixed_drag_area_m2 > 0.0) {
        fixed_area_aero_ = std::make_unique<aero::FixedAreaAero>(aero::FixedAreaAero{low.fixed_drag_area_m2,
                                                                                    low.fixed_drag_coefficient});
    }
}

aero::AeroResult Vehicle::hull_aero(const Vec3& air_velocity_body_mps, const environment::AtmosphereState& air,
                                    bool powered) const {
    if (fixed_area_aero_) return fixed_area_aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps);
    if (capsule_aero_) return capsule_aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps);
    return aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps, powered, air.mean_free_path_m);
}

MassProperties Vehicle::mass_properties(const std::vector<std::vector<double>>* flows, double tau_s) const {
    struct Part {
        double mass, station, ixx, iyy;
        double y = 0.0, z = 0.0;
        bool liquid = false;
        double izz = -1.0;  // < 0: = iyy (axisymmetric)
    };
    std::vector<Part> parts;
    const Vec3& down = liquid_down_body_;
    const bool settled_aft = down.x == -1.0 && down.y == 0.0 && down.z == 0.0;  // the exact old code path
    const double tilt_rad = std::acos(std::clamp(-down.x, -1.0, 1.0));
    const double lateral = std::hypot(down.y, down.z);
    const double lateral_y = lateral > 0.0 ? down.y / lateral : 0.0;
    const double lateral_z = lateral > 0.0 ? down.z / lateral : 0.0;
    for (size_t si = 0; si < stages_.size(); ++si) {
        const auto& s = stages_[si];
        const double r = 0.5 * s.config.diameter_m;
        const double l = s.config.length_m;
        // Dry structure: the configured principal moments (a station), else a thin-walled cylinder about its own CG [E].
        if (const auto& i = s.config.inertia_kg_m2) {
            parts.push_back({s.config.dry_mass_kg, s.base_station_m + s.config.dry_cg_station_m, i->x, i->y,
                             s.config.dry_cg_offset_m.y, s.config.dry_cg_offset_m.z, false, i->z});
        } else {
            parts.push_back({s.config.dry_mass_kg, s.base_station_m + s.config.dry_cg_station_m,
                             s.config.dry_mass_kg * r * r, s.config.dry_mass_kg * (0.5 * r * r + l * l / 12.0),
                             s.config.dry_cg_offset_m.y, s.config.dry_cg_offset_m.z});
        }
        if (s.extra_mass_kg > 0.0) {
            parts.push_back({s.extra_mass_kg, s.base_station_m + extra_mass_station_fraction * l, 0.0, 0.0});
        }
        // Settled liquid; the fluid does not spin with the tank (no roll inertia) [E]. Settled aft: a column on the
        // tank bottom; otherwise below a surface perpendicular to liquid_down_body_ (liquid_settling).
        for (size_t ti = 0; ti < s.tanks.size(); ++ti) {
            const auto& t = s.tanks[ti];
            const double mass = std::max(0.0, t.mass_kg - (flows ? (*flows)[si][ti] * tau_s : 0.0));
            if (mass <= 0.0) continue;
            const double base = s.base_station_m + t.config.base_station_m;
            if (settled_aft || t.length_m <= 0.0) {
                const double h = mass / (t.density_kg_m3 * t.area_m2);
                parts.push_back({mass, base + 0.5 * h, 0.0, mass * (0.25 * t.radius_m * t.radius_m + h * h / 12.0)});
                continue;
            }
            if (!t.settling) t.settling = settling_table(t.length_m / t.radius_m);
            const double fill = mass / (t.density_kg_m3 * t.area_m2 * t.length_m);
            const auto m = t.settling->query(fill, tilt_rad);
            const double rt = t.radius_m;
            parts.push_back({mass, base + 0.5 * t.length_m + rt * m.axial, 0.0,
                             mass * rt * rt * (m.axial_var + 0.5 * (m.lateral_var + m.transverse_var)),
                             rt * m.lateral * lateral_y, rt * m.lateral * lateral_z, true});
        }
    }
    if (attached_mass_) {
        parts.push_back({attached_mass_->mass_kg, attached_mass_->position_m.x, 0.0, 0.0, attached_mass_->position_m.y,
                         attached_mass_->position_m.z});
    }
    MassProperties mp;
    for (const auto& p : parts) {
        mp.mass_kg += p.mass;
        mp.cg_station_m += p.mass * p.station;
        mp.cg_offset_m.y += p.mass * p.y;
        mp.cg_offset_m.z += p.mass * p.z;
        if (!p.liquid) mp.roll_offset_m += Vec3{0.0, p.mass * p.y, p.mass * p.z};
    }
    mp.cg_station_m /= mp.mass_kg;
    // Lateral offset (capsule trim): enters the aerodynamic and RCS moments only; the inertia stays on the axis (its
    // parallel-axis term is < 0.1 %, d3 plan ruling 2).
    mp.cg_offset_m = mp.cg_offset_m / mp.mass_kg;
    mp.roll_offset_m = mp.roll_offset_m / mp.mass_kg;
    double ixx = 0.0, iyy = 0.0, izz = 0.0;
    for (const auto& p : parts) {
        const double d = p.station - mp.cg_station_m;
        ixx += p.ixx;
        iyy += p.iyy + p.mass * d * d;
        izz += (p.izz < 0.0 ? p.iyy : p.izz) + p.mass * d * d;
    }
    mp.inertia_kg_m2 = {ixx, iyy, izz};
    return mp;
}

void Vehicle::settle_propellant(const Vec3& specific_force_body_mps2, double dt_s) {
    const double a = math::norm(specific_force_body_mps2);
    if (a < settling_threshold_mps2 || dt_s <= 0.0) return;
    const Vec3 target = specific_force_body_mps2 / -a;
    const Vec3 v = liquid_down_body_;
    const double angle = std::acos(std::clamp(math::dot(v, target), -1.0, 1.0));
    if (angle <= 0.0) return;
    // Turn on the sphere about v x target (a reversal through 180 deg picks a perpendicular axis): never through zero.
    Vec3 axis = math::cross(v, target);
    if (math::norm(axis) < 1e-9) axis = math::cross(v, std::abs(v.x) < 0.9 ? Vec3{1.0, 0.0, 0.0} : Vec3{0.0, 0.0, 1.0});
    axis = math::normalized(axis);
    const double tau = std::clamp(std::sqrt(2.0 * diameter_m() / a), settling_tau_min_s, settling_tau_max_s);
    const double step = angle * (1.0 - std::exp(-dt_s / tau));
    const double c = std::cos(step), s = std::sin(step);
    liquid_down_body_ = math::normalized(v * c + math::cross(axis, v) * s + axis * (math::dot(axis, v) * (1.0 - c)));
}

void Vehicle::set_liquid_down_body(const Vec3& down_body) { liquid_down_body_ = math::normalized(down_body); }

bool settling_tank_kind(const Tank& t) {
    return t.config.role == config::TankRole::Main && t.config.contents != config::TankContents::Rcs &&
           !t.config.propellant_management_device && t.length_m > 0.0 && t.config.capacity_kg > 0.0;
}

namespace {

// A tank whose outlet needs settled liquid before an ignition now (refuelling f3).
bool settling_tank(const Tank& t) { return settling_tank_kind(t) && t.mass_kg > 0.0; }

}  // namespace

bool Vehicle::needs_settling(size_t stage_index) const {
    const auto& stage = stages_.at(stage_index);
    if (stage.feed != FeedSource::Main) return false;
    return std::any_of(stage.tanks.begin(), stage.tanks.end(), settling_tank);
}

double Vehicle::settling_travel_m(size_t stage_index) const {
    double travel = 0.0;
    for (const auto& t : stages_.at(stage_index).tanks) {
        if (settling_tank(t)) travel = std::max(travel, t.length_m * (1.0 - t.mass_kg / t.config.capacity_kg));
    }
    return std::max(travel, settling_min_travel_m);
}

void Vehicle::update_settling(const LiquidEnvironment& liquid, double dt_s) {
    for (size_t s = 0; s < stages_.size(); ++s) {
        auto& stage = stages_[s];
        if (!needs_settling(s)) continue;
        bool covered = true;
        double a = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < stage.tanks.size(); ++i) {
            const auto& t = stage.tanks[i];
            if (!settling_tank(t)) continue;
            covered = covered && outlet_check(*this, s, i, liquid, settling_max_tilt_rad).covered;
            const double centre_m = stage.base_station_m + t.config.base_station_m + 0.5 * t.length_m;
            a = std::min(a, norm(apparent_gravity_body_mps2(liquid, Vec3{centre_m, 0.0, 0.0} - mass_properties().cg_point_m())));
        }
        const double t_settle = settling_time_s(settling_travel_m(s), a);
        if (!std::isfinite(t_settle)) continue;  // no acceleration: the liquid stays where it is
        stage.settled = std::clamp(stage.settled + (covered ? 1.0 : -1.0) * dt_s / t_settle, 0.0, 1.0);
    }
}

bool Vehicle::stage_settled(size_t stage_index) const {
    return !needs_settling(stage_index) || stages_.at(stage_index).settled >= 1.0;
}

double Vehicle::propellant_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) {
        for (const auto& t : s.tanks) total += t.config.contents == config::TankContents::Rcs ? 0.0 : t.mass_kg;
    }
    return total;
}

double Vehicle::main_propellant_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) {
        total += s.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
                 s.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
    }
    return total;
}

void Vehicle::set_tank_mass(size_t stage_index, std::string_view tank_id, double mass_kg) {
    if (stage_index >= stages_.size()) {
        throw std::invalid_argument("Unbekannte Stufe " + std::to_string(stage_index));
    }
    auto& stage = stages_[stage_index];
    const auto tank = std::find_if(stage.tanks.begin(), stage.tanks.end(),
                                   [&](const Tank& t) { return t.config.id == tank_id; });
    if (tank == stage.tanks.end()) {
        throw std::invalid_argument("Unbekannter Tank '" + std::string(tank_id) + "'");
    }
    if (!std::isfinite(mass_kg)) {
        throw std::invalid_argument("Tankmasse '" + std::string(tank_id) + "' ist keine Zahl");
    }
    tank->mass_kg = std::clamp(mass_kg, 0.0, tank->config.capacity_kg);
    const auto role = stage.feed == FeedSource::Header ? config::TankRole::Header : config::TankRole::Main;
    if (stage.propellant_kg(config::TankContents::Oxidizer, role) > 0.0 &&
        stage.propellant_kg(config::TankContents::Fuel, role) > 0.0) {
        for (auto& e : stage.engines) e.starved = e.failed;
    }
    refresh_rcs_availability();
}

void Vehicle::select_feed(size_t stage_index, FeedSource source) {
    auto& stage = stages_.at(stage_index);
    if (!stage.has_header_tanks() || stage.feed == source) return;
    stage.feed = source;
    const auto role = source == FeedSource::Header ? config::TankRole::Header : config::TankRole::Main;
    if (stage.propellant_kg(config::TankContents::Oxidizer, role) > 0.0 &&
        stage.propellant_kg(config::TankContents::Fuel, role) > 0.0) {
        for (auto& e : stage.engines) e.starved = e.failed;
    }
}

double Vehicle::rcs_propellant_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) total += s.propellant_kg(config::TankContents::Rcs);
    return total;
}

double Vehicle::rcs_supply_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) {
        if (s.rcs.empty()) continue;
        if (s.config.rcs.propellant == "main") {
            total += s.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
                     s.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
        } else {
            total += s.propellant_kg(config::TankContents::Rcs);
        }
    }
    return total;
}

Vec3 throttle_torque_nm(const Engine& engine, double cg_station_m) {
    const Vec3 arm = engine.gimbal_point_m - Vec3{cg_station_m, 0.0, 0.0};
    return cross(arm, engine.axis_body * engine.performance->thrust_n(1.0, 0.0));
}

void Vehicle::update_engines(double dt_s, const VehicleCommand& command, double ambient_pressure_pa) {
    size_t index = 0;
    for (size_t stage_index = 0; stage_index < stages_.size(); ++stage_index) {
        auto& stage = stages_[stage_index];
        for (auto& e : stage.engines) {
            const auto& cfg = e.performance->config();
            const GroupCommand* group = find_group(command, e.group_id);
            const bool want_run = group != nullptr && group->run;
            const double trim = index < command.engine_throttle_trim.size() ? command.engine_throttle_trim[index] : 0.0;
            ++index;
            const double target_throttle =
                std::clamp((group ? group->throttle : 0.0) + trim, cfg.throttle_min, cfg.throttle_max);
            const bool can_start = (!e.has_run || e.relight) && !e.starved &&
                                   e.performance->can_operate(ambient_pressure_pa);

            if (want_run && e.mode == EngineMode::Off && can_start && ignition_check_ && !stage_settled(stage_index)) {
                // Refuelling f3: gas at the outlets - the turbopumps ingest it and the engine is lost [E].
                e.has_run = true;
                e.starved = true;
                e.failed = true;
                gas_ingestion_ = true;
            } else if (want_run && e.mode == EngineMode::Off && can_start) {
                e.mode = EngineMode::Starting;
                e.has_run = true;
                e.throttle = target_throttle;
            } else if ((!want_run || !e.performance->can_operate(ambient_pressure_pa)) &&
                       (e.mode == EngineMode::Starting || e.mode == EngineMode::Running)) {
                e.mode = EngineMode::ShuttingDown;
            }

            switch (e.mode) {
                case EngineMode::Starting:
                    e.spool = cfg.startup_s > 0.0 ? std::min(1.0, e.spool + dt_s / cfg.startup_s) : 1.0;
                    if (e.spool >= 1.0) e.mode = EngineMode::Running;
                    break;
                case EngineMode::ShuttingDown:
                    e.spool = cfg.shutdown_s > 0.0 ? std::max(0.0, e.spool - dt_s / cfg.shutdown_s) : 0.0;
                    if (e.spool <= 0.0) e.mode = EngineMode::Off;
                    break;
                default:
                    break;
            }
            if (e.mode == EngineMode::Running || e.mode == EngineMode::Starting) {
                e.throttle = approach(e.throttle, target_throttle, cfg.throttle_rate_per_s * dt_s);
            }

            // Thrust vector control: common pitch/yaw plus tangential deflection for roll.
            double pitch_target = 0.0, yaw_target = 0.0;
            if (e.gimbaled && e.producing_thrust()) {
                const double radius = std::hypot(e.gimbal_point_m.y, e.gimbal_point_m.z);
                const double roll = radius > 0.1 ? command.gimbal_roll_rad : 0.0;
                yaw_target = command.gimbal_yaw_rad - std::sin(e.ring_angle_rad) * roll;
                pitch_target = command.gimbal_pitch_rad + std::cos(e.ring_angle_rad) * roll;
                const double magnitude = std::hypot(yaw_target, pitch_target);
                if (magnitude > cfg.gimbal_max_rad) {
                    yaw_target *= cfg.gimbal_max_rad / magnitude;
                    pitch_target *= cfg.gimbal_max_rad / magnitude;
                }
            }
            const double max_step = cfg.gimbal_rate_rad_s * dt_s;
            e.gimbal_pitch_rad = approach(e.gimbal_pitch_rad, pitch_target, max_step);
            e.gimbal_yaw_rad = approach(e.gimbal_yaw_rad, yaw_target, max_step);
        }
    }
}

bool Vehicle::has_landing_legs() const {
    return !stages_.empty() && !stages_.front().config.landing_legs.azimuths_rad.empty();
}

void Vehicle::set_leg_deployment(double fraction) {
    leg_deployment_ = has_landing_legs() ? std::clamp(fraction, 0.0, 1.0) : 0.0;
    legs_deploying_ = leg_deployment_ > 0.0;
}

void Vehicle::update_landing_legs(double dt_s) {
    if (!legs_deploying_ || leg_deployment_ >= 1.0) return;
    leg_deployment_ = std::min(1.0, leg_deployment_ + dt_s / stages_.front().config.landing_legs.deploy_time_s);
}

void Vehicle::update_grid_fins(double dt_s, const VehicleCommand& command) {
    size_t index = 0;
    for (auto& stage : stages_) {
        for (auto& fin : stage.grid_fins) {
            const double target = index < command.grid_fin_rad.size() ? command.grid_fin_rad[index] : 0.0;
            fin.deflection_rad = approach(fin.deflection_rad,
                                          std::clamp(target, -fin.max_deflection_rad, fin.max_deflection_rad),
                                          fin.rate_rad_s * dt_s);
            ++index;
        }
    }
}

void Vehicle::update_flaps(double dt_s, const VehicleCommand& command) {
    size_t index = 0;
    for (auto& stage : stages_) {
        for (auto& flap : stage.flaps) {
            const double target = index < command.flap_rad.size() ? command.flap_rad[index] : flap.deflection_rad;
            flap.deflection_rad = approach(flap.deflection_rad, std::clamp(target, 0.0, flap.max_deflection_rad),
                                           flap.rate_rad_s * dt_s);
            ++index;
        }
    }
}

size_t Vehicle::flap_count() const {
    size_t n = 0;
    for (const auto& stage : stages_) n += stage.flaps.size();
    return n;
}

size_t Vehicle::grid_fin_count() const {
    size_t n = 0;
    for (const auto& stage : stages_) n += stage.grid_fins.size();
    return n;
}

std::vector<std::vector<double>> Vehicle::tank_mass_flows_kgps() const {
    std::vector<std::vector<double>> flows;
    for (const auto& stage : stages_) {
        std::vector<double> stage_flows(stage.tanks.size(), 0.0);
        const auto engine_role = stage.feed == FeedSource::Header ? config::TankRole::Header : config::TankRole::Main;
        double oxidizer = 0.0, fuel = 0.0;
        for (const auto& e : stage.engines) {
            if (!e.producing_thrust()) continue;
            const double mdot = thrust_factor_ * e.spool * e.performance->mass_flow_kgps(e.throttle);
            const double mr = e.performance->config().mixture_ratio;
            oxidizer += mdot * mr / (1.0 + mr);
            fuel += mdot / (1.0 + mr);
        }
        // Reaction control: firing thrusters draw their own tank, or ullage gas from the main tanks in proportion to
        // the oxidizer and fuel left - from the header tanks only once the main tanks are dry [E] (the gas is
        // pressurant: the attitude control must not die with the last main-tank residue).
        const bool main_left = stage.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
                                   stage.propellant_kg(config::TankContents::Fuel, config::TankRole::Main) >
                               0.0;
        const auto gas_role = main_left ? config::TankRole::Main : config::TankRole::Header;
        const double ox_kg = stage.propellant_kg(config::TankContents::Oxidizer, gas_role);
        const double fuel_kg = stage.propellant_kg(config::TankContents::Fuel, gas_role);
        double gas_oxidizer = 0.0, gas_fuel = 0.0;
        for (const auto& t : stage.rcs) {
            if (t.output <= 0.0) continue;
            const double mdot = t.output * t.thrust_n / (t.isp_s * constants::standard_gravity_mps2);
            if (t.tank >= 0) {
                stage_flows[static_cast<size_t>(t.tank)] += mdot;
            } else if (ox_kg + fuel_kg > 0.0) {
                gas_oxidizer += mdot * ox_kg / (ox_kg + fuel_kg);
                gas_fuel += mdot * fuel_kg / (ox_kg + fuel_kg);
            }
        }
        // Draw each propellant from the first tank of that kind and role that still holds some.
        const auto draw = [&](config::TankContents contents, config::TankRole role, double flow) {
            for (size_t i = 0; i < stage.tanks.size() && flow > 0.0; ++i) {
                const auto& tank = stage.tanks[i];
                if (tank.config.contents == contents && tank.config.role == role && tank.mass_kg > 0.0) {
                    stage_flows[i] += flow;
                    flow = 0.0;
                }
            }
        };
        draw(config::TankContents::Oxidizer, engine_role, oxidizer);
        draw(config::TankContents::Fuel, engine_role, fuel);
        draw(config::TankContents::Oxidizer, gas_role, gas_oxidizer);
        draw(config::TankContents::Fuel, gas_role, gas_fuel);
        flows.push_back(std::move(stage_flows));
    }
    return flows;
}

void Vehicle::consume_propellant(double dt_s) {
    const auto flows = tank_mass_flows_kgps();
    for (size_t s = 0; s < stages_.size(); ++s) {
        auto& stage = stages_[s];
        for (size_t i = 0; i < stage.tanks.size(); ++i) {
            stage.tanks[i].mass_kg = std::max(0.0, stage.tanks[i].mass_kg - flows[s][i] * dt_s);
        }
        // The selected feed source ran dry (no silent switch to the other one).
        const auto role = stage.feed == FeedSource::Header ? config::TankRole::Header : config::TankRole::Main;
        const bool dry = stage.propellant_kg(config::TankContents::Oxidizer, role) <= 0.0 ||
                         stage.propellant_kg(config::TankContents::Fuel, role) <= 0.0;
        if (dry) {
            for (auto& e : stage.engines) {
                if (e.mode != EngineMode::Off) {
                    e.mode = EngineMode::Off;  // flame-out
                    e.spool = 0.0;
                }
                e.starved = true;
            }
        }
    }
    refresh_rcs_availability();
}

bool Vehicle::payload_stacked() const {
    return !stages_.empty() && !stages_.front().config.payload && stages_.back().config.payload;
}

void Vehicle::refresh_rcs_availability() {
    bool changed = false;
    // A payload's stages (config::stack_payload) wait until the payload flies alone: its lowest stage is the vehicle's.
    const bool stacked = payload_stacked();
    for (auto& stage : stages_) {
        const bool main_dry = stage.propellant_kg(config::TankContents::Oxidizer) <= 0.0 ||
                              stage.propellant_kg(config::TankContents::Fuel) <= 0.0;
        for (auto& t : stage.rcs) {
            const bool starved = t.tank >= 0 ? stage.tanks[static_cast<size_t>(t.tank)].mass_kg <= 0.0 : main_dry;
            const bool blocked = (stacked && stage.config.payload) || (t.behind_nose_cone && !nose_cone_open_);
            if (starved != t.starved || blocked != t.blocked) {
                t.starved = starved;
                t.blocked = blocked;
                changed = true;
            }
        }
    }
    if (changed) rcs_allocator_.reset();
}

bool Vehicle::has_nose_cone() const {
    return std::any_of(stages_.begin(), stages_.end(), [](const Stage& s) { return s.config.nose_cone; });
}

void Vehicle::set_nose_cone_open(bool open) {
    if (!has_nose_cone()) return;
    nose_cone_open_ = open;
    refresh_rcs_availability();
}

bool Vehicle::rcs_allocator_current() const {
    return rcs_allocator_ && norm(rcs_allocator_->cg_m() - mass_properties().cg_point_m()) <= rcs_cg_tolerance_m;
}

std::vector<propulsion::RcsThrusterGeometry> Vehicle::rcs_geometry() const {
    std::vector<propulsion::RcsThrusterGeometry> geometry;
    for (const auto& s : stages_) {
        for (const auto& t : s.rcs) {
            geometry.push_back({t.position_m, t.direction, t.starved || t.blocked ? 0.0 : t.thrust_n});
        }
    }
    return geometry;
}

const propulsion::RcsAllocator& Vehicle::rcs_allocator() const {
    if (!rcs_allocator_current()) rcs_allocator_.emplace(rcs_geometry(), mass_properties().cg_point_m());
    return *rcs_allocator_;
}

void Vehicle::stop_rcs() {
    rcs_modulator_ = {};
    rcs_duty_.clear();
    rcs_torque_ = {};
    rcs_force_ = {};
    for (auto& s : stages_) {
        for (auto& t : s.rcs) t.output = 0.0;
    }
}

void Vehicle::update_rcs(double dt_s, const VehicleCommand& command) {
    refresh_rcs_availability();
    const auto& a = rcs_allocator();
    rcs_torque_ = {};
    rcs_force_ = {};
    if (a.size() == 0) return;
    const Vec3 capability = a.max_torque_nm();
    const Vec3& f = command.rcs_torque_fraction;
    const Vec3 torque{std::clamp(f.x, -1.0, 1.0) * capability.x, std::clamp(f.y, -1.0, 1.0) * capability.y,
                      std::clamp(f.z, -1.0, 1.0) * capability.z};
    Vec3 force;
    const double magnitude = norm(command.rcs_force_fraction);
    if (magnitude > 0.0) {
        const Vec3 u = command.rcs_force_fraction / magnitude;
        force = u * (std::min(magnitude, 1.0) *
                     (command.rcs_force_pure ? a.pure_force_capability_n(u) : a.force_capability_n(u)));
    }
    rcs_duty_ = a.allocate(torque, force, &rcs_duty_, command.rcs_keep_force);
    if (rcs_modulator_.firing().size() != rcs_duty_.size()) {
        double minimum_on_time_s = 0.0;
        for (const auto& s : stages_) {
            if (s.rcs.empty()) continue;
            const double t = s.config.rcs.minimum_on_time_s;
            minimum_on_time_s = minimum_on_time_s > 0.0 ? std::min(minimum_on_time_s, t) : t;
        }
        rcs_modulator_ = propulsion::RcsModulator(rcs_duty_.size(), minimum_on_time_s);
    }
    const auto& firing = rcs_modulator_.update(rcs_duty_, dt_s);
    std::vector<double> output(firing.size());
    size_t i = 0;
    for (auto& s : stages_) {
        for (auto& t : s.rcs) {
            t.output = firing[i] && !t.starved && !t.blocked ? 1.0 : 0.0;  // fixed thrust, on/off
            output[i++] = t.output;
        }
    }
    rcs_torque_ = a.torque_nm(output);
    rcs_force_ = a.force_n(output);
}

Vehicle::RcsCapability Vehicle::rcs_force_capability(const Vec3& direction_body) const {
    return rcs_capability(direction_body, false);
}

Vehicle::RcsCapability Vehicle::rcs_pure_force_capability(const Vec3& direction_body) const {
    return rcs_capability(direction_body, true);
}

Vehicle::RcsCapability Vehicle::rcs_capability(const Vec3& direction_body, bool pure) const {
    // A query (the f3 planners) leaves the flight's allocator where it is: rebuilt here at another CG it would move the
    // attitude control's reference within rcs_cg_tolerance_m and change the flight. A stale one is rebuilt locally.
    std::optional<propulsion::RcsAllocator> local;
    if (!rcs_allocator_current()) local.emplace(rcs_geometry(), mass_properties().cg_point_m());
    const auto& a = local ? *local : *rcs_allocator_;
    std::vector<double> duty;
    RcsCapability c;
    c.force_n = pure ? a.pure_force_capability_n(direction_body, &duty) : a.force_capability_n(direction_body, &duty);
    size_t i = 0;
    for (const auto& s : stages_) {
        for (const auto& t : s.rcs) {
            c.mass_flow_kgps += duty[i++] * t.thrust_n / (t.isp_s * constants::standard_gravity_mps2);
        }
    }
    return c;
}

bool Vehicle::any_engine_producing_thrust() const {
    for (const auto& s : stages_) {
        for (const auto& e : s.engines) {
            if (e.producing_thrust()) return true;
        }
    }
    return false;
}

void Vehicle::fail_engine(size_t stage_index, size_t engine_index) {
    auto& e = stages_.at(stage_index).engines.at(engine_index);
    e.mode = EngineMode::Off;
    e.spool = 0.0;
    e.starved = true;  // never restarts
    e.failed = true;
}

Vec3 Vehicle::max_rcs_torque_nm() const { return rcs_allocator().max_torque_nm(); }

bool Vehicle::deploy_parachutes(const std::string& role) {
    bool any = false;
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.config.role != role || chute.state != ParachuteState::Stowed) continue;
            chute.state = ParachuteState::Open;
            chute.deployed_s = 0.0;
            any = true;
        }
    }
    return any;
}

void Vehicle::release_parachutes(const std::string& role) {
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.config.role != role || chute.state != ParachuteState::Open) continue;
            chute.state = ParachuteState::Released;
            chute.drag_area_m2 = 0.0;
        }
    }
}

void Vehicle::update_parachutes(double dt_s) {
    for (auto& stage : stages_) {
        for (auto& chute : stage.parachutes) {
            if (chute.state != ParachuteState::Open) continue;
            chute.deployed_s += dt_s;
            chute.drag_area_m2 = chute.config.count * aero::parachute_drag_area_m2(chute.config, chute.deployed_s);
        }
    }
}

bool Vehicle::parachutes_out() const {
    for (const auto& stage : stages_) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state == ParachuteState::Open) return true;
        }
    }
    return false;
}

const char* to_string(ParachuteState state) {
    switch (state) {
        case ParachuteState::Stowed: return "verstaut";
        case ParachuteState::Open: return "offen";
        case ParachuteState::Released: return "abgeworfen";
        case ParachuteState::Torn: return "Fallschirm gerissen";
    }
    return "?";
}

bool Vehicle::parachute_torn() const {
    for (const auto& stage : stages_) {
        for (const auto& chute : stage.parachutes) {
            if (chute.state == ParachuteState::Torn) return true;
        }
    }
    return false;
}

void Vehicle::record_parachute_load(size_t stage, size_t set, double load_per_canopy_n) {
    auto& chute = stages_.at(stage).parachutes.at(set);
    if (chute.state != ParachuteState::Open) return;
    chute.peak_load_n = std::max(chute.peak_load_n, load_per_canopy_n);
    if (load_per_canopy_n > chute.config.max_load_n) {  // all canopies of a set carry the same load: they tear together
        chute.state = ParachuteState::Torn;
        chute.drag_area_m2 = 0.0;
    }
}

bool Vehicle::splashdown_recovery() const {
    return !stages_.empty() && stages_.front().config.recovery_method == "splashdown";
}

double Vehicle::max_splashdown_speed_mps() const {
    return stages_.empty() ? 0.0 : stages_.front().config.max_splashdown_speed_mps;
}

std::unique_ptr<Vehicle> Vehicle::copy_upper(size_t index) const {
    auto copy = std::unique_ptr<Vehicle>(new Vehicle());
    copy->id_ = id_;
    copy->config_dir_ = config_dir_;
    copy->stages_ = stages_;
    copy->nose_cone_open_ = nose_cone_open_;
    copy->hull_aero_scale_ = hull_aero_scale_;
    copy->thrust_factor_ = thrust_factor_;
    copy->liquid_down_body_ = liquid_down_body_;
    copy->aero_ = std::make_unique<aero::SlenderBodyAero>(*aero_);
    copy->rebuild_aero();
    if (index == 0) return copy;
    return copy->separate(index);
}

std::unique_ptr<Vehicle> Vehicle::separate(size_t index) {
    if (index == 0 || index >= stages_.size()) {
        throw std::out_of_range("Vehicle::separate: index must split the stack into two non-empty parts");
    }
    auto upper = std::unique_ptr<Vehicle>(new Vehicle());
    upper->id_ = id_ + "/" + stages_[index].config.id;
    upper->config_dir_ = config_dir_;
    const double shift = stages_[index].base_station_m;
    for (size_t i = index; i < stages_.size(); ++i) {
        Stage stage = std::move(stages_[i]);
        stage.base_station_m -= shift;
        for (auto& e : stage.engines) e.gimbal_point_m.x -= shift;
        for (auto& fin : stage.grid_fins) fin.geometry.station_m -= shift;
        for (auto& t : stage.rcs) t.position_m.x -= shift;
        for (auto& chute : stage.parachutes) chute.attach_station_m -= shift;
        for (auto& flap : stage.flaps) {
            flap.geometry.station_fwd_m -= shift;
            flap.geometry.station_aft_m -= shift;
        }
        upper->stages_.push_back(std::move(stage));
    }
    stages_.erase(stages_.begin() + static_cast<long>(index), stages_.end());
    upper->aero_ = std::make_unique<aero::SlenderBodyAero>(aero_->geometry(), aero_->drag_curve());
    upper->rebuild_aero();
    id_ += "/" + stages_.back().config.id;
    rebuild_aero();
    stop_rcs();
    upper->stop_rcs();
    rcs_allocator_.reset();  // each part keeps only its own thrusters
    upper->rcs_allocator_.reset();
    upper->hull_aero_scale_ = hull_aero_scale_;  // review d3 M5: the truth aero dispersion stays on both parts
    upper->thrust_factor_ = thrust_factor_;  // likewise the truth thrust dispersion
    upper->liquid_down_body_ = liquid_down_body_;  // one hull until now: the liquid lies alike
    upper->nose_cone_open_ = nose_cone_open_ && upper->has_nose_cone();
    nose_cone_open_ = nose_cone_open_ && has_nose_cone();
    refresh_rcs_availability();  // a payload split off flies alone: its thrusters are free now
    upper->refresh_rcs_availability();
    // Fixed legs are always out, also on a part split off (the upper part starts with them stowed otherwise).
    for (Vehicle* part : {this, upper.get()}) {
        if (part->has_landing_legs() && part->stages_.front().config.landing_legs.fixed) part->set_leg_deployment(1.0);
    }
    return upper;
}

}  // namespace starsim::vehicle
