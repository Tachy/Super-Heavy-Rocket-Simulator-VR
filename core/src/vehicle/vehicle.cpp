#include "starsim/vehicle/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <map>

#include "starsim/math/constants.hpp"

namespace starsim::vehicle {

using math::Vec3;

namespace {

// Payload and fairing sit in the upper part of their stage [E].
constexpr double extra_mass_station_fraction = 0.75;
// The RCS allocation is rebuilt when the CG has moved this far (lever change negligible below).
constexpr double rcs_cg_tolerance_m = 0.01;

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
    return normalized(Vec3{1.0, std::tan(gimbal_yaw_rad), std::tan(gimbal_pitch_rad)});
}

double Stage::propellant_kg(config::TankContents contents) const {
    double total = 0.0;
    for (const auto& t : tanks) {
        if (t.config.contents == contents) total += t.mass_kg;
    }
    return total;
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
        const double area = constants::pi * 0.25 * stage_config.diameter_m * stage_config.diameter_m;
        for (const auto& tank_config : stage_config.tanks) {
            const double density = tank_config.contents == config::TankContents::Oxidizer ? config.propellant.oxidizer_density_kg_m3
                                   : tank_config.contents == config::TankContents::Fuel   ? config.propellant.fuel_density_kg_m3
                                                                                          : tank_config.density_kg_m3;
            stage.tanks.push_back({tank_config, density, area, tank_config.mass_kg});
        }
        for (const auto& group : stage_config.engine_groups) {
            for (int k = 0; k < group.count; ++k) {
                const double angle = group.angles_rad.empty() ? constants::two_pi * k / group.count
                                                              : group.angles_rad[static_cast<size_t>(k)];
                Engine engine;
                engine.performance = performances.at(group.engine_id);
                engine.group_id = stage_config.id + "/" + group.id;
                engine.gimbal_point_m = {station, group.ring_radius_m * std::cos(angle),
                                         group.ring_radius_m * std::sin(angle)};
                engine.ring_angle_rad = angle;
                engine.gimbaled = group.gimbal;
                engine.relight = group.relight;
                engine.abort_only = group.abort_only;
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
}

aero::AeroResult Vehicle::hull_aero(const Vec3& air_velocity_body_mps, const environment::AtmosphereState& air,
                                    bool powered) const {
    if (capsule_aero_) return capsule_aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps);
    return aero_->compute(air_velocity_body_mps, air.density_kg_m3, air.speed_of_sound_mps, powered, air.mean_free_path_m);
}

MassProperties Vehicle::mass_properties(const std::vector<std::vector<double>>* flows, double tau_s) const {
    struct Part { double mass, station, ixx, iyy; double y = 0.0, z = 0.0; };
    std::vector<Part> parts;
    for (size_t si = 0; si < stages_.size(); ++si) {
        const auto& s = stages_[si];
        const double r = 0.5 * s.config.diameter_m;
        const double l = s.config.length_m;
        // Dry structure as a thin-walled cylinder about its own CG [E].
        parts.push_back({s.config.dry_mass_kg, s.base_station_m + s.config.dry_cg_station_m,
                         s.config.dry_mass_kg * r * r, s.config.dry_mass_kg * (0.5 * r * r + l * l / 12.0),
                         s.config.dry_cg_offset_m.y, s.config.dry_cg_offset_m.z});
        if (s.extra_mass_kg > 0.0) {
            parts.push_back({s.extra_mass_kg, s.base_station_m + extra_mass_station_fraction * l, 0.0, 0.0});
        }
        // Settled liquid column; the fluid does not spin with the tank (no roll inertia) [E].
        for (size_t ti = 0; ti < s.tanks.size(); ++ti) {
            const auto& t = s.tanks[ti];
            const double mass = std::max(0.0, t.mass_kg - (flows ? (*flows)[si][ti] * tau_s : 0.0));
            if (mass <= 0.0) continue;
            const double h = mass / (t.density_kg_m3 * t.area_m2);
            parts.push_back({mass, s.base_station_m + t.config.base_station_m + 0.5 * h, 0.0,
                             mass * (0.25 * r * r + h * h / 12.0)});
        }
    }
    MassProperties mp;
    for (const auto& p : parts) {
        mp.mass_kg += p.mass;
        mp.cg_station_m += p.mass * p.station;
        mp.cg_offset_m.y += p.mass * p.y;
        mp.cg_offset_m.z += p.mass * p.z;
    }
    mp.cg_station_m /= mp.mass_kg;
    // Lateral offset (capsule trim): enters the aerodynamic and RCS moments only; the inertia stays on the axis (its
    // parallel-axis term is < 0.1 %, d3 plan ruling 2).
    mp.cg_offset_m = mp.cg_offset_m / mp.mass_kg;
    double ixx = 0.0, iyy = 0.0;
    for (const auto& p : parts) {
        const double d = p.station - mp.cg_station_m;
        ixx += p.ixx;
        iyy += p.iyy + p.mass * d * d;
    }
    mp.inertia_kg_m2 = {ixx, iyy, iyy};
    return mp;
}

double Vehicle::propellant_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) {
        for (const auto& t : s.tanks) total += t.config.contents == config::TankContents::Rcs ? 0.0 : t.mass_kg;
    }
    return total;
}

double Vehicle::rcs_propellant_kg() const {
    double total = 0.0;
    for (const auto& s : stages_) total += s.propellant_kg(config::TankContents::Rcs);
    return total;
}

void Vehicle::update_engines(double dt_s, const VehicleCommand& command, double ambient_pressure_pa) {
    for (auto& stage : stages_) {
        for (auto& e : stage.engines) {
            const auto& cfg = e.performance->config();
            const GroupCommand* group = find_group(command, e.group_id);
            const bool want_run = group != nullptr && group->run;
            const double target_throttle =
                std::clamp(group ? group->throttle : 0.0, cfg.throttle_min, cfg.throttle_max);
            const bool can_start = (!e.has_run || e.relight) && !e.starved &&
                                   e.performance->can_operate(ambient_pressure_pa);

            if (want_run && e.mode == EngineMode::Off && can_start) {
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
    if (!legs_deploying_) return;
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
        double oxidizer = 0.0, fuel = 0.0;
        for (const auto& e : stage.engines) {
            if (!e.producing_thrust()) continue;
            const double mdot = e.spool * e.performance->mass_flow_kgps(e.throttle);
            const double mr = e.performance->config().mixture_ratio;
            oxidizer += mdot * mr / (1.0 + mr);
            fuel += mdot / (1.0 + mr);
        }
        // Reaction control: firing thrusters draw their own tank, or ullage gas from the main tanks in proportion to
        // the oxidizer and fuel left.
        const double ox_kg = stage.propellant_kg(config::TankContents::Oxidizer);
        const double fuel_kg = stage.propellant_kg(config::TankContents::Fuel);
        for (const auto& t : stage.rcs) {
            if (t.output <= 0.0) continue;
            const double mdot = t.output * t.thrust_n / (t.isp_s * constants::standard_gravity_mps2);
            if (t.tank >= 0) {
                stage_flows[static_cast<size_t>(t.tank)] += mdot;
            } else if (ox_kg + fuel_kg > 0.0) {
                oxidizer += mdot * ox_kg / (ox_kg + fuel_kg);
                fuel += mdot * fuel_kg / (ox_kg + fuel_kg);
            }
        }
        // Draw each propellant from the first tank of that kind that still holds some.
        for (auto [contents, flow] : {std::pair{config::TankContents::Oxidizer, oxidizer},
                                      std::pair{config::TankContents::Fuel, fuel}}) {
            for (size_t i = 0; i < stage.tanks.size() && flow > 0.0; ++i) {
                if (stage.tanks[i].config.contents == contents && stage.tanks[i].mass_kg > 0.0) {
                    stage_flows[i] += flow;
                    flow = 0.0;
                }
            }
        }
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
        const bool dry = stage.propellant_kg(config::TankContents::Oxidizer) <= 0.0 ||
                         stage.propellant_kg(config::TankContents::Fuel) <= 0.0;
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

void Vehicle::refresh_rcs_availability() {
    bool changed = false;
    // A payload's stages (config::stack_payload) wait until the payload flies alone: its lowest stage is the vehicle's.
    const bool stacked = !stages_.empty() && !stages_.front().config.payload;
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

const propulsion::RcsAllocator& Vehicle::rcs_allocator() const {
    const Vec3 cg = mass_properties().cg_point_m();
    if (!rcs_allocator_ || norm(rcs_allocator_->cg_m() - cg) > rcs_cg_tolerance_m) {
        std::vector<propulsion::RcsThrusterGeometry> geometry;
        for (const auto& s : stages_) {
            for (const auto& t : s.rcs) {
                geometry.push_back({t.position_m, t.direction, t.starved || t.blocked ? 0.0 : t.thrust_n});
            }
        }
        rcs_allocator_.emplace(std::move(geometry), cg);
    }
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
        force = u * (std::min(magnitude, 1.0) * a.force_capability_n(u));
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
    const auto& a = rcs_allocator();
    std::vector<double> duty;
    RcsCapability c;
    c.force_n = a.force_capability_n(direction_body, &duty);
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
}

Vec3 Vehicle::max_rcs_torque_nm() const { return rcs_allocator().max_torque_nm(); }

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
    upper->nose_cone_open_ = nose_cone_open_ && upper->has_nose_cone();
    nose_cone_open_ = nose_cone_open_ && has_nose_cone();
    refresh_rcs_availability();  // a payload split off flies alone: its thrusters are free now
    upper->refresh_rcs_availability();
    return upper;
}

}  // namespace starsim::vehicle
