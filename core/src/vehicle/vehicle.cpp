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
            const double density = tank_config.contents == config::TankContents::Oxidizer
                                       ? config.propellant.oxidizer_density_kg_m3
                                       : config.propellant.fuel_density_kg_m3;
            stage.tanks.push_back({tank_config, density, area, tank_config.mass_kg});
        }
        for (const auto& group : stage_config.engine_groups) {
            for (int k = 0; k < group.count; ++k) {
                const double angle = constants::two_pi * k / group.count;
                Engine engine;
                engine.performance = performances.at(group.engine_id);
                engine.group_id = stage_config.id + "/" + group.id;
                engine.gimbal_point_m = {station, group.ring_radius_m * std::cos(angle),
                                         group.ring_radius_m * std::sin(angle)};
                engine.ring_angle_rad = angle;
                engine.gimbaled = group.gimbal;
                engine.relight = group.relight;
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
        station += stage_config.length_m;
        stages_.push_back(std::move(stage));
    }
    rebuild_aero();
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
    aero_ = std::make_unique<aero::SlenderBodyAero>(
        aero::SlenderBodyGeometry{length_m(), diameter_m(), stages_.back().config.aero.nose_length_m}, drag,
        tail_first);
}

MassProperties Vehicle::mass_properties(const std::vector<std::vector<double>>* flows, double tau_s) const {
    struct Part { double mass, station, ixx, iyy; };
    std::vector<Part> parts;
    for (size_t si = 0; si < stages_.size(); ++si) {
        const auto& s = stages_[si];
        const double r = 0.5 * s.config.diameter_m;
        const double l = s.config.length_m;
        // Dry structure as a thin-walled cylinder about its own CG [E].
        parts.push_back({s.config.dry_mass_kg, s.base_station_m + s.config.dry_cg_station_m,
                         s.config.dry_mass_kg * r * r, s.config.dry_mass_kg * (0.5 * r * r + l * l / 12.0)});
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
    }
    mp.cg_station_m /= mp.mass_kg;
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
        for (const auto& t : s.tanks) total += t.mass_kg;
    }
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
        // Draw each propellant from the first tank of that kind that still holds some.
        for (auto [contents, flow] : {std::pair{config::TankContents::Oxidizer, oxidizer},
                                      std::pair{config::TankContents::Fuel, fuel}}) {
            for (size_t i = 0; i < stage.tanks.size() && flow > 0.0; ++i) {
                if (stage.tanks[i].config.contents == contents && stage.tanks[i].mass_kg > 0.0) {
                    stage_flows[i] = flow;
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

Vec3 Vehicle::max_rcs_torque_nm() const {
    double force = 0.0;
    for (const auto& s : stages_) force += 0.5 * s.config.rcs_thruster_count * s.config.rcs_thrust_n;
    const double lever = 0.5 * length_m();
    return {force * 0.5 * diameter_m(), force * lever, force * lever};
}

double Vehicle::max_rcs_force_n() const {
    double force = 0.0;
    for (const auto& s : stages_) force += 0.5 * s.config.rcs_thruster_count * s.config.rcs_thrust_n;
    return force;
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
        upper->stages_.push_back(std::move(stage));
    }
    stages_.erase(stages_.begin() + static_cast<long>(index), stages_.end());
    upper->aero_ = std::make_unique<aero::SlenderBodyAero>(aero_->geometry(), aero_->drag_curve());
    upper->rebuild_aero();
    id_ += "/" + stages_.back().config.id;
    rebuild_aero();
    return upper;
}

}  // namespace starsim::vehicle
