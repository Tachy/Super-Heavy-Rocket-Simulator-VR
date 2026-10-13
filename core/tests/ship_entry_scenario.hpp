#pragma once

// Test scenario: the ship alone entering the atmosphere under central gravity over a rotating Earth, flown by
// AttitudeController (flaps + RCS) on the entry attitude setpoint (alpha, bank). Shared by the corridor and the
// telemetry replay tests.

#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <vector>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/gnc/entry_attitude.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::test {

class EntryGravity final : public environment::GravityModel {
public:
    ephemeris::Body central_body() const override { return ephemeris::Body::Earth; }
    math::Vec3 acceleration_mps2(const math::Vec3& r, time::Epoch) const override {
        const double d = norm(r);
        return r * (-3.986004418e14 / (d * d * d));  // GM Earth (IERS 2010)
    }
};

inline const time::Epoch entry_t0 = time::Epoch::from_tdb_seconds_past_j2000(8.4e8);
inline constexpr double earth_rate_radps = 7.2921150e-5;

// Earth rotating about GCRF z with the sidereal rate; ITRF = GCRF at entry_t0.
class EntryEarth final : public frames::BodyOrientation {
public:
    frames::FixedToInertial fixed_to_gcrf(time::Epoch epoch) const override {
        const double a = earth_rate_radps * (epoch - entry_t0);
        math::Mat3 r = math::Mat3::identity(), dr;
        r(0, 0) = std::cos(a); r(0, 1) = -std::sin(a); r(1, 0) = std::sin(a); r(1, 1) = std::cos(a);
        dr(0, 0) = -earth_rate_radps * std::sin(a); dr(0, 1) = -earth_rate_radps * std::cos(a);
        dr(1, 0) = earth_rate_radps * std::cos(a); dr(1, 1) = -earth_rate_radps * std::sin(a);
        return {r, dr};
    }
};

struct EntryStart {
    double height_m = 120000.0;
    double speed_relative_mps = 7400.0;  // relative to the rotating Earth
    double flight_path_angle_rad = -1.0 * constants::deg_to_rad;
    double heading_rad = 90.0 * constants::deg_to_rad;  // clockwise from north
    double latitude_rad = 10.0 * constants::deg_to_rad;
    double mass_propellant_kg = -1.0;  // < 0: the stage's landing_propellant_kg
    double bank_rad = 0.0;             // initial attitude (alpha 60 deg)
};

struct EntrySample {
    double t_s = 0.0;
    vehicle::FlightTelemetry telemetry;
    double drag_mps2 = 0.0;  // along the relative velocity
    double lift_vertical_mps2 = 0.0;
    // Accelerometer (aerodynamic acceleration) split along / across the air velocity.
    double aero_drag_mps2 = 0.0;
    double aero_lift_mps2 = 0.0;
};

// Commanded angle of attack and bank from the current telemetry.
using EntryLaw = std::function<void(const vehicle::FlightTelemetry&, double& alpha_rad, double& bank_rad)>;

class ShipEntryScenario {
public:
    explicit ShipEntryScenario(const EntryStart& start, std::function<void(config::VehicleConfig&)> tweak = {}) {
        auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, "starship_block3");
        cfg.stages.erase(cfg.stages.begin());
        auto& ship = cfg.stages[0];
        const double propellant = start.mass_propellant_kg >= 0.0 ? start.mass_propellant_kg : ship.landing_propellant_kg;
        double full = 0.0;
        for (const auto& tank : ship.tanks) full += tank.mass_kg;
        for (auto& tank : ship.tanks) tank.mass_kg *= propellant / full;
        for (auto& f : ship.flaps) f.initial_deflection_rad = 0.5 * f.max_deflection_rad;
        if (tweak) tweak(cfg);
        belly_azimuth_rad_ = ship.aero.belly_azimuth_rad;
        initial_bank_rad_ = start.bank_rad;

        const frames::GeodeticPosition g{start.latitude_rad, 0.0, start.height_m};
        const math::Vec3 r = frames::geodetic_to_fixed(g);
        const double sl = std::sin(g.latitude_rad), cl = std::cos(g.latitude_rad);
        const math::Vec3 east{0.0, 1.0, 0.0}, north{-sl, 0.0, cl}, up{cl, 0.0, sl};
        const double cg = std::cos(start.flight_path_angle_rad), sg = std::sin(start.flight_path_angle_rad);
        const math::Vec3 horizontal = north * std::cos(start.heading_rad) + east * std::sin(start.heading_rad);
        const math::Vec3 v_rel = (horizontal * cg + up * sg) * start.speed_relative_mps;
        const math::Vec3 v = v_rel + cross(math::Vec3{0.0, 0.0, earth_rate_radps}, r);
        const vehicle::RigidBodyState state{
            r, v, gnc::entry_attitude_quaternion(v_rel, up, 60.0 * constants::deg_to_rad, start.bank_rad, belly_azimuth_rad_),
            {}};
        body_ = std::make_unique<vehicle::FlightBody>(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR), state);
    }

    // Flies until the Mach number (in continuum flow, below 40 km) drops below `until_mach`, the altitude below
    // `stop_altitude_m`, the body breaks up or lands, the ship skips out and climbs above 130 km, or `max_time_s`
    // passes. One sample per second.
    std::vector<EntrySample> run(const EntryLaw& law, double until_mach, double max_time_s = 3000.0,
                                 double stop_altitude_m = -1.0) {
        std::vector<EntrySample> samples;
        gnc::AttitudeController::Gains gains;
        gains.flaps = true;
        gains.natural_frequency_rad_s = 0.5;  // the flaps move at ~20 deg/s: 0.8 rad/s limit-cycles in roll
        const gnc::AttitudeController controller(gains);
        const double dt = 0.01;
        double alpha = 60.0 * constants::deg_to_rad, bank = initial_bank_rad_;
        math::Vec3 previous_v_rel;
        for (int k = 0; k * dt < max_time_s; ++k) {
            const time::Epoch now = entry_t0 + k * dt;
            const auto transform = env_.earth_orientation.fixed_to_gcrf(now);
            const auto& s = body_->state();
            const math::Vec3 v_rel = s.velocity_gcrf_mps - transform.rotation_rate_per_s * (transpose(transform.rotation) * s.position_gcrf_m);
            if (k % 100 == 0) {
                const auto t = body_->telemetry(env_, now);
                law(t, alpha, bank);
                EntrySample sample{k * dt, t, 0.0, 0.0};
                if (!samples.empty()) {
                    const math::Vec3 a = (v_rel - previous_v_rel) / 1.0;
                    const math::Vec3 u = math::normalized(v_rel);
                    const math::Vec3 up = math::normalized(s.position_gcrf_m);
                    const math::Vec3 g = gravity_.acceleration_mps2(s.position_gcrf_m, now);
                    sample.drag_mps2 = -dot(a - g, u);
                    (void)up;
                }
                const math::Vec3 a_aero = s.attitude.rotate(body_->aero_acceleration_body_mps2());
                const math::Vec3 u_air = math::normalized(v_rel);
                sample.aero_drag_mps2 = -dot(a_aero, u_air);
                sample.aero_lift_mps2 = norm(a_aero + u_air * sample.aero_drag_mps2);
                previous_v_rel = v_rel;
                samples.push_back(sample);
                if (t.altitude_m < 40000.0 && t.mach < until_mach) break;
                if (t.altitude_m < stop_altitude_m) break;
                if (body_->skipped_out() && t.altitude_m > 130000.0) break;
            }
            if (body_->impacted() || body_->resting()) break;
            const math::Vec3 up = math::normalized(s.position_gcrf_m);
            const auto setpoint = gnc::entry_attitude(v_rel, up, alpha, bank, belly_azimuth_rad_);
            vehicle::VehicleCommand command;
            const double height = frames::fixed_to_geodetic(transpose(transform.rotation) * s.position_gcrf_m).height_m;
            const double pressure = atmosphere_.at(height).pressure_pa;
            controller.steer(*body_, pressure, setpoint.axis_gcrf, setpoint.roll_reference_gcrf).apply_to(command);
            body_->step(env_, now, dt, command);
        }
        return samples;
    }

    vehicle::FlightBody& body() { return *body_; }

private:
    EntryGravity gravity_;
    environment::Us76Atmosphere atmosphere_;
    EntryEarth earth_;
    vehicle::FlightEnvironment env_{gravity_, atmosphere_, earth_, nullptr, false};
    std::unique_ptr<vehicle::FlightBody> body_;
    double belly_azimuth_rad_ = 0.0;
    double initial_bank_rad_ = 0.0;
};

}  // namespace starsim::test
