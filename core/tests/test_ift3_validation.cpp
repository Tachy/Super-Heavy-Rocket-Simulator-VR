// Ascent validation against Starship IFT-3 (2024-03-14, Super Heavy B10 / Ship 28, Block 1).
//
// The simulated stack follows the *measured* flight-path angle (steering by telemetry), so speed and
// altitude test what the vehicle model predicts by itself: thrust, mass flow, drag, gravity.
// SpaceX's throttle schedule is not public, so thrust cannot be validated independently from webcast
// data. Instead the throttle is RECONSTRUCTED every tick from the measured sensed acceleration
// (T = m a_sensed + D). What is then tested is the physical consistency of the vehicle model:
//   - speed and altitude follow the measurement (mass, drag, gravity, kinematics are consistent)
//   - the propellant left at MECO (T+162 s) is a plausible landing reserve (5..20 % of the load)
// Finding (2026-10-12): mean reconstructed throttle 0.75 (~91 % at liftoff, ~75 % after max-Q);
// speed within 4.3 %, altitude within 7.9 % (T+60..145 s); 541 t (15.9 %) left at MECO.
// Reference: webcast telemetry scraped by meithan (GPL-3.0, fetched locally, not redistributed):
//   python tools/data_pipeline/fetch.py ift3_booster_telemetry
// Writes <build>/ift3_sim.csv; compare with tools/plot_telemetry.py.

#include <cmath>
#include <cstdio>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/environment/gravity.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/attitude_controller.hpp"
#include "starsim/sim/telemetry_recorder.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "test_support.hpp"

using namespace starsim;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double launch_azimuth_rad = 97.0 * deg;  // [E] Starbase towards the Florida Straits
constexpr double dt = 0.005;
// The smoothed reference is unreliable in the first minute (speed rounded to 1 km/h, altitude to
// 0.1 km, smoothing starts at a constant 3.8 m/s^2), so deviations are evaluated from T+60 s.
constexpr double compare_from_s = 60.0;
constexpr double compare_until_s = 145.0;            // MECO sequence starts after ~T+145 s
constexpr double meco_s = 162.0;

struct Reference {
    std::vector<double> t, speed, altitude, horizontal, vertical, vertical_acceleration, horizontal_acceleration;

    double at(const std::vector<double>& column, double time) const {
        const size_t i = std::min(static_cast<size_t>(std::max(0.0, time)), t.size() - 2);
        const double f = time - t[i];
        return column[i] + f * (column[i + 1] - column[i]);
    }
};

}  // namespace

TEST_CASE("IFT-3 booster ascent matches webcast telemetry", "[vehicle][validation][ift3]") {
    const auto reference_path = std::filesystem::path(STARSIM_DATA_DIR) / "validation/IFT3_full_data_booster.csv";
    if (!std::filesystem::exists(reference_path)) {
        SKIP("reference missing: python tools/data_pipeline/fetch.py ift3_booster_telemetry");
    }
    const auto table = config::load_csv_table(reference_path);
    const Reference ref{table.column("Time [s]"), table.column("Smoothed speed [m/s]"),
                        table.column("Smoothed altitude [km]"), table.column("Horizontal speed [m/s]"),
                        table.column("Vertical speed [m/s]"),
                        table.column("Vertical acceleration [m/s^2]"),
                        table.column("Horizontal acceleration [m/s^2]")};

    test::spice_session();
    const spice::SpiceEphemeris ephemeris;
    const auto earth = spice::SpiceOrientation::earth();
    const environment::PointMassGravity gravity(ephemeris, earth);
    const environment::Us76Atmosphere atmosphere;
    const vehicle::FlightEnvironment env{gravity, atmosphere, earth, &ephemeris, true};
    const spice::SpiceTimeConverter time;

    const auto liftoff = time.from_utc({2024, 3, 14, 13, 25, 0.0});
    const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
    auto body = vehicle::FlightBody::on_pad(
        std::make_unique<vehicle::Vehicle>(config::load_vehicle(config_dir, "starship_block1"), config_dir),
        25.99686 * deg, -97.15394 * deg, 10.0, launch_azimuth_rad, earth, liftoff - 3.0);

    const auto& engines = body->vehicle().stages().front().engines;
    const auto& raptor = engines.front().performance->config();
    const double vacuum_thrust = raptor.thrust_ref_n + raptor.reference_pressure_pa * raptor.nozzle_exit_area_m2;
    const double reference_area = constants::pi * 0.25 * 9.0 * 9.0;
    const double loaded_propellant = body->vehicle().stages().front().propellant_kg(config::TankContents::Oxidizer) +
                                     body->vehicle().stages().front().propellant_kg(config::TankContents::Fuel);

    // Throttle that reproduces the measured sensed acceleration (vertical-plane approximation).
    auto reconstruct_throttle = [&](double t, double ambient_pressure, const vehicle::FlightTelemetry& now) {
        const double g = constants::standard_gravity_mps2;
        const double a_vertical = ref.at(ref.vertical_acceleration, std::max(t, 0.0)) + g;
        const double a_horizontal = ref.at(ref.horizontal_acceleration, std::max(t, 0.0));
        const double sensed = std::hypot(a_vertical, a_horizontal);
        const double drag = now.dynamic_pressure_pa * reference_area *
                            body->vehicle().aero().drag_curve().cd(now.mach, true);
        const double thrust = now.mass_kg * sensed + drag;
        return (thrust / engines.size() + ambient_pressure * raptor.nozzle_exit_area_m2) / vacuum_thrust;
    };
    double throttle_sum = 0.0;
    int throttle_samples = 0;

    vehicle::VehicleCommand command;
    command.groups = {{"booster/center", true, 1.0}, {"booster/middle", true, 1.0}, {"booster/outer", true, 1.0}};
    const gnc::AttitudeController controller;
    sim::TelemetryRecorder recorder(std::filesystem::path(STARSIM_OUTPUT_DIR) / "ift3_sim.csv");

    double worst_speed = 0.0, worst_altitude = 0.0;
    std::printf("[validation] IFT-3      t   speed sim/ref [m/s]    altitude sim/ref [km]\n");
    const int ticks = static_cast<int>((meco_s + 3.0) / dt);
    for (int i = 0; i <= ticks; ++i) {
        const double t = -3.0 + i * dt;
        const auto epoch = liftoff + t;
        if (body->clamped() && t >= 0.0) {
            body->release_clamps();
        }

        // Steering: follow the measured flight-path angle in the launch azimuth plane, with feedback.
        const auto transform = earth.fixed_to_gcrf(epoch);
        const auto geo = frames::fixed_to_geodetic(transpose(transform.rotation) * body->state().position_gcrf_m);
        const Vec3 up = transform.rotation * Vec3{std::cos(geo.latitude_rad) * std::cos(geo.longitude_rad),
                                                  std::cos(geo.latitude_rad) * std::sin(geo.longitude_rad),
                                                  std::sin(geo.latitude_rad)};
        const Vec3 east = transform.rotation * Vec3{-std::sin(geo.longitude_rad), std::cos(geo.longitude_rad), 0.0};
        const Vec3 north = cross(up, east);
        const Vec3 downrange = std::cos(launch_azimuth_rad) * north + std::sin(launch_azimuth_rad) * east;
        const auto telemetry = body->telemetry(env, epoch);
        double gamma = 90.0 * deg;
        if (t > 10.0) {
            const double gamma_ref = std::atan2(ref.at(ref.vertical, t), ref.at(ref.horizontal, t));
            gamma = std::clamp(gamma_ref + 2.0 * (gamma_ref - telemetry.flight_path_angle_rad), 0.0, 90.0 * deg);
        }
        const Vec3 axis = std::cos(gamma) * downrange + std::sin(gamma) * up;
        const double ambient = atmosphere.at(geo.height_m).pressure_pa;
        const double throttle = std::clamp(reconstruct_throttle(t, ambient, telemetry), 0.4, 1.0);
        for (auto& group : command.groups) group.throttle = throttle;
        if (t >= 0.0) {
            throttle_sum += throttle;
            ++throttle_samples;
        }
        const auto steering = controller.steer(*body, ambient, axis, downrange);
        command.gimbal_pitch_rad = steering.pitch_rad;
        command.gimbal_yaw_rad = steering.yaw_rad;
        command.gimbal_roll_rad = steering.roll_rad;

        if (t >= 0.0 && std::abs(t - std::round(t)) < 0.5 * dt) {
            recorder.record(t, telemetry);
            if (static_cast<int>(std::round(t)) % 15 == 0) {
                std::printf("[validation] IFT-3  %5.0f   %7.1f / %7.1f      %6.2f / %6.2f\n", t,
                            telemetry.speed_relative_mps, ref.at(ref.speed, t), telemetry.altitude_m / 1000.0,
                            ref.at(ref.altitude, t));
            }
            if (t >= compare_from_s && t <= compare_until_s) {
                worst_speed = std::max(worst_speed, std::abs(telemetry.speed_relative_mps / ref.at(ref.speed, t) - 1.0));
                worst_altitude = std::max(worst_altitude,
                                          std::abs(telemetry.altitude_m / 1000.0 / ref.at(ref.altitude, t) - 1.0));
            }
        }
        body->step(env, epoch, dt, command);
        REQUIRE_FALSE(body->impacted());
    }
    const auto& booster = body->vehicle().stages().front();
    const double remaining = booster.propellant_kg(config::TankContents::Oxidizer) +
                             booster.propellant_kg(config::TankContents::Fuel);
    std::printf("[validation] IFT-3 (T+%.0f..%.0f s): max speed deviation %.1f %%, max altitude deviation %.1f %%\n",
                compare_from_s, compare_until_s, worst_speed * 100.0, worst_altitude * 100.0);
    std::printf("[validation] IFT-3 reconstructed mean throttle %.3f (%.0f tf vac per engine); propellant at MECO "
                "%.0f t = %.1f %% of the load\n",
                throttle_sum / throttle_samples, throttle_sum / throttle_samples * vacuum_thrust / 9806.65,
                remaining / 1000.0, 100.0 * remaining / loaded_propellant);
    CHECK(worst_speed < 0.05);
    CHECK(worst_altitude < 0.08);
    CHECK(remaining / loaded_propellant > 0.05);
    // Boostback + landing reserve: no published figure; sources quote 3,250 or 3,400 t loaded, which
    // moves the result between ~12 % and ~16 %. Accept 5..20 %.
    CHECK(remaining / loaded_propellant < 0.20);
}
