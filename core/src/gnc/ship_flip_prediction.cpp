// The ship's flip flown forward from a predicted gate state, and the terminal entry guidance's gate re-aim built on it
// (plan 2026-11-03 planner-vehicle-parameters Task 3, fix round 3).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <set>

#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

Vec3 site_up(const LandingSite& site) {
    return {std::cos(site.latitude_rad) * std::cos(site.longitude_rad),
            std::cos(site.latitude_rad) * std::sin(site.longitude_rad), std::sin(site.latitude_rad)};
}

// Earth-fixed position and velocity of a body.
void fixed_state(const vehicle::FlightBody& body, const frames::FixedToInertial& tr, Vec3& r, Vec3& v) {
    const auto& s = body.state();
    r = transpose(tr.rotation) * s.position_gcrf_m;
    v = transpose(tr.rotation) * (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
}

}  // namespace

vehicle::RigidBodyState ship_gate_state(const vehicle::FlightBody& live, const frames::FixedToInertial& transform,
                                        const Vec3& position_itrf_m, const Vec3& velocity_itrf_mps,
                                        const Vec3& nose_itrf) {
    const Vec3 omega = vehicle::earth_rotation_vector(transform);
    const auto& s = live.state();
    // the live hull's flow side (body axes): the air velocity across the axis
    const Vec3 v_air_body = s.attitude.inverse_rotate(s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    const Vec3 across{0.0, v_air_body.y, v_air_body.z};
    const Vec3 belly = norm(across) > 1e-6 ? math::normalized(across) : Vec3{0.0, 0.0, 1.0};
    // at the gate: the axis along the nose, the flow side towards the gate's flow
    const Vec3 x = math::normalized(nose_itrf);
    const Vec3 flow = math::normalized(velocity_itrf_mps);
    Vec3 b = flow - x * dot(flow, x);
    b = norm(b) > 1e-9 ? math::normalized(b) : Vec3{0.0, 0.0, -1.0};
    const Vec3 x_body{1.0, 0.0, 0.0};
    const math::Mat3 body_basis = math::Mat3::from_columns(x_body, belly, cross(x_body, belly));
    const math::Mat3 fixed_basis = math::Mat3::from_columns(x, b, cross(x, b));
    vehicle::RigidBodyState gate;
    gate.position_gcrf_m = transform.rotation * position_itrf_m;
    gate.velocity_gcrf_mps = transform.rotation * velocity_itrf_mps + cross(omega, gate.position_gcrf_m);
    gate.attitude = math::Quat::from_matrix(transform.rotation * fixed_basis * transpose(body_basis));
    gate.angular_velocity_body_radps = gate.attitude.inverse_rotate(omega);  // held still relative to the Earth
    return gate;
}

ShipFlipPrediction predict_ship_flip(const vehicle::Vehicle& ship, const vehicle::RigidBodyState& gate_state,
                                     const vehicle::FlightEnvironment& env, time::Epoch epoch, const LandingSite& site,
                                     const ShipLandingParameters& parameters, const AttitudeController& controller,
                                     double tick_s, const Vec3& along_itrf, double max_s) {
    const auto wall_start = std::chrono::steady_clock::now();
    ShipFlipPrediction f;
    // the planner's model: nominal hull aerodynamics, thrust and air (no truth dispersion)
    auto copy = ship.copy_upper(0);
    copy->set_hull_aero_scale(1.0);
    copy->set_thrust_factor(1.0);
    copy->select_feed(0, vehicle::FeedSource::Header);  // as the session from the flip on
    vehicle::FlightBody body(std::move(copy), gate_state);
    body.set_ground_height(site.height_m);
    vehicle::FlightEnvironment nominal = env;
    nominal.air_density_factor = 1.0;
    ShipLandingGuidance guidance(site, parameters);
    guidance.set_diagnostics(false);

    std::set<std::string> groups;  // every engine group of the ship (the session commands each, running or not)
    for (const auto& stage : body.vehicle().stages()) {
        for (const auto& e : stage.engines) groups.insert(e.group_id);
    }
    const Vec3 up_fixed = site_up(site);
    Vec3 r0, v0;
    fixed_state(body, env.earth_orientation.fixed_to_gcrf(epoch), r0, v0);
    f.gate_speed_mps = dot(v0, along_itrf);
    time::Epoch t = epoch;
    for (double elapsed = 0.0; elapsed <= max_s; elapsed += tick_s) {
        const auto tr = env.earth_orientation.fixed_to_gcrf(t);
        const Vec3 up_gcrf = tr.rotation * up_fixed;
        Vec3 r, v;
        fixed_state(body, tr, r, v);
        const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        if (f.flip_end_s == 0.0 && dot(axis, up_gcrf) > std::cos(parameters.flip_tolerance_rad)) {
            f.flip_end_s = elapsed;
            f.flip_end_travel_m = dot(r - r0, along_itrf);
            f.flip_end_speed_mps = dot(v, along_itrf);
        }
        if (ship_flip_ready(parameters, body, up_gcrf)) {
            f.reached = true;
            f.duration_s = elapsed;
            f.travel_m = dot(r - r0, along_itrf);
            f.speed_mps = dot(v, along_itrf);
            break;
        }
        const auto cmd = guidance.update(body, nominal, t);
        if (cmd.phase != ReturnPhase::Flip || body.impacted()) break;
        // the session's command (FlightSession::steering_command_for / apply_return_command)
        vehicle::VehicleCommand command;
        for (const auto& g : groups) {
            const bool run = std::find(cmd.run_groups.begin(), cmd.run_groups.end(), g) != cmd.run_groups.end();
            command.groups.push_back({g, run, cmd.throttle});
        }
        const double pressure = nominal.atmosphere.at(frames::fixed_to_geodetic(r).height_m).pressure_pa;
        controller.steer(body, pressure, cmd.axis_gcrf, cmd.roll_reference_gcrf, cmd.feedforward ? &*cmd.feedforward : nullptr)
            .apply_to(command);
        command.rcs_force_fraction = body.state().attitude.inverse_rotate(cmd.rcs_force_gcrf);
        if (!cmd.flap_rad.empty()) command.flap_rad = cmd.flap_rad;
        body.step(nominal, t, tick_s, command);
        t = t + tick_s;
    }
    f.wall_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - wall_start).count();
    return f;
}

ShipGateAim::ShipGateAim(LandingSite site, ShipLandingParameters parameters, AttitudeController controller, double tick_s)
    : site_(std::move(site)), parameters_(std::move(parameters)), controller_(controller), tick_s_(tick_s) {}

EntryTarget ShipGateAim::operator()(const EntryGuidance::TerminalGate& gate) {
    const auto aim = ship_entry_target(site_, parameters_.gate_lead_m);
    if (!site_.catch_target) return aim;
    const Vec3& along = aim.nose_direction_itrf;  // horizontal
    const double along_mps = dot(gate.velocity_itrf_mps, along);
    const double mass_kg = gate.body.vehicle().mass_properties().mass_kg;
    if (!flip_epoch_ || gate.epoch - *flip_epoch_ >= flip_refresh_s) {
        flip_epoch_ = gate.epoch;
        const auto tr = gate.env.earth_orientation.fixed_to_gcrf(gate.epoch);
        const Vec3 up = site_up(site_);
        // the gate on the static aim's point at the hand-over height (the lead hardly moves the air it flies in)
        const Vec3 position = aim.position_itrf_m + up * gate.height_m;
        const auto state = ship_gate_state(gate.body, tr, position, gate.velocity_itrf_mps, along);
        flip_ = predict_ship_flip(gate.body.vehicle(), state, gate.env, gate.epoch, site_, parameters_, controller_,
                                  tick_s_, along);
        if (std::getenv("STARSIM_DEBUG_GATE")) {  // diagnostics
            std::printf("[gate-flip] gate along %.2f m/s, sink %.1f m/s, height %.0f m: flip end %.2f s %.1f m %.2f m/s, "
                        "plan start %.2f s %.1f m %.2f m/s%s (%.0f ms)\n",
                        flip_->gate_speed_mps, -dot(gate.velocity_itrf_mps, up), gate.height_m, flip_->flip_end_s,
                        flip_->flip_end_travel_m, flip_->flip_end_speed_mps, flip_->duration_s, flip_->travel_m,
                        flip_->speed_mps, flip_->reached ? "" : " NOT SETTLED", flip_->wall_ms);
        }
    }
    const double lead_m = flip_ && flip_->reached ? ship_gate_lead_m(parameters_, *flip_, along_mps, mass_kg)
                                                  : ship_gate_lead_m(parameters_, along_mps, mass_kg);
    if (std::getenv("STARSIM_DEBUG_GATE")) {  // diagnostics
        std::printf("[gate-aim] along %.2f m/s, mass %.1f t: lead %.1f m (static %.1f m, analytic %.1f m)\n", along_mps,
                    mass_kg / 1000.0, lead_m, parameters_.gate_lead_m, ship_gate_lead_m(parameters_, along_mps, mass_kg));
    }
    return ship_entry_target(site_, lead_m);
}

}  // namespace starsim::gnc
