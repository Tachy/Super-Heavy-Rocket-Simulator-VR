#include "starsim/gnc/deorbit_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "starsim/frames/geodetic.hpp"
#include "starsim/orbit/ballistic_propagator.hpp"
#include "starsim/vehicle/flight_body.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

integrators::Dop853Options<6> planning_options() {
    auto options = orbit::default_coast_options();
    options.relative_tolerance = 1e-10;  // ~cm over an orbit: far below what the entry guidance absorbs
    options.absolute_tolerance = integrators::State<6>{{0.01, 0.01, 0.01, 1e-5, 1e-5, 1e-5}};
    return options;
}

struct Sample {
    time::Epoch epoch;
    math::StateVector state;
};

// Earth-fixed position and velocity relative to the rotating Earth.
void to_fixed(const frames::BodyOrientation& earth, time::Epoch epoch, const math::StateVector& s, Vec3& r, Vec3& v) {
    const auto t = earth.fixed_to_gcrf(epoch);
    r = transpose(t.rotation) * s.position_m;
    v = transpose(t.rotation) * (s.velocity_mps - cross(vehicle::earth_rotation_vector(t), s.position_m));
}

// One shot: coast to the burn, retrograde impulse, coast to the entry interface, entry prediction at +-bank.
struct Shot {
    bool valid = false;
    time::Epoch entry_epoch;
    double flight_path_rad = 0.0;
    double speed_mps = 0.0;
    double downrange_m = 0.0;
    double crossrange_m = 0.0;
};

class Shooter {
public:
    Shooter(const std::vector<Sample>& samples, const environment::GravityModel& gravity,
            const frames::BodyOrientation& earth, const EntryPredictorModel& model, const EntryTarget& target,
            const DeorbitParameters& parameters)
        : samples_(samples), gravity_(gravity), earth_(earth), model_(model), target_(target), parameters_(parameters),
          options_(planning_options()) {}

    void set_mass(double mass_kg) { model_.mass_kg = mass_kg; }

    math::StateVector state_at(time::Epoch epoch) const {
        const double sample_s = samples_.size() > 1 ? samples_[1].epoch - samples_[0].epoch : 1.0;
        const auto i = static_cast<size_t>(
            std::clamp(std::floor((epoch - samples_.front().epoch) / sample_s), 0.0, static_cast<double>(samples_.size() - 1)));
        if (epoch - samples_[i].epoch == 0.0) return samples_[i].state;
        return orbit::propagate_coast(gravity_, samples_[i].state, samples_[i].epoch, epoch, {}, options_).state;
    }

    Shot shoot(time::Epoch burn, double delta_v_mps) const {
        Shot shot;
        auto s = state_at(burn);
        s.velocity_mps -= math::normalized(s.velocity_mps) * delta_v_mps;
        const integrators::Event<6> interface{
            "entry interface",
            [&](double t_s, const integrators::State<6>& y) {
                const Vec3 r = transpose(earth_.fixed_to_gcrf(burn + t_s).rotation) * Vec3{y[0], y[1], y[2]};
                return frames::fixed_to_geodetic(r).height_m - parameters_.entry_interface_height_m;
            },
            integrators::Crossing::Falling};
        const auto coast = orbit::propagate_coast(gravity_, s, burn, burn + 6000.0, std::span(&interface, 1), options_);
        if (coast.event_index != 0) return shot;
        Vec3 r, v;
        to_fixed(earth_, coast.epoch, coast.state, r, v);
        const auto g = frames::fixed_to_geodetic(r);
        const Vec3 up{std::cos(g.latitude_rad) * std::cos(g.longitude_rad), std::cos(g.latitude_rad) * std::sin(g.longitude_rad),
                      std::sin(g.latitude_rad)};
        shot.entry_epoch = coast.epoch;
        shot.speed_mps = norm(v);
        shot.flight_path_rad = std::asin(std::clamp(dot(v, up) / shot.speed_mps, -1.0, 1.0));
        shot.valid = true;
        for (const double sign : {1.0, -1.0}) {  // bank reversals: the guidance flies both signs
            EntryPredictorState start;
            start.position_itrf_m = r;
            start.velocity_itrf_mps = v;
            start.bank_rad = sign * parameters_.nominal_bank_rad;
            const auto p = predict_entry(model_, start, start.bank_rad, target_, parameters_.gate_height_m, 0.0, 2.0);
            shot.valid = shot.valid && p.reached_gate;
            shot.downrange_m += 0.5 * p.downrange_error_m;
            shot.crossrange_m += 0.5 * p.crossrange_m;
        }
        return shot;
    }

private:
    const std::vector<Sample>& samples_;
    const environment::GravityModel& gravity_;
    const frames::BodyOrientation& earth_;
    EntryPredictorModel model_;
    const EntryTarget& target_;
    const DeorbitParameters& parameters_;
    integrators::Dop853Options<6> options_;
};

// Two-body retrograde delta-v from a circular-ish orbit at radius r (speed v, flight path gamma) that crosses the
// entry interface radius at the entry flight-path angle (bisection).
double initial_delta_v(double r_m, double v_mps, double gamma_rad, double entry_radius_m, double entry_gamma_rad,
                       double gm) {
    const auto gamma_at_entry = [&](double dv) {
        const double u = v_mps - dv;
        const double energy = 0.5 * u * u - gm / r_m;
        const double h = r_m * u * std::cos(gamma_rad);
        const double ve2 = 2.0 * (energy + gm / entry_radius_m);
        if (ve2 <= 0.0) return 0.0;  // never gets there: shallow
        return -std::acos(std::clamp(h / (entry_radius_m * std::sqrt(ve2)), -1.0, 1.0));
    };
    double lo = 0.0, hi = 1000.0;  // gamma falls with dv
    for (int i = 0; i < 60; ++i) {
        const double mid = 0.5 * (lo + hi);
        (gamma_at_entry(mid) > entry_gamma_rad ? lo : hi) = mid;
    }
    return 0.5 * (lo + hi);
}

}  // namespace

DeorbitPlan plan_deorbit(const math::StateVector& state_gcrf, time::Epoch epoch, const environment::GravityModel& gravity,
                         const frames::BodyOrientation& earth, EntryPredictorModel model, const EntryTarget& target,
                         double mass_kg, double exhaust_velocity_mps, double available_propellant_kg,
                         double reserve_propellant_kg, const DeorbitParameters& parameters) {
    DeorbitPlan plan;
    const bool debug = std::getenv("STARSIM_DEBUG_DEORBIT") != nullptr;
    const double gm = model.gm_m3ps2;
    const auto burn_propellant = [&](double dv) { return mass_kg * (1.0 - std::exp(-dv / exhaust_velocity_mps)); };

    // Coast samples over the window.
    std::vector<Sample> samples;
    const auto options = planning_options();
    double next = 0.0;
    const integrators::StepObserver<6> observer = [&](const integrators::Dop853<6>& solver) {
        while (next <= solver.t()) {
            samples.push_back({epoch + next, math::unpack(solver.dense_output(next))});
            next += parameters.sample_s;
        }
    };
    integrators::integrate<6>(orbit::ballistic_dynamics(gravity, epoch), 0.0, math::pack(state_gcrf),
                              parameters.search_window_s, options, {}, observer);
    if (samples.size() < 3) {
        plan.reason = "Bahn nicht berechenbar";
        return plan;
    }

    // Passes: closest approaches of the ground track to the target, prefiltered on the track's cross distance.
    const Vec3 target_direction = math::normalized(target.position_itrf_m);
    const double radius_m = norm(target.position_itrf_m);
    std::vector<double> angle(samples.size());
    std::vector<double> cross_track(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        Vec3 r, v;
        to_fixed(earth, samples[i].epoch, samples[i].state, r, v);
        angle[i] = std::acos(std::clamp(dot(math::normalized(r), target_direction), -1.0, 1.0));
        cross_track[i] = std::asin(std::clamp(dot(target_direction, math::normalized(cross(r, v))), -1.0, 1.0)) * radius_m;
    }
    const double r0 = norm(state_gcrf.position_m);
    const double period_s = 2.0 * constants::pi * std::sqrt(r0 * r0 * r0 / gm);
    const double entry_radius_m = radius_m + parameters.entry_interface_height_m;

    Shooter shooter(samples, gravity, earth, model, target, parameters);
    for (size_t i = 1; i + 1 < samples.size(); ++i) {
        if (!(angle[i] < angle[i - 1] && angle[i] <= angle[i + 1])) continue;
        if (std::abs(cross_track[i]) > 5.0 * parameters.max_crossrange_m) continue;
        const time::Epoch pass = samples[i].epoch;
        ++plan.passes_checked;

        const auto& s = samples[i].state;
        const double v0 = norm(s.velocity_mps);
        const double gamma0 = std::asin(dot(s.velocity_mps, s.position_m) / (v0 * norm(s.position_m)));
        double dv = initial_delta_v(norm(s.position_m), v0, gamma0, entry_radius_m, parameters.entry_flight_path_rad, gm);
        shooter.set_mass(mass_kg - burn_propellant(dv));

        // Bracket the burn epoch on the downrange error (later burn: later entry, overshoot).
        const time::Epoch earliest = epoch + parameters.lead_s;
        bool bracketed = false;
        time::Epoch t_lo, t_hi;
        Shot s_lo;
        bool have_previous = false;
        Shot previous;
        time::Epoch previous_t;
        for (double back = period_s; back >= 0.05 * period_s; back -= parameters.sample_s) {
            const time::Epoch t = pass - back;
            if (t < earliest) continue;
            const Shot shot = shooter.shoot(t, dv);
            if (!shot.valid) {
                have_previous = false;
                continue;
            }
            if (have_previous && previous.downrange_m < 0.0 && shot.downrange_m >= 0.0 &&
                shot.downrange_m - previous.downrange_m < 4.0e6) {
                t_lo = previous_t;
                t_hi = t;
                s_lo = previous;
                bracketed = true;
                (void)t_hi;
                break;
            }
            previous = shot;
            previous_t = t;
            have_previous = true;
        }
        if (debug) {
            std::printf("[deorbit] pass %+.0f s cross track %.0f km: dv0 %.1f m/s, %s\n", pass - epoch,
                        cross_track[i] / 1000.0, dv, bracketed ? "bracketed" : "no bracket");
        }
        if (!bracketed) continue;

        // Newton on (flight-path angle at the entry interface, downrange error) in (burn epoch, delta-v).
        time::Epoch t = t_lo;
        Shot shot = s_lo;
        bool converged = false;
        for (int iteration = 0; iteration < 25; ++iteration) {
            shot = shooter.shoot(t, dv);
            if (!shot.valid) break;
            const double f1 = (shot.flight_path_rad - parameters.entry_flight_path_rad) / (0.01 * constants::deg_to_rad);
            const double f2 = shot.downrange_m / 1000.0;
            if (debug) {
                std::printf("[deorbit]   it %d burn %+.1f s dv %.2f m/s: gamma %.4f deg downrange %.2f km cross %.1f km\n",
                            iteration, t - epoch, dv, shot.flight_path_rad / constants::deg_to_rad, f2,
                            shot.crossrange_m / 1000.0);
            }
            if (std::abs(f1) < 0.3 && std::abs(f2) < 0.3) {
                converged = true;
                break;
            }
            const double dt = 2.0, ddv = 0.2;
            const Shot st = shooter.shoot(t + dt, dv);
            const Shot sv = shooter.shoot(t, dv + ddv);
            if (!st.valid || !sv.valid) break;
            const double a11 = (st.flight_path_rad - shot.flight_path_rad) / (0.01 * constants::deg_to_rad) / dt;
            const double a21 = (st.downrange_m - shot.downrange_m) / 1000.0 / dt;
            const double a12 = (sv.flight_path_rad - shot.flight_path_rad) / (0.01 * constants::deg_to_rad) / ddv;
            const double a22 = (sv.downrange_m - shot.downrange_m) / 1000.0 / ddv;
            const double det = a11 * a22 - a12 * a21;
            if (std::abs(det) < 1e-12) break;
            double step_t = -(a22 * f1 - a12 * f2) / det;
            double step_v = -(-a21 * f1 + a11 * f2) / det;
            step_t = std::clamp(step_t, -120.0, 120.0);
            step_v = std::clamp(step_v, -20.0, 20.0);
            t = std::max(t + step_t, earliest);
            dv = std::max(dv + step_v, 1.0);
            shooter.set_mass(mass_kg - burn_propellant(dv));
        }
        if (!converged || std::abs(shot.crossrange_m) > parameters.max_crossrange_m) continue;

        plan.node = {t, -math::normalized(shooter.state_at(t).velocity_mps) * dv};
        plan.delta_v_mps = dv;
        plan.entry_interface_epoch = shot.entry_epoch;
        plan.entry_flight_path_rad = shot.flight_path_rad;
        plan.entry_speed_mps = shot.speed_mps;
        plan.downrange_error_m = shot.downrange_m;
        plan.crossrange_m = shot.crossrange_m;
        plan.propellant_kg = burn_propellant(dv);
        if (available_propellant_kg - plan.propellant_kg < reserve_propellant_kg) {
            char text[160];
            std::snprintf(text, sizeof text, "Zu wenig Treibstoff: Deorbit %.1f t + Landung %.1f t, an Bord %.1f t",
                          plan.propellant_kg / 1000.0, reserve_propellant_kg / 1000.0, available_propellant_kg / 1000.0);
            plan.reason = text;
            return plan;
        }
        plan.feasible = true;
        return plan;
    }
    char text[160];
    std::snprintf(text, sizeof text, "Kein Überflug des Ziels in Reichweite (%.0f h gesucht)",
                  parameters.search_window_s / 3600.0);
    plan.reason = text;
    return plan;
}

}  // namespace starsim::gnc
