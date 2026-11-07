#include "starsim/mission/lunar_orbit_transfer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/lambert.hpp"

namespace starsim::mission {

using math::StateVector;
using math::Vec3;

namespace {
constexpr double min_lead_s = 600.0;              // [E] a burn is never planned closer than this to "now"
// LLO -> NRHO scan [E]: arrival epochs on the reference in one-day windows, coarse time of flight, and the departure
// phase on the LLO within one LLO period (adds to the time of flight).
constexpr double ascent_window_s = 86400.0;
constexpr double ascent_arrival_step_s = 7200.0;
constexpr double ascent_min_tof_s = 0.25 * 86400.0;
constexpr double ascent_max_tof_s = 4.0 * 86400.0;
constexpr double ascent_tof_step_s = 3.0 * 3600.0;
constexpr double ascent_phase_step_s = 120.0;
constexpr int ascent_refined_per_window = 3;      // [E] cheapest two-body candidates refined per window
constexpr double ascent_tolerance_m = 100.0;      // [E] position aim on the reference
constexpr double station_keeping_tolerance_m = 100.0;
// NRHO -> LLO for a site: touchdowns scanned over the wait at this step (the site turns ~0.09 deg inertially) [E]
constexpr double site_wait_scan_step_s = 600.0;

bool debug_enabled() { return std::getenv("STARSIM_DEBUG_LUNAR_TRANSFER") != nullptr; }

// First perilune of the reference strictly after `t`, if any.
std::optional<time::Epoch> next_perilune(const NrhoReference& nrho, time::Epoch t) {
    for (const auto& tp : nrho.perilune_epochs()) {
        if (tp > t) return tp;
    }
    return std::nullopt;
}
}  // namespace

std::optional<NrhoSiteDeparture> nrho_site_departure(const NrhoReference& nrho, const ephemeris::Ephemeris& ephemeris,
                                                     const frames::BodyOrientation& moon, time::Epoch after,
                                                     double latitude_rad, double longitude_rad, double llo_altitude_m,
                                                     std::optional<time::Epoch> touchdown,
                                                     double first_pass_after_arrival_s, double site_wait_s) {
    const auto tp = next_perilune(nrho, after + min_lead_s);
    if (!tp || !nrho.covers(*tp)) return std::nullopt;
    const double gm_moon = ephemeris.gm_m3ps2(ephemeris::Body::Moon);
    const auto rel = moon_relative(ephemeris, nrho.state_gcrf(*tp), *tp);
    const Vec3 r_hat = math::normalized(rel.position_m);
    const double r = norm(rel.position_m);
    const double r_llo = constants::moon_mean_radius_m + llo_altitude_m;
    const double v_llo = std::sqrt(gm_moon / r_llo);
    const double v_aposelene = std::sqrt(gm_moon * 2.0 * r_llo / (r * (r + r_llo)));  // transfer ellipse r -> r_llo
    const double half_period = constants::pi * std::sqrt(std::pow(0.5 * (r + r_llo), 3) / gm_moon);
    // the departure burn into a plane through the perilune point: the difference of the velocity vectors (the perilune
    // velocity may keep a small radial part) to the aposelene speed, horizontal, in that plane
    const auto burn = [&](const Vec3& normal) { return norm(cross(normal, r_hat) * v_aposelene - rel.velocity_mps); };
    // the flown transfer (nrho_to_llo: inclination 90 deg): the polar plane through the perilune point, in the NRHO's
    // sense
    const Vec3 pole = moon.fixed_to_gcrf(*tp).rotation * Vec3{0.0, 0.0, 1.0};
    const Vec3 nrho_normal = math::normalized(cross(rel.position_m, rel.velocity_mps));
    Vec3 polar = cross(pole, r_hat);
    // a perilune over a pole is in every polar plane: then the NRHO's own (the transfer turns nothing)
    polar = norm(polar) < 1e-6 ? nrho_normal : math::normalized(polar);
    if (dot(polar, nrho_normal) < 0.0) polar = -polar;
    const double polar_burn = burn(polar);

    NrhoSiteDeparture d;
    d.departure = *tp;
    const time::Epoch first_pass = *tp + half_period + first_pass_after_arrival_s;
    const auto evaluate = [&](time::Epoch t) {
        NrhoSiteDeparture c = d;
        c.touchdown = t;
        // (a) in the departure burn: the plane through the perilune point and the site
        const Vec3 site_plane = llo_plane_through_site(moon, latitude_rad, longitude_rad, t, rel.position_m, polar);
        // not credited below the polar burn: the flight flies the polar transfer
        const double in_burn = std::max(burn(site_plane) - polar_burn, 0.0);
        // (b) in the polar LLO: the least turn that holds the site
        const auto rotation = moon.fixed_to_gcrf(t).rotation;
        const Vec3 site = rotation * Vec3{std::cos(latitude_rad) * std::cos(longitude_rad),
                                          std::cos(latitude_rad) * std::sin(longitude_rad), std::sin(latitude_rad)};
        const double llo_turn = std::asin(std::min(std::abs(dot(polar, site)), 1.0));
        const double in_llo = 2.0 * v_llo * std::sin(0.5 * llo_turn);
        c.in_llo = in_llo < in_burn;
        c.plane_change_rad = c.in_llo ? llo_turn : std::acos(std::clamp(dot(site_plane, polar), -1.0, 1.0));
        c.extra_delta_v_mps = std::min(in_burn, in_llo);
        // (b): the polar plane turned least onto the site (its normal's component along the site removed)
        const Vec3 turned = polar - site * dot(polar, site);
        c.llo_plane_normal_moon_inertial =
            c.in_llo ? (norm(turned) < 1e-12 ? polar : math::normalized(turned)) : site_plane;
        return c;
    };
    if (touchdown) return evaluate(*touchdown);
    d = evaluate(first_pass);
    for (double wait = site_wait_scan_step_s; wait <= site_wait_s; wait += site_wait_scan_step_s) {
        const auto c = evaluate(first_pass + wait);  // the pass of least cost within the wait
        if (c.extra_delta_v_mps < d.extra_delta_v_mps) d = c;
    }
    return d;
}

LunarOrbitTransfer::LunarOrbitTransfer(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                                       const frames::BodyOrientation& moon_orientation)
    : ephemeris_(ephemeris),
      transfer_(gravity, ephemeris, moon_orientation),
      position_(gravity),
      gm_moon_(ephemeris.gm_m3ps2(ephemeris::Body::Moon)) {}

TwoBurnTransfer LunarOrbitTransfer::nrho_to_llo(const StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                                const LloTarget& llo) const {
    TwoBurnTransfer result;
    const auto tp = next_perilune(nrho, epoch + min_lead_s);
    if (!tp) {
        result.reason = report::make(report::Code::lunar_no_nrho_to_llo_transfer);
        return result;
    }
    // Hohmann-like first guess: the perilune becomes the aposelene of an ellipse down to the LLO radius.
    const auto rel = moon_relative(ephemeris_, position_.coast(state, epoch, *tp), *tp);
    const double r = norm(rel.position_m);
    const double r_llo = constants::moon_mean_radius_m + llo.altitude_m;
    const double v = norm(rel.velocity_mps);
    const double v_apo = std::sqrt(gm_moon_ * 2.0 * r_llo / (r * (r + r_llo)));
    const Vec3 guess = math::normalized(rel.velocity_mps) * (v_apo - v);
    const double half_period = constants::pi * std::sqrt(std::pow(0.5 * (r + r_llo), 3) / gm_moon_);

    // The targeter's "periselene" is the first closest approach after the burn: the burn sits at the transfer's
    // aposelene, so the radial velocity first falls and rises through zero at the far side, at the LLO altitude.
    const auto solution = transfer_.solve(state, epoch, *tp,
                                          {llo.altitude_m, llo.inclination_rad, *tp + half_period}, guess);
    if (!solution.converged) {
        result.reason = report::make(report::Code::lunar_no_nrho_to_llo_transfer);
        return result;
    }
    result.feasible = true;
    result.first = solution.burn;
    result.second = LunarTransferTargeter::orbit_insertion(solution.arrival, gm_moon_);
    result.total_delta_v_mps = norm(result.first.delta_v_gcrf_mps) + norm(result.second.delta_v_gcrf_mps);
    return result;
}

TwoBurnTransfer LunarOrbitTransfer::llo_to_nrho(const StateVector& state, time::Epoch epoch, const NrhoReference& nrho,
                                                double search_s, double max_total_mps) const {
    TwoBurnTransfer best;
    best.total_delta_v_mps = std::numeric_limits<double>::infinity();
    const auto rel0 = moon_relative(ephemeris_, state, epoch);
    const Vec3 llo_normal = cross(rel0.position_m, rel0.velocity_mps);
    const double llo_period = 2.0 * constants::pi * std::sqrt(std::pow(norm(rel0.position_m), 3) / gm_moon_);

    struct Candidate {
        double total_mps;
        time::Epoch departure, arrival;
        Vec3 delta_v_mps;
    };
    for (double window = 0.0; window < search_s; window += ascent_window_s) {
        // Two-body scan: both ends relative to the Moon's centre at their own epochs; best per arrival epoch.
        std::vector<Candidate> candidates;
        for (double ta = std::max(window, ascent_min_tof_s); ta < std::min(window + ascent_window_s, search_s);
             ta += ascent_arrival_step_s) {
            const time::Epoch arrival = epoch + ta;
            if (!nrho.covers(arrival)) continue;
            const auto target = moon_relative(ephemeris_, nrho.state_gcrf(arrival), arrival);
            Candidate c{std::numeric_limits<double>::infinity(), {}, arrival, {}};
            for (double tof = ascent_min_tof_s; tof <= ascent_max_tof_s; tof += ascent_tof_step_s) {
                for (double phase = 0.0; phase < llo_period; phase += ascent_phase_step_s) {
                    const double td = ta - tof - phase;
                    if (td < min_lead_s) break;
                    const auto at_td = orbit::propagate_two_body(rel0, gm_moon_, td);
                    const auto arc = orbit::solve_lambert(at_td.position_m, target.position_m, tof + phase, gm_moon_,
                                                          llo_normal);
                    if (!arc) continue;
                    const double total = norm(arc->departure_velocity_mps - at_td.velocity_mps) +
                                         norm(target.velocity_mps - arc->arrival_velocity_mps);
                    if (total < c.total_mps) c = {total, epoch + td, arrival, arc->departure_velocity_mps -
                                                                                   at_td.velocity_mps};
                }
            }
            if (std::isfinite(c.total_mps)) candidates.push_back(c);
        }
        std::sort(candidates.begin(), candidates.end(),
                  [](const Candidate& a, const Candidate& b) { return a.total_mps < b.total_mps; });
        if (candidates.size() > ascent_refined_per_window) candidates.resize(ascent_refined_per_window);

        // Refine in the full force model: reach the reference's position at the arrival epoch.
        TwoBurnTransfer window_best;
        window_best.total_delta_v_mps = std::numeric_limits<double>::infinity();
        for (const auto& c : candidates) {
            const auto solution = position_.solve(state, epoch, c.departure, {nrho.state_gcrf(c.arrival).position_m,
                                                                              c.arrival},
                                                  c.delta_v_mps, ascent_tolerance_m);
            TwoBurnTransfer candidate;
            if (solution.converged) {
                candidate.feasible = true;
                candidate.first = solution.burn;
                candidate.second = {c.arrival,
                                    nrho.state_gcrf(c.arrival).velocity_mps - solution.arrival_gcrf.velocity_mps};
                candidate.total_delta_v_mps =
                    norm(candidate.first.delta_v_gcrf_mps) + norm(candidate.second.delta_v_gcrf_mps);
                if (candidate.total_delta_v_mps < window_best.total_delta_v_mps) window_best = candidate;
            }
            if (debug_enabled()) {
                std::printf("[lunar transfer] arrival %+.2f d, tof %.2f d: two-body %.0f m/s, refined %s %.0f m/s\n",
                            (c.arrival - epoch) / 86400.0, (c.arrival - c.departure) / 86400.0, c.total_mps,
                            solution.converged ? "ok" : "failed", candidate.total_delta_v_mps);
            }
        }
        if (!window_best.feasible) continue;
        if (window_best.total_delta_v_mps <= max_total_mps) return window_best;
        if (window_best.total_delta_v_mps < best.total_delta_v_mps) best = window_best;
    }
    if (!best.feasible) {
        best = {};
        best.reason = report::make(report::Code::lunar_no_llo_to_nrho_transfer);
    }
    return best;
}

std::optional<gnc::BurnNode> LunarOrbitTransfer::station_keeping(const StateVector& state, time::Epoch epoch,
                                                                 const NrhoReference& nrho, double deadband_m,
                                                                 double lead_s) const {
    const auto tp = next_perilune(nrho, epoch);
    if (!tp || !nrho.covers(*tp)) return std::nullopt;
    const Vec3 aim = nrho.state_gcrf(*tp).position_m;
    if (norm(position_.coast(state, epoch, *tp).position_m - aim) < deadband_m) return std::nullopt;
    const auto solution = position_.solve(state, epoch, epoch + lead_s, {aim, *tp}, Vec3{},
                                          station_keeping_tolerance_m);
    if (!solution.converged) return std::nullopt;
    return solution.burn;
}

std::optional<gnc::BurnNode> LunarOrbitTransfer::arrival_burn(const StateVector& state, time::Epoch epoch,
                                                              const TwoBurnTransfer& transfer, LunarOrbitKind target,
                                                              const NrhoReference* nrho, const LloTarget&) const {
    if (target == LunarOrbitKind::Llo) {
        const auto arrival = transfer_.evaluate(state, epoch, epoch, Vec3{});
        if (!arrival.reached) return std::nullopt;
        return LunarTransferTargeter::orbit_insertion(arrival, gm_moon_);
    }
    const time::Epoch t = transfer.second.epoch;
    if (!nrho || !nrho->covers(t)) return std::nullopt;
    const auto arrived = position_.coast(state, epoch, t);
    // Velocity match only: the position error at the perilune is left to the station keeping.
    return gnc::BurnNode{t, nrho->state_gcrf(t).velocity_mps - arrived.velocity_mps};
}

}  // namespace starsim::mission
