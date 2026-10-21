// Refuelling f2: the station's reboost - the depot keeps its altitude between the tanker flights (spec f2 "Depot
// station keeping, flown"): released from its ideal attitude, a Hohmann pair from its mean orbit, a station again.

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "starsim/gnc/burn_node.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

double FlightSession::station_mean_height_m() const {
    if (!station_index_) throw std::logic_error("station_mean_height_m: no station");
    const auto& s = bodies_[*station_index_].body->state();
    const math::Vec3 pole = orbit::true_equator_to_gcrf(earth_, epoch_) * math::Vec3{0.0, 0.0, 1.0};
    return orbit::energy_mean_semi_major_axis_m({s.position_gcrf_m, s.velocity_gcrf_mps}, pole, gm_earth_,
                                                constants::earth_j2, constants::wgs84_semi_major_axis_m) -
           constants::wgs84_semi_major_axis_m;
}

bool FlightSession::start_station_reboost(double height_m, std::vector<std::string> groups) {
    const auto refuse = [&](const char* why) {
        reboost_status_ = why;
        return false;
    };
    if (reboost_) return refuse("Reboost läuft bereits");
    if (!station_index_) return refuse("Reboost: keine Station");
    if (bodies_.size() != 1) return refuse("Reboost nur ohne andere Fahrzeuge");
    if (groups.empty()) return refuse("Reboost: keine Triebwerksgruppen");
    if (station_mean_height_m() > height_m - mission::raise_tolerance_m) return refuse("Reboost: Station hoch genug");
    const size_t index = *station_index_;
    auto& body = *bodies_[index].body;
    Reboost r;
    r.height_m = height_m;
    r.groups = std::move(groups);
    r.station_body = index;
    r.start_main_kg = body.vehicle().main_propellant_kg();
    r.record.start = epoch_;
    r.impulses = std::move(station_impulses_);
    // released: the depot flies (its ideal attitude ends, the burn executor turns it)
    station_index_.reset();
    station_impulses_.clear();
    body.set_ideal_lvlh_attitude(std::nullopt);
    controlled_ = index;
    reset_mission_state();
    reboost_ = std::move(r);
    reboost_status_ = "Reboost";
    run_reboost();  // the first burn now (or the end)
    return true;
}

void FlightSession::run_reboost() {
    auto& r = *reboost_;
    if (r.burning) {
        if (autopilot_.state == AutopilotState::Burn) return;  // flying (turn, settling, burn)
        if (!burn_abort_reason_.empty()) return end_reboost(false, "Reboost abgebrochen: " + burn_abort_reason_);
        // the tail-off first, as the Raise leg (f1): a burn planned during it lit the Raptor again on the unsettled
        // liquid (f2 probe: gas ingestion on a 1000 t depot right after its first burn)
        if (bodies_[controlled_].body->vehicle().any_engine_producing_thrust()) return;
        r.burning = false;
    }
    // A Hohmann pair from the energy-mean semi-major axis [D]: the depot's osculating apsides swing by +-10 km with J2
    // (f2 probe: periapsis 175-193 km on a 191 km mean orbit), so apsis burns planned from them (the Raise leg's
    // next_raise_node) asked 6.7 m/s for a 2 m/s circularisation. The first burn after reboost_lead_s, the second half a
    // transfer orbit later, sized from the mean semi-major axis the first one left.
    const auto& s = bodies_[controlled_].body->state();
    const math::StateVector state{s.position_gcrf_m, s.velocity_gcrf_mps};
    const math::Vec3 pole = orbit::true_equator_to_gcrf(earth_, epoch_) * math::Vec3{0.0, 0.0, 1.0};
    const double a = orbit::energy_mean_semi_major_axis_m(state, pole, gm_earth_, constants::earth_j2,
                                                          constants::wgs84_semi_major_axis_m);
    const double r2 = constants::wgs84_semi_major_axis_m + r.height_m;
    double dv_mps = 0.0;
    time::Epoch at = epoch_;
    if (r.record.burns == 0) {
        const double r1 = a;  // near-circular: the mean radius is the mean semi-major axis
        dv_mps = std::sqrt(gm_earth_ / r1) * (std::sqrt(2.0 * r2 / (r1 + r2)) - 1.0);
        at = epoch_ + reboost_lead_s;
        r.second_burn = at + constants::pi * std::sqrt(std::pow(0.5 * (r1 + r2), 3) / gm_earth_);
    } else if (r.record.burns == 1) {
        dv_mps = std::sqrt(gm_earth_ / r2) - std::sqrt(gm_earth_ * (2.0 / r2 - 1.0 / a));  // at the apoapsis r2
        at = std::max(r.second_burn, epoch_ + burn_preparation_s);
    } else {
        return end_reboost(true, {});
    }
    if (!(dv_mps > 0.0)) return end_reboost(true, {});
    const auto ahead = orbit::propagate_two_body(state, gm_earth_, at - epoch_);
    const gnc::BurnNode node{at, dv_mps * math::normalized(ahead.velocity_mps)};
    if (!start_burn(node, burn_configurations(&r.groups), true, small_burn_transient_share)) {
        return end_reboost(false, "Reboost: Brennvorgang nicht möglich");
    }
    r.burning = true;
    r.record.cold_gas = r.record.cold_gas || burn_config_.rcs;
    ++r.record.burns;
}

void FlightSession::end_reboost(bool completed, std::string reason) {
    auto r = std::move(*reboost_);
    reboost_.reset();
    disengage_autopilot();
    r.record.end = epoch_;
    r.record.completed = completed;
    r.record.reason = std::move(reason);
    r.record.propellant_kg = r.start_main_kg - bodies_[r.station_body].body->vehicle().main_propellant_kg();
    make_station(r.station_body);  // the ideal attitude at once
    station_impulses_ = std::move(r.impulses);
    reboost_status_ = completed ? std::string("Reboost abgeschlossen") : r.record.reason;
    last_reboost_ = std::move(r.record);
}

}  // namespace starsim::sim
