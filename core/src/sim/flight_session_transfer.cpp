// Refuelling r4: a docked pair's boil-off and the propellant transfer between its ships.

#include <algorithm>

#include "starsim/sim/flight_session.hpp"
#include "starsim/vehicle/boil_off.hpp"

namespace starsim::sim {

using math::Vec3;

vehicle::LiquidEnvironment FlightSession::docked_liquid(size_t body_index, const Vec3& push_station_mps2) const {
    const auto& station = *bodies_[*station_index_].body;
    const auto& ss = station.state();
    // the compound's specific force: the drag of the station body (it carries the compound's mass) plus the push
    const Vec3 specific_station = station.liquid_specific_force_body_mps2() + push_station_mps2;
    if (body_index == *station_index_) {  // the station vehicle's CG is the compound CG (attached mass included)
        return {specific_station, ss.angular_velocity_body_radps, ss.position_gcrf_m, ss.attitude, {}};
    }
    const auto& chaser = *bodies_[body_index].body;
    return {docked_attitude_station_.inverse_rotate(specific_station), chaser.state().angular_velocity_body_radps,
            ss.position_gcrf_m, chaser.state().attitude,
            docked_attitude_station_.inverse_rotate(docked_position_station_m_)};
}

double FlightSession::vent_docked_boil_off(double dt_s, const Vec3& push_station_mps2) {
    double vented = 0.0;
    const std::array<size_t, 2> index{controlled_, *station_index_};
    for (size_t b = 0; b < 2; ++b) {
        auto& body = *bodies_[index[b]].body;
        auto now = body.boil_off_rates_now(env_, epoch_, docked_liquid(index[b], push_station_mps2));
        for (const auto& tank : now) {
            double flow = tank.mass_flow_kgps;
            for (const auto& before : docked_boil_off_last_[b]) {
                if (before.stage_index == tank.stage_index && before.tank_index == tank.tank_index) {
                    flow = 0.5 * (flow + before.mass_flow_kgps);
                }
            }
            const auto& t = body.vehicle().stages()[tank.stage_index].tanks[tank.tank_index];
            const double kg = std::min(t.mass_kg, flow * dt_s);
            if (kg <= 0.0) continue;
            set_tank_mass(index[b], tank.stage_index, t.config.id, t.mass_kg - kg);
            vented += kg;
        }
        body.set_boil_off_rate(vehicle::total_boil_off(body.vehicle(), now));
        docked_boil_off_last_[b] = std::move(now);
    }
    return vented;
}

void FlightSession::docked_update(double dt_s, bool after_coast) {
    if (!docked()) return;
    docked_elapsed_s_ += dt_s;
    if (!after_coast && docked_elapsed_s_ < vehicle::FlightBody::boil_off_interval_s - 1e-9) return;
    vent_docked_boil_off(docked_elapsed_s_, {});
    docked_elapsed_s_ = 0.0;
}

void FlightSession::reset_docked_update() {
    docked_elapsed_s_ = 0.0;
    docked_boil_off_last_ = {};
}

}  // namespace starsim::sim
