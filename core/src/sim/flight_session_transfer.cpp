// Refuelling r4: a docked pair's boil-off and the propellant transfer between its ships.

#include <algorithm>

#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"

namespace starsim::sim {

using math::Vec3;

namespace {

// The first connector of `contents` on any stage of the vehicle (its stage in *stage_index), or nullptr.
const config::TransferConnectorConfig* connector_for(const vehicle::Vehicle& v, config::TankContents contents,
                                                     size_t* stage_index) {
    for (size_t s = 0; s < v.stages().size(); ++s) {
        for (const auto& c : v.stages()[s].config.transfer_connectors) {
            if (c.contents == contents) {
                *stage_index = s;
                return &c;
            }
        }
    }
    return nullptr;
}

const vehicle::Tank* find_tank(const vehicle::Vehicle& v, size_t stage, const std::string& id) {
    for (const auto& t : v.stages()[stage].tanks) {
        if (t.config.id == id) return &t;
    }
    return nullptr;
}

}  // namespace

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
    if (transfer_ && transfer_->active()) {  // the boil-off runs on the transfer's steps
        // half a tick: a sum of ticks misses the step epoch by the Epoch's rounding (~1e-7 s at 8e8 s past J2000)
        if (epoch_ < transfer_next_ - 0.5 * tick_s) return;
        run_transfer_step(docked_elapsed_s_);
        transfer_next_ = epoch_ + transfer_->request().step_s;
        docked_elapsed_s_ = 0.0;
        return;
    }
    transfer_thrust_n_ = 0.0;
    if (!after_coast && docked_elapsed_s_ < vehicle::FlightBody::boil_off_interval_s - 1e-9) return;
    vent_docked_boil_off(docked_elapsed_s_, {});
    docked_elapsed_s_ = 0.0;
}

void FlightSession::reset_docked_update() {
    docked_elapsed_s_ = 0.0;
    docked_boil_off_last_ = {};
}

bool FlightSession::refuse_transfer(const char* reason) {
    transfer_.reset();
    transfer_refusal_ = {};
    transfer_refusal_.status = reason;
    return false;
}

const TransferState& FlightSession::transfer_state() const {
    return transfer_ ? transfer_->state() : transfer_refusal_;
}

void FlightSession::end_transfer() {
    if (transfer_) transfer_->stop();
    transfer_thrust_n_ = 0.0;
}

void FlightSession::stop_transfer() { end_transfer(); }

bool FlightSession::start_transfer(const TransferRequest& request) {
    if (transfer_ && transfer_->active()) {
        docking_status_ = "Transfer läuft bereits";
        return false;
    }
    if (!docked()) return refuse_transfer("Transfer nicht möglich: nicht angedockt");
    if (autopilot_.state != AutopilotState::Off && autopilot_.state != AutopilotState::Orbit) {
        return refuse_transfer("Transfer nicht möglich: Autopilot aktiv");
    }
    const size_t chaser = controlled_, station = *station_index_;
    TransferSides sides;
    sides.delivering = request.direction == TransferDirection::ChaserToStation ? chaser : station;
    sides.receiving = sides.delivering == chaser ? station : chaser;
    const auto& dv = bodies_[sides.delivering].body->vehicle();
    const auto& rv = bodies_[sides.receiving].body->vehicle();
    std::array<TransferLine, 2> lines;
    const std::array<config::TankContents, 2> contents{config::TankContents::Oxidizer, config::TankContents::Fuel};
    for (size_t k = 0; k < 2; ++k) {
        const auto* dc = connector_for(dv, contents[k], &sides.delivering_stage[k]);
        const auto* rc = connector_for(rv, contents[k], &sides.receiving_stage[k]);
        if (!dc || !rc) return refuse_transfer("Transfer nicht möglich: keine Transferkupplung");
        const auto& dp = dv.stages()[sides.delivering_stage[k]].config.propellant_transfer;
        const auto& rp = rv.stages()[sides.receiving_stage[k]].config.propellant_transfer;
        if (!dp || !rp) return refuse_transfer("Transfer nicht möglich: kein Transfersystem");
        sides.delivering_tank[k] = dc->tank_id;
        sides.receiving_tank[k] = rc->tank_id;
        sides.pressure_difference_pa = dp->delivery_pressure_pa - rp->receive_pressure_pa;
        const auto* dt = find_tank(dv, sides.delivering_stage[k], dc->tank_id);
        const auto* rt = find_tank(rv, sides.receiving_stage[k], rc->tank_id);
        lines[k] = {vehicle::pair_discharge_area_m2(*dc, *rc), dt->density_kg_m3, dc->chilldown_kg + rc->chilldown_kg};
        sides.longest_tank_m = std::max({sides.longest_tank_m, dt->length_m, rt->length_m});
    }
    sides.gas_stage = sides.delivering == chaser ? sides.delivering_stage[0] : sides.receiving_stage[0];
    transfer_sides_ = sides;
    const auto tanks = transfer_tanks();
    if (!PropellantTransfer::any_line_open(request, tanks)) {
        return refuse_transfer("Transfer nicht möglich: nichts zu übertragen");
    }
    transfer_.emplace(lines, request, tanks);
    transfer_next_ = epoch_ + request.step_s;
    transfer_thrust_n_ = 0.0;
    docked_elapsed_s_ = 0.0;  // the boil-off now runs on the transfer's steps
    run_transfer_step(0.0);   // the push for the first step and its outlets
    return true;
}

TransferTanks FlightSession::transfer_tanks() const {
    TransferTanks t;
    const auto& d = transfer_sides_;
    const auto& dv = bodies_[d.delivering].body->vehicle();
    const auto& rv = bodies_[d.receiving].body->vehicle();
    for (size_t k = 0; k < 2; ++k) {
        t.delivering_kg[k] = find_tank(dv, d.delivering_stage[k], d.delivering_tank[k])->mass_kg;
        const auto* r = find_tank(rv, d.receiving_stage[k], d.receiving_tank[k]);
        t.receiving_kg[k] = r->mass_kg;
        t.receiving_capacity_kg[k] = r->config.capacity_kg;
    }
    return t;
}

// One transfer step over the `dt_s` that just ended: the settling push and its gas, both ships' boil-off, then the
// push for the next step, the outlets under it and the state machine's moves. dt_s = 0 at the start only sets the push.
void FlightSession::run_transfer_step(double dt_s) {
    auto& chaser = *bodies_[controlled_].body;
    auto& station = *bodies_[*station_index_].body;
    const auto& request = transfer_->request();
    const auto& d = transfer_sides_;
    const double compound_kg = station.vehicle().mass_properties().mass_kg;  // carries the chaser's mass
    if (dt_s > 0.0) {
        // 1. the push of the step that ended: average thrust as one impulse along the chaser's +x
        if (transfer_thrust_n_ > 0.0) {
            auto s = station.state();
            s.velocity_gcrf_mps +=
                chaser.state().attitude.rotate(Vec3{1.0, 0.0, 0.0}) * (transfer_thrust_n_ * dt_s / compound_kg);
            station.set_state(s);
            sync_docked();
            // gas from the chaser's main tanks in proportion to their oxidizer and fuel (Vehicle::tank_mass_flows_kgps)
            const auto& st = chaser.vehicle().stages()[d.gas_stage];
            const double gas = transfer_thrust_n_ * dt_s / (st.config.rcs.isp_s * constants::standard_gravity_mps2);
            const double ox = st.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main);
            const double fuel = st.propellant_kg(config::TankContents::Fuel, config::TankRole::Main);
            double drawn = 0.0;
            if (ox + fuel > 0.0) {
                for (size_t i = 0; i < st.tanks.size(); ++i) {
                    const auto& t = chaser.vehicle().stages()[d.gas_stage].tanks[i];
                    if (t.config.role != config::TankRole::Main) continue;
                    const double share = t.config.contents == config::TankContents::Oxidizer ? ox / (ox + fuel)
                                         : t.config.contents == config::TankContents::Fuel  ? fuel / (ox + fuel)
                                                                                             : 0.0;
                    const double kg = std::min(t.mass_kg, gas * share);
                    if (kg <= 0.0) continue;
                    set_tank_mass(controlled_, d.gas_stage, t.config.id, t.mass_kg - kg);
                    drawn += kg;
                }
            }
            transfer_->add_settling(drawn, transfer_thrust_n_);
        }
        // 2. both ships' boil-off over the step, in the pushed compound's environment
        const Vec3 push = docked_attitude_station_.rotate(Vec3{transfer_thrust_n_ / compound_kg, 0.0, 0.0});
        transfer_->add_boil_off(vent_docked_boil_off(dt_s, push));
    }
    // 3. the push for the next step and the delivering outlets under it (the receiving tanks take liquid in)
    const auto& gas_stage = chaser.vehicle().stages()[d.gas_stage];
    const double capability = chaser.vehicle().rcs_pure_force_capability({1.0, 0.0, 0.0}).force_n;
    const bool gas_left = gas_stage.propellant_kg(config::TankContents::Oxidizer, config::TankRole::Main) +
                              gas_stage.propellant_kg(config::TankContents::Fuel, config::TankRole::Main) >
                          0.0;
    const double thrust = gas_left ? std::min(capability, request.settling_acceleration_mps2 * compound_kg) : 0.0;
    const double a = thrust / compound_kg;
    const auto& dv = bodies_[d.delivering].body->vehicle();
    const auto liquid = docked_liquid(d.delivering, docked_attitude_station_.rotate(Vec3{a, 0.0, 0.0}));
    bool settled = a > 0.0;
    for (size_t k = 0; k < 2 && settled; ++k) {
        if (!transfer_->state().line_open[k]) continue;
        const auto& tanks = dv.stages()[d.delivering_stage[k]].tanks;
        for (size_t i = 0; i < tanks.size(); ++i) {
            if (tanks[i].config.id == d.delivering_tank[k]) {
                settled = vehicle::outlet_check(dv, d.delivering_stage[k], i, liquid, request.max_settled_tilt_rad)
                              .covered;
            }
        }
    }
    transfer_thrust_n_ = transfer_->active() ? thrust : 0.0;
    if (dt_s <= 0.0) return;
    TransferStepInput in;
    in.dt_s = dt_s;
    in.tanks = transfer_tanks();  // after the gas and the boil-off: the floors hold against them
    in.settled = settled;
    in.thrust_available = capability > 0.0 && gas_left;
    in.settle_time_s = a > 0.0 ? vehicle::settling_time_s(d.longest_tank_m, a) : 0.0;
    in.pressure_difference_pa = d.pressure_difference_pa;
    const auto out = transfer_->step(in);
    // 4. move the propellant (the compound follows)
    for (size_t k = 0; k < 2; ++k) {
        if (out.moved_kg[k] + out.vented_kg[k] > 0.0) {
            set_tank_mass(d.delivering, d.delivering_stage[k], d.delivering_tank[k],
                          in.tanks.delivering_kg[k] - out.moved_kg[k] - out.vented_kg[k]);
        }
        if (out.moved_kg[k] > 0.0) {
            set_tank_mass(d.receiving, d.receiving_stage[k], d.receiving_tank[k],
                          in.tanks.receiving_kg[k] + out.moved_kg[k]);
        }
    }
    transfer_thrust_n_ = out.thrust ? thrust : 0.0;
}

}  // namespace starsim::sim
