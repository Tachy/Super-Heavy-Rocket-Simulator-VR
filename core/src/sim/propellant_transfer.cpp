#include "starsim/sim/propellant_transfer.hpp"

#include <algorithm>

#include "starsim/vehicle/propellant_transfer.hpp"

namespace starsim::sim {

namespace {

constexpr double closed_margin_kg = 1e-3;  // a line within a gram of its end is closed (rounding of the tank sums)

bool line_open(const TransferRequest& r, const TransferTanks& t, double delivered_kg, size_t k) {
    return t.delivering_kg[k] > r.delivering_floor_kg[k] + closed_margin_kg &&
           t.receiving_kg[k] < t.receiving_capacity_kg[k] - closed_margin_kg &&
           delivered_kg < r.limit_kg[k] - closed_margin_kg;
}

}  // namespace

bool PropellantTransfer::any_line_open(const TransferRequest& request, const TransferTanks& tanks) {
    return line_open(request, tanks, 0.0, transfer_oxidizer) || line_open(request, tanks, 0.0, transfer_fuel);
}

PropellantTransfer::PropellantTransfer(std::array<TransferLine, 2> lines, TransferRequest request,
                                       const TransferTanks& tanks)
    : lines_(lines), request_(request) {
    for (size_t k = 0; k < 2; ++k) state_.line_open[k] = line_open(request_, tanks, 0.0, k);
    state_.phase = TransferPhase::Settling;
    state_.status = "Treibstoff wird gesetzt";
}

bool PropellantTransfer::active() const {
    return state_.phase == TransferPhase::Settling || state_.phase == TransferPhase::Chilldown ||
           state_.phase == TransferPhase::Flowing || state_.phase == TransferPhase::Paused;
}

void PropellantTransfer::stop() {
    if (!active()) return;
    state_.phase = TransferPhase::Stopped;
    state_.status = "Transfer gestoppt";
    state_.flow_kgps = {};
}

void PropellantTransfer::add_settling(double gas_kg, double thrust_n) {
    state_.settling_gas_kg += gas_kg;
    state_.settling_thrust_n = thrust_n;
}

// A pause during Settling resumes into Chilldown: when the outlets are covered again, the liquid has settled.
void PropellantTransfer::pause(const TransferStepInput& in) {
    if (state_.phase != TransferPhase::Paused) {
        paused_from_ = state_.phase == TransferPhase::Flowing ? TransferPhase::Flowing : TransferPhase::Chilldown;
    }
    state_.phase = TransferPhase::Paused;
    state_.status = in.thrust_available ? "Transfer unterbrochen: Treibstoff nicht am Auslass"
                                        : "Transfer unterbrochen: kein Kaltgas zum Setzen";
    state_.flow_kgps = {};
}

void PropellantTransfer::close_lines(const TransferTanks& tanks) {
    for (size_t k = 0; k < 2; ++k) {
        state_.line_open[k] = state_.line_open[k] && line_open(request_, tanks, state_.delivered_kg[k], k);
    }
    if (!state_.line_open[0] && !state_.line_open[1]) {
        state_.phase = TransferPhase::Complete;
        state_.status = "Transfer abgeschlossen";
        state_.flow_kgps = {};
    }
}

TransferStepOutput PropellantTransfer::step(const TransferStepInput& in) {
    TransferStepOutput out;
    if (!active()) return out;
    state_.elapsed_s += in.dt_s;
    phase_elapsed_s_ += in.dt_s;
    state_.flow_kgps = {};
    switch (state_.phase) {
    case TransferPhase::Settling:
        if (!in.thrust_available) {
            pause(in);
        } else if (phase_elapsed_s_ >= in.settle_time_s) {
            if (in.settled) {
                state_.phase = TransferPhase::Chilldown;
                state_.status = "Leitungen werden gekühlt";
                phase_elapsed_s_ = 0.0;
            } else {
                pause(in);
            }
        }
        break;
    case TransferPhase::Paused:
        if (in.settled && in.thrust_available) {
            state_.phase = paused_from_;
            state_.status = paused_from_ == TransferPhase::Flowing ? "Transfer läuft" : "Leitungen werden gekühlt";
            if (paused_from_ == TransferPhase::Chilldown) phase_elapsed_s_ = 0.0;  // chilled again from the start
        }
        break;
    case TransferPhase::Chilldown:
        if (!in.settled || !in.thrust_available) {
            pause(in);
            break;
        }
        for (size_t k = 0; k < 2; ++k) {
            if (!state_.line_open[k]) continue;
            const double left = lines_[k].chilldown_kg - state_.chilldown_kg[k];
            const double room = std::max(in.tanks.delivering_kg[k] - request_.delivering_floor_kg[k], 0.0);
            const double vent =
                std::clamp(std::min(left, lines_[k].chilldown_kg * in.dt_s / request_.chilldown_s), 0.0, room);
            out.vented_kg[k] = vent;
            state_.chilldown_kg[k] += vent;
        }
        if (phase_elapsed_s_ >= request_.chilldown_s - 1e-9) {
            state_.phase = TransferPhase::Flowing;
            state_.status = "Transfer läuft";
            phase_elapsed_s_ = 0.0;
        }
        break;
    case TransferPhase::Flowing: {
        if (!in.settled || !in.thrust_available) {
            pause(in);
            break;
        }
        TransferTanks after = in.tanks;
        for (size_t k = 0; k < 2; ++k) {
            if (!state_.line_open[k]) continue;
            const double flow = vehicle::line_mass_flow_kgps(lines_[k].discharge_area_m2, lines_[k].density_kg_m3,
                                                             in.pressure_difference_pa);
            const double moved = std::max(
                0.0, std::min({flow * in.dt_s, in.tanks.delivering_kg[k] - request_.delivering_floor_kg[k],
                               in.tanks.receiving_capacity_kg[k] - in.tanks.receiving_kg[k],
                               request_.limit_kg[k] - state_.delivered_kg[k]}));
            out.moved_kg[k] = moved;
            state_.delivered_kg[k] += moved;
            state_.flow_kgps[k] = moved / in.dt_s;
            after.delivering_kg[k] -= moved;
            after.receiving_kg[k] += moved;
        }
        close_lines(after);
        break;
    }
    default:
        break;
    }
    out.thrust = active();
    return out;
}

}  // namespace starsim::sim
