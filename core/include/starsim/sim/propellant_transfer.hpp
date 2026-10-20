#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>

#include "starsim/math/constants.hpp"

namespace starsim::sim {

// Ship-to-ship propellant transfer (refuelling r4): the state machine of one transfer between a docked pair, fed by the
// session with tank masses and whether the outlets are covered. Settling (the settling push runs for the settling
// time) -> Chilldown (the lines are chilled, the propellant vented) -> Flowing (pressure-driven, per line until the
// delivering tank reaches its floor, the receiving tank is full or the line's limit is delivered) -> Complete. An
// uncovered outlet pauses Chilldown / Flowing (and ends a Settling that did not settle); a covered one resumes.
enum class TransferDirection { ChaserToStation, StationToChaser };
enum class TransferPhase { Idle, Settling, Chilldown, Flowing, Paused, Complete, Stopped };
inline constexpr size_t transfer_oxidizer = 0;  // line indices
inline constexpr size_t transfer_fuel = 1;

struct TransferRequest {
    TransferDirection direction = TransferDirection::ChaserToStation;
    std::array<double, 2> delivering_floor_kg{0.0, 0.0};  // the delivering main tanks keep this (the tanker reserve)
    std::array<double, 2> limit_kg{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    double settling_acceleration_mps2 = 1.0e-4 * constants::standard_gravity_mps2;  // [E] spec r4
    double max_settled_tilt_rad = 30.0 * constants::deg_to_rad;  // [E] the aft dome's outlet stays covered
    double chilldown_s = 120.0;                                  // [E]
    double step_s = 10.0;                                        // [E] update cadence (flow, thrust, boil-off)
    double max_pause_s = 600.0;  // [E] a longer pause aborts (the push would drain the reserve, undocking blocked)
};

struct TransferLine {
    double discharge_area_m2 = 0.0;  // C_d A of the coupled pair
    double density_kg_m3 = 0.0;      // of the delivered liquid
    double chilldown_kg = 0.0;       // both connectors' together, vented from the delivering tank
};

struct TransferTanks {
    std::array<double, 2> delivering_kg{};  // main tanks of the delivering ship
    std::array<double, 2> receiving_kg{};
    std::array<double, 2> receiving_capacity_kg{};
};

struct TransferStepInput {
    double dt_s = 0.0;
    TransferTanks tanks;
    bool settled = false;           // every involved delivering tank's outlet covered (vehicle::outlet_check)
    bool thrust_available = false;  // the chaser can push (+x capability and cold gas left)
    double settle_time_s = 0.0;     // the longest involved tank's settling time at the achieved push
    double pressure_difference_pa = 0.0;
};

struct TransferStepOutput {
    std::array<double, 2> moved_kg{};   // from the delivering to the receiving tank
    std::array<double, 2> vented_kg{};  // chill-down, from the delivering tank
    bool thrust = false;                // the settling push stays on
};

struct TransferState {
    TransferPhase phase = TransferPhase::Idle;
    std::string status;  // German (UI)
    double elapsed_s = 0.0;
    std::array<double, 2> delivered_kg{};
    std::array<double, 2> flow_kgps{};     // of the last step
    std::array<double, 2> chilldown_kg{};  // vented
    std::array<bool, 2> line_open{};
    double settling_gas_kg = 0.0;
    double settling_thrust_n = 0.0;  // of the last step
    double boil_off_kg = 0.0;        // both ships, since the start
};

class PropellantTransfer {
public:
    PropellantTransfer(std::array<TransferLine, 2> lines, TransferRequest request, const TransferTanks& tanks);
    TransferStepOutput step(const TransferStepInput& in);
    void stop();
    void add_settling(double gas_kg, double thrust_n);
    void add_boil_off(double kg) { state_.boil_off_kg += kg; }
    bool active() const;  // Settling, Chilldown, Flowing or Paused: the valves armed, the settling push on
    const TransferState& state() const { return state_; }
    const TransferRequest& request() const { return request_; }
    // Whether a line can move anything (the session refuses a start without one).
    static bool any_line_open(const TransferRequest& request, const TransferTanks& tanks);

private:
    void pause(const TransferStepInput& in);
    void close_lines(const TransferTanks& tanks);

    std::array<TransferLine, 2> lines_;
    TransferRequest request_;
    TransferState state_;
    TransferPhase paused_from_ = TransferPhase::Chilldown;
    double phase_elapsed_s_ = 0.0;
};

}  // namespace starsim::sim
