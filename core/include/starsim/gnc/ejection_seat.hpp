#pragma once

#include "starsim/config/crew_escape_config.hpp"

namespace starsim::gnc {

// ACES II-class seat modes (config/crew_escape): 1 slow and low - main straight after the rocket, no drogue; 2 fast -
// drogue, then the seat-man separation and the main; 3 high - the drogue is held until the low-mode height, then as 2.
enum class SeatMode { Mode1, Mode2, Mode3 };
enum class SeatPhase { Waiting, Catapult, Rocket, Drogue, Separated, Main, Down };
const char* to_string(SeatPhase phase);  // German UI text
const char* to_string(SeatMode mode);

// Equivalent airspeed of a dynamic pressure (US76 sea-level density).
double equivalent_airspeed_mps(double dynamic_pressure_pa);
// Firing is survivable here: height above the ground below the ceiling and equivalent airspeed below the limit.
bool inside_seat_envelope(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa);
// The mode the sequencer latches at firing.
SeatMode seat_mode(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa);

struct SeatInput {
    double t_since_fire_s = 0.0;  // this seat's own firing
    double height_m = 0.0;        // above the ground / sea
    double dynamic_pressure_pa = 0.0;
    bool on_surface = false;
};

// One tick's actions; each event is reported once.
struct SeatActions {
    double rail_thrust_n = 0.0;  // catapult + rocket along the rail
    bool deploy_drogue = false;
    bool separate = false;       // seat-man separation: the occupant continues as a parachutist body
    bool deploy_main = false;    // on the parachutist
};

// Time-based sequence of one seat from its firing to the touchdown (mode latched at construction). Mode 1 separates
// the seat as the main deploys (no drogue).
class SeatSequencer {
public:
    SeatSequencer(const config::CrewEscapeConfig& c, SeatMode mode);
    SeatActions update(const SeatInput& in);
    SeatPhase phase() const { return phase_; }
    SeatMode mode() const { return mode_; }

private:
    const config::CrewEscapeConfig* config_;
    SeatMode mode_;
    SeatPhase phase_ = SeatPhase::Waiting;
};

}  // namespace starsim::gnc
