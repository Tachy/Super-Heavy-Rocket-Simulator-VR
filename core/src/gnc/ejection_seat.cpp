#include "starsim/gnc/ejection_seat.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/math/constants.hpp"

namespace starsim::gnc {

const char* to_string(SeatPhase phase) {
    switch (phase) {
        case SeatPhase::Waiting: return "wartet";
        case SeatPhase::Catapult: return "Katapult";
        case SeatPhase::Rocket: return "Rakete";
        case SeatPhase::Drogue: return "Bremsschirm";
        case SeatPhase::Separated: return "vom Sitz getrennt";
        case SeatPhase::Main: return "Hauptschirm";
        case SeatPhase::Down: return "gelandet";
    }
    return "";
}

const char* to_string(SeatMode mode) {
    switch (mode) {
        case SeatMode::Mode1: return "Modus 1";
        case SeatMode::Mode2: return "Modus 2";
        case SeatMode::Mode3: return "Modus 3";
    }
    return "";
}

double equivalent_airspeed_mps(double dynamic_pressure_pa) {
    return std::sqrt(2.0 * std::max(dynamic_pressure_pa, 0.0) / constants::standard_sea_level_density_kg_m3);
}

bool inside_seat_envelope(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa) {
    return height_m <= c.envelope_max_height_m && equivalent_airspeed_mps(dynamic_pressure_pa) <= c.envelope_max_eas_mps;
}

SeatMode seat_mode(const config::CrewEscapeConfig& c, double height_m, double dynamic_pressure_pa) {
    if (height_m > c.mode_low_height_m) return SeatMode::Mode3;
    return equivalent_airspeed_mps(dynamic_pressure_pa) < c.mode1_max_eas_mps ? SeatMode::Mode1 : SeatMode::Mode2;
}

SeatSequencer::SeatSequencer(const config::CrewEscapeConfig& c, SeatMode mode) : config_(&c), mode_(mode) {}

SeatActions SeatSequencer::update(const SeatInput& in) {
    const auto& c = *config_;
    SeatActions a;
    const double t = in.t_since_fire_s;
    const bool seated = phase_ == SeatPhase::Waiting || phase_ == SeatPhase::Catapult || phase_ == SeatPhase::Rocket ||
                        phase_ == SeatPhase::Drogue;
    a.rail_thrust_n = seated ? config::thrust_at(c.catapult, t) + config::thrust_at(c.rocket, t) : 0.0;
    if (phase_ == SeatPhase::Waiting) phase_ = SeatPhase::Catapult;
    if (phase_ == SeatPhase::Catapult && config::thrust_at(c.catapult, t) <= 0.0 && t > c.catapult.front().time_s) {
        phase_ = SeatPhase::Rocket;
    }
    const bool rocket_phase = phase_ == SeatPhase::Catapult || phase_ == SeatPhase::Rocket;
    if (mode_ == SeatMode::Mode1) {
        // the rocket carries seat and occupant: the separation (and the main) not before its burnout
        const double burnout_s = std::max(c.catapult.back().time_s, c.rocket.back().time_s);
        if (rocket_phase && t >= std::max(c.mode1_main_delay_s, burnout_s)) {
            a.separate = a.deploy_main = true;
            phase_ = SeatPhase::Main;
        }
    } else {
        if (rocket_phase && t >= c.mode2_drogue_delay_s) {
            a.deploy_drogue = true;
            phase_ = SeatPhase::Drogue;
        }
        // mode 3 holds the drogue until the low-mode height, then separates as mode 2 does
        const bool low = mode_ == SeatMode::Mode2 || in.height_m < c.mode_low_height_m;
        if (phase_ == SeatPhase::Drogue && !a.deploy_drogue && t >= c.mode2_separation_delay_s && low) {
            a.separate = a.deploy_main = true;
            phase_ = SeatPhase::Main;
        }
    }
    if (in.on_surface && phase_ != SeatPhase::Waiting) phase_ = SeatPhase::Down;
    return a;
}

}  // namespace starsim::gnc
