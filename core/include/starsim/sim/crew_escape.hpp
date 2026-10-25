#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/ejection_seat.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::sim {

// Crew escape of a crewed ship return (docs/superpowers/archive/specs/2026-10-22-crew-escape-design.md): what the session
// reports per crew member and for the given-up ship. German reasons and UI texts.
enum class CrewOutcome { Flying, Rescued, Lost };
const char* to_string(CrewOutcome outcome);

struct CrewMemberState {
    std::optional<size_t> body_index;  // seat, then parachutist body; empty before this seat fired
    gnc::SeatPhase phase = gnc::SeatPhase::Waiting;
    gnc::SeatMode mode = gnc::SeatMode::Mode1;
    double fire_time_s = 0.0;          // mission time of this seat's firing
    double load_g = 0.0;               // specific force now
    double peak_load_g = 0.0;
    double min_clearance_m = 1e9;      // smallest distance to the ship hull, its plume or the tower so far
    std::optional<math::Vec3> touchdown_enu_m;  // site frame (east, north, up from the pad centre)
    bool touchdown_over_water = false;
    double touchdown_sink_mps = 0.0;
    CrewOutcome outcome = CrewOutcome::Flying;
    std::string reason;                // why lost (empty otherwise)
};

struct CrewEscapeState {
    bool armed = false;                // the onboard computer gave the ship up
    bool fired = false;                // the first seat fired
    std::string give_up_reason;
    double give_up_time_s = 0.0;       // mission time
    bool early = false;                // given up in the entry above the minimum firing height: fire over water
    std::vector<CrewMemberState> members;
    std::optional<math::Vec3> ship_impact_enu_m;
    double ship_impact_speed_mps = 0.0;
    double ship_impact_tower_distance_m = 0.0;
    double ship_impact_shore_distance_m = 0.0;  // > 0: beyond the shoreline (seaward)
    bool ship_impact_over_water = false;
};

// Distance of `point` from a hull capsule (axis bottom..top, radius); negative inside.
double hull_clearance_m(const math::Vec3& bottom, const math::Vec3& top, double radius_m, const math::Vec3& point);
// Distance of `point` from an exhaust cone (apex at the engines, `axis` unit along the exhaust, half angle, length);
// 0 inside.
double plume_clearance_m(const math::Vec3& apex, const math::Vec3& axis, double half_angle_rad, double length_m,
                         const math::Vec3& point);

// Sustained load: true once the load stayed above `max_g` for longer than `max_s` (time above accumulates while above,
// resets below).
class LoadJudge {
public:
    bool exceeded(double load_g, double dt_s, double max_g, double max_s);

private:
    double above_s_ = 0.0;
};

}  // namespace starsim::sim
