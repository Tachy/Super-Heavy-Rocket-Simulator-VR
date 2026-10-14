#include "starsim/sim/mission_start.hpp"

#include <stdexcept>

#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/capsule_return_mission.hpp"

namespace starsim::sim {

MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::mission_kind(config_dir, mission_id);
    switch (s.kind) {
        case config::MissionKind::Ascent: {
            const auto m = config::load_ascent_mission(config_dir, mission_id);
            start_ascent_mission(session, m, config_dir, requested);
            s.site_id = m.site_id;
            s.epoch = requested;
            break;
        }
        case config::MissionKind::CapsuleReturn: {
            const auto m = config::load_capsule_return_mission(config_dir, mission_id);
            s.epoch = time.from_utc(m.epoch_utc);
            place_capsule_return_mission(session, m, config_dir, s.epoch);
            s.return_planned = session.plan_capsule_return(capsule_return_parameters(m));
            break;
        }
        case config::MissionKind::ShipReturn:
            throw std::invalid_argument("mission '" + std::string(mission_id) +
                                        "' is a ship return: it is flown from orbit with plan_ship_return (key R), "
                                        "not started");
    }
    return s;
}

}  // namespace starsim::sim
