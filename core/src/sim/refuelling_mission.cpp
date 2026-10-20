#include "starsim/sim/refuelling_mission.hpp"

#include "starsim/config/mission_config.hpp"
#include "starsim/sim/ascent_mission.hpp"

namespace starsim::sim {

MissionStart start_depot_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::DepotLaunchMission& mission, const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::MissionKind::DepotLaunch;
    const auto ascent = config::load_ascent_mission(config_dir, mission.ascent_id);
    s.epoch = time.from_utc(mission.epoch_utc);
    start_ascent_mission(session, ascent, config_dir, s.epoch);
    IssMissionSequence q;
    q.from_launch = true;
    q.ascent_to_station = true;
    session.set_iss_mission(q);
    s.site_id = ascent.site_id;
    s.return_planned = true;
    return s;
}

}  // namespace starsim::sim
