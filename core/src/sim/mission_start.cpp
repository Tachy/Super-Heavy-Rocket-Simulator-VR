#include "starsim/sim/mission_start.hpp"

#include <stdexcept>

#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/capsule_return_mission.hpp"
#include "starsim/sim/iss_mission.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/lunar_return_mission.hpp"
#include "starsim/sim/refuelling_campaign_flight.hpp"
#include "starsim/sim/refuelling_mission.hpp"

namespace starsim::sim {

MissionStart start_mission(FlightSession& session, const std::filesystem::path& config_dir, std::string_view mission_id,
                           time::Epoch requested, const time::TimeScaleConverter& time) {
    session.set_supervisor(nullptr);  // refuelling f2: a new mission ends a campaign
    MissionStart s;
    s.kind = config::mission_kind(config_dir, mission_id);
    session.set_boil_off(true);  // a mission without its own setting: boil-off on (a lunar return sets its own)
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
        case config::MissionKind::LunarLanding: {
            const auto m = config::load_lunar_landing_mission(config_dir, mission_id);
            s = start_lunar_landing_mission(session, config_dir, m, time.from_utc(m.epoch_utc));
            break;
        }
        case config::MissionKind::LunarReturn: {
            const auto m = config::load_lunar_return_mission(config_dir, mission_id);
            s = start_lunar_return_mission(session, config_dir, m, time.from_utc(m.epoch_utc));
            break;
        }
        case config::MissionKind::IssRendezvous:
            s = start_iss_rendezvous_mission(session, config_dir,
                                             config::load_iss_rendezvous_mission(config_dir, mission_id),
                                             iss_archive(config_dir, time), time);
            break;
        case config::MissionKind::IssLaunch:
            s = start_iss_launch_mission(session, config_dir, config::load_iss_launch_mission(config_dir, mission_id),
                                         iss_archive(config_dir, time), time);
            break;
        case config::MissionKind::IssDeparture:
            s = start_iss_departure_mission(session, config_dir,
                                            config::load_iss_departure_mission(config_dir, mission_id),
                                            iss_archive(config_dir, time), time);
            break;
        case config::MissionKind::DepotLaunch:
            s = start_depot_launch_mission(session, config_dir, config::load_depot_launch_mission(config_dir, mission_id),
                                           time);
            break;
        case config::MissionKind::TankerFlight:
        case config::MissionKind::HlsRefuel:
            s = start_refuelling_flight_mission(session, config_dir,
                                                config::load_refuelling_flight_mission(config_dir, mission_id), time);
            break;
        case config::MissionKind::RefuellingCampaignFlown:
            s = start_refuelling_campaign_flight(
                session, config_dir, config::load_refuelling_campaign_flown_mission(config_dir, mission_id), time);
            break;
        case config::MissionKind::ShipReturn:
            throw std::invalid_argument("mission '" + std::string(mission_id) +
                                        "' is a ship return: it is flown from orbit with plan_ship_return (key R), "
                                        "not started");
    }
    return s;
}

}  // namespace starsim::sim
