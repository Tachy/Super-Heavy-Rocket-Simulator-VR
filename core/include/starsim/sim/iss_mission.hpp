#pragma once

#include <filesystem>

#include "starsim/config/mission_config.hpp"
#include "starsim/mission/iss_launch_window.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

// data/iss_oem next to config/ (a missing directory: an empty archive).
orbit::OemArchive iss_archive(const std::filesystem::path& config_dir, const time::TimeScaleConverter& time);

// The launch window of an iss_launch mission from its earliest_utc: the site, target inclination and separation orbit
// of its ascent, the ISS of the mission's iss_orbit from `archive`, the phasing limit counted from the separation.
// Throws std::runtime_error (German) for an ISS source without data (iss_start).
mission::LaunchWindow iss_launch_window(const FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::IssLaunchMission& mission, const orbit::OemArchive& archive,
                                        const time::TimeScaleConverter& time);

// start_mission's iss_rendezvous branch: the Dragon at the separation of the first window of the mission's launch
// after epoch_utc (or phase_rad behind the ISS at epoch_utc, its plane J2-biased to the boost), the ISS of iss_orbit,
// the phasing planned (plan_rendezvous) and the sequence set (phasing -> proximity -> docking). Refused (German
// refusal, return_planned false): no window or the planner's reason. Throws std::runtime_error (German) for an
// unknown port or an ISS source without data, before anything is placed.
MissionStart start_iss_rendezvous_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                          const config::IssRendezvousMission& mission,
                                          const orbit::OemArchive& archive, const time::TimeScaleConverter& time);

// start_mission's iss_launch branch: the launch window from earliest_utc, then the ascent mission at the window's
// liftoff, the ISS added, the sequence set (ascent -> phasing after the separation -> proximity -> docking). Epoch:
// the liftoff. Refused (German refusal, nothing launched): no window. Warnings: the ISS source note and station
// reboosts between the liftoff and the mission limit.
MissionStart start_iss_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                      const config::IssLaunchMission& mission, const orbit::OemArchive& archive,
                                      const time::TimeScaleConverter& time);

// start_mission's iss_departure branch: the first capsule-return pass at least undock_to_deorbit_s after epoch_utc
// (previewed on the docked Dragon), the Dragon docked at the port (dock_in_place) iss_departure_start_lead_s before the
// undocking (that pass - undock_to_deorbit_s), the sequence set (undocking -> departure -> capsule return of
// return_id, aimed at that pass). Epoch: the start. Refused (German refusal): no return pass in the search window
// (the probe placement stays). Throws std::runtime_error (German) for an unknown port or an ISS source without data.
MissionStart start_iss_departure_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::IssDepartureMission& mission, const orbit::OemArchive& archive,
                                         const time::TimeScaleConverter& time);

}  // namespace starsim::sim
