#include "starsim/sim/iss_mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/config/splashdown_zone_config.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/capsule_return_mission.hpp"

namespace starsim::sim {

namespace {

using math::Vec3;
constexpr double gm = constants::earth_gm_m3ps2;

// The ISS configuration with the mission's port checked (std::runtime_error, German).
config::VehicleConfig iss_vehicle(const std::filesystem::path& config_dir, const std::string& station_id,
                                  const std::string& port, const std::string& mission_id) {
    auto iss = config::load_vehicle(config_dir, station_id);
    for (const auto& stage : iss.stages) {
        for (const auto& p : stage.docking_ports) {
            if (p.id == port) return iss;
        }
    }
    throw std::runtime_error("Mission " + mission_id + ": unbekannter Andockport '" + port + "' an " + station_id);
}

// The station's state at any epoch of one source. A scenario argument of latitude (synthetic orbit only) holds at
// `reference`; other epochs keep its offset from the J2-advanced one, so the station moves on its orbit.
struct StationSource {
    config::VehicleConfig iss;
    const orbit::OemArchive* archive = nullptr;
    const time::TimeScaleConverter* time = nullptr;
    const frames::BodyOrientation* earth = nullptr;
    IssOrbitSource source = IssOrbitSource::Auto;
    double offset_rad = 0.0;  // about the orbit normal, from the advanced argument of latitude

    StationSource(config::VehicleConfig vehicle, const orbit::OemArchive& a, const time::TimeScaleConverter& t,
                  const frames::BodyOrientation& e, IssOrbitSource s, std::optional<double> argument_of_latitude_rad,
                  time::Epoch reference)
        : iss(std::move(vehicle)), archive(&a), time(&t), earth(&e), source(s) {
        const auto advanced = start(reference);  // throws the German message without data
        if (argument_of_latitude_rad && !advanced.from_oem) {
            const auto set = iss_start(iss, a, t, e, reference, source, argument_of_latitude_rad);
            const Vec3 n = math::normalized(cross(advanced.state_gcrf.position_m, advanced.state_gcrf.velocity_mps));
            const Vec3 r0 = math::normalized(advanced.state_gcrf.position_m);
            const Vec3 r1 = math::normalized(set.state_gcrf.position_m);
            offset_rad = std::atan2(dot(cross(r0, r1), n), dot(r0, r1));
        }
    }
    StationStart start(time::Epoch epoch) const {
        auto st = iss_start(iss, *archive, *time, *earth, epoch, source);
        if (offset_rad != 0.0) {
            const auto& s = st.state_gcrf;
            const auto turn = math::Quat::from_axis_angle(math::normalized(cross(s.position_m, s.velocity_mps)), offset_rad);
            st.state_gcrf = {turn.rotate(s.position_m), turn.rotate(s.velocity_mps)};
        }
        return st;
    }
    mission::StationStateAt state_at() const {
        return [this](time::Epoch t) { return start(t).state_gcrf; };
    }
};

mission::LaunchWindowParameters window_parameters(const config::AscentMission& ascent,
                                                  const config::IssLaunchMission& launch) {
    mission::LaunchWindowParameters p;
    p.inclination_rad = ascent.inclination_rad;
    p.northerly = ascent.northerly;
    p.insertion_s = launch.window.insertion_s;
    p.insertion_arc_rad = launch.window.insertion_arc_rad;
    p.separation_periapsis_radius_m = constants::wgs84_semi_major_axis_m + ascent.periapsis_height_m;
    p.separation_apoapsis_radius_m = constants::wgs84_semi_major_axis_m + ascent.apoapsis_height_m;
    p.search_s = launch.window.search_s;
    p.phasing.mission_limit_s = launch.mission_limit_s - launch.window.insertion_s;  // from the separation
    p.phasing.earliest_arrival_s = std::max(0.0, launch.earliest_arrival_s - launch.window.insertion_s);
    return p;
}

mission::LaunchWindow window_for(const std::filesystem::path& config_dir, const config::IssLaunchMission& launch,
                                 const frames::BodyOrientation& earth, const StationSource& station,
                                 const time::TimeScaleConverter& time) {
    const auto ascent = config::load_ascent_mission(config_dir, launch.ascent_id);
    const auto site = config::load_launch_site(config_dir, ascent.site_id);
    const Vec3 pad_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    return mission::plan_launch_window(pad_itrf, earth, station.state_at(), time.from_utc(launch.earliest_utc),
                                       window_parameters(ascent, launch), gm);
}

// The 200 x 210 km separation orbit `phase_rad` behind the station at `epoch`, at its periapsis, in the station's
// plane turned by the J2 bias that makes both planes meet at the boost.
math::StateVector phased_start(const math::StateVector& iss, time::Epoch epoch, double phase_rad,
                               const mission::PhasingParameters& phasing, const frames::BodyOrientation& earth) {
    const auto e_iss = orbit::elements_from_state(iss, gm);
    orbit::KeplerianElements e;
    const double rp = constants::wgs84_semi_major_axis_m + 200e3, ra = constants::wgs84_semi_major_axis_m + 210e3;
    e.semi_major_axis_m = 0.5 * (rp + ra);  // [S] the falcon9_crew_dragon separation orbit
    e.eccentricity = (ra - rp) / (ra + rp);
    e.inclination_rad = e_iss.inclination_rad;
    e.raan_rad = e_iss.raan_rad;
    e.argument_of_periapsis_rad = e_iss.argument_of_periapsis_rad + e_iss.true_anomaly_rad - phase_rad;
    e.true_anomaly_rad = 0.0;
    const auto unbiased = orbit::state_from_elements(e, gm);
    const Vec3 pole = earth.fixed_to_gcrf(epoch).rotation * Vec3{0.0, 0.0, 1.0};
    const auto schedule = mission::phasing_schedule(unbiased, iss, epoch, phasing, gm);
    const double match_s = schedule.feasible ? schedule.boost_epoch - epoch : 0.5 * phasing.mission_limit_s;
    const auto turn = math::Quat::from_axis_angle(
        pole, mission::plane_bias_rad(e.semi_major_axis_m, e_iss.semi_major_axis_m, e.inclination_rad, match_s));
    return {turn.rotate(unbiased.position_m), turn.rotate(unbiased.velocity_mps)};
}

// The wait on the pad (StartOptions::wait_on_pad) from where the ISS comes from the same source as at the liftoff -
// the OEM file the window was planned with, or the synthetic orbit: a station placed from another file (or the synthetic
// orbit before the file's coverage) would reach the liftoff off the planned one, and IssOrbitSource::Oem has no state
// outside the coverage. Moved up to the first such epoch; none before the liftoff: no wait (nullopt).
std::optional<time::Epoch> station_consistent_wait(const StationSource& station, std::optional<time::Epoch> wait_from,
                                                   time::Epoch liftoff) {
    constexpr int max_steps = 16;   // files passed over (the archive holds a few)
    constexpr double past_s = 1.0;  // beyond a file's usable stop (its covers() includes the stop)
    if (!wait_from || station.source == IssOrbitSource::Synthetic) return wait_from;
    const orbit::Oem* at_liftoff = station.archive->for_epoch(liftoff);
    time::Epoch pad = *wait_from;
    for (int i = 0; i < max_steps && pad < liftoff; ++i) {
        const orbit::Oem* at_pad = station.archive->for_epoch(pad);
        if (at_pad == at_liftoff) return pad;
        const time::Epoch next = at_pad ? at_pad->header().usable_stop + past_s : at_liftoff->header().usable_start;
        if (!(pad < next)) break;
        pad = next;
    }
    return std::nullopt;
}

// German: the station impulses (reboosts) in [from, to].
void reboost_warnings(const StationStart& st, time::Epoch from, time::Epoch to, const time::TimeScaleConverter& time,
                      std::vector<std::string>& out) {
    for (const auto& impulse : st.impulses) {
        if (impulse.epoch < from || impulse.epoch > to) continue;
        char dv[32];
        std::snprintf(dv, sizeof dv, "%.2f m/s", impulse.delta_v_mps);
        out.push_back("Bahnmanöver der ISS um " + time.to_utc(impulse.epoch).to_iso_string().substr(0, 19) +
                      " UTC: " + dv + " (" + impulse.name + ")");
    }
}

// The Dragon docked at `port` at `epoch`: placed near the station, the station added, the nose cone open, the docking
// mechanism armed and mated at once (FlightSession::dock_in_place).
StationStart place_docked(FlightSession& session, const config::VehicleConfig& dragon, const StationSource& station,
                          const std::string& port, time::Epoch epoch) {
    const auto st = station.start(epoch);
    std::string chaser_port;
    for (const auto& stage : dragon.stages) {
        if (!stage.docking_ports.empty() && chaser_port.empty()) chaser_port = stage.docking_ports.front().id;
    }
    if (chaser_port.empty()) throw std::runtime_error("Das Fahrzeug " + dragon.id + " hat keinen Andockport");
    // anywhere near the station (dock_in_place moves it to the mated pose)
    const Vec3 radial = math::normalized(st.state_gcrf.position_m);
    session.place_in_orbit(dragon, {st.state_gcrf.position_m - 100.0 * radial, st.state_gcrf.velocity_mps}, epoch);
    const size_t index = session.add_station(st.vehicle, st.state_gcrf, st.epoch, st.impulses);
    session.set_nose_cone(session.controlled_index(), true);
    session.set_rendezvous_target(index, chaser_port, port);
    session.dock_in_place();
    return st;
}

}  // namespace

orbit::OemArchive iss_archive(const std::filesystem::path& config_dir, const time::TimeScaleConverter& time) {
    return orbit::OemArchive::scan(config_dir.parent_path() / "data" / "iss_oem", time);
}

mission::LaunchWindow iss_launch_window(const FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::IssLaunchMission& mission, const orbit::OemArchive& archive,
                                        const time::TimeScaleConverter& time) {
    const StationSource station(iss_vehicle(config_dir, mission.station_id, mission.port, mission.id), archive, time,
                                session.earth_orientation(), parse_iss_orbit_source(mission.iss_orbit), std::nullopt,
                                time.from_utc(mission.earliest_utc));
    return window_for(config_dir, mission, session.earth_orientation(), station, time);
}

MissionStart start_iss_rendezvous_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                          const config::IssRendezvousMission& mission,
                                          const orbit::OemArchive& archive, const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::MissionKind::IssRendezvous;
    const auto& earth = session.earth_orientation();
    const time::Epoch requested = time.from_utc(mission.epoch_utc);
    const StationSource station(iss_vehicle(config_dir, mission.station_id, mission.port, mission.id), archive, time,
                                earth, parse_iss_orbit_source(mission.iss_orbit), mission.iss_argument_of_latitude_rad,
                                requested);
    mission::PhasingParameters phasing;
    phasing.mission_limit_s = mission.mission_limit_s;
    phasing.earliest_arrival_s = mission.earliest_arrival_s;
    math::StateVector dragon;
    if (mission.phase_rad) {
        s.epoch = requested;
        dragon = phased_start(station.start(requested).state_gcrf, requested, *mission.phase_rad, phasing, earth);
    } else {
        auto launch = config::load_iss_launch_mission(config_dir, mission.launch_id);
        launch.earliest_utc = mission.epoch_utc;
        launch.mission_limit_s = mission.mission_limit_s;
        launch.earliest_arrival_s = mission.earliest_arrival_s;
        const auto w = window_for(config_dir, launch, earth, station, time);
        s.launch_window = w;
        if (!w.feasible) {
            s.refusal = w.reason;
            return s;
        }
        s.epoch = w.separation;
        dragon = w.separation_state_gcrf;
        phasing.mission_limit_s = mission.mission_limit_s - launch.window.insertion_s;
        phasing.earliest_arrival_s = std::max(0.0, mission.earliest_arrival_s - launch.window.insertion_s);
    }
    const auto st = station.start(s.epoch);
    session.place_in_orbit(config::load_vehicle(config_dir, mission.vehicle_id), dragon, s.epoch);
    session.add_station(st.vehicle, st.state_gcrf, st.epoch, st.impulses);
    s.warnings.push_back(st.note);
    if (!session.plan_rendezvous(phasing)) {
        s.refusal = session.rendezvous_status();
        session.clear_bodies();  // the mission does not start
        return s;
    }
    for (const auto& w : session.rendezvous_warnings()) s.warnings.push_back(w);
    IssMissionSequence q;
    q.station_port = mission.port;
    q.phasing = phasing;
    session.set_iss_mission(q);
    s.return_planned = true;
    return s;
}

MissionStart start_iss_launch_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                      const config::IssLaunchMission& mission, const orbit::OemArchive& archive,
                                      const time::TimeScaleConverter& time, std::optional<time::Epoch> wait_from) {
    MissionStart s;
    s.kind = config::MissionKind::IssLaunch;
    const auto& earth = session.earth_orientation();
    const StationSource station(iss_vehicle(config_dir, mission.station_id, mission.port, mission.id), archive, time,
                                earth, parse_iss_orbit_source(mission.iss_orbit), std::nullopt,
                                time.from_utc(mission.earliest_utc));
    const auto w = window_for(config_dir, mission, earth, station, time);
    s.launch_window = w;
    if (!w.feasible) {
        s.refusal = w.reason;
        return s;
    }
    const auto ascent = config::load_ascent_mission(config_dir, mission.ascent_id);
    const auto launch = start_ascent_mission_for_liftoff(session, ascent, config_dir, w.liftoff,
                                                         station_consistent_wait(station, wait_from, w.liftoff));
    s.epoch = launch.placed;
    if (!launch.profile.feasible) {
        s.refusal = launch.profile.reason;
        return s;
    }
    const auto st = station.start(s.epoch);
    session.add_station(st.vehicle, st.state_gcrf, st.epoch, st.impulses);  // after the launch (it clears the bodies)
    IssMissionSequence q;
    q.station_port = mission.port;
    q.from_launch = true;
    q.phasing.mission_limit_s = mission.mission_limit_s - mission.window.insertion_s;  // from the separation
    q.phasing.earliest_arrival_s = std::max(0.0, mission.earliest_arrival_s - mission.window.insertion_s);
    session.set_iss_mission(q);
    s.site_id = ascent.site_id;
    s.warnings.push_back(st.note);
    reboost_warnings(st, w.liftoff, w.liftoff + mission.mission_limit_s, time, s.warnings);
    s.return_planned = true;
    return s;
}

MissionStart start_iss_departure_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::IssDepartureMission& mission, const orbit::OemArchive& archive,
                                         const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::MissionKind::IssDeparture;
    const time::Epoch earliest = time.from_utc(mission.epoch_utc);
    const StationSource station(iss_vehicle(config_dir, mission.station_id, mission.port, mission.id), archive, time,
                                session.earth_orientation(), parse_iss_orbit_source(mission.iss_orbit),
                                mission.iss_argument_of_latitude_rad, earliest);
    auto dragon = config::load_vehicle(config_dir, mission.vehicle_id);
    for (auto& stage : dragon.stages) {
        for (auto& tank : stage.tanks) {
            if (tank.contents == config::TankContents::Rcs) tank.mass_kg = mission.rcs_propellant_kg;
        }
    }
    const auto ret = config::load_capsule_return_mission(config_dir, mission.return_id);
    const auto zone = config::load_splashdown_zone(config_dir, ret.zone_id);
    auto p = capsule_return_parameters(ret);
    p.search_window_s = mission.search_window_s;
    // the probe: docked at the earliest undocking, the first pass at least undock_to_deorbit_s later
    s.epoch = earliest;
    place_docked(session, dragon, station, mission.port, earliest);
    session.set_splashdown_zone(zone);
    auto probe = p;
    probe.lead_s = mission.undock_to_deorbit_s;
    const auto pass = session.capsule_return_preview(probe);
    if (!pass.feasible) {
        s.refusal = "Keine Rückkehrgelegenheit: " + pass.reason;
        session.clear_bodies();  // the probe placement goes: the mission does not start
        return s;
    }
    const time::Epoch undock = pass.node.epoch - mission.undock_to_deorbit_s;
    s.epoch = undock - FlightSession::iss_departure_start_lead_s;
    const auto st = place_docked(session, dragon, station, mission.port, s.epoch);
    session.set_splashdown_zone(zone);
    IssMissionSequence q;
    q.station_port = mission.port;
    q.undock_epoch = undock;
    q.capsule_return = p;
    q.deorbit_epoch = pass.node.epoch;
    session.set_iss_mission(q);
    s.warnings.push_back(st.note);
    reboost_warnings(st, s.epoch, pass.node.epoch, time, s.warnings);
    s.return_planned = true;
    return s;
}

}  // namespace starsim::sim
