#include "starsim/sim/refuelling_mission.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/ascent_mission.hpp"

namespace starsim::sim {

namespace {

using math::Vec3;
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double equatorial_radius_m = constants::wgs84_semi_major_axis_m;

// The stage's main oxidizer and fuel tanks filled with `kg` in the ratio of their capacities (capped at capacity).
void fill_main_tanks(config::StageConfig& stage, double kg) {
    double capacity = 0.0;
    for (const auto& tank : stage.tanks) {
        if (tank.role == config::TankRole::Main) capacity += tank.capacity_kg;
    }
    for (auto& tank : stage.tanks) {
        if (tank.role != config::TankRole::Main) continue;
        tank.mass_kg = capacity > 0.0 ? std::min(tank.capacity_kg, kg * tank.capacity_kg / capacity) : 0.0;
    }
}

}  // namespace

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

mission::StationStateAt depot_orbit(const config::DepotOrbitConfig& depot, time::Epoch reference,
                                    const frames::BodyOrientation& earth) {
    orbit::StationMeanElements m;
    m.epoch = reference;
    m.semi_major_axis_m = equatorial_radius_m + depot.height_m;
    m.inclination_rad = depot.inclination_rad;
    m.raan_rad = depot.raan_rad;
    m.argument_of_latitude_rad = depot.argument_of_latitude_rad;
    return [m, &earth](time::Epoch t) {
        const auto at = orbit::advance_mean_elements(m, t, gm, constants::earth_j2, equatorial_radius_m);
        return orbit::synthetic_station_state(at, gm, earth);
    };
}

math::StateVector phasing_start_state(const math::StateVector& station, double behind_rad, double periapsis_height_m,
                                      double apoapsis_height_m) {
    const Vec3 n = math::normalized(cross(station.position_m, station.velocity_mps));
    const auto back = math::Quat::from_axis_angle(n, -behind_rad);
    const Vec3 r_hat = math::normalized(back.rotate(station.position_m));
    const Vec3 v_hat = cross(n, r_hat);
    const double rp = equatorial_radius_m + periapsis_height_m;
    const double ra = equatorial_radius_m + apoapsis_height_m;
    const double a = 0.5 * (rp + ra);
    const double vp = std::sqrt(gm * (2.0 / rp - 1.0 / a));
    return {rp * r_hat, vp * v_hat};
}

gnc::DeorbitParameters ship_deorbit_parameters(const config::ShipReturnDeorbit& d) {
    gnc::DeorbitParameters p;
    p.entry_interface_height_m = d.entry_interface_height_m;
    p.entry_flight_path_rad = d.entry_flight_path_rad;
    p.nominal_bank_rad = d.nominal_bank_rad;
    p.max_crossrange_m = d.max_crossrange_m;
    p.search_window_s = FlightSession::tanker_reserve_search_s;
    return p;
}

MissionStart start_refuelling_flight_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                             const config::RefuellingFlightMission& m,
                                             const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = m.kind;
    const auto& earth = session.earth_orientation();
    const time::Epoch earliest = time.from_utc(m.earliest_utc);
    const auto depot_at = depot_orbit(m.depot, earliest, earth);
    const auto ascent = config::load_ascent_mission(config_dir, m.ascent_id);
    const auto site = config::load_launch_site(config_dir, ascent.site_id);
    auto depot_vehicle = config::top_stage_config(config::load_vehicle(config_dir, m.depot.vehicle_id));
    fill_main_tanks(depot_vehicle.stages.back(), m.depot.propellant_kg);
    // the phasing orbit: the ascent's target, or with its perigee raised at apoapsis (user 2026-10-20: the tanker)
    const double phasing_periapsis_m = m.phasing_periapsis_height_m.value_or(ascent.periapsis_height_m);
    IssMissionSequence q;
    q.station_port = m.depot.port;
    if (m.start == config::RefuellingStart::Pad) {
        const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
        mission::LaunchWindowParameters p;
        p.inclination_rad = ascent.inclination_rad;
        p.northerly = ascent.northerly;
        p.insertion_s = m.window.insertion_s;
        p.insertion_arc_rad = m.window.insertion_arc_rad;
        p.separation_periapsis_radius_m = equatorial_radius_m + phasing_periapsis_m;  // the phasing after the raise
        p.separation_apoapsis_radius_m = equatorial_radius_m + ascent.apoapsis_height_m;
        p.search_s = m.window.search_s;
        p.phasing.mission_limit_s = m.mission_limit_s - m.window.insertion_s;  // from the orbit insertion
        p.phasing.earliest_arrival_s = std::max(0.0, m.earliest_arrival_s - m.window.insertion_s);
        const auto w = mission::plan_launch_window(pad, earth, depot_at, earliest, p, gm);
        s.launch_window = w;
        if (!w.feasible) {
            s.refusal = w.reason;
            return s;
        }
        start_ascent_mission(session, ascent, config_dir, w.liftoff);
        session.add_station(depot_vehicle, depot_at(w.liftoff), w.liftoff);  // after the launch (it clears the bodies)
        q.from_launch = true;
        q.phasing = p.phasing;
        if (m.phasing_periapsis_height_m) q.raise_periapsis_radius_m = equatorial_radius_m + phasing_periapsis_m;
        s.epoch = w.liftoff;
        s.site_id = ascent.site_id;
    } else {
        mission::PhasingParameters phasing;
        phasing.mission_limit_s = m.mission_limit_s;
        phasing.earliest_arrival_s = m.earliest_arrival_s;
        auto chaser = config::top_stage_config(config::load_vehicle(config_dir, ascent.vehicle_id));
        fill_main_tanks(chaser.stages.back(), m.start_propellant_kg);
        const auto depot = depot_at(earliest);
        session.place_in_orbit(
            chaser, phasing_start_state(depot, m.phasing_behind_rad, phasing_periapsis_m, ascent.apoapsis_height_m),
            earliest);
        session.set_launch_site(launch_site_from_config(site));  // the return site
        session.add_station(depot_vehicle, depot, earliest);
        if (!session.plan_rendezvous(phasing)) {
            s.refusal = session.rendezvous_status();
            session.clear_bodies();  // the mission does not start
            return s;
        }
        q.phasing = phasing;
        s.epoch = earliest;
    }
    q.transfer = m.kind == config::MissionKind::TankerFlight ? MissionTransfer::TankerReserve
                                                             : MissionTransfer::FillChaser;
    if (m.kind == config::MissionKind::TankerFlight) {
        q.ship_return = ship_deorbit_parameters(config::load_ship_return_mission(config_dir, m.return_id).deorbit);
    }
    session.set_iss_mission(q);
    s.return_planned = true;
    return s;
}

}  // namespace starsim::sim
