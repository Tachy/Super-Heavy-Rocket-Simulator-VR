#include "starsim/sim/refuelling_mission.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/vehicle/propellant_transfer.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"

namespace starsim::sim {

namespace {

using math::Vec3;
constexpr double gm = constants::earth_gm_m3ps2;
constexpr double equatorial_radius_m = constants::wgs84_semi_major_axis_m;
// [E] an autonomous refuelling flight resumes the approach this often after a failed capture (r5: the light HLS
// bounced off the depot's drogues once), then the mission ends
constexpr int capture_retries = 2;
// the single Raptor of the raise, phasing and deorbit (r3, r5) and the depot's reboost [E]
constexpr const char* phasing_raptor_group = "sea_level_3";

// The depot orbit through a GCRF parking state at `at` (hls_refuel): its plane in the true equator frame of `at`, run
// back by the J2 rates to `earliest`; the config's argument of latitude holds at `earliest`.
config::DepotOrbitConfig depot_in_plane(const config::DepotOrbitConfig& depot, const math::StateVector& parking_gcrf,
                                        time::Epoch at, time::Epoch earliest, const frames::BodyOrientation& earth) {
    const math::Mat3 to_tod = transpose(orbit::true_equator_to_gcrf(earth, at));
    const auto e = orbit::elements_from_state({to_tod * parking_gcrf.position_m, to_tod * parking_gcrf.velocity_mps}, gm);
    orbit::StationMeanElements mean;
    mean.epoch = at;
    mean.semi_major_axis_m = equatorial_radius_m + depot.height_m;
    mean.inclination_rad = e.inclination_rad;
    mean.raan_rad = e.raan_rad;
    mean.argument_of_latitude_rad = e.argument_of_periapsis_rad + e.true_anomaly_rad;
    const auto back = orbit::advance_mean_elements(mean, earliest, gm, constants::earth_j2, equatorial_radius_m);
    auto out = depot;
    out.inclination_rad = back.inclination_rad;
    out.raan_rad = back.raan_rad;
    return out;
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

double phasing_exhaust_velocity_mps(const vehicle::Vehicle& v) {
    // engine groups are qualified by their stage: "ship/sea_level_3"
    const std::string group = v.stages().back().config.id + "/" + phasing_raptor_group;
    return gnc::groups_vacuum_isp_s(v, {group}) * constants::standard_gravity_mps2;
}

double phasing_settling_gas_kg(const vehicle::Vehicle& v, bool ignition_settling) {
    return ignition_settling ? vehicle::settling_gas_bound_kg(v, v.stages().size() - 1) : 0.0;
}

mission::LaunchWindowParameters refuelling_window_parameters(const config::RefuellingFlightMission& m,
                                                             const config::AscentMission& ascent,
                                                             double exhaust_velocity_mps, double gas_per_burn_kg,
                                                             bool reference_phasing) {
    mission::LaunchWindowParameters p;
    p.inclination_rad = ascent.inclination_rad;
    p.northerly = ascent.northerly;
    p.insertion_s = m.window.insertion_s;
    p.insertion_arc_rad = m.window.insertion_arc_rad;
    // f1: the separation on the insertion orbit; the raise to the phasing orbit is part of the window's estimate
    p.separation_periapsis_radius_m = equatorial_radius_m + ascent.periapsis_height_m;
    p.separation_apoapsis_radius_m = equatorial_radius_m + ascent.apoapsis_height_m;
    if (m.phasing_periapsis_height_m && (reference_phasing || m.phasing_candidates_m.empty())) {
        p.phasing_orbits = {{equatorial_radius_m + *m.phasing_periapsis_height_m,
                             equatorial_radius_m + ascent.apoapsis_height_m}};
    } else {
        for (const auto& [periapsis_m, apoapsis_m] : m.phasing_candidates_m) {
            p.phasing_orbits.push_back({equatorial_radius_m + periapsis_m, equatorial_radius_m + apoapsis_m});
        }
    }
    p.cost = {m.planning_mass_kg, exhaust_velocity_mps, m.planning_boil_off_kgps, gas_per_burn_kg};
    p.search_s = m.window.search_s;
    // from the orbit insertion, window_margin_s short of the limit: the flown raise and the drag of a long phasing
    // move the arrival (f1: a window needing 47.0 of 48 h no longer closed at the re-plan 40 h in)
    p.phasing.mission_limit_s = m.mission_limit_s - m.window.insertion_s - m.window_margin_s;
    p.phasing.earliest_arrival_s = std::max(0.0, m.earliest_arrival_s - m.window.insertion_s);
    return p;
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
    // hls_refuel: the depot orbit lies in the lunar mission's parking plane at its epoch (r5 plan decision 8)
    config::LunarLandingMission lunar;
    time::Epoch lunar_epoch{};
    auto depot_cfg = m.depot;
    if (m.kind == config::MissionKind::HlsRefuel) {
        if (!session.moon_orientation()) {
            s.refusal = "HLS-Betankung: die Sitzung hat keine Mondorientierung";
            return s;
        }
        lunar = config::load_lunar_landing_mission(config_dir, m.lunar_mission_id);
        if (std::abs(lunar.parking_inclination_rad - m.depot.inclination_rad) > 1e-9) {
            s.refusal = "HLS-Betankung: Depotbahn und Parkbahn der Mondmission haben verschiedene Neigungen";
            return s;
        }
        lunar_epoch = time.from_utc(lunar.epoch_utc);
        if (earliest >= lunar_epoch) {
            s.refusal = "HLS-Betankung: frühester Start nach dem Start der Mondmission";
            return s;
        }
        const auto parking = lunar_parking_state(session, lunar, lunar_epoch, lunar.parking_inclination_rad);
        if (!parking) {
            s.refusal = "Kein Parkorbit durch die Mondbahnebene: Bahnneigung unter der Deklination des Mondes";
            return s;
        }
        depot_cfg = depot_in_plane(m.depot, *parking, lunar_epoch, earliest, earth);
    }
    const auto depot_at = depot_orbit(depot_cfg, earliest, earth);
    const auto ascent = config::load_ascent_mission(config_dir, m.ascent_id);
    const auto site = config::load_launch_site(config_dir, ascent.site_id);
    auto depot_vehicle = config::top_stage_config(config::load_vehicle(config_dir, m.depot.vehicle_id));
    fill_main_tanks(depot_vehicle.stages.back(), m.depot.propellant_kg);
    // the orbit start's phasing orbit: the ascent's target, or with the reference perigee (r5)
    const double phasing_periapsis_m = m.phasing_periapsis_height_m.value_or(ascent.periapsis_height_m);
    IssMissionSequence q;
    q.station_port = m.depot.port;
    q.max_capture_retries = capture_retries;
    if (m.start == config::RefuellingStart::Pad) {
        const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
        // the phasing engine of the chaser alone, tanks as configured (an empty vehicle's engines count as starved)
        const vehicle::Vehicle chaser(config::top_stage_config(config::load_vehicle(config_dir, ascent.vehicle_id)),
                                      config_dir);
        const auto p = refuelling_window_parameters(m, ascent, phasing_exhaust_velocity_mps(chaser),
                                                    phasing_settling_gas_kg(chaser, session.ignition_settling()));
        const auto w = mission::plan_launch_window(pad, earth, depot_at, earliest, p, gm);
        s.launch_window = w;
        if (!w.feasible) {
            s.refusal = w.reason;
            return s;
        }
        if (m.kind == config::MissionKind::HlsRefuel && w.liftoff >= lunar_epoch) {
            s.refusal = "HLS-Betankung: Startfenster nach dem Start der Mondmission";
            return s;
        }
        start_ascent_mission(session, ascent, config_dir, w.liftoff);
        session.add_station(depot_vehicle, depot_at(w.liftoff), w.liftoff);  // after the launch (it clears the bodies)
        q.from_launch = true;
        q.phasing = p.phasing;
        q.phasing.mission_limit_s += m.window_margin_s;  // the flown phasing keeps the full limit
        q.raise_to = w.phasing_orbit;  // f1: the window's choice (none: the insertion orbit is the phasing orbit)
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
    } else {  // hls_refuel: the lunar landing mission from the refuelled HLS at its epoch
        q.continue_epoch = lunar_epoch;
        q.continuation = [config_dir, lunar, lunar_epoch](FlightSession& f) {
            return continue_lunar_landing_mission(f, config_dir, lunar, lunar_epoch).refusal;
        };
    }
    session.set_iss_mission(q);
    s.return_planned = true;
    return s;
}

}  // namespace starsim::sim
