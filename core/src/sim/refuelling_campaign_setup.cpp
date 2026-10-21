#include "starsim/sim/refuelling_campaign_setup.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/environment/atmosphere.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_budget.hpp"
#include "starsim/mission/refuelling_costs.hpp"
#include "starsim/mission/tanker_reserve.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/refuelling_mission.hpp"
#include "starsim/vehicle/boil_off.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::sim {

namespace {

constexpr double gm = constants::earth_gm_m3ps2;
constexpr double equatorial_radius_m = constants::wgs84_semi_major_axis_m;
constexpr double deorbit_perigee_height_m = 50.0e3;  // [E] the return's entry perigee; only differences count
// [E] the campaign's window search from each earliest launch: the depot phase over Starbase's crossings repeats in
// weeks (the missions' 12 days found no second 200 km window; at 300 km they came 5 and 12 days apart)
constexpr double campaign_window_search_s = 60.0 * 86400.0;
// [E] review I1: later launches reaching Waypoint 0 within this of the first window's arrival fly the same rendezvous
// with a shorter phasing (the next arrival comes ~14 days later); they are probed over this span from each launch
constexpr double same_arrival_s = 3.0 * 3600.0;
constexpr double later_launch_search_s = 3.0 * 86400.0;
constexpr double later_launch_step_s = 3600.0;  // [E] probe step of later launches to the same Waypoint 0

// The vehicle's top stage alone, tanks as configured (full: built empty, its engines would count as starved and
// phasing_exhaust_velocity_mps would read 0).
config::VehicleConfig top_stage(const std::filesystem::path& dir, const std::string& id) {
    return config::top_stage_config(config::load_vehicle(dir, id));
}

// The main tanks at `kg`, split by their capacities (as fill_main_tanks).
void set_main(vehicle::Vehicle& v, double kg) {
    double capacity = 0.0;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) capacity += t.config.capacity_kg;
    }
    const size_t stage = v.stages().size() - 1;
    std::vector<std::pair<std::string, double>> masses;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) masses.emplace_back(t.config.id, kg * t.config.capacity_kg / capacity);
    }
    for (const auto& [id, mass] : masses) v.set_tank_mass(stage, id, mass);
}

double main_capacity_kg(const vehicle::Vehicle& v) {
    double capacity = 0.0;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) capacity += t.config.capacity_kg;
    }
    return capacity;
}

// Orbit-averaged boil-off (both propellants) with the main tanks at `main_kg`; the vehicle flies the depot's attitude
// (identity in LVLH: nose along +V, dorsal to nadir).
double orbit_boil_off_kgps(vehicle::Vehicle& v, double main_kg, const math::StateVector& orbit,
                           const math::Vec3& sun_gcrf_m) {
    set_main(v, main_kg);
    const auto rate = vehicle::orbit_average_boil_off(v, orbit, {}, sun_gcrf_m);
    return rate.oxidizer_kgps + rate.fuel_kgps;
}

std::string tonnes(double kg) {
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, "%.1f t", kg / 1000.0);
    return buffer;
}

struct HlsDemand {
    std::string refusal;  // German; empty when planned
    CampaignHlsDemand values;
};

// The smallest HLS mass that closes the lunar mission's budget from its planned outbound flight (plan decision 2).
HlsDemand hls_demand(const CampaignSources& src, const config::LunarLandingMission& lunar, time::Epoch lunar_epoch,
                     const config::VehicleConfig& hls_full, bool ignition_settling) {
    HlsDemand d;
    FlightSession session(src.ephemeris, src.earth, src.config_dir, environment::PointMassGravityConfig{}, &src.moon);
    session.set_ignition_settling(ignition_settling);  // the same flag as the windows' gas (f3 minor M7)
    try {
        place_lunar_landing_mission(session, lunar, src.config_dir, lunar_epoch, &hls_full);
    } catch (const std::runtime_error& e) {  // no plane, terrain files missing (German)
        d.refusal = e.what();
        return d;
    }
    if (!session.plan_lunar_outbound(lunar_outbound_parameters(lunar, session.gravity(), session.ephemeris(),
                                                               lunar_epoch))) {
        d.refusal = session.lunar_status();
        return d;
    }
    const auto& v = session.body(session.controlled_index()).vehicle();
    const auto& sets = lunar_landing_setup(lunar).guidance.braking_sets;
    const auto legs = lunar_mission_legs(lunar, *session.lunar_outbound_plan(), gnc::groups_vacuum_isp_s(v, sets.front()),
                                         gnc::groups_vacuum_isp_s(v, sets.back()));
    auto& out = d.values;
    out.capacity_kg = v.propellant_kg();
    out.headers_kg = v.propellant_kg() - v.main_propellant_kg();
    out.dry_mass_kg = v.mass_properties().mass_kg - v.propellant_kg();
    // f3: each in-orbit ignition settles first (its bound over the fill: the tanks drain over the legs)
    const double settling_kg = session.settling_gas_bound_kg() * static_cast<double>(lunar_in_orbit_ignitions(legs));
    out.required_mass_kg = mission::lunar_budget_required_mass(out.dry_mass_kg, lunar.reserve_kg + settling_kg, legs);
    return d;
}

}  // namespace

std::string campaign_lever_error(const CampaignLevers& l) {
    const auto positive = [](const std::optional<double>& v) { return !v || (std::isfinite(*v) && *v > 0.0); };
    const auto non_negative = [](const std::optional<double>& v) { return !v || (std::isfinite(*v) && *v >= 0.0); };
    if (!positive(l.depot_height_m)) return "Kampagne: Depothöhe muss positiv sein";
    if (!positive(l.tanker_dry_mass_kg)) return "Kampagne: Trockenmasse des Tankers muss positiv sein";
    if (!non_negative(l.tanker_reserve_kg)) return "Kampagne: Reserve des Tankers darf nicht negativ sein";
    if (!positive(l.phasing_limit_s)) return "Kampagne: Grenze des Phasenflugs muss positiv sein";
    if (!non_negative(l.depot_initial_kg)) return "Kampagne: Anfangsfüllung darf nicht negativ sein";
    if (!positive(l.depot_insulation_scale)) return "Kampagne: Isolierfaktor muss positiv sein";
    if (!positive(l.launch_interval_s)) return "Kampagne: Startabstand muss positiv sein";
    if (!non_negative(l.hls_wait_s)) return "Kampagne: Wartezeit des HLS darf nicht negativ sein";
    return {};
}

CampaignReport plan_refuelling_campaign(const CampaignSources& src, const CampaignLevers& levers,
                                        const CampaignLive* live) {
    CampaignReport report;
    if (auto error = campaign_lever_error(levers); !error.empty()) {
        report.result.reason = std::move(error);
        return report;
    }
    if (live && levers.depot_height_m) {
        report.result.reason = "Kampagne: im geflogenen Lauf gilt die Höhe des Depots";
        return report;
    }
    const auto& dir = src.config_dir;
    const auto ref = config::load_refuelling_reference(dir);
    const auto tanker_mission = config::load_refuelling_flight_mission(dir, ref.tanker.mission);
    const auto hls_mission = config::load_refuelling_flight_mission(dir, ref.hls.mission);
    const auto tanker_ascent = config::load_ascent_mission(dir, tanker_mission.ascent_id);
    const auto hls_ascent = config::load_ascent_mission(dir, hls_mission.ascent_id);
    const auto site = config::load_launch_site(dir, tanker_ascent.site_id);
    const auto hls_site = config::load_launch_site(dir, hls_ascent.site_id);
    // the reference windows (the measured flights' baselines) stay at the config's start; a live re-plan (f2) starts now
    const time::Epoch config_start = src.time.from_utc(tanker_mission.earliest_utc);
    const time::Epoch start = live ? live->epoch : config_start;
    report.start = start;
    const auto sun_at = [&](time::Epoch t) { return src.ephemeris.state_gcrf(ephemeris::Body::Sun, t).position_m; };
    const auto pad_of = [](const config::LaunchSiteConfig& s) {
        return frames::geodetic_to_fixed({s.latitude_rad, s.longitude_rad, s.height_m});
    };

    // Depot: tanker_flight's [depot] at the altitude lever (plan decision 7)
    auto depot_cfg = tanker_mission.depot;
    const double reference_height_m = depot_cfg.height_m;
    depot_cfg.height_m = levers.depot_height_m.value_or(reference_height_m);
    std::optional<orbit::StationMeanElements> flown;  // f2: the flown depot's mean orbit
    if (live) {
        flown = orbit::station_mean_elements_from_state(live->depot_gcrf, live->epoch, gm, constants::earth_j2,
                                                        equatorial_radius_m, src.earth);
        depot_cfg.height_m = flown->semi_major_axis_m - equatorial_radius_m;
    }
    const double dh = depot_cfg.height_m - reference_height_m;
    report.depot_height_m = depot_cfg.height_m;
    const auto depot_at = flown ? station_orbit(*flown, src.earth) : depot_orbit(depot_cfg, start, src.earth);
    const auto reference_depot_at = depot_orbit(tanker_mission.depot, config_start, src.earth);
    auto depot_vehicle_cfg = top_stage(dir, depot_cfg.vehicle_id);
    if (auto& thermal = depot_vehicle_cfg.stages.back().thermal) {
        thermal->hull.wall_resistance_m2k_w *= levers.depot_insulation_scale;
    }
    vehicle::Vehicle depot(depot_vehicle_cfg, dir);
    report.depot_capacity_kg = main_capacity_kg(depot);

    // Drag makeup end-on (plan decision 4)
    const double r_ref = equatorial_radius_m + reference_height_m;
    const double r_new = equatorial_radius_m + depot_cfg.height_m;
    const double diameter_m = depot_vehicle_cfg.stages.back().diameter_m;
    report.drag_makeup_kgps = mission::drag_makeup_kgps(
        environment::Us76Atmosphere{}.at(depot_cfg.height_m).density_kg_m3, std::sqrt(gm / r_new),
        FlightSession::dragon_free_molecular_cd, constants::pi * 0.25 * diameter_m * diameter_m,
        phasing_exhaust_velocity_mps(depot));

    // Tanker net per flight at the reference phasing time (plan decision 3)
    auto tanker_cfg = top_stage(dir, tanker_ascent.vehicle_id);
    const double dry_kg = levers.tanker_dry_mass_kg.value_or(ref.tanker.dry_mass_kg);
    tanker_cfg.stages.back().dry_mass_kg = dry_kg;
    vehicle::Vehicle tanker(tanker_cfg, dir);
    const double reserve_kg = levers.tanker_reserve_kg.value_or(ref.tanker.main_at_undock_kg);
    const double ve_tanker = phasing_exhaust_velocity_mps(tanker);
    set_main(tanker, reserve_kg);
    const double return_mass_kg = tanker.mass_properties().mass_kg;
    const double rp = equatorial_radius_m + deorbit_perigee_height_m;
    // f1: the way up to another depot height is in each window's phasing cost (its candidate orbits shift with the
    // depot); the altitude cost keeps the deorbit difference
    const double dv_back = mission::deorbit_delta_v_mps(r_new, rp, gm) - mission::deorbit_delta_v_mps(r_ref, rp, gm);
    report.altitude_cost_kg = return_mass_kg * (std::exp(dv_back / ve_tanker) - 1.0) *
                                  mission::TankerReserveParameters{}.burn_margin;
    report.tanker_base_delivered_kg = ref.tanker.delivered_oxidizer_kg + ref.tanker.delivered_fuel_kg +
                                      (ref.tanker.dry_mass_kg - dry_kg) * ref.tanker.dry_mass_sensitivity +
                                      (ref.tanker.main_at_undock_kg - reserve_kg) - report.altitude_cost_kg;

    // Windows (plan decision 1): the flown window logic against the depot. f1: the insertion stays, the candidate
    // phasing orbits shift with the depot
    const auto window_parameters = [&](const config::RefuellingFlightMission& m, const config::AscentMission& a,
                                       double exhaust_velocity_mps, double gas_per_burn_kg) {
        auto p = refuelling_window_parameters(m, a, exhaust_velocity_mps, gas_per_burn_kg);
        for (auto& o : p.phasing_orbits) {
            o.periapsis_radius_m += dh;
            o.apoapsis_radius_m += dh;
        }
        p.phasing.mission_limit_s =
            levers.phasing_limit_s.value_or(m.mission_limit_s) - m.window.insertion_s - m.window_margin_s;
        p.search_s = std::max(p.search_s, campaign_window_search_s);
        return p;
    };
    const vehicle::Vehicle hls_vehicle(top_stage(dir, hls_ascent.vehicle_id), dir);
    vehicle::Vehicle hls_phasing(top_stage(dir, hls_ascent.vehicle_id), dir);  // its boil-off while phasing
    const double ve_hls = phasing_exhaust_velocity_mps(hls_vehicle);
    // f3: the phasing burns settle first (the campaign plans with the default fidelity; one source for the windows'
    // gas and the HLS demand - f3 minor M7)
    const bool settling = vehicle::ignition_settling_default;
    const double gas_tanker = phasing_settling_gas_kg(tanker, settling);
    const double gas_hls = phasing_settling_gas_kg(hls_vehicle, settling);
    auto tanker_window = window_parameters(tanker_mission, tanker_ascent, ve_tanker, gas_tanker);
    auto hls_window = window_parameters(hls_mission, hls_ascent, ve_hls, gas_hls);
    if (levers.reference_phasing) {  // f1 lever: the r5 single phasing orbit, shifted with the depot
        const auto single = [&](const config::RefuellingFlightMission& m, const config::AscentMission& a, double ve,
                                mission::LaunchWindowParameters& p) {
            p.phasing_orbits = refuelling_window_parameters(m, a, ve, 0.0, true).phasing_orbits;
            for (auto& o : p.phasing_orbits) {
                o.periapsis_radius_m += dh;
                o.apoapsis_radius_m += dh;
            }
        };
        try {
            single(tanker_mission, tanker_ascent, ve_tanker, tanker_window);
            single(hls_mission, hls_ascent, ve_hls, hls_window);
        } catch (const std::invalid_argument& e) {  // a mission config without the reference orbit (German)
            report.result.reason = e.what();
            return report;
        }
    }
    const auto tanker_pad = pad_of(site);
    const auto hls_pad = pad_of(hls_site);
    // The reference flights' own windows (the measured numbers were flown in them): their phasing time and cost are
    // the baselines of every window's delivery (f1: the propellant of the chosen orbit against the reference's)
    mission::PhasingCost tanker_reference_cost, hls_reference_cost;
    double hls_reference_phasing_s = 0.0;
    {
        const auto p = refuelling_window_parameters(tanker_mission, tanker_ascent, ve_tanker, gas_tanker);
        const auto w = mission::plan_launch_window(tanker_pad, src.earth, reference_depot_at, config_start, p, gm);
        if (!w.feasible) {
            report.result.reason = "Referenzfenster des Tankers: " + w.reason;
            return report;
        }
        report.reference_phasing_s = w.schedule.waypoint0_epoch - w.separation;
        tanker_reference_cost = w.cost;
        // the HLS's against the same depot orbit (its measured flight phased into the lunar plane's depot) [E]
        const auto ph = refuelling_window_parameters(hls_mission, hls_ascent, ve_hls, gas_hls);
        const auto wh = mission::plan_launch_window(hls_pad, src.earth, reference_depot_at, config_start, ph, gm);
        if (!wh.feasible) {
            report.result.reason = "Referenzfenster des HLS: " + wh.reason;
            return report;
        }
        hls_reference_cost = wh.cost;
        hls_reference_phasing_s = wh.schedule.waypoint0_epoch - wh.separation;
    }
    // The latest launch reaching the same Waypoint 0 as `w` (review I1: the first feasible launch may take extra laps).
    const auto latest_launch = [&](mission::LaunchWindow w, const math::Vec3& pad, mission::LaunchWindowParameters p) {
        p.search_s = later_launch_search_s;
        while (true) {
            const auto next =
                mission::plan_launch_window(pad, src.earth, depot_at, w.liftoff + later_launch_step_s, p, gm);
            if (!next.feasible ||
                std::abs(next.schedule.waypoint0_epoch - w.schedule.waypoint0_epoch) > same_arrival_s) {
                return w;
            }
            w = next;
        }
    };
    const double base_kg = report.tanker_base_delivered_kg;
    const double residue_kg = ref.tanker.residue_after_ascent_kg;
    const double reference_phasing_s = report.reference_phasing_s;
    mission::CampaignModels models;
    models.next_tanker = [&, base_kg, residue_kg, reference_phasing_s](double earliest_s) {
        const auto first =
            mission::plan_launch_window(tanker_pad, src.earth, depot_at, start + earliest_s, tanker_window, gm);
        if (!first.feasible) return std::optional<mission::CampaignFlight>{};
        const auto w = latest_launch(first, tanker_pad, tanker_window);
        const double phasing_s = w.schedule.waypoint0_epoch - w.separation;
        // the tanker's own boil-off over the phasing time's difference to the reference flight, and the propellant
        // of the chosen phasing orbit's raise and boost against the reference window's (f1)
        const double rate = orbit_boil_off_kgps(tanker, residue_kg, depot_at(w.separation), sun_at(w.separation));
        mission::CampaignFlight flight{w.liftoff - start, w.schedule.waypoint0_epoch - start,
                                       base_kg + rate * (reference_phasing_s - phasing_s) -
                                           (w.cost.propellant_kg - tanker_reference_cost.propellant_kg)};
        flight.phasing_orbit = w.phasing_orbit;
        flight.window = w;
        return std::optional{flight};
    };
    models.next_hls = [&](double earliest_s) {
        const auto first = mission::plan_launch_window(hls_pad, src.earth, depot_at, start + earliest_s, hls_window, gm);
        if (!first.feasible) return std::optional<mission::CampaignFlight>{};
        const auto w = latest_launch(first, hls_pad, hls_window);
        mission::CampaignFlight flight{w.liftoff - start, w.schedule.waypoint0_epoch - start, 0.0};
        // f1 minor M3: signed - a cheaper window than the reference needs less - and with the HLS's own boil-off over
        // the phasing time's difference (as the tankers' deliveries)
        const double rate =
            orbit_boil_off_kgps(hls_phasing, ref.hls.main_at_docking_kg, depot_at(w.separation), sun_at(w.separation));
        flight.extra_demand_kg = w.cost.propellant_kg - hls_reference_cost.propellant_kg +
                                 rate * (w.schedule.waypoint0_epoch - w.separation - hls_reference_phasing_s);
        flight.phasing_orbit = w.phasing_orbit;
        flight.window = w;
        return std::optional{flight};
    };
    models.boil_off_kgps = [&](double t_s, double fill_kg) {
        return orbit_boil_off_kgps(depot, fill_kg, depot_at(start + t_s), sun_at(start + t_s));
    };
    const double drag_kgps = report.drag_makeup_kgps;
    models.drag_makeup_kgps = [drag_kgps](double) { return drag_kgps; };

    // HLS demand (plan decision 2)
    const auto lunar = config::load_lunar_landing_mission(dir, hls_mission.lunar_mission_id);
    report.lunar_mission_id = hls_mission.lunar_mission_id;
    report.lunar_epoch = src.time.from_utc(lunar.epoch_utc);
    const auto hls_full = top_stage(dir, hls_ascent.vehicle_id);
    auto lunar_at_depot = lunar;  // r6 minor: the HLS leaves for the Moon from the depot's altitude
    lunar_at_depot.parking_height_m += dh;
    HlsDemand planned;
    if (live && live->demand) {
        planned.values = *live->demand;  // f2: the first plan's (the lunar outbound flight is planned once)
    } else {
        planned = hls_demand(src, lunar_at_depot, report.lunar_epoch, hls_full, settling);
    }
    if (!planned.refusal.empty()) {
        report.result.reason = planned.refusal;
        return report;
    }
    const auto& demand = planned.values;
    report.hls_demand = demand;
    report.hls_required_mass_kg = demand.required_mass_kg;
    report.hls_dry_mass_kg = demand.dry_mass_kg;
    report.hls_capacity_kg = demand.capacity_kg;
    const double required_propellant_kg = demand.required_mass_kg - demand.dry_mass_kg;
    if (required_propellant_kg > demand.capacity_kg) {
        report.result.reason = "HLS-Tanks zu klein: " + tonnes(required_propellant_kg) + " gefordert, " +
                               tonnes(demand.capacity_kg) + " Tankvolumen";
        return report;
    }
    vehicle::Vehicle hls(hls_full, dir);
    set_main(hls, ref.hls.main_at_docking_kg);
    report.hls_propellant_at_docking_kg = hls.propellant_kg();
    // r6 minor: the HLS's wait loaded for the mission boils off with the Sun of its arrival, and must fit its tanks
    if (levers.hls_wait_s > 0.0) {
        const double main_kg = required_propellant_kg - demand.headers_kg;
        models.hls_wait_kg = [&, main_kg](double arrival_s) {
            return orbit_boil_off_kgps(hls, main_kg, depot_at(start + arrival_s), sun_at(start + arrival_s)) *
                   levers.hls_wait_s;
        };
    }
    mission::CampaignInputs inputs;
    inputs.hls_tank_room_kg = demand.capacity_kg - required_propellant_kg;
    report.demand_kg = mission::gross_for_net_kg(std::max(0.0, required_propellant_kg - report.hls_propellant_at_docking_kg),
                                                 inputs.transfer_vent_fraction) +
                       ref.tanker.chilldown_kg;
    inputs.depot_capacity_kg = report.depot_capacity_kg;
    inputs.depot_initial_kg = levers.depot_initial_kg.value_or(ref.depot.initial_propellant_kg);
    inputs.demand_kg = report.demand_kg;
    inputs.launch_interval_s = levers.launch_interval_s;
    if (live) {  // f2: the flown depot's fill and the last flight
        inputs.depot_initial_kg = live->depot_propellant_kg;
        if (live->previous_launch) inputs.previous_launch_s = *live->previous_launch - start;
        if (live->previous_arrival) inputs.previous_arrival_s = *live->previous_arrival - start;
        inputs.earliest_launch_s = live->launch_lead_s;
    }
    report.tanker_window = tanker_window;
    report.hls_window = hls_window;
    report.result = mission::refuelling_campaign(inputs, models);
    return report;
}

}  // namespace starsim::sim
