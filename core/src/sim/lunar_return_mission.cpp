#include "starsim/sim/lunar_return_mission.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include "starsim/aero/radiative_heating.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/entry_aero_table.hpp"
#include "starsim/gnc/entry_guidance.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/gnc/ship_landing.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/quat.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/vehicle/flight_body.hpp"
#include "starsim/vehicle/vehicle.hpp"

namespace starsim::sim {

namespace {

constexpr double nrho_first_perilune_s = 2.0 * 86400.0;  // [E] l5 Task 6: the departure burn ~0.75 d before it
constexpr double asymptote_lead_s = 3600.0;              // [E] the LLO plane holds the asymptote of a departure 1 h on
constexpr double min_entry_speed_mps = 8000.0;           // [E] floor of the entry-speed search (below: LEO entries)
constexpr double max_entry_speed_mps = 11500.0;          // [E] above the natural lunar return (~11.0 km/s inertial)
constexpr double entry_heading_rad = 0.5 * constants::pi;  // [E] eastward arrival (l5 Task 6: westward +0.8 km/s)
// [E] aerocapture: perigee raise out of the pass (~50-60 m/s from a -64 km periapsis, l5 Task 4/6) and the b4 deorbit
// burn (~100 m/s with margin) on the deorbit engines
constexpr double aerocapture_raise_mps = 60.0;
constexpr double aerocapture_deorbit_mps = 100.0;
constexpr int max_propellant_iterations = 5;
constexpr double propellant_tolerance_kg = 100.0;

// tank capacity (the configured loads are full tanks)
double capacity_kg(const config::StageConfig& stage) {
    double full = 0.0;
    for (const auto& tank : stage.tanks) full += tank.mass_kg;
    return full;
}

void scale_tanks(config::StageConfig& stage, double propellant_kg) {
    const double full = capacity_kg(stage);
    for (auto& tank : stage.tanks) tank.mass_kg *= propellant_kg / full;
}

// Vacuum thrust and mass flow of the engines in `groups` (all at full throttle).
mission::FiniteBurnModel vacuum_engines(const vehicle::Vehicle& vehicle, const std::vector<std::string>& groups,
                                        double mass_kg) {
    mission::FiniteBurnModel model{mass_kg, 0.0, 0.0};
    for (const auto& stage : vehicle.stages()) {
        for (const auto& e : stage.engines) {
            bool in = false;
            for (const auto& g : groups) in = in || g == e.group_id;
            if (!in) continue;
            model.thrust_n += e.performance->thrust_n(1.0, 0.0);
            model.mass_flow_kgps += e.performance->mass_flow_kgps(1.0);
        }
    }
    return model;
}

// Engine group ids of the vehicle ("<stage>/<group>") from the configuration's ("<group>", as burns force them).
std::vector<std::string> qualified(const std::string& stage_id, const std::vector<std::string>& groups) {
    std::vector<std::string> ids;
    for (const auto& g : groups) ids.push_back(stage_id + "/" + g);
    return ids;
}

std::string tonnes(double kg) {
    char text[32];
    std::snprintf(text, sizeof text, "%.1f", kg / 1000.0);
    std::string s = text;
    for (auto& c : s) {
        if (c == '.') c = ',';
    }
    return s;
}

}  // namespace

config::VehicleConfig lunar_return_ship(const config::LunarReturnMission& m, const std::filesystem::path& config_dir,
                                        const config::VehicleConfig* vehicle) {
    auto cfg = vehicle ? *vehicle : config::load_vehicle(config_dir, m.vehicle_id);
    while (!cfg.stages.empty() && cfg.stages.front().id != m.stage_id) cfg.stages.erase(cfg.stages.begin());
    if (cfg.stages.empty()) throw std::runtime_error("Mondrückkehr: Stufe '" + m.stage_id + "' fehlt im Fahrzeug");
    return cfg;
}

LunarReturnSetup lunar_return_setup(const config::LunarReturnMission& m, const math::Vec3& site_itrf_m,
                                    const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                                    time::Epoch epoch) {
    LunarReturnSetup setup;
    auto& ret = setup.ret;
    if (m.orbit == "nrho") {
        ret.orbit = mission::LunarOrbitKind::Nrho;
        const mission::NrhoSeed seed{{m.nrho_apolune_position, m.nrho_apolune_velocity}, m.nrho_revolutions,
                                     m.nrho_synodic_months};
        ret.nrho = std::make_shared<mission::NrhoReference>(
            gravity, ephemeris, seed, epoch + nrho_first_perilune_s,
            mission::NrhoReferenceOptions{.revolutions = m.nrho_reference_revolutions});
    } else {
        ret.orbit = mission::LunarOrbitKind::Llo;
    }
    ret.target.site_itrf_m = site_itrf_m;
    ret.target.interface_height_m = m.entry_interface_height_m;
    ret.target.braking_height_m = m.braking_height_m;
    ret.target.nominal_bank_rad = m.nominal_bank_rad;
    ret.flight_time_guess_s = m.flight_time_s;
    ret.search_s = m.search_s;
    ret.mcc_after_tei_s = m.mcc_after_tei_s;
    ret.mcc_allowance_mps = m.mcc_allowance_mps;
    setup.aerocapture = m.aerocapture;
    if (m.aerocapture) {
        // the exit orbit's inclination is chosen with the pass (Task 9); none here: no reversals on it
        setup.entry = gnc::EntryGuidanceParameters::aerocapture(m.aerocapture_apoapsis_m, -1.0);
        setup.aerocapture_apoapsis_m = m.aerocapture_apoapsis_m;
        setup.perigee_raise_height_m = m.perigee_raise_height_m;
        setup.deorbit.entry_interface_height_m = m.deorbit.entry_interface_height_m;
        setup.deorbit.entry_flight_path_rad = m.deorbit.entry_flight_path_rad;
        setup.deorbit.nominal_bank_rad = m.deorbit.nominal_bank_rad;
        setup.deorbit.max_crossrange_m = m.deorbit.max_crossrange_m;
        setup.deorbit.search_window_s = m.deorbit.search_window_s;
    } else {
        setup.entry = gnc::EntryGuidanceParameters::lunar_return();
    }
    return setup;
}

math::StateVector lunar_return_start_state(const FlightSession& session, const config::LunarReturnMission& m,
                                           time::Epoch epoch, const mission::EarthReturnParameters& ret) {
    if (ret.orbit == mission::LunarOrbitKind::Nrho) return ret.nrho->state_gcrf(epoch);
    if (!session.moon_orientation()) throw std::runtime_error("Mondrückkehr: die Sitzung hat keine Mondorientierung");
    const auto& ephemeris = session.ephemeris();
    const math::Vec3 s = mission::return_asymptote(ephemeris, epoch + asymptote_lead_s, ret.flight_time_guess_s,
                                                   ret.target.site_itrf_m, session.earth_orientation());
    const auto rot = session.moon_orientation()->fixed_to_gcrf(epoch).rotation;
    const math::Vec3 pole{rot(0, 2), rot(1, 2), rot(2, 2)};
    // The plane of the mission's inclination containing the asymptote: n = a e1 + b e2 with n.s = 0, e1 the pole's part
    // across s, e2 = s x e1; n.pole = a cos(declination) = cos(inclination). Of the two planes the one that is
    // n = cross(pole, s) when polar. The departure point (velocity along s) lies a quarter revolution ahead.
    const double sin_declination = std::clamp(dot(s, pole), -1.0, 1.0);
    const double cos_declination = std::sqrt(1.0 - sin_declination * sin_declination);
    const double a = std::cos(m.llo_inclination_rad) / cos_declination;
    if (std::abs(a) > 1.0) {
        char text[160];
        std::snprintf(text, sizeof text,
                      "Keine LLO mit %.1f° Bahnneigung enthält die Rückflug-Asymptote (Deklination %.1f°)",
                      m.llo_inclination_rad / constants::deg_to_rad, std::asin(sin_declination) / constants::deg_to_rad);
        throw std::runtime_error(text);
    }
    const math::Vec3 e1 = math::normalized(pole - s * sin_declination);
    const math::Vec3 e2 = cross(s, e1);
    const math::Vec3 n = math::normalized(e1 * a - e2 * std::sqrt(1.0 - a * a));
    const double r = constants::moon_mean_radius_m + m.llo_altitude_m;
    const double v = std::sqrt(ephemeris.gm_m3ps2(ephemeris::Body::Moon) / r);
    const math::Vec3 position = s * -1.0;
    const auto moon = ephemeris.state_gcrf(ephemeris::Body::Moon, epoch);
    return {moon.position_m + position * r, moon.velocity_mps + cross(n, position) * v};
}

void place_lunar_return_ship(FlightSession& session, const config::LunarReturnMission& m,
                             const config::VehicleConfig& ship, time::Epoch epoch, double propellant_kg,
                             const mission::EarthReturnParameters& ret) {
    const auto state = lunar_return_start_state(session, m, epoch, ret);
    auto cfg = ship;
    scale_tanks(cfg.stages.front(), propellant_kg);
    const auto moon = session.ephemeris().state_gcrf(ephemeris::Body::Moon, epoch);
    const math::Vec3 x = math::normalized(state.velocity_mps - moon.velocity_mps);  // nose prograde (Moon-relative)
    const math::Vec3 z = math::normalized(state.position_m - moon.position_m);      // belly (body -z) to the Moon
    const math::Vec3 y = cross(z, x);
    math::Mat3 r;
    const math::Vec3 columns[3] = {x, y, z};
    for (int c = 0; c < 3; ++c) {
        r(0, c) = columns[c].x;
        r(1, c) = columns[c].y;
        r(2, c) = columns[c].z;
    }
    session.place_body(cfg, {state.position_m, state.velocity_mps, math::Quat::from_matrix(r), {}}, epoch);
}

LunarReturnBudget lunar_return_budget(const FlightSession& session, const std::filesystem::path& config_dir,
                                      const config::LunarReturnMission& m, const config::VehicleConfig& ship,
                                      time::Epoch epoch, LunarReturnSetup& setup) {
    LunarReturnBudget b;
    const auto& stage = ship.stages.front();
    if (stage.aero.entry_alpha_table.empty() || stage.heat_shield.nose_radius_m <= 0.0) {
        b.reason = "Kein Eintrittskorridor: das Fahrzeug hat keinen Hitzeschild";
        return b;
    }
    // Onboard entry model at the entry mass (dry + landing propellant), as the entry guidance will fly it.
    auto entry_cfg = ship;
    scale_tanks(entry_cfg.stages.front(), stage.landing_propellant_kg);
    const vehicle::FlightBody entry_body(std::make_unique<vehicle::Vehicle>(entry_cfg, config_dir),
                                         vehicle::RigidBodyState{});
    const auto& entry_vehicle = entry_body.vehicle();
    const auto gate = gnc::ship_entry_target(session.return_site(entry_vehicle), gnc::ship_landing_parameters());
    const gnc::EntryGuidance onboard(
        entry_body, gnc::load_alpha_schedule(config_dir, stage.aero.entry_alpha_table), gate, setup.entry,
        std::make_shared<const aero::RadiativeHeatingTable>(aero::RadiativeHeatingTable::load(config_dir)));

    // Entry speed: the highest with a corridor of the minimum width (2-D question: sphere, no J2).
    auto corridor_model = onboard.model();
    corridor_model.j2 = 0.0;
    gnc::CorridorQuery q;
    q.interface_height_m = m.entry_interface_height_m;
    q.latitude_rad = session.launch_site().latitude_rad;
    q.heading_rad = entry_heading_rad;
    q.limits = stage.heat_shield;
    if (m.aerocapture) q.aerocapture_apoapsis_m = m.aerocapture_apoapsis_m;
    b.entry_speed_mps =
        gnc::max_entry_speed(corridor_model, q, m.min_corridor_width_rad, min_entry_speed_mps, max_entry_speed_mps);
    if (b.entry_speed_mps < min_entry_speed_mps) {
        b.reason = "Kein Eintrittskorridor innerhalb der Hitzeschild-Grenzen";
        return b;
    }
    q.speed_mps = b.entry_speed_mps;
    b.corridor = gnc::entry_corridor(corridor_model, q);
    auto& target = setup.ret.target;
    target.speed_mps = b.entry_speed_mps;
    target.flight_path_rad = b.corridor.middle_rad;
    if (!m.aerocapture) {
        target.entry_model = onboard.model();
        target.gate = gate;
        if (setup.entry.final_bank_rad >= 0.0) {
            // A constant bank that stays in dives once below circular speed (Task 8 finding): the profiled entry's
            // initial bank must lie in its window; plan where the window is widest, aim with its middle.
            b.bank_window = gnc::widest_bank_window(corridor_model, q, b.corridor, setup.entry.max_bank_rad);
            if (!b.bank_window.exists) {
                b.reason = "Kein Eintrittsfenster: keine Querneigung fängt ein und bleibt in den Grenzen";
                return b;
            }
            target.flight_path_rad = b.bank_window.flight_path_rad;
            target.nominal_bank_rad = b.bank_window.middle_rad;
            setup.entry.initial_bank_rad = b.bank_window.middle_rad;  // the guidance's warm start
        }
    }

    // Propellant fixed point over the planned burns (plane-only targeting: the gate moves the arrival time by hours,
    // the delta-v by ~1-2 %; the session aims at the gate when it plans).
    const vehicle::Vehicle full(ship, config_dir);
    const double dry_kg = full.mass_properties().mass_kg - full.propellant_kg();
    const double capacity = capacity_kg(stage);
    const auto burn_groups = qualified(stage.id, setup.burn_groups);
    const double isp_s = gnc::groups_vacuum_isp_s(entry_vehicle, burn_groups);
    if (!(isp_s > 0.0)) {
        b.reason = "Kein Rückflug: keine Triebwerke für die Brennvorgänge";
        return b;
    }
    const double ve = isp_s * constants::standard_gravity_mps2;
    double end_kg = dry_kg + stage.landing_propellant_kg;
    if (m.aerocapture) {
        const double isp_deorbit_s =
            gnc::groups_vacuum_isp_s(entry_vehicle, qualified(stage.id, stage.deorbit_engine_groups));
        end_kg *= std::exp((aerocapture_raise_mps + aerocapture_deorbit_mps) /
                           (isp_deorbit_s * constants::standard_gravity_mps2));
    }
    auto planning = setup.ret;
    planning.target.entry_model.reset();
    const auto state = lunar_return_start_state(session, m, epoch, setup.ret);
    mission::EarthReturnPlanner planner(session.gravity(), session.ephemeris(), session.earth_orientation());
    // First iteration impulsive (the mass is not known yet), then finite burns at the mass of the previous one.
    double initial_kg = dry_kg + capacity;
    for (int i = 0; i < max_propellant_iterations; ++i) {
        if (i > 0) planner.set_finite_burn(vacuum_engines(entry_vehicle, burn_groups, initial_kg));
        const auto plan = planner.plan(state, epoch, planning, initial_kg, isp_s, initial_kg - dry_kg);
        if (!plan.feasible) {
            b.reason = plan.reason;
            return b;
        }
        b.tei_delta_v_mps = plan.tei_delta_v_mps;
        b.braking_delta_v_mps = plan.braking_delta_v_mps;
        const double coast_days = (plan.arrival.interface_epoch - epoch) / 86400.0;
        const double next_kg = end_kg * std::exp((plan.braking_delta_v_mps + m.mcc_allowance_mps) / ve) *
                                   std::exp(plan.tei_delta_v_mps / ve) +
                               coast_days * m.coast_attitude_kg_per_day + m.reserve_kg;
        b.propellant_kg = next_kg - dry_kg;
        if (b.propellant_kg > capacity) {
            b.reason = "Kein Rückflug: Treibstoff (" + tonnes(b.propellant_kg) + " t) übersteigt die Tanks (" +
                       tonnes(capacity) + " t)";
            return b;
        }
        const bool converged = std::abs(next_kg - initial_kg) < propellant_tolerance_kg;
        initial_kg = next_kg;
        if (converged) break;
    }
    b.closes = true;
    return b;
}

MissionStart start_lunar_return_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                        const config::LunarReturnMission& m, time::Epoch epoch,
                                        const config::VehicleConfig* vehicle, LunarReturnBudget* budget) {
    MissionStart s;
    s.kind = config::MissionKind::LunarReturn;
    s.epoch = epoch;
    const auto refuse = [&](const std::string& reason) {
        session.disengage_autopilot();
        s.return_planned = false;
        s.refusal = reason;
        return s;
    };
    if (!session.moon_orientation()) return refuse("Mondrückkehr: die Sitzung hat keine Mondorientierung");
    const auto site = config::load_launch_site(config_dir, m.site_id);
    session.set_launch_site(launch_site_from_config(site));
    const auto site_itrf = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    LunarReturnSetup setup;
    config::VehicleConfig ship;
    try {
        ship = lunar_return_ship(m, config_dir, vehicle);
        setup = lunar_return_setup(m, site_itrf, session.gravity(), session.ephemeris(), epoch);
    } catch (const std::runtime_error& e) {  // stage missing, NRHO reference not converged (German)
        return refuse(e.what());
    }
    LunarReturnBudget b;
    try {
        b = lunar_return_budget(session, config_dir, m, ship, epoch, setup);
    } catch (const std::runtime_error& e) {  // no LLO of the mission's inclination through the asymptote (German)
        b.reason = e.what();
    }
    if (budget) *budget = b;
    if (!b.closes) return refuse(b.reason);
    place_lunar_return_ship(session, m, ship, epoch, b.propellant_kg, setup.ret);
    s.return_planned = session.plan_lunar_return(setup);
    if (!s.return_planned) return refuse(session.lunar_status());
    return s;
}

}  // namespace starsim::sim
