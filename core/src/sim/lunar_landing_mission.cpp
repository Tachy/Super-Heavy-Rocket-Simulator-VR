#include "starsim/sim/lunar_landing_mission.hpp"

#include <memory>
#include <stdexcept>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/lunar_site.hpp"

namespace starsim::sim {

namespace {

struct PlacedSite {
    std::shared_ptr<const environment::LunarTerrain> terrain;
    LunarSite site;
};

PlacedSite place(FlightSession& session, const config::LunarLandingMission& m, const std::filesystem::path& config_dir,
                 time::Epoch epoch, const config::VehicleConfig* vehicle) {
    if (!session.moon_orientation()) {
        throw std::runtime_error("Mondlandung: die Sitzung hat keine Mondorientierung");
    }
    const auto site = config::load_lunar_site(config_dir, m.site_id);
    const mission::LunarTransferTargeter targeter(session.gravity(), session.ephemeris(), *session.moon_orientation());
    const auto raan = targeter.parking_orbit_raan(epoch + m.flight_time_s, m.parking_inclination_rad);
    if (!raan) {
        throw std::runtime_error("Kein Parkorbit durch die Mondbahnebene: Bahnneigung unter der Deklination des Mondes");
    }
    // circular: the argument of latitude is the anomaly from the ascending node
    const orbit::KeplerianElements elements{constants::wgs84_semi_major_axis_m + m.parking_height_m, 0.0,
                                            m.parking_inclination_rad, *raan, 0.0, m.argument_of_latitude_rad};
    const auto state = orbit::state_from_elements(elements, session.gm_earth_m3ps2());
    // the data pipeline's directory next to config/ (repository layout)
    PlacedSite placed;
    placed.terrain = load_lunar_terrain(config_dir, config_dir.parent_path() / "data");
    placed.site = lunar_site(site, *placed.terrain);
    if (vehicle) {
        session.place_in_orbit(*vehicle, state, epoch);
    } else {
        session.place_in_orbit(m.vehicle_id, state, epoch);
    }
    session.set_moon_terrain(placed.terrain);
    session.set_lunar_landing_site(placed.site);
    return placed;
}

}  // namespace

void place_lunar_landing_mission(FlightSession& session, const config::LunarLandingMission& m,
                                 const std::filesystem::path& config_dir, time::Epoch epoch,
                                 const config::VehicleConfig* vehicle) {
    place(session, m, config_dir, epoch, vehicle);
}

LunarLandingSetup lunar_landing_setup(const config::LunarLandingMission& m) {
    LunarLandingSetup setup;
    setup.descent.pdi_altitude_above_site_m = m.pdi_altitude_above_site_m;
    setup.descent.braking_range_m = m.braking_range_m;
    setup.descent.max_crossrange_m = m.max_crossrange_m;
    setup.descent.slope_limit_rad = m.slope_limit_rad;
    setup.descent.touchdown_search_radius_m = m.touchdown_search_radius_m;
    setup.surface_stay_s = m.surface_stay_s;
    setup.descent_delta_v_mps = m.descent_delta_v_mps;
    setup.ascent_delta_v_mps = m.ascent_delta_v_mps;
    setup.reserve_kg = m.reserve_kg;
    gnc::LunarAscentParameters ascent;
    ascent.insertion_altitude_above_site_m = m.insertion_altitude_above_site_m;
    ascent.aposelene_altitude_m = m.ascent_aposelene_m;
    setup.ascent = ascent;
    return setup;
}

mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& m, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg, double vacuum_isp_s,
                                           double approach_isp_s) {
    const auto setup = lunar_landing_setup(m);
    const bool nrho = m.lunar_orbit == "nrho";
    std::vector<mission::BudgetLeg> legs{{"TLI", norm(plan.tli.delta_v_gcrf_mps), vacuum_isp_s}};
    if (nrho) {
        if (plan.powered_flyby) legs.push_back({"Swing-by", norm(plan.powered_flyby->delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"Einschuss", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"NRHO -> LLO", m.nrho_to_llo_delta_v_mps, vacuum_isp_s});
    } else {
        legs.push_back({"LOI", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
    }
    legs.push_back({"Abstieg (Bremsphase)", setup.braking_delta_v_mps, vacuum_isp_s});
    legs.push_back({"Abstieg (Anflug)", m.descent_delta_v_mps - setup.braking_delta_v_mps, approach_isp_s});
    legs.push_back({"Aufstieg", m.ascent_delta_v_mps, vacuum_isp_s});
    if (nrho) legs.push_back({"LLO -> NRHO", m.llo_to_nrho_delta_v_mps, vacuum_isp_s});
    return mission::lunar_budget(initial_mass_kg, dry_mass_kg, m.reserve_kg, legs);
}

MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& m, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle) {
    MissionStart s;
    s.kind = config::MissionKind::LunarLanding;
    s.epoch = epoch;
    const auto refuse = [&](const std::string& reason) {
        session.disengage_autopilot();
        s.return_planned = false;
        s.refusal = reason;
        return s;
    };
    PlacedSite placed;
    try {
        placed = place(session, m, config_dir, epoch, vehicle);
    } catch (const std::runtime_error& e) {  // no plane, no Moon orientation, terrain files missing (German)
        return refuse(e.what());
    }
    const auto setup = lunar_landing_setup(m);
    const auto& v = session.body(session.controlled_index()).vehicle();
    // A touchdown point within the slope limit for the approach of a polar pass (either direction).
    const double footprint_m = v.stages().front().config.landing_legs.footprint_radius_m;
    bool reachable = false;
    for (const double azimuth : {0.0, constants::pi}) {
        reachable = reachable ||
                    mission::select_touchdown_point(*placed.terrain, placed.site.config.latitude_rad,
                                                    placed.site.config.longitude_rad, azimuth, footprint_m, setup.descent)
                        .has_value();
    }
    if (!reachable) return refuse("Mission nicht durchführbar: kein Landepunkt innerhalb der Hangneigungsgrenze");
    const auto outbound = lunar_outbound_parameters(m, session.gravity(), session.ephemeris(), epoch);
    if (!session.plan_lunar_outbound(outbound)) return refuse(session.lunar_status());
    const double mass_kg = v.mass_properties().mass_kg;
    const auto& sets = setup.guidance.braking_sets;
    const auto budget = lunar_mission_budget(m, *session.lunar_outbound_plan(), mass_kg, mass_kg - v.propellant_kg(),
                                             gnc::groups_vacuum_isp_s(v, sets.front()),
                                             gnc::groups_vacuum_isp_s(v, sets.back()));
    if (!budget.closes) return refuse("Mission nicht durchführbar: " + budget.reason);
    LunarMissionSequence sequence;
    sequence.landing = setup;
    sequence.via_nrho = outbound.orbit == mission::LunarOrbitKind::Nrho;
    sequence.nrho = outbound.nrho;
    if (!sequence.via_nrho) sequence.llo = outbound.llo;
    session.set_lunar_mission(std::move(sequence));
    s.return_planned = true;
    return s;
}

mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& m,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch) {
    mission::LunarOutboundParameters p;
    p.arrival_epoch = epoch + m.flight_time_s;
    p.earliest_departure_s = m.earliest_departure_s;
    p.mcc_after_tli_s = m.mcc_after_tli_s;
    if (m.lunar_orbit == "nrho") {
        p.orbit = mission::LunarOrbitKind::Nrho;
        const mission::NrhoSeed seed{{m.nrho_apolune_position, m.nrho_apolune_velocity}, m.nrho_revolutions,
                                     m.nrho_synodic_months};
        p.nrho = std::make_shared<mission::NrhoReference>(
            gravity, ephemeris, seed, p.arrival_epoch,
            mission::NrhoReferenceOptions{.revolutions = m.nrho_reference_revolutions});
    } else {
        p.orbit = mission::LunarOrbitKind::Llo;
        p.llo = {m.llo_altitude_m, m.llo_inclination_rad};
    }
    return p;
}

}  // namespace starsim::sim
