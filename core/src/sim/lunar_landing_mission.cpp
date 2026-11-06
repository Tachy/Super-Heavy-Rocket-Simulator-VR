#include "starsim/sim/lunar_landing_mission.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "starsim/config/lunar_site_config.hpp"
#include "starsim/gnc/lunar_ascent_guidance.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/lunar_descent.hpp"
#include "starsim/mission/lunar_transfer.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/mission/touchdown_search.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/report/report.hpp"
#include "flight_session_lunar_detail.hpp"
#include "starsim/sim/lunar_site.hpp"
#include "starsim/sim/terrain_cache.hpp"

namespace starsim::sim {

namespace {

struct PlacedSite {
    std::shared_ptr<const environment::LunarTerrain> terrain;  // LOLA: the site height, the planners
    std::shared_ptr<const environment::DetailedLunarTerrain> surface;  // the physics surface (all parts loaded)
    LunarSite site;
};

// The Moon's terrain (the data pipeline's directory next to config/: repository layout), the composed surface with
// every part (the session picks the fidelity flags from them) and the mission's site on it.
PlacedSite load_site(const config::LunarLandingMission& m, const std::filesystem::path& config_dir) {
    const auto site = config::load_lunar_site(config_dir, m.site_id);
    PlacedSite placed;
    placed.surface = shared_moon_surface(config_dir, config_dir.parent_path() / "data", {true, true}, site);
    placed.terrain = placed.surface->lola_shared();
    placed.site = lunar_site(site, *placed.terrain);
    return placed;
}

void set_site(FlightSession& session, const PlacedSite& placed) {
    session.set_moon_terrain(placed.surface);
    session.set_lunar_landing_site(placed.site);
}

PlacedSite place(FlightSession& session, const config::LunarLandingMission& m, const std::filesystem::path& config_dir,
                 time::Epoch epoch, const config::VehicleConfig* vehicle) {
    if (!session.moon_orientation()) {
        throw report::Refusal(report::make(report::Code::lunar_landing_no_moon_orientation));
    }
    const auto state = lunar_parking_state(session, m, epoch, m.parking_inclination_rad);
    if (!state) {
        throw report::Refusal(report::make(report::Code::lunar_mission_no_parking_plane));
    }
    const auto placed = load_site(m, config_dir);
    if (vehicle) {
        session.place_in_orbit(*vehicle, *state, epoch);
    } else {
        session.place_in_orbit(m.vehicle_id, *state, epoch);
    }
    set_site(session, placed);
    return placed;
}

// After the touchdown check (`reachable`: a point was found): outbound plan, budget, legs.
MissionStart plan_after_touchdown_check(FlightSession& session, const config::LunarLandingMission& m, time::Epoch epoch,
                                        bool reachable) {
    MissionStart s;
    s.kind = config::MissionKind::LunarLanding;
    s.epoch = epoch;
    const auto refuse = [&](const report::Report& reason) {
        session.disengage_autopilot();
        s.return_planned = false;
        s.refusal = reason;
        return s;
    };
    const auto setup = lunar_landing_setup(m);
    const auto& v = session.body(session.controlled_index()).vehicle();
    if (!reachable) {
        return refuse(report::with_cause(report::make(report::Code::lunar_mission_infeasible),
                                         report::make(report::Code::lunar_mission_no_touchdown_point)));
    }
    const auto outbound = lunar_outbound_parameters(m, session.gravity(), session.ephemeris(), epoch);
    if (!session.plan_lunar_outbound(outbound)) return refuse(session.lunar_status());
    const double mass_kg = v.mass_properties().mass_kg;
    const auto& sets = setup.guidance.braking_sets;
    const auto budget = lunar_mission_budget(m, *session.lunar_outbound_plan(), mass_kg, mass_kg - v.propellant_kg(),
                                             gnc::groups_vacuum_isp_s(v, sets.front()),
                                             gnc::groups_vacuum_isp_s(v, sets.back()), session.settling_gas_bound_kg());
    if (!budget.closes) return refuse(report::with_cause(report::make(report::Code::lunar_mission_infeasible), budget.reason));
    LunarMissionSequence sequence;
    sequence.landing = setup;
    sequence.via_nrho = outbound.orbit == mission::LunarOrbitKind::Nrho;
    sequence.nrho = outbound.nrho;
    if (!sequence.via_nrho) sequence.llo = outbound.llo;
    session.set_lunar_mission(std::move(sequence));
    s.return_planned = true;
    return s;
}

// The mission from the controlled body as it is (after the placement, or the refuelled HLS): a touchdown point within
// the slope limit for the approach of a polar pass (either direction), then plan_after_touchdown_check. On the session's
// composed surface (its fidelity flags) with the hazard check, as the descent planner selects - that search runs on a
// background thread (a refused site checks every candidate: up to ~30 s); LOLA alone (milliseconds) is decided at once.
DeferredMissionStart plan_from_live_state(FlightSession& session, const config::LunarLandingMission& m,
                                          time::Epoch epoch, const PlacedSite& placed) {
    const auto setup = lunar_landing_setup(m);
    const auto& stage = session.body(session.controlled_index()).vehicle().stages().front().config;
    const double footprint_m = stage.landing_legs.footprint_radius_m;
    const double lat = placed.site.config.latitude_rad, lon = placed.site.config.longitude_rad;
    const std::vector<double> azimuths{0.0, constants::pi};
    auto surface = std::dynamic_pointer_cast<const environment::DetailedLunarTerrain>(session.moon_surface_shared());
    const bool hazards = surface && surface->detail_on() && stage.landing_legs.foot_pad_radius_m > 0.0;
    if (!hazards) {
        bool reachable = false;
        for (const double azimuth : azimuths) {
            reachable = reachable || mission::select_touchdown_point(*placed.terrain, lat, lon, azimuth, footprint_m,
                                                                     setup.descent)
                                         .has_value();
        }
        return {plan_after_touchdown_check(session, m, epoch, reachable)};
    }
    auto search = std::make_shared<std::shared_future<std::optional<mission::TouchdownPoint>>>(
        mission::select_touchdown_point_async(std::move(surface), lat, lon, azimuths, footprint_m, setup.descent,
                                              mission::touchdown_hazard_limits(stage))
            .share());
    DeferredMissionStart d;
    d.status = report::make(report::Code::start_checking_landing_site);
    d.poll = [search, m, epoch](FlightSession& f) -> std::optional<MissionStart> {
        if (search->wait_for(std::chrono::seconds(0)) != std::future_status::ready) return std::nullopt;
        return plan_after_touchdown_check(f, m, epoch, search->get().has_value());
    };
    d.wait = [search, m, epoch](FlightSession& f) {
        return plan_after_touchdown_check(f, m, epoch, search->get().has_value());
    };
    return d;
}

MissionStart decided_now(FlightSession& session, DeferredMissionStart start) {
    return start.done ? std::move(*start.done) : start.wait(session);
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

std::vector<mission::BudgetLeg> lunar_mission_legs(const config::LunarLandingMission& m,
                                                   const mission::OutboundPlan& plan, double vacuum_isp_s,
                                                   double approach_isp_s) {
    const auto setup = lunar_landing_setup(m);
    const bool nrho = m.lunar_orbit == "nrho";
    std::vector<mission::BudgetLeg> legs{{"tli", norm(plan.tli.delta_v_gcrf_mps), vacuum_isp_s}};
    if (nrho) {
        if (plan.powered_flyby) legs.push_back({"swing_by", norm(plan.powered_flyby->delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"insertion", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"nrho_to_llo", m.nrho_to_llo_delta_v_mps, vacuum_isp_s, 2});  // departure + LLO arrival
    } else {
        legs.push_back({"loi", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
    }
    legs.push_back({"descent_braking", setup.braking_delta_v_mps, vacuum_isp_s, 2});  // DOI + PDI
    // the approach continues the braking burn
    legs.push_back({"descent_approach", m.descent_delta_v_mps - setup.braking_delta_v_mps, approach_isp_s, 0});
    legs.push_back({"ascent", m.ascent_delta_v_mps, vacuum_isp_s, 1});  // lit landed; the circularization in orbit
    if (nrho) legs.push_back({"llo_to_nrho", m.llo_to_nrho_delta_v_mps, vacuum_isp_s, 2});
    return legs;
}

size_t lunar_in_orbit_ignitions(const std::vector<mission::BudgetLeg>& legs) {
    size_t ignitions = 0;
    for (const auto& leg : legs) ignitions += static_cast<size_t>(std::max(leg.ignitions, 0));
    return ignitions;
}

mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& m, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg, double vacuum_isp_s,
                                           double approach_isp_s, double settling_gas_kg) {
    const auto legs = lunar_mission_legs(m, plan, vacuum_isp_s, approach_isp_s);
    return mission::lunar_budget(initial_mass_kg, dry_mass_kg,
                                 m.reserve_kg + settling_gas_kg * static_cast<double>(lunar_in_orbit_ignitions(legs)),
                                 legs);
}

DeferredMissionStart begin_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                                 const config::LunarLandingMission& m, time::Epoch epoch,
                                                 const config::VehicleConfig* vehicle) {
    PlacedSite placed;
    try {
        placed = place(session, m, config_dir, epoch, vehicle);
    } catch (const std::runtime_error& e) {  // no plane, no Moon orientation, terrain files missing
        session.disengage_autopilot();
        MissionStart s;
        s.kind = config::MissionKind::LunarLanding;
        s.epoch = epoch;
        s.refusal = detail::exception_report(e);
        return {s};
    }
    return plan_from_live_state(session, m, epoch, placed);
}

MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& m, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle) {
    return decided_now(session, begin_lunar_landing_mission(session, config_dir, m, epoch, vehicle));
}

std::optional<math::StateVector> lunar_parking_state(const FlightSession& session, const config::LunarLandingMission& m,
                                                     time::Epoch epoch, double inclination_rad) {
    const mission::LunarTransferTargeter targeter(session.gravity(), session.ephemeris(), *session.moon_orientation());
    const auto raan = targeter.parking_orbit_raan(epoch + m.flight_time_s, inclination_rad);
    if (!raan) return std::nullopt;
    // circular: the argument of latitude is the anomaly from the ascending node
    const orbit::KeplerianElements elements{constants::wgs84_semi_major_axis_m + m.parking_height_m, 0.0,
                                            inclination_rad, *raan, 0.0, m.argument_of_latitude_rad};
    return orbit::state_from_elements(elements, session.gm_earth_m3ps2());
}

MissionStart continue_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                            const config::LunarLandingMission& m, time::Epoch epoch) {
    const auto refuse = [&](const report::Report& reason) {
        session.disengage_autopilot();
        MissionStart s;
        s.kind = config::MissionKind::LunarLanding;
        s.epoch = epoch;
        s.refusal = reason;
        return s;
    };
    if (!session.moon_orientation()) return refuse(report::make(report::Code::lunar_landing_no_moon_orientation));
    if (session.epoch() > epoch + 0.5 * FlightSession::tick_s) {
        return refuse(report::make(report::Code::lunar_mission_too_late));
    }
    PlacedSite placed;
    try {
        placed = load_site(m, config_dir);
    } catch (const std::runtime_error& e) {  // terrain files missing
        return refuse(detail::exception_report(e));
    }
    set_site(session, placed);
    // A mission leg inside the flight (refuelling: the HLS after the depot): decided here, on the session's thread.
    return decided_now(session, plan_from_live_state(session, m, epoch, placed));
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
