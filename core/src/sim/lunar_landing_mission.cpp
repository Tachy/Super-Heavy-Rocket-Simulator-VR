#include "starsim/sim/lunar_landing_mission.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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

// [E] revolutions from the LLO arrival until the session is "in the LLO" and plans the descent: the first trim check
// 600 s after the LOI, then up to two apsis trims (FlightSession::llo_trim_interval_s, max_llo_trims).
constexpr double llo_settling_revolutions = 2.0;
// [D] NRHO variant: the LLO's wait for the site its [budget] legs were measured with (l4 Task 9 mission flight: site
// wait ~4 d, llo_to_nrho 986 m/s 5.3 d after the LLO arrival); the plane through the site is chosen within it
// (mission::nrho_site_departure) - a longer wait is not in the return leg's budget.
constexpr double nrho_site_wait_s = 4.0 * 86400.0;

struct PlacedSite {
    std::shared_ptr<const environment::LunarTerrain> terrain;  // LOLA: the site height, the planners
    std::shared_ptr<const environment::DetailedLunarTerrain> surface;  // the physics surface (all parts loaded)
    LunarSite site;
};

// The Moon's terrain (the data pipeline's directory next to config/: repository layout), the composed surface with
// every part (the session picks the fidelity flags from them) and the mission's site on it (`site_override`: another
// site than the mission's, e.g. a free one).
PlacedSite load_site(const config::LunarLandingMission& m, const std::filesystem::path& config_dir,
                     const config::LunarSiteConfig* site_override = nullptr) {
    const auto site = site_override ? *site_override : config::load_lunar_site(config_dir, m.site_id);
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
                 time::Epoch epoch, const config::VehicleConfig* vehicle, const config::LunarSiteConfig* site) {
    if (!session.moon_orientation()) {
        throw report::Refusal(report::make(report::Code::lunar_landing_no_moon_orientation));
    }
    const auto state = lunar_parking_state(session, m, epoch, m.parking_inclination_rad);
    if (!state) {
        throw report::Refusal(report::make(report::Code::lunar_mission_no_parking_plane));
    }
    const auto placed = load_site(m, config_dir, site);
    if (vehicle) {
        session.place_in_orbit(*vehicle, *state, epoch);
    } else {
        session.place_in_orbit(m.vehicle_id, *state, epoch);
    }
    set_site(session, placed);
    return placed;
}

MissionStart refused_start(FlightSession& session, time::Epoch epoch, const report::Report& reason) {
    session.disengage_autopilot();
    MissionStart s;
    s.kind = config::MissionKind::LunarLanding;
    s.epoch = epoch;
    s.return_planned = false;
    s.refusal = reason;
    return s;
}

// After the outbound plan and the touchdown check (`reachable`: a point was found): budget, legs.
MissionStart plan_after_touchdown_check(FlightSession& session, const config::LunarLandingMission& m, time::Epoch epoch,
                                        const mission::LunarOutboundParameters& outbound, bool reachable) {
    const auto refuse = [&](const report::Report& reason) { return refused_start(session, epoch, reason); };
    const auto setup = lunar_landing_setup(m);
    const auto& v = session.body(session.controlled_index()).vehicle();
    if (!reachable) {
        return refuse(report::with_cause(report::make(report::Code::lunar_mission_infeasible),
                                         report::make(report::Code::lunar_mission_no_touchdown_point)));
    }
    const auto* site = session.lunar_landing_site();
    const auto* outbound_plan = session.lunar_outbound_plan();
    const bool via_nrho = outbound.orbit == mission::LunarOrbitKind::Nrho;
    // NRHO: the site's share of the departure burn (ruling R12) - the NRHO stays as it is
    std::optional<mission::NrhoSiteDeparture> departure;
    if (via_nrho && site && session.moon_orientation()) {
        departure = lunar_nrho_site_departure(m, outbound, *outbound_plan, session.ephemeris(),
                                              *session.moon_orientation(), site->config, epoch);
    }
    // the Moon's rotation over the stay, turned back in the LLO before the return (ruling R15)
    const double stay_plane_change_mps = lunar_mission_stay_plane_change_mps(session, m, epoch, departure);
    const double mass_kg = v.mass_properties().mass_kg;
    const auto& sets = setup.guidance.braking_sets;
    const auto budget = lunar_mission_budget(m, *outbound_plan, mass_kg, mass_kg - v.propellant_kg(),
                                             gnc::groups_vacuum_isp_s(v, sets.front()),
                                             gnc::groups_vacuum_isp_s(v, sets.back()), session.settling_gas_bound_kg(),
                                             departure ? departure->extra_delta_v_mps : 0.0, stay_plane_change_mps);
    if (!budget.closes && via_nrho) {  // the site is out of the NRHO's reach on this budget
        const auto unreachable = report::make(
            report::Code::lunar_nrho_site_unreachable,
            {report::angle("latitude", site ? site->config.latitude_rad : 0.0), report::mass("budget_short", -budget.margin_kg)});
        return refuse(report::with_cause(report::make(report::Code::lunar_mission_infeasible),
                                         report::with_cause(unreachable, budget.reason)));
    }
    if (!budget.closes) return refuse(report::with_cause(report::make(report::Code::lunar_mission_infeasible), budget.reason));
    MissionStart s;
    s.kind = config::MissionKind::LunarLanding;
    s.epoch = epoch;
    s.lunar_budget_margin_kg = budget.margin_kg;
    s.lunar_stay_plane_change_mps = stay_plane_change_mps;
    LunarMissionSequence sequence;
    sequence.landing = setup;
    // the descent takes the pass the LLO plane was chosen for (or the mission's planned touchdown)
    sequence.landing.descent.planned_touchdown =
        outbound_plan->llo_touchdown ? outbound_plan->llo_touchdown : lunar_planned_touchdown(m, epoch);
    s.lunar_planned_touchdown = sequence.landing.descent.planned_touchdown;
    if (departure) s.lunar_planned_touchdown = departure->touchdown;
    sequence.via_nrho = via_nrho;
    sequence.nrho = outbound.nrho;
    if (!sequence.via_nrho) {
        sequence.llo = outbound.llo;
        sequence.llo.plane_normal_moon_inertial = outbound_plan->llo_plane_normal_moon_inertial;
    }
    session.set_lunar_mission(std::move(sequence));
    s.return_planned = true;
    return s;
}

// The approaches the touchdown point is checked for (ruling R13): the planned LLO plane's over the site at its
// touchdown; an inclination-targeted LLO or the NRHO's (polar, the descent waits for a pass): north- and southbound.
std::vector<double> approach_azimuths(const FlightSession& session, const config::LunarLandingMission& m,
                                      const PlacedSite& placed) {
    const auto* plan = session.lunar_outbound_plan();
    if (!plan || !plan->llo_plane_normal_moon_inertial || !plan->llo_touchdown || !session.moon_orientation()) {
        return {0.0, constants::pi};
    }
    return {mission::llo_approach_azimuth(*session.moon_orientation(), placed.site.config.latitude_rad,
                                          placed.site.config.longitude_rad, *plan->llo_touchdown,
                                          *plan->llo_plane_normal_moon_inertial,
                                          constants::moon_mean_radius_m + m.llo_altitude_m,
                                          session.ephemeris().gm_m3ps2(ephemeris::Body::Moon))};
}

// The mission from the controlled body as it is (after the placement, or the refuelled HLS): the outbound plan first
// (its LLO plane gives the approach over the site), then a touchdown point within the slope limit for that approach,
// then plan_after_touchdown_check. On the session's composed surface (its fidelity flags) with the hazard check, as
// the descent planner selects - that search runs on a background thread (a refused site checks every candidate: up to
// ~30 s); LOLA alone (milliseconds) is decided at once.
DeferredMissionStart plan_from_live_state(FlightSession& session, const config::LunarLandingMission& m,
                                          time::Epoch epoch, const PlacedSite& placed) {
    const auto outbound = std::make_shared<const mission::LunarOutboundParameters>(lunar_outbound_parameters(
        m, session.gravity(), session.ephemeris(), epoch, &placed.site.config));
    if (!session.plan_lunar_outbound(*outbound)) return {refused_start(session, epoch, session.lunar_status())};
    const auto setup = lunar_landing_setup(m);
    const auto& stage = session.body(session.controlled_index()).vehicle().stages().front().config;
    const double footprint_m = stage.landing_legs.footprint_radius_m;
    const double lat = placed.site.config.latitude_rad, lon = placed.site.config.longitude_rad;
    const auto azimuths = approach_azimuths(session, m, placed);
    auto surface = std::dynamic_pointer_cast<const environment::DetailedLunarTerrain>(session.moon_surface_shared());
    const bool hazards = surface && surface->detail_on() && stage.landing_legs.foot_pad_radius_m > 0.0;
    if (!hazards) {
        bool reachable = false;
        for (const double azimuth : azimuths) {
            reachable = reachable || mission::select_touchdown_point(*placed.terrain, lat, lon, azimuth, footprint_m,
                                                                     setup.descent)
                                         .has_value();
        }
        return {plan_after_touchdown_check(session, m, epoch, *outbound, reachable)};
    }
    auto search = std::make_shared<std::shared_future<std::optional<mission::TouchdownPoint>>>(
        mission::select_touchdown_point_async(std::move(surface), lat, lon, azimuths, footprint_m, setup.descent,
                                              mission::touchdown_hazard_limits(stage))
            .share());
    DeferredMissionStart d;
    d.status = report::make(report::Code::start_checking_landing_site);
    d.poll = [search, m, epoch, outbound](FlightSession& f) -> std::optional<MissionStart> {
        if (search->wait_for(std::chrono::seconds(0)) != std::future_status::ready) return std::nullopt;
        return plan_after_touchdown_check(f, m, epoch, *outbound, search->get().has_value());
    };
    d.wait = [search, m, epoch, outbound](FlightSession& f) {
        return plan_after_touchdown_check(f, m, epoch, *outbound, search->get().has_value());
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
    place(session, m, config_dir, epoch, vehicle, nullptr);
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
                                                   double approach_isp_s, double nrho_plane_change_mps,
                                                   double stay_plane_change_mps) {
    const auto setup = lunar_landing_setup(m);
    const bool nrho = m.lunar_orbit == "nrho";
    std::vector<mission::BudgetLeg> legs{{"tli", norm(plan.tli.delta_v_gcrf_mps), vacuum_isp_s}};
    if (nrho) {
        if (plan.powered_flyby) legs.push_back({"swing_by", norm(plan.powered_flyby->delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"insertion", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
        legs.push_back({"nrho_to_llo", m.nrho_to_llo_delta_v_mps, vacuum_isp_s, 2});  // departure + LLO arrival
        // the turn onto the plane through the site, in the departure burn (no ignition of its own)
        legs.push_back({"nrho_plane_change", nrho_plane_change_mps, vacuum_isp_s, 0});
    } else {
        legs.push_back({"loi", norm(plan.insertion.delta_v_gcrf_mps), vacuum_isp_s});
    }
    legs.push_back({"descent_braking", setup.braking_delta_v_mps, vacuum_isp_s, 2});  // DOI + PDI
    // the approach continues the braking burn
    legs.push_back({"descent_approach", m.descent_delta_v_mps - setup.braking_delta_v_mps, approach_isp_s, 0});
    legs.push_back({"ascent", m.ascent_delta_v_mps, vacuum_isp_s, 1});  // lit landed; the circularization in orbit
    // back onto the landing plane the return starts from (ruling R15), a burn of its own in the LLO
    if (stay_plane_change_mps > 0.0) legs.push_back({"stay_plane_change", stay_plane_change_mps, vacuum_isp_s, 1});
    if (nrho) legs.push_back({"llo_to_nrho", m.llo_to_nrho_delta_v_mps, vacuum_isp_s, 2});
    return legs;
}

double lunar_stay_plane_change_mps(const frames::BodyOrientation& moon, const config::LunarSiteConfig& site,
                                   const std::optional<math::Vec3>& llo_plane_normal_moon_inertial,
                                   time::Epoch touchdown, double stay_s, double llo_radius_m, double gm_moon_m3ps2) {
    if (stay_s <= 0.0) return 0.0;
    const math::Vec3 site_fixed{std::cos(site.latitude_rad) * std::cos(site.longitude_rad),
                                std::cos(site.latitude_rad) * std::sin(site.longitude_rad), std::sin(site.latitude_rad)};
    const math::Vec3 at_liftoff = moon.fixed_to_gcrf(touchdown + stay_s).rotation * site_fixed;
    double turn_rad = 0.0;
    if (llo_plane_normal_moon_inertial) {
        turn_rad = std::asin(std::min(std::abs(dot(math::normalized(*llo_plane_normal_moon_inertial), at_liftoff)), 1.0));
    } else {
        const math::Vec3 at_touchdown = moon.fixed_to_gcrf(touchdown).rotation * site_fixed;
        turn_rad = std::acos(std::clamp(dot(at_touchdown, at_liftoff), -1.0, 1.0));
    }
    return 2.0 * std::sqrt(gm_moon_m3ps2 / llo_radius_m) * std::sin(0.5 * turn_rad);
}

double lunar_mission_stay_plane_change_mps(const FlightSession& session, const config::LunarLandingMission& m,
                                           time::Epoch epoch,
                                           const std::optional<mission::NrhoSiteDeparture>& departure) {
    const auto* site = session.lunar_landing_site();
    const auto* moon = session.moon_orientation();
    if (!site || !moon || m.surface_stay_s <= 0.0) return 0.0;
    const double gm_moon = session.ephemeris().gm_m3ps2(ephemeris::Body::Moon);
    // the NRHO variant descends into the default LLO (no [lunar_orbit] altitude_km)
    const double altitude_m = m.lunar_orbit == "nrho" ? mission::LloTarget{}.altitude_m : m.llo_altitude_m;
    const double r_llo = constants::moon_mean_radius_m + altitude_m;
    const auto stay = [&](const std::optional<math::Vec3>& normal, time::Epoch touchdown) {
        return lunar_stay_plane_change_mps(*moon, site->config, normal, touchdown, m.surface_stay_s, r_llo, gm_moon);
    };
    if (departure) return stay(departure->llo_plane_normal_moon_inertial, departure->touchdown);
    const auto* plan = session.lunar_outbound_plan();
    if (plan && plan->llo_plane_normal_moon_inertial && plan->llo_touchdown) {
        return stay(plan->llo_plane_normal_moon_inertial, *plan->llo_touchdown);
    }
    const auto planned = lunar_planned_touchdown(m, epoch);
    return stay(std::nullopt, planned ? *planned
                                      : epoch + m.flight_time_s + lunar_first_pass_after_arrival_s(m, gm_moon));
}

size_t lunar_in_orbit_ignitions(const std::vector<mission::BudgetLeg>& legs) {
    size_t ignitions = 0;
    for (const auto& leg : legs) ignitions += static_cast<size_t>(std::max(leg.ignitions, 0));
    return ignitions;
}

mission::BudgetResult lunar_mission_budget(const config::LunarLandingMission& m, const mission::OutboundPlan& plan,
                                           double initial_mass_kg, double dry_mass_kg, double vacuum_isp_s,
                                           double approach_isp_s, double settling_gas_kg,
                                           double nrho_plane_change_mps, double stay_plane_change_mps) {
    const auto legs =
        lunar_mission_legs(m, plan, vacuum_isp_s, approach_isp_s, nrho_plane_change_mps, stay_plane_change_mps);
    return mission::lunar_budget(initial_mass_kg, dry_mass_kg,
                                 m.reserve_kg + settling_gas_kg * static_cast<double>(lunar_in_orbit_ignitions(legs)),
                                 legs);
}

DeferredMissionStart begin_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                                 const config::LunarLandingMission& m, time::Epoch epoch,
                                                 const config::VehicleConfig* vehicle,
                                                 const config::LunarSiteConfig* site) {
    PlacedSite placed;
    try {
        placed = place(session, m, config_dir, epoch, vehicle, site);
    } catch (const std::runtime_error& e) {  // no plane, no Moon orientation, terrain files missing
        session.disengage_autopilot();
        MissionStart s;
        s.kind = config::MissionKind::LunarLanding;
        s.epoch = epoch;
        s.refusal = detail::exception_report(e);
        return {s};
    }
    // The outbound plan (seconds; the NRHO reference ~15 s) runs at the first poll, so the status is published first
    // (Task 7 N2: the event line stayed blank while it ran inside this call).
    auto inner = std::make_shared<std::optional<DeferredMissionStart>>();
    const auto planned = [inner, m, epoch, placed](FlightSession& f) -> DeferredMissionStart& {
        if (!*inner) *inner = plan_from_live_state(f, m, epoch, placed);
        return **inner;
    };
    DeferredMissionStart d;
    d.status = report::make(report::Code::start_checking_landing_site);
    d.poll = [planned](FlightSession& f) -> std::optional<MissionStart> {
        auto& later = planned(f);
        if (later.done) return std::move(*later.done);
        return later.poll(f);
    };
    d.wait = [planned](FlightSession& f) { return decided_now(f, std::move(planned(f))); };
    return d;
}

MissionStart start_lunar_landing_mission(FlightSession& session, const std::filesystem::path& config_dir,
                                         const config::LunarLandingMission& m, time::Epoch epoch,
                                         const config::VehicleConfig* vehicle, const config::LunarSiteConfig* site) {
    return decided_now(session, begin_lunar_landing_mission(session, config_dir, m, epoch, vehicle, site));
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

std::optional<time::Epoch> lunar_planned_touchdown(const config::LunarLandingMission& m, time::Epoch epoch) {
    if (!m.touchdown_after_epoch_s) return std::nullopt;
    return epoch + *m.touchdown_after_epoch_s;
}

namespace {
double llo_period_s(const config::LunarLandingMission& m, double gm_moon) {
    // the NRHO variant descends into the default LLO (no [lunar_orbit] altitude_km)
    const double altitude_m = m.lunar_orbit == "nrho" ? mission::LloTarget{}.altitude_m : m.llo_altitude_m;
    const double r = constants::moon_mean_radius_m + altitude_m;
    return constants::two_pi * std::sqrt(r * r * r / gm_moon);
}
}  // namespace

double lunar_llo_settling_s(const config::LunarLandingMission& m, double gm_moon) {
    return llo_settling_revolutions * llo_period_s(m, gm_moon);
}

double lunar_first_pass_after_arrival_s(const config::LunarLandingMission& m, double gm_moon) {
    const mission::LunarDescentParameters descent;
    return lunar_llo_settling_s(m, gm_moon) + descent.min_lead_s + 1.25 * llo_period_s(m, gm_moon);
}

std::optional<mission::NrhoSiteDeparture> lunar_nrho_site_departure(const config::LunarLandingMission& m,
                                                                    const mission::LunarOutboundParameters& outbound,
                                                                    const mission::OutboundPlan& plan,
                                                                    const ephemeris::Ephemeris& ephemeris,
                                                                    const frames::BodyOrientation& moon,
                                                                    const config::LunarSiteConfig& site,
                                                                    time::Epoch epoch) {
    if (outbound.orbit != mission::LunarOrbitKind::Nrho || !outbound.nrho) return std::nullopt;
    return mission::nrho_site_departure(*outbound.nrho, ephemeris, moon, plan.insertion.epoch, site.latitude_rad,
                                        site.longitude_rad, outbound.llo.altitude_m, lunar_planned_touchdown(m, epoch),
                                        lunar_first_pass_after_arrival_s(m, ephemeris.gm_m3ps2(ephemeris::Body::Moon)),
                                        nrho_site_wait_s);
}

mission::LunarOutboundParameters lunar_outbound_parameters(const config::LunarLandingMission& m,
                                                           const environment::GravityModel& gravity,
                                                           const ephemeris::Ephemeris& ephemeris, time::Epoch epoch,
                                                           const config::LunarSiteConfig* site) {
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
        p.llo.altitude_m = m.llo_altitude_m;
        if (m.llo_inclination_rad) {
            p.llo.inclination_rad = *m.llo_inclination_rad;
        } else if (site) {  // the plane is free (user 2026-11-06): chosen through the site at the planned touchdown
            p.llo_site = mission::LloSiteTarget{
                site->latitude_rad, site->longitude_rad, lunar_planned_touchdown(m, epoch),
                lunar_first_pass_after_arrival_s(m, ephemeris.gm_m3ps2(ephemeris::Body::Moon))};
        }
    }
    return p;
}

}  // namespace starsim::sim
