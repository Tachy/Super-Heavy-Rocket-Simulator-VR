#pragma once

// Helpers for return/landing scenarios: a booster with a chosen propellant load, placed over the launch site.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/primitives.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"

namespace starsim::test {

// Single-stage booster of `vehicle_id` with its main tanks scaled to `propellant_kg` in total (RCS supplies stay full).
inline config::VehicleConfig booster_config(const std::string& vehicle_id, double propellant_kg) {
    auto cfg = config::load_vehicle(STARSIM_CONFIG_DIR, vehicle_id);
    cfg.stages.resize(1);
    double total = 0.0;
    for (const auto& tank : cfg.stages[0].tanks) total += tank.contents == config::TankContents::Rcs ? 0.0 : tank.mass_kg;
    for (auto& tank : cfg.stages[0].tanks) {
        if (tank.contents != config::TankContents::Rcs) tank.mass_kg *= propellant_kg / total;
    }
    return cfg;
}

// Booster state over `site`: CG at `cg_enu_m` (east, north, up from the pad centre on the ground), velocity relative
// to the ground, body x up and body +y east, then rotated by `tilt_enu` (site frame); co-rotating with the Earth.
inline vehicle::RigidBodyState state_over_site(const sim::LaunchSite& site, const frames::BodyOrientation& earth,
                                               time::Epoch epoch, const math::Vec3& cg_enu_m,
                                               const math::Vec3& velocity_enu_mps, const math::Quat& tilt_enu = {}) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    const math::Mat3 enu_to_itrf = contact::from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
    const math::Vec3 origin = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    const math::Mat3 body_to_enu =
        tilt_enu.to_matrix() * contact::from_columns({0.0, 0.0, 1.0}, {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0});
    const auto t = earth.fixed_to_gcrf(epoch);
    const math::Vec3 r_itrf = origin + enu_to_itrf * cg_enu_m;
    vehicle::RigidBodyState s;
    s.position_gcrf_m = t.position(r_itrf);
    s.velocity_gcrf_mps = t.velocity(r_itrf, enu_to_itrf * velocity_enu_mps);
    s.attitude = math::Quat::from_matrix(t.rotation * enu_to_itrf * body_to_enu);
    s.angular_velocity_body_radps = s.attitude.inverse_rotate(vehicle::earth_rotation_vector(t));
    return s;
}

// CG height above the pad at which the return guidance's own burn prediction ignites for a booster falling straight
// down at `sink_mps` (stop height 0), by bisection.
inline double ignition_height_m(const sim::FlightSession& session, const config::VehicleConfig& cfg,
                                const frames::BodyOrientation& earth, time::Epoch epoch, double sink_mps,
                                const gnc::BoosterReturnParameters& parameters = {}) {
    double low = 200.0, high = 8000.0;  // [m] bracket
    for (int i = 0; i < 40; ++i) {
        const double h = 0.5 * (low + high);
        const vehicle::FlightBody body(std::make_unique<vehicle::Vehicle>(cfg, STARSIM_CONFIG_DIR),
                                       state_over_site(session.launch_site(), earth, epoch, {0.0, 0.0, h}, {0.0, 0.0, -sink_mps}));
        const gnc::BoosterReturnGuidance guidance(session.return_site(body.vehicle()), parameters);
        (guidance.predict_landing_burn(body, session.environment(), epoch).stop_height_m > 0.0 ? high : low) = h;
    }
    return 0.5 * (low + high);
}

// End of a landing: first pin contact (offset of the CG from the catch axis, horizontal and vertical speed, site
// frame) and how it ended.
struct CatchRun {
    bool contact = false;
    double offset_m = 0.0;
    double lateral_mps = 0.0;
    double sink_mps = 0.0;
    sim::CatchPhase tower_phase = sim::CatchPhase::Open;
    bool tower_struck = false;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
    sim::ReturnResult result = sim::ReturnResult::Flying;  // ship returns: caught / upright abort / lost
    gnc::ScvxLandingGuidance::Statistics scvx;  // of the SCvx landing burn, booster or ship (zero without one)
    bool caught_within_tolerance() const {
        return tower_phase == sim::CatchPhase::Caught && contact && offset_m < 1.0 && lateral_mps < 0.5;
    }
};

// Advances body 0 tick by tick until it is caught, rests, impacts or `max_s` pass.
inline CatchRun fly_landing(sim::FlightSession& session, const frames::BodyOrientation& earth, time::Epoch t0, double max_s) {
    CatchRun run;
    const auto& booster = session.body(0);
    const auto* tower = session.tower();
    for (double t = sim::FlightSession::tick_s; t <= max_s; t += sim::FlightSession::tick_s) {
        session.advance_to(t0 + t, 10);
        if (std::getenv("STARSIM_DEBUG_CATCH") && tower && static_cast<int>(std::lround(t / sim::FlightSession::tick_s)) % 20 == 0) {  // diagnostics
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto& s = booster.state();
            const auto& g = tower->geometry();
            const math::Vec3 p = g.to_site(transpose(tr.rotation) * s.position_gcrf_m);
            const math::Vec3 v = transpose(g.site_to_itrf()) * transpose(tr.rotation) *
                                 (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            const auto* r = session.return_status(0);
            std::printf("  [catch] t %.1f cg %.2f,%.2f,%.1f v %.2f,%.2f,%.1f tower %s open %.3f contact %d thr %.2f groups %zu abort %d\n", t,
                        p.x, p.y, p.z, v.x, v.y, v.z, sim::to_string(tower->phase()), tower->arms().opening_rad(),
                        tower->in_contact(booster), r ? r->throttle : -1.0, r ? r->run_groups.size() : 0, r ? r->abort : 0);
        }
        if (!run.contact && tower && tower->in_contact(booster)) {
            const auto tr = earth.fixed_to_gcrf(session.epoch());
            const auto& s = booster.state();
            const auto& g = tower->geometry();
            const math::Vec3 p = g.to_site(transpose(tr.rotation) * s.position_gcrf_m);
            const math::Vec3 v = transpose(g.site_to_itrf()) * transpose(tr.rotation) *
                                 (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            run.contact = true;
            run.offset_m = std::hypot(p.x, p.y);
            run.lateral_mps = std::hypot(v.x, v.y);
            run.sink_mps = -v.z;
        }
        if (booster.caught() || booster.impacted() || booster.resting()) break;
    }
    if (tower) {
        run.tower_phase = tower->phase();
        run.tower_struck = tower->tower_struck();
    }
    run.outcome = session.landing_outcome(0);
    if (const auto* g = session.return_guidance(0); g && g->scvx()) {
        run.scvx = g->scvx()->statistics();
    } else if (const auto* s = session.ship_landing(0); s && s->scvx()) {
        run.scvx = s->scvx()->statistics();
    }
    // The LQR's clipping (the descent model's gaps show here): one line per landing burn for the campaign tables.
    std::printf("  [scvx clip] %d plans (%d failed), gimbal use <= %.2f (clipped %d of %d ticks), last 10 s <= %.2f (%d of %d)\n",
                run.scvx.plans, run.scvx.failures, run.scvx.peak_gimbal_use, run.scvx.clipped_ticks, run.scvx.lateral_ticks,
                run.scvx.terminal_peak_gimbal_use, run.scvx.terminal_clipped_ticks, run.scvx.terminal_ticks);
    return run;
}

// End of a landing on legs: first foot contact (CG offset from the zone centre, sink, lateral speed, site frame), the
// leg judgement and the session's outcome.
struct LegRun {
    bool contact = false;
    double contact_offset_m = 0.0;
    double contact_sink_mps = 0.0;
    double contact_lateral_mps = 0.0;
    double legs_at_contact = 0.0;  // deployment fraction
    double offset_m = 0.0;         // where it ended
    double propellant_kg = 0.0;    // left at the end
    sim::LegTouchdown touchdown = sim::LegTouchdown::None;
    sim::LandingOutcome outcome = sim::LandingOutcome::Flying;
    gnc::ScvxLandingGuidance::Statistics scvx;  // of the SCvx landing burn (zero without one)
};

inline LegRun fly_to_touchdown(sim::FlightSession& session, const frames::BodyOrientation& earth, time::Epoch t0,
                               double max_s) {
    LegRun run;
    const auto& stage = session.body(0);
    const auto zone = session.launch_site();
    auto enu = [&](math::Vec3* velocity) {
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const auto& s = stage.state();
        if (velocity) {
            const math::Vec3 v_fixed = transpose(tr.rotation) * (s.velocity_gcrf_mps - cross(vehicle::earth_rotation_vector(tr), s.position_gcrf_m));
            const math::Vec3 origin = frames::geodetic_to_fixed({zone.latitude_rad, zone.longitude_rad, zone.height_m});
            *velocity = sim::site_enu_m(zone, origin + v_fixed);
        }
        return sim::site_enu_m(zone, transpose(tr.rotation) * s.position_gcrf_m);
    };
    for (double t = sim::FlightSession::tick_s; t <= max_s; t += sim::FlightSession::tick_s) {
        session.advance_to(t0 + t, 10);
        if (!run.contact && session.leg_landing().in_contact(stage)) {
            math::Vec3 v;
            const math::Vec3 p = enu(&v);
            run.contact = true;
            run.contact_offset_m = std::hypot(p.x, p.y);
            run.contact_sink_mps = -v.z;
            run.contact_lateral_mps = std::hypot(v.x, v.y);
            run.legs_at_contact = stage.vehicle().leg_deployment();
        }
        if (session.leg_touchdown(0) != sim::LegTouchdown::None || stage.impacted()) break;
    }
    const math::Vec3 p = enu(nullptr);
    run.offset_m = std::hypot(p.x, p.y);
    run.propellant_kg = stage.vehicle().propellant_kg();
    run.touchdown = session.leg_touchdown(0);
    run.outcome = session.landing_outcome(0);
    if (const auto* g = session.return_guidance(0); g && g->scvx()) run.scvx = g->scvx()->statistics();
    return run;
}

}  // namespace starsim::test
