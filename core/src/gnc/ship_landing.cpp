#include "starsim/gnc/ship_landing.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/mat3.hpp"

namespace starsim::gnc {

using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;
constexpr double no_sink_limit_mps = 1.0e3;  // an engine-set switch without a sink window
constexpr double catch_missed_height_m = 5.0;  // CG this far below the catch point: the catch is over (as the booster)

Vec3 up_at(double latitude_rad, double longitude_rad) {
    return {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
}

}  // namespace

ShipLandingParameters ship_landing_parameters() {
    ShipLandingParameters p;
    auto& s = p.scvx;
    s.snapshot.phase_groups = {std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2", "ship/sea_level_3"},
                               std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2"},
                               std::vector<std::string>{"ship/sea_level_1", "ship/sea_level_2"}};
    s.snapshot.reserve_kg = 1000.0;  // [E] the ship's landing propellant is small (header tanks)
    // [E] lateral LQR effort: the booster's 0.2 swung the ship's gimbals stop to stop after the flip (20 deg/s slew)
    s.lateral_effort_fraction = 0.1;
    auto& l = s.settings.limits;
    // [E] the more engines, the more gimbal authority. The lateral phase took 15 deg until f3: it saturated on the
    // divert after the flip (lunar return seed 1, ~27 m/s across: overshoot, hover, headers dry).
    l.tilt_max_rad = {20.0 * deg, 20.0 * deg, 15.0 * deg};
    l.rate_max_radps = {0.3, 0.2, 0.2};                     // [E] the ship is shorter and lighter than the booster
    // 3 -> 2 at 20-40 m/s sink [D]: flight 14 video shows two engines from ~200 m (user 2026-10-13), where flights
    // 13/14 sank at 31 / 35 m/s (webcast telemetry). The planner's third phase flies the same pair (no switch).
    l.switches = {{{0.0, 0.0, 20.0, 40.0}, {0.0, 0.0, 0.0, no_sink_limit_mps}}};
    l.throttle_margin = 0.05;          // [E]
    l.terminal_thrust_margin = 1.0;    // no "gentle end" cap: two engines at minimum are below the weight
    return p;
}

EntryTarget ship_entry_target(const LandingSite& site, const ShipLandingParameters& parameters) {
    const Vec3 pad = frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m});
    EntryTarget target{pad, site.height_m, {}};
    if (!site.catch_target) return target;
    const auto& c = *site.catch_target;
    const Vec3 up = up_at(site.latitude_rad, site.longitude_rad);
    Vec3 to_tower = c.tower_center_itrf_m - c.pin_point_itrf_m;
    to_tower = math::normalized(to_tower - up * dot(to_tower, up));
    const Vec3 axis = c.pin_point_itrf_m - up * dot(c.pin_point_itrf_m - pad, up);  // catch axis at the pad height
    target.position_itrf_m = axis - to_tower * parameters.gate_lead_m;
    target.nose_direction_itrf = to_tower;
    return target;
}

ShipLandingGuidance::ShipLandingGuidance(LandingSite site, ShipLandingParameters parameters)
    : site_(std::move(site)), parameters_(std::move(parameters)) {}

void ShipLandingGuidance::set_realtime(bool realtime) {
    parameters_.scvx.realtime = realtime;
    if (scvx_) scvx_->set_realtime(realtime);
}

ReturnCommand ShipLandingGuidance::update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                          time::Epoch epoch) {
    ReturnCommand cmd;
    const auto transform = env.earth_orientation.fixed_to_gcrf(epoch);
    const Vec3 site_up = up_at(site_.latitude_rad, site_.longitude_rad);
    const Vec3 up_gcrf = transform.rotation * site_up;
    if (!started_) {
        started_ = true;
        next_attempt_ = epoch;
        flip_start_ = epoch;
    }
    if (phase_ == ReturnPhase::Flip) {
        cmd.run_groups = parameters_.flip_groups;
        cmd.throttle = parameters_.flip_throttle;
        cmd.axis_gcrf = up_gcrf;  // engines down
        if (site_.catch_target) {
            // pins across the arms already in the flip, while three engines and the flaps can roll the ship (two
            // off-axis engines in the burn have little roll authority)
            const Vec3 body_y = transpose(transform.rotation) * body.state().attitude.rotate(Vec3{0.0, 1.0, 0.0});
            cmd.roll_reference_gcrf = transform.rotation * catch_roll_reference(site_.catch_target->across_itrf, site_up,
                                                                                site_.catch_target->pin_azimuth_rad, body_y);
        }
        const Vec3 axis = body.state().attitude.rotate(Vec3{1.0, 0.0, 0.0});
        const double tilt = std::acos(std::clamp(dot(axis, up_gcrf), -1.0, 1.0));
        const Vec3 w = body.state().angular_velocity_body_radps;
        const bool settled = std::hypot(w.y, w.z) < parameters_.flip_settled_rate_radps;  // the plans' rate cone
        if (std::getenv("STARSIM_DEBUG_FLIP")) {  // diagnostics
            double spool = 0.0, thrust = 0.0;
            for (const auto& e : body.vehicle().stages().front().engines) {
                if (e.producing_thrust()) thrust += e.spool * e.performance->thrust_n(e.throttle, 0.0);
                spool = std::max(spool, e.spool);
            }
            std::printf("[flip] t %.2f tilt %.1f rate %.3f spool %.2f thrust %.2f MN flaps", epoch - flip_start_,
                        tilt * constants::rad_to_deg, std::hypot(w.y, w.z), spool, thrust / 1e6);
            for (const auto& f : body.vehicle().stages().front().flaps) {
                std::printf(" %.0f", f.deflection_rad * constants::rad_to_deg);
            }
            std::printf(" w %.3f/%.3f\n", w.y, w.z);
        }
        if (tilt > parameters_.flip_flaps_until_tilt_rad) {
            const auto& stage = body.vehicle().stages().front();
            const double cg = body.vehicle().mass_properties().cg_station_m;
            for (const auto& f : stage.flaps) {
                const bool forward = 0.5 * (f.geometry.station_fwd_m + f.geometry.station_aft_m) > cg;
                cmd.flap_rad.push_back(forward ? f.max_deflection_rad : 0.0);
            }
        }
        if (tilt < parameters_.flip_tolerance_rad && flip_duration_s_ == 0.0) {
            flip_duration_s_ = epoch - flip_start_;
            const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
            flip_end_height_m_ = frames::fixed_to_geodetic(r_fixed).height_m - site_.height_m;
        }
        if (tilt < parameters_.flip_tolerance_rad && settled && epoch >= next_attempt_ && site_.catch_target) {
            ++first_plan_attempts_;
            auto guidance = std::make_unique<ScvxLandingGuidance>(site_, parameters_.scvx);
            if (abort_requested_) guidance->request_abort();
            if (guidance->start(body, env, epoch)) {
                scvx_ = std::move(guidance);
                phase_ = ReturnPhase::LandingBurn;
            } else {
                next_attempt_ = epoch + parameters_.first_plan_retry_s;
            }
        }
    }
    if (phase_ == ReturnPhase::LandingBurn) {
        const auto c = scvx_->update(body, env, epoch);
        cmd.run_groups = c.run_groups;
        cmd.throttle = c.throttle;
        cmd.axis_gcrf = c.axis_gcrf;
        cmd.roll_reference_gcrf = c.roll_reference_gcrf;
        cmd.feedforward = c.feedforward;
        cmd.torque_body_nm = c.torque_body_nm;
        cmd.scvx = true;
        cmd.abort = c.abort;
        cmd.time_to_go_s = c.time_to_go_s;
        cmd.impact_miss_m = c.offset_m;
        // [E] flaps folded against the hull in the burn: the plans' model has no flaps
        cmd.flap_rad.assign(body.vehicle().stages().front().flaps.size(), 0.0);
        const auto mass = body.vehicle().mass_properties();
        const Vec3 r_fixed = transpose(transform.rotation) * body.state().position_gcrf_m;
        const Vec3 r_catch = site_.catch_target->pin_point_itrf_m -
                             site_up * (site_.catch_target->pin_station_m - mass.cg_station_m);
        const bool catch_over = !c.abort && (catch_contact_ || dot(r_fixed - r_catch, site_up) < -catch_missed_height_m);
        if (catch_over) phase_ = ReturnPhase::Landed;
    }
    if (body.resting() || body.impacted()) phase_ = ReturnPhase::Landed;
    if (phase_ == ReturnPhase::Landed) {
        cmd.run_groups.clear();
        cmd.axis_gcrf = up_gcrf;
    }
    cmd.phase = phase_;
    return cmd;
}

bool ShipLandingGuidance::request_abort() {
    if (!site_.sea || phase_ == ReturnPhase::Landed || catch_contact_) return false;
    if (scvx_) return abort_requested_ = scvx_->request_abort();
    abort_requested_ = true;  // still flipping: the first plan goes to the sea
    return true;
}

}  // namespace starsim::gnc
