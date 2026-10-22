#include "starsim/sim/catch_tower_sim.hpp"

#include <cstdio>
#include <cstdlib>

#include <algorithm>
#include <cmath>
#include <limits>

#include "starsim/contact/tower_collision.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::sim {

using math::Vec3;

namespace {

constexpr double rest_speed_mps = 0.1;      // caught: body and carriage this slow ...
constexpr double rest_rate_radps = 0.01;
constexpr double rest_time_s = 2.0;         // ... for this long, engines off, both pins on the arms
constexpr double missed_drop_m = 5.0;       // pins this far below the arm tops: the catch is lost
constexpr double tower_zone_m = 1000.0;     // bodies closer than this need fixed ticks
// [E] The tower serves a catchable body only inside this zone (CG horizontally from the catch axis, height above the
// arm tops): a stage with pins elsewhere (e.g. the ship after staging) is neither tracked nor judged.
constexpr double catch_zone_radius_m = 2000.0;
constexpr double catch_zone_height_m = 5000.0;
// Bound on a coasting body's acceleration towards the tower (gravity; drag only slows the fall).
constexpr double coast_acceleration_bound_mps2 = 10.0;
// The arms are force limited (they stall against the hull): damage comes from the hull being driven into an arm.
constexpr double hull_damage_force_n = 6.0e6;  // [E] hull against one arm, ~3x the arm actuator force
constexpr double hull_overlap_m = 1.0;         // [E] gross overlap of hull and arm the contact points cannot see

}  // namespace

const char* to_string(CatchPhase phase) {
    switch (phase) {
        case CatchPhase::Open: return "offen";
        case CatchPhase::Standby: return "bereit";
        case CatchPhase::Closing: return "schließt";
        case CatchPhase::Caught: return "gefangen";
        case CatchPhase::Missed: return "verfehlt";
        case CatchPhase::Aborted: return "Abbruch";
    }
    return "";
}

CatchTowerSim::CatchTowerSim(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m)
    : geometry_(config, latitude_rad, longitude_rad, height_m), arms_(config) {}

void CatchTowerSim::close_arms(bool instantly) {
    if (instantly) arms_.set_closed();
    arms_.command_close(true);
    if (phase_ == CatchPhase::Open || phase_ == CatchPhase::Standby) phase_ = CatchPhase::Closing;
}

void CatchTowerSim::standby_arms(bool instantly) {
    if (instantly) arms_.set_standby();
    arms_.command(contact::ArmCommand::Standby);
    if (phase_ == CatchPhase::Open) phase_ = CatchPhase::Standby;
}

void CatchTowerSim::abort_catch() {
    if (phase_ == CatchPhase::Caught) return;
    arms_.command_close(false);
    phase_ = CatchPhase::Aborted;
}

bool CatchTowerSim::catchable(const vehicle::FlightBody& body) {
    const auto& stages = body.vehicle().stages();
    return stages.size() == 1 && !stages.front().config.catch_pins.azimuths_rad.empty() && !body.clamped() &&
           !body.impacted() && !body.caught();
}

contact::SitePose CatchTowerSim::site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const {
    const math::Mat3 to_fixed = transpose(t.rotation);
    const math::Mat3 to_site = transpose(geometry_.site_to_itrf()) * to_fixed;
    const Vec3 omega = vehicle::earth_rotation_vector(t);
    const auto& s = body.state();
    contact::SitePose p;
    p.cg_m = geometry_.to_site(to_fixed * s.position_gcrf_m);
    p.velocity_mps = to_site * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    p.body_to_site = to_site * s.attitude.to_matrix();
    p.angular_velocity_radps = to_site * (s.attitude.rotate(s.angular_velocity_body_radps) - omega);
    p.cg_station_m = body.vehicle().mass_properties().cg_station_m;
    return p;
}

std::array<contact::ArmContactState, 2> CatchTowerSim::arm_states() const {
    std::array<contact::ArmContactState, 2> states;
    for (int i = 0; i < 2; ++i) {
        const int side = i == 0 ? 1 : -1;
        auto& st = states[static_cast<size_t>(i)];
        st.box = geometry_.arm_box(side, arms_.opening_rad(), arms_.carriage_offset_m());
        st.hinge_m = geometry_.hinge(side) + Vec3{0.0, 0.0, arms_.carriage_offset_m()};
        st.yaw_rate_radps = -side * arms_.opening_rate_rad_s();  // arm direction turns by -side * dopening/dt
        st.carriage_velocity_mps = arms_.carriage_velocity_mps();
        st.outward = geometry_.across() * side;
    }
    return states;
}

CatchTowerSim::Track& CatchTowerSim::track(const vehicle::FlightBody& body) {
    auto it = tracks_.find(&body);
    if (it == tracks_.end()) {
        const auto& stage = body.vehicle().stages().front();
        Track t;
        t.geometry = contact::make_catch_geometry(stage.config.catch_pins, stage.base_station_m,
                                                  0.5 * body.vehicle().diameter_m(), geometry_.config().arm_thickness_m);
        it = tracks_.emplace(&body, t).first;
    }
    return it->second;
}

void CatchTowerSim::before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                time::Epoch epoch) {
    const auto t = earth.fixed_to_gcrf(epoch);
    const auto arms = arm_states();
    const math::Mat3 site_to_gcrf = t.rotation * geometry_.site_to_itrf();
    hull_push_n_ = 0.0;
    vertical_load_n_ = caught_load_n_;
    for (auto* body : bodies) {
        if (!catchable(*body)) continue;
        const auto pose = site_pose(*body, t);
        if (tracks_.count(body) == 0 &&
            (std::hypot(pose.cg_m.x, pose.cg_m.y) > catch_zone_radius_m ||
             pose.cg_m.z - geometry_.arm_top_height_m(arms_.carriage_offset_m()) > catch_zone_height_m)) {
            continue;  // not (yet) approaching this tower
        }
        auto& tr = track(*body);
        tr.contact = contact::catch_contact(tr.geometry, pose, arms);
        if (tr.contact.any_contact() && !tr.first_contact) tr.first_contact = epoch;  // the catch moment
        if (tr.contact.any_contact()) {
            body->apply_external_wrench(site_to_gcrf * tr.contact.force_n,
                                        body->state().attitude.inverse_rotate(site_to_gcrf * tr.contact.torque_nm));
        }
        hull_push_n_ = std::max({hull_push_n_, tr.contact.hull_push_n[0], tr.contact.hull_push_n[1]});
        vertical_load_n_ += tr.contact.vertical_load_n;
        max_pin_load_n_ = std::max(max_pin_load_n_, tr.contact.max_pin_load_n);
        if (phase_ == CatchPhase::Open || phase_ == CatchPhase::Standby) {
            const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
            const double pin_height = pin_axis.z - geometry_.arm_top_height_m(arms_.carriage_offset_m());
            const auto command = contact::arm_command(trigger_, pin_height, std::hypot(pose.cg_m.x, pose.cg_m.y),
                                                      -pose.velocity_mps.z, arms_.final_closing_time_s());
            if (command == contact::ArmCommand::Closed) {
                close_arms(false);
            } else {
                arms_.command(command);
                phase_ = command == contact::ArmCommand::Standby ? CatchPhase::Standby : CatchPhase::Open;
            }
        }
    }
}

void CatchTowerSim::after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                               time::Epoch epoch_end, double dt_s) {
    arms_.step(dt_s, hull_push_n_, vertical_load_n_);
    const auto t = earth.fixed_to_gcrf(epoch_end);
    for (auto* body : bodies) {
        if (body->clamped() || body->caught()) continue;
        const bool was_tracked = tracks_.count(body) > 0;
        if (!body->impacted()) {
            const auto pose = site_pose(*body, t);
            const Vec3 bottom = pose.cg_m + pose.body_to_site * Vec3{-pose.cg_station_m, 0.0, 0.0};
            const Vec3 top = pose.cg_m + pose.body_to_site * Vec3{body->vehicle().length_m() - pose.cg_station_m, 0.0, 0.0};
            if (contact::hits_tower(geometry_, bottom, top, 0.5 * body->vehicle().diameter_m())) {
                body->mark_impacted();
                tower_struck_ = true;
                if (std::getenv("STARSIM_DEBUG_CATCH")) std::printf("[tower] hull hits tower/mount: cg %.2f,%.2f,%.2f bottom %.2f top %.2f opening %.3f\n", pose.cg_m.x, pose.cg_m.y, pose.cg_m.z, bottom.z, top.z, arms_.opening_rad());  // diagnostics
            }
            if (was_tracked) {
                auto& tr = tracks_.at(body);
                if (tr.contact.max_pin_load_n > tr.geometry.max_pin_load_n || arms_.hit_hard_stop()) body->mark_impacted();
                const double radius = 0.5 * body->vehicle().diameter_m();
                for (const int side : {1, -1}) {
                    const auto arm = geometry_.arm_box(side, arms_.opening_rad(), arms_.carriage_offset_m());
                    if (contact::hull_penetration_m(arm, bottom, top, radius) > hull_overlap_m ||
                        std::max(tr.contact.hull_push_n[0], tr.contact.hull_push_n[1]) > hull_damage_force_n) {
                        body->mark_impacted();
                        tower_struck_ = true;
                        if (std::getenv("STARSIM_DEBUG_CATCH")) std::printf("[tower] arm %d in hull: cg %.2f,%.2f,%.2f bottom %.2f opening %.3f\n", side, pose.cg_m.x, pose.cg_m.y, pose.cg_m.z, bottom.z, arms_.opening_rad());  // diagnostics
                    }
                }
                const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
                if (pin_axis.z < geometry_.arm_top_height_m(arms_.carriage_offset_m()) - missed_drop_m &&
                    phase_ != CatchPhase::Caught && phase_ != CatchPhase::Aborted) {
                    phase_ = CatchPhase::Missed;
                }
                const bool at_rest = tr.contact.all_pins_in_contact() && !body->vehicle().any_engine_producing_thrust() &&
                                     norm(pose.velocity_mps) < rest_speed_mps &&
                                     norm(pose.angular_velocity_radps) < rest_rate_radps &&
                                     std::abs(arms_.carriage_velocity_mps()) < rest_speed_mps;
                tr.at_rest_s = at_rest ? tr.at_rest_s + dt_s : 0.0;
                if (tr.at_rest_s >= rest_time_s && !body->impacted()) {
                    caught_load_n_ += body->vehicle().mass_properties().mass_kg * constants::standard_gravity_mps2;
                    body->hold_fixed_to_earth(earth, epoch_end);
                    phase_ = CatchPhase::Caught;
                }
            }
        }
        if (was_tracked && (body->impacted() || body->resting()) && phase_ != CatchPhase::Caught &&
            phase_ != CatchPhase::Aborted) {
            phase_ = CatchPhase::Missed;
        }
    }
}

std::optional<time::Epoch> CatchTowerSim::first_contact_epoch(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    if (it == tracks_.end()) return std::nullopt;
    return it->second.first_contact;
}

bool CatchTowerSim::in_contact(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it != tracks_.end() && it->second.contact.any_contact();
}

double CatchTowerSim::coast_limit_s(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                    time::Epoch epoch) const {
    const auto t = earth.fixed_to_gcrf(epoch);
    const Vec3 omega = vehicle::earth_rotation_vector(t);
    double limit = std::numeric_limits<double>::infinity();
    for (const auto* body : bodies) {
        if (body->clamped() || body->impacted() || body->caught() || body->resting()) continue;
        const auto& s = body->state();
        const double gap = norm(geometry_.to_site(transpose(t.rotation) * s.position_gcrf_m)) - tower_zone_m;
        if (gap <= 0.0) return 0.0;
        // Earliest entry: gap = v t + a t^2 / 2 with the speed over the ground and the acceleration bound.
        const double v = norm(s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
        const double a = coast_acceleration_bound_mps2;
        limit = std::min(limit, (std::sqrt(v * v + 2.0 * a * gap) - v) / a);
    }
    return limit;
}

}  // namespace starsim::sim
