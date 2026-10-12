#include "starsim/sim/catch_tower_sim.hpp"

#include <algorithm>
#include <cmath>

#include "starsim/contact/tower_collision.hpp"

namespace starsim::sim {

using math::Vec3;

namespace {

constexpr double rest_speed_mps = 0.1;      // caught: body and carriage this slow ...
constexpr double rest_rate_radps = 0.01;
constexpr double rest_time_s = 2.0;         // ... for this long, engines off, both pins on the arms
constexpr double missed_drop_m = 5.0;       // pins this far below the arm tops: the catch is lost
constexpr double tower_zone_m = 1000.0;     // bodies closer than this need fixed ticks
constexpr double hull_crush_depth_m = 0.2;  // [E] an arm this deep in the hull: struck it (centred catch: 0.3 m clear)

}  // namespace

const char* to_string(CatchPhase phase) {
    switch (phase) {
        case CatchPhase::Open: return "offen";
        case CatchPhase::Closing: return "schließt";
        case CatchPhase::Caught: return "gefangen";
        case CatchPhase::Missed: return "verfehlt";
    }
    return "";
}

CatchTowerSim::CatchTowerSim(const config::TowerConfig& config, double latitude_rad, double longitude_rad, double height_m)
    : geometry_(config, latitude_rad, longitude_rad, height_m), arms_(config) {}

void CatchTowerSim::close_arms(bool instantly) {
    if (instantly) arms_.set_closed();
    arms_.command_close(true);
    if (phase_ == CatchPhase::Open) phase_ = CatchPhase::Closing;
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
    vertical_load_n_ = 0.0;
    for (auto* body : bodies) {
        if (!catchable(*body)) continue;
        auto& tr = track(*body);
        const auto pose = site_pose(*body, t);
        tr.contact = contact::catch_contact(tr.geometry, pose, arms);
        if (tr.contact.any_contact()) {
            body->apply_external_wrench(site_to_gcrf * tr.contact.force_n,
                                        body->state().attitude.inverse_rotate(site_to_gcrf * tr.contact.torque_nm));
        }
        hull_push_n_ = std::max({hull_push_n_, tr.contact.hull_push_n[0], tr.contact.hull_push_n[1]});
        vertical_load_n_ += tr.contact.vertical_load_n;
        max_pin_load_n_ = std::max(max_pin_load_n_, tr.contact.max_pin_load_n);
        if (phase_ == CatchPhase::Open) {
            const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
            const double pin_height = pin_axis.z - geometry_.arm_top_height_m(arms_.carriage_offset_m());
            if (contact::should_close_arms(trigger_, pin_height, std::hypot(pose.cg_m.x, pose.cg_m.y),
                                           -pose.velocity_mps.z, arms_.closing_time_s())) {
                close_arms(false);
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
            if (contact::hits_tower(geometry_, bottom, top, 0.5 * body->vehicle().diameter_m())) body->mark_impacted();
            if (was_tracked) {
                auto& tr = tracks_.at(body);
                if (tr.contact.max_pin_load_n > tr.geometry.max_pin_load_n || arms_.hit_hard_stop()) body->mark_impacted();
                const double radius = 0.5 * body->vehicle().diameter_m();
                for (const int side : {1, -1}) {
                    const auto arm = geometry_.arm_box(side, arms_.opening_rad(), arms_.carriage_offset_m());
                    if (contact::hull_penetration_m(arm, bottom, top, radius) > hull_crush_depth_m) body->mark_impacted();
                }
                const Vec3 pin_axis = pose.cg_m + pose.body_to_site * Vec3{tr.geometry.pin_station_m - pose.cg_station_m, 0.0, 0.0};
                if (pin_axis.z < geometry_.arm_top_height_m(arms_.carriage_offset_m()) - missed_drop_m &&
                    phase_ != CatchPhase::Caught) {
                    phase_ = CatchPhase::Missed;
                }
                const bool at_rest = tr.contact.all_pins_in_contact() && !body->vehicle().any_engine_producing_thrust() &&
                                     norm(pose.velocity_mps) < rest_speed_mps &&
                                     norm(pose.angular_velocity_radps) < rest_rate_radps &&
                                     std::abs(arms_.carriage_velocity_mps()) < rest_speed_mps;
                tr.at_rest_s = at_rest ? tr.at_rest_s + dt_s : 0.0;
                if (tr.at_rest_s >= rest_time_s && !body->impacted()) {
                    body->hold_fixed_to_earth(earth, epoch_end);
                    phase_ = CatchPhase::Caught;
                }
            }
        }
        if (was_tracked && (body->impacted() || body->resting()) && phase_ != CatchPhase::Caught) phase_ = CatchPhase::Missed;
    }
}

bool CatchTowerSim::in_contact(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it != tracks_.end() && it->second.contact.any_contact();
}

bool CatchTowerSim::needs_ticks(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                time::Epoch epoch) const {
    const auto t = earth.fixed_to_gcrf(epoch);
    for (const auto* body : bodies) {
        if (body->clamped() || body->impacted() || body->caught() || body->resting()) continue;
        if (norm(geometry_.to_site(transpose(t.rotation) * body->state().position_gcrf_m)) < tower_zone_m) return true;
    }
    return false;
}

}  // namespace starsim::sim
