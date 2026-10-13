#include "starsim/sim/leg_landing_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "starsim/contact/primitives.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::sim {

using math::Vec3;

namespace {

constexpr double tip_over_tilt_rad = 30.0 * constants::deg_to_rad;  // spec: beyond this the stage falls over
constexpr double rest_speed_mps = 0.05;
constexpr double rest_rate_radps = 0.01;
constexpr double rest_time_s = 2.0;
// A crushed core shortens its leg: the stage then stands (and may rock) on three feet, like a table with a short leg.
constexpr int stable_feet = 3;
constexpr double contact_zone_height_m = 50.0;  // feet tracked below this height of the CG above the surface

bool engines_off(const vehicle::FlightBody& body) { return !body.vehicle().any_engine_producing_thrust(); }

}  // namespace

const char* to_string(LegTouchdown touchdown) {
    switch (touchdown) {
        case LegTouchdown::None: return "";
        case LegTouchdown::Landed: return "gelandet";
        case LegTouchdown::TippedOver: return "umgekippt";
        case LegTouchdown::Impact: return "Aufprall";
        case LegTouchdown::Splashdown: return "Wasserung";
    }
    return "";
}

LegLandingSim::LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface,
                             double tick_s)
    : origin_itrf_m_(frames::geodetic_to_fixed({latitude_rad, longitude_rad, height_m})),
      surface_(std::move(surface)),
      tick_s_(tick_s) {
    const double sl = std::sin(latitude_rad), cl = std::cos(latitude_rad);
    const double so = std::sin(longitude_rad), co = std::cos(longitude_rad);
    site_to_itrf_ = contact::from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl});
}

Vec3 LegLandingSim::to_site(const Vec3& itrf_m) const { return transpose(site_to_itrf_) * (itrf_m - origin_itrf_m_); }

contact::SitePose LegLandingSim::site_pose(const vehicle::FlightBody& body, const frames::FixedToInertial& t) const {
    const math::Mat3 to_fixed = transpose(t.rotation);
    const math::Mat3 to_site = transpose(site_to_itrf_) * to_fixed;
    const Vec3 omega = vehicle::earth_rotation_vector(t);
    const auto& s = body.state();
    contact::SitePose p;
    p.cg_m = this->to_site(to_fixed * s.position_gcrf_m);
    p.velocity_mps = to_site * (s.velocity_gcrf_mps - cross(omega, s.position_gcrf_m));
    p.body_to_site = to_site * s.attitude.to_matrix();
    p.angular_velocity_radps = to_site * (s.attitude.rotate(s.angular_velocity_body_radps) - omega);
    p.cg_station_m = body.vehicle().mass_properties().cg_station_m;
    return p;
}

LegLandingSim::Track& LegLandingSim::track(const vehicle::FlightBody& body) {
    auto it = tracks_.find(&body);
    if (it == tracks_.end()) {
        const auto& stage = body.vehicle().stages().front();
        Track t;
        t.geometry = contact::make_leg_geometry(stage.config.landing_legs, stage.base_station_m, 0.5 * stage.config.diameter_m);
        it = tracks_.emplace(&body, std::move(t)).first;
    }
    return it->second;
}

void LegLandingSim::before_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                                time::Epoch epoch) {
    const auto t = earth.fixed_to_gcrf(epoch);
    const math::Mat3 site_to_gcrf = t.rotation * site_to_itrf_;
    for (auto* body : bodies) {
        const auto& v = body->vehicle();
        if (!v.has_landing_legs() || v.leg_deployment() <= 0.0 || body->impacted() || body->resting()) continue;
        const auto pose = site_pose(*body, t);
        if (tracks_.count(body) == 0 && pose.cg_m.z > contact_zone_height_m) continue;
        auto& tr = track(*body);
        if (v.leg_deployment() < 1.0) {  // legs still moving: no support, only the base can touch
            auto base_only = tr.geometry;
            base_only.feet.clear();
            tr.contact = contact::leg_contact(base_only, tr.crushed_m, pose, surface_);
            continue;
        }
        const auto mass = v.mass_properties();
        const contact::FrictionLimit limit{mass.mass_kg, mass.inertia_kg_m2, tick_s_};
        tr.contact = contact::leg_contact(tr.geometry, tr.crushed_m, pose, surface_, &limit);
        if (tr.contact.feet_in_contact > 0) {
            body->apply_external_wrench(site_to_gcrf * tr.contact.force_n,
                                        body->state().attitude.inverse_rotate(site_to_gcrf * tr.contact.torque_nm));
        }
    }
}

void LegLandingSim::after_tick(const std::vector<vehicle::FlightBody*>& bodies, const frames::BodyOrientation& earth,
                               time::Epoch epoch_end, double dt_s) {
    const auto t = earth.fixed_to_gcrf(epoch_end);
    for (auto* body : bodies) {
        auto it = tracks_.find(body);
        if (it == tracks_.end() || it->second.touchdown != LegTouchdown::None) continue;
        auto& tr = it->second;
        const auto& c = tr.contact;
        const auto pose = site_pose(*body, t);
        const Vec3 axis = pose.body_to_site * Vec3{1.0, 0.0, 0.0};
        const double tilt = std::acos(std::clamp(axis.z, -1.0, 1.0));
        if (std::getenv("STARSIM_DEBUG_LEGS")) {  // diagnostics
            static int n = 0;
            if (++n % 20 == 0) {
                std::printf("[legs] cg %.2f %.2f %.3f  v %.3f %.3f %.3f  w %.4f %.4f %.4f  tilt %.2f deg  feet %d  crushed",
                            pose.cg_m.x, pose.cg_m.y, pose.cg_m.z, pose.velocity_mps.x, pose.velocity_mps.y,
                            pose.velocity_mps.z, pose.angular_velocity_radps.x, pose.angular_velocity_radps.y,
                            pose.angular_velocity_radps.z, tilt / constants::deg_to_rad, c.feet_in_contact);
                for (double x : tr.crushed_m) std::printf(" %.3f", x);
                std::printf("  F %.0f %.0f %.0f  T %.0f %.0f %.0f\n", c.force_n.x, c.force_n.y, c.force_n.z, c.torque_nm.x, c.torque_nm.y, c.torque_nm.z);
            }
        }
        auto end = [&](LegTouchdown result) {
            tr.touchdown = result;
            if (result == LegTouchdown::Landed) body->hold_landed(earth, epoch_end);
            else body->mark_impacted();
        };
        if (c.stroke_exhausted || c.base_contact) {
            end(LegTouchdown::Impact);
        } else if (c.water) {
            end(LegTouchdown::Splashdown);
        } else if (tilt > tip_over_tilt_rad && c.feet_in_contact > 0) {
            end(LegTouchdown::TippedOver);
        } else if (c.feet_in_contact >= stable_feet && engines_off(*body) &&
                   norm(pose.velocity_mps) < rest_speed_mps && norm(pose.angular_velocity_radps) < rest_rate_radps) {
            tr.at_rest_s += dt_s;
            if (tr.at_rest_s >= rest_time_s) end(LegTouchdown::Landed);
        } else {
            tr.at_rest_s = 0.0;
        }
    }
}

LegTouchdown LegLandingSim::touchdown(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it == tracks_.end() ? LegTouchdown::None : it->second.touchdown;
}

bool LegLandingSim::in_contact(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it != tracks_.end() && it->second.contact.feet_in_contact > 0;
}

std::vector<double> LegLandingSim::crushed_m(const vehicle::FlightBody& body) const {
    const auto it = tracks_.find(&body);
    return it == tracks_.end() ? std::vector<double>{} : it->second.crushed_m;
}

}  // namespace starsim::sim
