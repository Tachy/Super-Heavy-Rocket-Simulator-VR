#include "starsim/sim/leg_landing_sim.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

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

LegLandingSim::SiteFrame LegLandingSim::site_frame(const LegSite& site, const frames::Ellipsoid& shape) {
    const double sl = std::sin(site.latitude_rad), cl = std::cos(site.latitude_rad);
    const double so = std::sin(site.longitude_rad), co = std::cos(site.longitude_rad);
    return {frames::geodetic_to_fixed({site.latitude_rad, site.longitude_rad, site.height_m}, shape),
            contact::from_columns({-so, co, 0.0}, {-sl * co, -sl * so, cl}, {cl * co, cl * so, sl})};
}

LegLandingSim::LegLandingSim(const LegSite& site, const frames::Ellipsoid& shape, contact::LandingSurface surface,
                             double tick_s)
    : site_(site), frame_(site_frame(site, shape)), surface_(std::move(surface)), tick_s_(tick_s) {}

LegLandingSim::LegLandingSim(double latitude_rad, double longitude_rad, double height_m, contact::LandingSurface surface,
                             double tick_s)
    : LegLandingSim(LegSite{ephemeris::Body::Earth, latitude_rad, longitude_rad, height_m}, frames::wgs84,
                    std::move(surface), tick_s) {}

Vec3 LegLandingSim::to_site(const Vec3& fixed_m) const {
    return transpose(frame_.site_to_fixed) * (fixed_m - frame_.origin_fixed_m);
}

contact::SitePose LegLandingSim::site_pose(const vehicle::FlightBody& body, const environment::BodyFrame& frame) const {
    const math::Mat3 to_site = transpose(frame_.site_to_fixed) * transpose(frame.transform.rotation);
    const auto& s = body.state();
    contact::SitePose p;
    p.cg_m = this->to_site(frame.to_fixed(s.position_gcrf_m));
    p.velocity_mps = to_site * frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
    p.body_to_site = to_site * s.attitude.to_matrix();
    p.angular_velocity_radps = to_site * (s.attitude.rotate(s.angular_velocity_body_radps) - frame.rotation_gcrf_radps);
    p.cg_station_m = body.vehicle().mass_properties().cg_station_m;
    return p;
}

void LegLandingSim::set_deck_motion(std::shared_ptr<const contact::DeckMotion> motion, time::Epoch sea_origin) {
    if (motion && !surface_.deck) throw std::logic_error("set_deck_motion: the site has no deck");
    deck_motion_ = std::move(motion);
    sea_origin_ = sea_origin;
    carrier_cache_.reset();
}

void LegLandingSim::change_deck_motion(std::shared_ptr<const contact::DeckMotion> motion, time::Epoch sea_origin,
                                       const std::vector<vehicle::FlightBody*>& bodies,
                                       const vehicle::FlightEnvironment& env, time::Epoch epoch) {
    const auto frame = vehicle::reference_body_of(env, site_.body).frame(epoch);
    std::vector<vehicle::FlightBody*> landed;
    for (auto* body : bodies) {
        const auto it = tracks_.find(body);
        if (it == tracks_.end() || it->second.touchdown != LegTouchdown::Landed || !body->resting()) continue;
        landed.push_back(body);
        if (it->second.held_bottom_m) {  // riding the old motion: fixed where it is now (its velocity: the Earth's)
            body->hold_landed(frame, site_.body);
            it->second.held_bottom_m.reset();
        }
    }
    set_deck_motion(std::move(motion), sea_origin);
    if (!deck_motion_ || landed.empty()) return;
    const auto c = carrier(epoch);
    for (auto* body : landed) {  // the new motion takes them along from their pose relative to the deck now
        auto& tr = tracks_.at(body);
        const auto pose = relative_pose(site_pose(*body, frame), c);
        tr.held_body_to_deck = pose.body_to_site;
        tr.held_bottom_m = pose.cg_m - pose.body_to_site * Vec3{pose.cg_station_m, 0.0, 0.0};
    }
    follow_deck(landed, env, epoch);
}

std::optional<contact::DeckPose> LegLandingSim::deck_pose(time::Epoch epoch) const {
    if (!deck_motion_) return std::nullopt;
    return deck_motion_->pose(epoch - sea_origin_);
}

LegLandingSim::Carrier LegLandingSim::carrier(time::Epoch epoch) const {
    const double key = epoch.tdb_seconds_past_j2000();
    if (carrier_cache_ && carrier_cache_->first == key) return carrier_cache_->second;
    const auto f = deck_motion_->frame(epoch - sea_origin_);
    // the level deck axes (bow, port, up) of the zero pose: deck_to_site x level^T rotates site axes with the deck
    const auto level = contact::deck_frame(contact::DeckPose{}, surface_.deck->heading_rad).deck_to_site;
    const Carrier c{f.deck_to_site * transpose(level), f.origin_site_m, f.velocity_site_mps, f.angular_velocity_site_radps};
    carrier_cache_.emplace(key, c);
    return c;
}

// cg' = M^T (cg - o); v' = M^T (v - v_o - w_d x (cg - o)); w' = M^T (w - w_d); body_to_deck = M^T body_to_site.
contact::SitePose LegLandingSim::relative_pose(const contact::SitePose& p, const Carrier& c) {
    const math::Mat3 to_carrier = transpose(c.to_site);
    const Vec3 r = p.cg_m - c.origin_site_m;
    contact::SitePose q = p;
    q.cg_m = to_carrier * r;
    q.velocity_mps = to_carrier * (p.velocity_mps - c.velocity_site_mps - cross(c.angular_velocity_site_radps, r));
    q.angular_velocity_radps = to_carrier * (p.angular_velocity_radps - c.angular_velocity_site_radps);
    q.body_to_site = to_carrier * p.body_to_site;
    return q;
}

void LegLandingSim::follow_deck(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                                time::Epoch epoch) {
    if (!deck_motion_) return;
    const auto frame = vehicle::reference_body_of(env, site_.body).frame(epoch);
    const auto c = carrier(epoch);
    for (auto* body : bodies) {
        const auto it = tracks_.find(body);
        if (it == tracks_.end() || it->second.touchdown != LegTouchdown::Landed || !it->second.held_bottom_m ||
            !body->resting()) {
            continue;
        }
        const auto& tr = it->second;
        const Vec3 bottom = c.origin_site_m + c.to_site * *tr.held_bottom_m;
        const math::Mat3 body_to_site = c.to_site * tr.held_body_to_deck;
        const double cg_station = body->vehicle().mass_properties().cg_station_m;
        const Vec3 cg = bottom + body_to_site * Vec3{cg_station, 0.0, 0.0};
        const Vec3 velocity = c.velocity_site_mps + cross(c.angular_velocity_site_radps, cg - c.origin_site_m);
        const math::Mat3& s = frame_.site_to_fixed;
        body->move_landed_hold(frame, frame_.origin_fixed_m + s * bottom, math::Quat::from_matrix(s * body_to_site),
                               s * velocity, s * c.angular_velocity_site_radps);
    }
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

void LegLandingSim::before_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                                time::Epoch epoch) {
    const auto frame = vehicle::reference_body_of(env, site_.body).frame(epoch);
    const math::Mat3 site_to_gcrf = frame.transform.rotation * frame_.site_to_fixed;
    // A moving deck: contact in the deck-attached frame, the wrench rotated back to the site frame.
    const std::optional<Carrier> deck = deck_motion_ ? std::optional<Carrier>(carrier(epoch)) : std::nullopt;
    const math::Mat3 contact_to_gcrf = deck ? site_to_gcrf * deck->to_site : site_to_gcrf;
    // The deck supports in the carrier frame; the water beside it stays at the mean sea level (DeckSeaField).
    contact::LandingSurface moving;
    if (deck) moving.field = std::make_shared<contact::DeckSeaField>(*surface_.deck, deck->to_site, deck->origin_site_m);
    const contact::LandingSurface& contact_surface = deck ? moving : surface_;
    for (auto* body : bodies) {
        const auto& v = body->vehicle();
        if (!v.has_landing_legs()) continue;
        // On this site: near it on its body (an Earth site's plane reaches everywhere). Elsewhere the body keeps its own
        // ground contact even with its legs out (lunar l2: an HLS touching the Moon away from the site).
        const auto pose = site_pose(*body, frame);
        // Also above the site's plane: on the far side of the Moon (an orbit after the ascent, near the antipode) the
        // horizontal offset is small too, and the feet "touched" the plane ~3,500 km above them (l4: an impact in orbit).
        const bool on_site = body->reference_body(env, epoch) == site_.body &&
                             std::hypot(pose.cg_m.x, pose.cg_m.y) <= site_.tracking_radius_m &&
                             pose.cg_m.z > -site_.tracking_radius_m;
        body->set_legs_on_site(on_site);
        if (auto it = tracks_.find(body); it != tracks_.end()) it->second.on_site = on_site;
        if (!on_site || v.leg_deployment() <= 0.0 || body->impacted() || body->resting()) continue;
        // A track starts near the ground: near the site's plane AND near the body's own ground (lunar l3: an Earth site's
        // plane passes through low Earth orbit ~14 deg from the site; an HLS with fixed legs struck it there).
        if (tracks_.count(body) == 0 &&
            (pose.cg_m.z > contact_zone_height_m || body->ground_clearance(env, epoch).height_m > contact_zone_height_m)) {
            continue;
        }
        auto& tr = track(*body);
        const auto contact_pose = deck ? relative_pose(pose, *deck) : pose;
        if (v.leg_deployment() < 1.0) {  // legs still moving: no support, only the base can touch
            auto base_only = tr.geometry;
            base_only.feet.clear();
            tr.contact = contact::leg_contact(base_only, tr.crushed_m, contact_pose, contact_surface);
            continue;
        }
        const auto mass = v.mass_properties();
        const contact::FrictionLimit limit{mass.mass_kg, mass.inertia_kg_m2, tick_s_};
        tr.contact = contact::leg_contact(tr.geometry, tr.crushed_m, contact_pose, contact_surface, &limit);
        if (tr.contact.feet_in_contact > 0) {
            body->apply_external_wrench(contact_to_gcrf * tr.contact.force_n,
                                        body->state().attitude.inverse_rotate(contact_to_gcrf * tr.contact.torque_nm));
        }
    }
}

void LegLandingSim::after_tick(const std::vector<vehicle::FlightBody*>& bodies, const vehicle::FlightEnvironment& env,
                               time::Epoch epoch_end, double dt_s) {
    const auto frame = vehicle::reference_body_of(env, site_.body).frame(epoch_end);
    const std::optional<Carrier> deck = deck_motion_ ? std::optional<Carrier>(carrier(epoch_end)) : std::nullopt;
    for (auto* body : bodies) {
        auto it = tracks_.find(body);
        if (it == tracks_.end() || it->second.touchdown != LegTouchdown::None || !it->second.on_site) continue;
        auto& tr = it->second;
        const auto& c = tr.contact;
        // judged relative to the deck when it moves (tilt to the deck normal, motion relative to the deck)
        const auto pose = deck ? relative_pose(site_pose(*body, frame), *deck) : site_pose(*body, frame);
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
            if (result == LegTouchdown::Landed) {
                body->hold_landed(frame, site_.body);
                if (deck) {  // rides the deck from here: its pose relative to the deck, kept
                    tr.held_body_to_deck = pose.body_to_site;
                    tr.held_bottom_m = pose.cg_m - pose.body_to_site * Vec3{pose.cg_station_m, 0.0, 0.0};
                }
            } else {
                body->mark_impacted();
            }
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
    follow_deck(bodies, env, epoch_end);  // the landed stages ride the deck (and the ones just landed join it)
}

void LegLandingSim::release(const vehicle::FlightBody& body) {
    auto it = tracks_.find(&body);
    if (it == tracks_.end()) return;
    it->second.touchdown = LegTouchdown::None;
    it->second.at_rest_s = 0.0;
    it->second.held_bottom_m.reset();
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
