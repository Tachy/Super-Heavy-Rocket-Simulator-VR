// FlightSession: powered lunar descent and the surface steps (lunar l4).
//
// Design decisions:
// - The descent is a step of the lunar step machine (state Lunar): PoweredDescent flies gnc::LunarLandingGuidance's
//   commands as the ship's landing burn (apply_return_command with the ship-landing controller: gimbals + RCS), ticked.
// - The touchdown record is taken at the first leg contact (sink and lateral speed relative to the Moon); the legs'
//   judgement ends the step: "gelandet" -> Landed, "umgekippt" / "Aufprall" (or an impact without legs) -> Lost
//   (autopilot off).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "starsim/math/constants.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::sim {

namespace {

constexpr double far_future_s = 1e9;  // "nothing timed"

std::string german(double value, int decimals) {
    char text[32];
    std::snprintf(text, sizeof(text), "%.*f", decimals, value);
    std::string s = text;
    std::replace(s.begin(), s.end(), '.', ',');
    return s;
}

}  // namespace

bool FlightSession::begin_powered_descent(const mission::TouchdownPoint& point, double approach_azimuth_rad,
                                          gnc::LunarLandingParameters parameters) {
    if (!moon_) {
        lunar_status_ = "Landeanflug nicht möglich: keine Mondorientierung";
        return false;
    }
    if (legs_->site().body != ephemeris::Body::Moon) {
        lunar_status_ = "Landeanflug nicht möglich: keine Landestelle auf dem Mond (set_lunar_landing_site)";
        return false;
    }
    if (bodies_.empty() || reference_body(controlled_) != ephemeris::Body::Moon) {
        lunar_status_ = "Landeanflug nicht möglich: Fahrzeug nicht am Mond";
        return false;
    }
    auto& vehicle = bodies_[controlled_].body->vehicle();
    const auto& legs = vehicle.stages().front().config.landing_legs;
    if (legs.azimuths_rad.empty()) {
        lunar_status_ = "Landeanflug nicht möglich: keine Landebeine";
        return false;
    }
    if (!legs.fixed && vehicle.leg_deployment() == 0.0) vehicle.deploy_legs();
    gnc::LandingSite site{ephemeris::Body::Moon, point.latitude_rad, point.longitude_rad, point.height_m, {}, {},
                          gnc::PadTarget{-legs.foot_station_m, 0.0}};  // feet -foot_station_m below the stage bottom
    lunar_landing_ = std::make_unique<gnc::LunarLandingGuidance>(site, approach_azimuth_rad, std::move(parameters));
    lunar_landing_->set_realtime(realtime_planning_);
    lunar_touchdown_.reset();
    lunar_touchdown_point_ = point;
    attitude_mode_ = {};
    autopilot_.state = AutopilotState::Lunar;
    lunar_step_ = LunarStep::PoweredDescent;
    lunar_step_epoch_ = epoch_;
    lunar_status_ = "Landeanflug";
    return true;
}

void FlightSession::run_lunar_descent_steps(Flying& flying) {
    switch (lunar_step_) {
        case LunarStep::PoweredDescent: {
            auto& body = *flying.body;
            const auto frame = moon_->frame(epoch_);
            const auto& s = body.state();
            const bool contact = legs_->in_contact(body);
            const auto record_contact = [&] {
                const math::Vec3 rel = frame.relative_velocity(s.position_gcrf_m, s.velocity_gcrf_mps);
                const math::Vec3 up = math::normalized(s.position_gcrf_m - frame.centre_gcrf_m);
                lunar_touchdown_ = LunarTouchdown{LegTouchdown::None, 0.0, -dot(rel, up), norm(rel - up * dot(rel, up))};
            };
            if (contact && !lunar_touchdown_) record_contact();
            flying.return_command = lunar_landing_->update(body, env_, epoch_, contact);
            for (const auto& stage : body.vehicle().stages()) {
                for (const auto& e : stage.engines) {
                    const auto& groups = flying.return_command.run_groups;
                    flying.running_groups[e.group_id] = std::find(groups.begin(), groups.end(), e.group_id) != groups.end();
                }
            }
            if (!lunar_landing_->status().empty() && lunar_landing_->status() != lunar_status_ &&
                lunar_landing_->phase() == gnc::LunarDescentPhase::Aborted) {
                lunar_status_ = "Landeanflug: " + lunar_landing_->status();
            }
            const auto judged = legs_->touchdown(body);
            if (judged == LegTouchdown::None && !body.impacted()) break;
            if (!lunar_touchdown_) record_contact();  // an impact before any leg contact
            lunar_touchdown_->outcome = body.impacted() ? LegTouchdown::Impact : judged;
            // Where it stands: the footprint centre (the hull axis at the feet's station - on a slope the 21 m high CG
            // leans ~0.9 m downhill) against the touchdown point, both in the leg site's east / north / up frame.
            const double r_point = constants::moon_mean_radius_m + lunar_touchdown_point_.height_m;
            const double lat = lunar_touchdown_point_.latitude_rad, lon = lunar_touchdown_point_.longitude_rad;
            const math::Vec3 point_fixed{r_point * std::cos(lat) * std::cos(lon), r_point * std::cos(lat) * std::sin(lon),
                                         r_point * std::sin(lat)};
            const auto& vehicle = body.vehicle();
            const double below_cg_m =
                vehicle.mass_properties().cg_station_m - vehicle.stages().front().config.landing_legs.foot_station_m;
            const math::Vec3 footprint = s.position_gcrf_m - s.attitude.rotate(math::Vec3{1.0, 0.0, 0.0}) * below_cg_m;
            const math::Vec3 d = legs_->to_site(frame.to_fixed(footprint)) - legs_->to_site(point_fixed);
            lunar_touchdown_->offset_m = std::hypot(d.x, d.y);
            if (lunar_touchdown_->outcome == LegTouchdown::Landed) {
                lunar_landing_->mark_landed();
                lunar_step_ = LunarStep::Landed;
                lunar_step_epoch_ = epoch_ + far_future_s;
                lunar_status_ = "gelandet: " + german(lunar_touchdown_->offset_m, 2) + " m vom Zielpunkt";
            } else {
                lunar_step_ = LunarStep::Lost;
                lunar_status_ = std::string("verloren: ") + to_string(lunar_touchdown_->outcome);
                autopilot_.state = AutopilotState::Off;
            }
            for (auto& [group, run] : flying.running_groups) run = false;
            break;
        }
        case LunarStep::Landed:
        case LunarStep::Lost:
        default:
            break;
    }
}

}  // namespace starsim::sim
