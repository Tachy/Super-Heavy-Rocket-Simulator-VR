#include "starsim/sim/ascent_profile_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <mutex>
#include <sstream>
#include <vector>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/gnc/ascent_guidance.hpp"
#include "starsim/orbit/kepler.hpp"
#include "starsim/sim/ascent_mission.hpp"
#include "starsim/sim/ascent_plan_cache.hpp"
#include "starsim/sim/staging_sequence.hpp"

namespace starsim::sim {

namespace {

constexpr double nan = std::numeric_limits<double>::quiet_NaN();
constexpr double deg = constants::deg_to_rad;

// [E] A trial ends at SECO; one that has not cut off this long after the placement never will (Starship and Falcon 9
// cut off ~8-9 min after liftoff).
constexpr double trial_max_flight_s = 1200.0;
// [E] The trial is checked this often (ticks run at the flight's 200 Hz in between).
constexpr double trial_check_interval_s = 2.0;
// [E] The orbit at SECO counts as the target when its lowest and highest radius over the next revolution are within this
// of the target's periapsis and apoapsis (J2 swings a circular 200 km orbit by ~2 km: 199.7 .. 201.4 km).
constexpr double orbit_tolerance_m = 5e3;
constexpr double orbit_check_step_s = 5.0;  // [E] sampling of that revolution

// [E] Search start: between the optima found so far (Starship 14.9 deg, Falcon 9 11.6 deg) and the first step: wide
// enough that the parabola through the first three trials already lands near the optimum.
constexpr double initial_kick_rad = 13.0 * deg;
constexpr double initial_step_rad = 2.5 * deg;
constexpr double golden = 0.381966011250105;  // (3 - sqrt 5) / 2
constexpr double expansion = 1.618033988749895;  // bracketing: each step this much longer than the last
constexpr int max_trials = 16;                   // [E] safeguard; Starship and Falcon 9 converge in 4

// [E] Provisional station of the drone ship while the kick is searched, this far downrange along the launch azimuth: away
// from the pad (its deck replaces the ground near it), the MECO plan without a boostback does not depend on it; the ship
// is then placed at the predicted impact (Starlink-class: ~600 km downrange).
constexpr double provisional_ship_downrange_m = 600e3;

struct Trial {
    double propellant_kg = nan;
    std::string reason;  // German, when infeasible
    std::optional<BoosterImpact> booster_impact;  // drone-ship requests
};

// Where `stage` (a first stage coasting without a boostback) comes down: BoosterReturnGuidance::predict_impact with
// drag and its entry burn ahead, on the session's landing site (the drone ship's deck).
std::optional<BoosterImpact> predict_booster_impact(const FlightSession& session, const vehicle::FlightBody& stage) {
    const auto p = session.return_parameters(stage.vehicle().stages().front().config);
    const auto burn = gnc::BoosterReturnGuidance::entry_burn_ahead(stage.vehicle(), p);
    const auto impact = gnc::BoosterReturnGuidance::predict_impact(stage, session.environment(), session.epoch(),
                                                                   session.landing_zone()->height_m, nullptr, &burn);
    if (!impact.valid) return std::nullopt;
    const auto geo = frames::fixed_to_geodetic(impact.position_itrf_m);
    const math::Vec3 east{-std::sin(geo.longitude_rad), std::cos(geo.longitude_rad), 0.0};
    const math::Vec3 north{-std::sin(geo.latitude_rad) * std::cos(geo.longitude_rad),
                           -std::sin(geo.latitude_rad) * std::sin(geo.longitude_rad), std::cos(geo.latitude_rad)};
    const auto& v = impact.velocity_itrf_mps;
    return BoosterImpact{geo.latitude_rad, geo.longitude_rad, std::atan2(dot(v, east), dot(v, north))};
}

// The boostback of the stack's return planned from the state now has a Lambert arc (the session's MECO plan:
// FlightSession's Booster state, flip from the MECO to the separation included); true without a boostback.
bool boostback_arc_found(const FlightSession& session, const vehicle::FlightBody& stack) {
    auto parameters = session.return_parameters(stack.vehicle().stages().front().config);
    if (!parameters.boostback) return true;
    parameters.flip_duration_s += separation_after_meco_s(stack.vehicle());
    const auto& s = stack.state();
    const auto delta_v = gnc::BoosterReturnGuidance::boostback_delta_v_mps(
        {s.position_gcrf_m, s.velocity_gcrf_mps}, session.earth_orientation(), session.epoch(),
        session.return_site(stack.vehicle()), parameters);
    return delta_v && std::isfinite(*delta_v);
}

double stage_propellant_kg(const vehicle::Stage& stage) {
    return stage.propellant_kg(config::TankContents::Oxidizer) + stage.propellant_kg(config::TankContents::Fuel);
}

config::VehicleConfig request_stack(const std::filesystem::path& dir, const AscentProfileRequest& request) {
    if (request.stack) return *request.stack;
    return build_ascent_stack(config::load_vehicle(dir, request.vehicle_id), dir,
                              {request.payload_kg, request.fairing, request.payload, {}});
}

Trial fly_trial(const FlightSession& like, const AscentProfileRequest& request, double kick_angle_rad) {
    const auto& dir = like.config_dir();
    // The trial reads `like`'s (already interpolated) sources directly: no second interpolation layer.
    FlightSession session(like.ephemeris(), like.earth_orientation(), dir, like.gravity().config(), nullptr, false);
    const auto stack = request_stack(dir, request);
    session.launch_from_pad(stack, launch_site_from_config(config::load_launch_site(dir, request.site_id)),
                            request.heading_rad, request.liftoff);
    if (const auto zone = recovery_landing_site(dir, request.recovery)) session.set_landing_zone(*zone);
    session.set_booster_return(true);
    gnc::PitchProgram program;
    program.kick_angle_rad = kick_angle_rad;
    session.engage_ascent_autopilot(request.target, program);

    const char* return_unplannable = "Die Rückkehr der ersten Stufe ist nicht planbar.";
    const bool drone_ship = request.recovery.kind == RecoveryKind::DroneShip;
    const std::string booster_id = stack.stages.front().id;
    bool upper_lit = false;
    bool near_end = false;  // cut off, the engines' tail-off under way
    // The return checked at MECO (the last Booster tick): the boostback's Lambert arc of the plan (a failed arc makes
    // meco_propellant_kg count no boostback: finite, but not flyable).
    std::optional<bool> arc_found;
    bool meco_checked = false;
    double booster_kg = nan, booster_rate_kgps = 0.0;  // booster propellant at the last check, burned per second
    time::Epoch booster_epoch;
    std::optional<time::Epoch> separation;
    std::optional<BoosterImpact> impact;
    const int check_ticks = static_cast<int>(std::ceil(trial_check_interval_s / FlightSession::tick_s));
    while (session.epoch() - request.liftoff < trial_max_flight_s) {
        // single ticks over the cutoff: the propellant is read at SECO itself, not up to a check interval later; so is
        // the return at MECO (within two checks' propellant of the cutoff) and the separation (the staging sequence)
        const auto state_before = session.autopilot().state;
        const auto* guidance = session.ascent_guidance();
        const bool near_cutoff = state_before == AutopilotState::UpperStage && guidance && guidance->cutoff_epoch() &&
                                 *guidance->cutoff_epoch() - session.epoch() < 2.0 * trial_check_interval_s;
        const auto& cutoff_kg = session.booster_cutoff_propellant_kg();
        const bool near_meco = state_before == AutopilotState::Booster && cutoff_kg &&
                               booster_kg - *cutoff_kg < 2.0 * trial_check_interval_s * booster_rate_kgps;
        const bool single = near_cutoff || near_end || near_meco || state_before == AutopilotState::Staging;
        const int ticks = single ? 1 : check_ticks;
        const auto before = session.epoch();
        // half a tick beyond: advance_to ticks while at least a whole tick is left (rounding)
        if (!(session.advance_to(before + (ticks + 0.5) * FlightSession::tick_s, ticks) > before)) break;
        const auto state = session.autopilot().state;
        const auto index = session.controlled_index();
        const auto& body = session.body(index);
        if (body.impacted()) return {nan, "Die Rakete ist abgestürzt."};
        if (!separation && session.body_count() > 1) separation = session.epoch();
        if (drone_ship && separation && !impact &&
            !(session.epoch() - *separation < drone_ship_prediction_after_separation_s)) {
            for (size_t i = 0; i < session.body_count() && !impact; ++i) {
                const auto& b = session.body(i);
                if (i != index && b.vehicle().stages().front().config.id == booster_id) {
                    impact = predict_booster_impact(session, b);
                    if (!impact) return {nan, "Der Aufschlagpunkt der ersten Stufe ist nicht vorhersagbar."};
                }
            }
        }
        if (state == AutopilotState::Booster) {
            const auto& reserve = session.booster_cutoff_propellant_kg();
            if (reserve && !std::isfinite(*reserve)) return {nan, return_unplannable};
            if (!session.returns_after_staging(body.vehicle())) return {nan, return_unplannable};
            arc_found = boostback_arc_found(session, body);
            const double kg = stage_propellant_kg(body.vehicle().stages().front());
            if (std::isfinite(booster_kg)) booster_rate_kgps = (booster_kg - kg) / (session.epoch() - booster_epoch);
            booster_kg = kg;
            booster_epoch = session.epoch();
            continue;
        }
        if (!meco_checked && arc_found) {
            meco_checked = true;
            if (!*arc_found) return {nan, return_unplannable};
        }
        if (state == AutopilotState::UpperStage) {
            // The booster's return is settled by the MECO plan; flying it on would cost ~45 % of the trial time
            // (the return guidance's predictions on every tick) and change nothing for the upper stage.
            for (size_t i = 0; i < session.body_count(); ++i) {
                if (i != index) session.end_booster_return(i);
            }
            const bool running = session.telemetry(index).engines_running > 0;
            if (upper_lit && !running) return {nan, "Die Oberstufe ist vor Brennschluss leer."};
            upper_lit = upper_lit || running;
            continue;
        }
        if (state != AutopilotState::Orbit) continue;  // PreLaunch, Staging
        // SECO: the guidance cut off; the engines' shutdown tail-off still burns (PEG counts on its impulse: ~9 m/s,
        // ~30 km of periapsis on Starship): the orbit and the propellant are read once it has ended.
        if (body.vehicle().any_engine_producing_thrust()) {
            near_end = true;
            continue;
        }
        // The orbit judged as the guidance aims it. A circular target: on the radius flown over the next revolution (the
        // session's coast prediction) - PEG's J2-aware insertion aims that, the osculating apsides at cutoff differ by
        // design (199.7 x 212 km for a 200 km target). An elliptic target (the tankers' 120 x 190 km): PEG aims the
        // osculating (Kepler) apsides at cutoff; J2 makes the radius flown differ from them (120 x 180 km).
        const auto& s = body.state();
        const auto elements =
            orbit::elements_from_state({s.position_gcrf_m, s.velocity_gcrf_mps}, session.gm_earth_m3ps2());
        double periapsis_m = std::numeric_limits<double>::infinity(), apoapsis_m = -periapsis_m;
        if (gnc::AscentGuidance::circular_target(request.target)) {
            const double period_s = constants::two_pi * std::sqrt(std::pow(elements.semi_major_axis_m, 3.0) /
                                                                  session.gm_earth_m3ps2());
            for (const auto& p : session.predict_trajectory(index, std::isfinite(period_s) ? period_s : 0.0,
                                                            orbit_check_step_s)) {
                const double h = math::norm(p) - constants::wgs84_semi_major_axis_m;  // the target's altitudes
                periapsis_m = std::min(periapsis_m, h);
                apoapsis_m = std::max(apoapsis_m, h);
            }
        } else {
            periapsis_m = elements.semi_major_axis_m * (1.0 - elements.eccentricity) - constants::wgs84_semi_major_axis_m;
            apoapsis_m = elements.semi_major_axis_m * (1.0 + elements.eccentricity) - constants::wgs84_semi_major_axis_m;
        }
        const auto& target = request.target;
        if (std::getenv("STARSIM_DEBUG_ASCENT_PLANNER")) {
            std::printf("  [ascent planner] SECO T+%.1f s: orbit %.1f .. %.1f km, %.1f kg in the upper stage\n",
                        session.mission_time_s(), periapsis_m / 1e3, apoapsis_m / 1e3,
                        stage_propellant_kg(body.vehicle().stages().front()));
        }
        if (std::abs(periapsis_m - target.periapsis_altitude_m) > orbit_tolerance_m ||
            std::abs(apoapsis_m - target.apoapsis_altitude_m) > orbit_tolerance_m) {
            return {nan, "Die Oberstufe erreicht den Zielorbit nicht."};
        }
        if (drone_ship && !impact) return {nan, "Der Aufschlagpunkt der ersten Stufe ist nicht vorhersagbar."};
        return {stage_propellant_kg(body.vehicle().stages().front()), {}, impact};
    }
    if (std::getenv("STARSIM_DEBUG_ASCENT_PLANNER")) {
        std::printf("  [ascent planner] no SECO: state %s at %.1f s after placement, T+%.1f s, %zu bodies\n",
                    to_string(session.autopilot().state), session.epoch() - request.liftoff, session.mission_time_s(),
                    session.body_count());
    }
    return {nan, "Die Oberstufe erreicht den Zielorbit nicht."};
}

// Everything of the request that shapes the trajectory: not the liftoff epoch (the Earth turns the same way at any
// hour; the Moon's and Sun's pull over the ascent move the optimum by far less than kick_tolerance_rad).
std::string memo_key(const FlightSession& like, const AscentProfileRequest& r) {
    std::ostringstream key;
    key.precision(17);
    key << like.config_dir().string() << '|' << r.vehicle_id << '|' << r.site_id << '|' << r.heading_rad << '|'
        << r.target.periapsis_altitude_m << '|' << r.target.apoapsis_altitude_m << '|' << r.target.inclination_rad << '|'
        << r.target.northerly << '|' << r.payload_kg << '|' << r.fairing << '|';
    const auto& g = like.gravity().config();  // the trials fly `like`'s gravity model
    key << static_cast<int>(g.central_body) << ',' << g.earth << ',' << g.moon << ',' << g.sun << ',' << g.earth_j2 << '|';
    if (r.payload) {
        key << r.payload->vehicle_id << ',' << r.payload->separation_delay_s << ',' << r.payload->separation_dv_mps << ','
            << r.payload->nose_cone_open_s;
    }
    key << '|';
    const auto& c = r.recovery;
    key << static_cast<int>(c.kind) << ',' << c.zone_or_ship_id << ',' << c.latitude_rad << ',' << c.longitude_rad << ','
        << c.heading_rad;
    return key.str();
}

std::mutex& memo_mutex() {
    static std::mutex m;
    return m;
}

std::map<std::string, AscentProfile>& memo() {
    static std::map<std::string, AscentProfile> m;
    return m;
}

// Kick search: a parabola through the best trial and its two neighbours gives the next kick (a Newton step on the
// propellant's derivative, its slope and curvature from the three trials: secant on f'); golden-section steps
// where no parabola is possible (an infeasible neighbour, a vertex outside the bracket); expanding steps until the best
// trial has a worse neighbour on either side or sits at a bound.
class KickSearch {
public:
    KickSearch(const FlightSession& like, const AscentProfileRequest& request) : like_(like), request_(request) {}

    AscentProfile run() {
        AscentProfile result;
        for (double k : {initial_kick_rad - initial_step_rad, initial_kick_rad, initial_kick_rad + initial_step_rad}) {
            evaluate(k);
        }
        while (static_cast<int>(points_.size()) < max_trials) {
            const auto next = next_kick();
            if (!next) break;
            evaluate(*next);
        }
        const auto best = best_index();
        result.trial_flights = static_cast<int>(points_.size());
        if (!best) {
            result.reason = "Startprofil nicht planbar: " + last_reason_;
            return result;
        }
        result.feasible = true;
        result.kick_angle_rad = points_[*best].kick_rad;
        result.upper_propellant_at_seco_kg = points_[*best].propellant_kg;
        result.booster_impact = points_[*best].booster_impact;
        return result;
    }

private:
    struct Point {
        double kick_rad = 0.0;
        double propellant_kg = nan;
        std::optional<BoosterImpact> booster_impact;
    };

    void evaluate(double kick_rad) {
        kick_rad = std::clamp(kick_rad, min_kick_angle_rad, max_kick_angle_rad);
        const auto start = std::chrono::steady_clock::now();
        const auto trial = fly_trial(like_, request_, kick_rad);
        if (std::getenv("STARSIM_DEBUG_ASCENT_PLANNER")) {
            std::printf("  [ascent planner] kick %6.3f deg: %s %.1f kg (%.1f s)\n", kick_rad / deg,
                        std::isnan(trial.propellant_kg) ? trial.reason.c_str() : "", trial.propellant_kg,
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
        }
        if (std::isnan(trial.propellant_kg)) last_reason_ = trial.reason;
        points_.push_back({kick_rad, trial.propellant_kg, trial.booster_impact});
        std::sort(points_.begin(), points_.end(), [](const Point& a, const Point& b) { return a.kick_rad < b.kick_rad; });
    }

    std::optional<size_t> best_index() const {
        std::optional<size_t> best;
        for (size_t i = 0; i < points_.size(); ++i) {
            if (std::isnan(points_[i].propellant_kg)) continue;
            if (!best || points_[i].propellant_kg > points_[*best].propellant_kg) best = i;
        }
        return best;
    }

    bool known(double kick_rad) const {
        return std::any_of(points_.begin(), points_.end(),
                           [&](const Point& p) { return std::abs(p.kick_rad - kick_rad) < 0.5 * kick_tolerance_rad; });
    }

    // The vertex of the parabola through three trials (by kick) when it is a maximum (a Newton step on the propellant's
    // derivative: slope and curvature from the three trials); none for a straight or convex fit or an infeasible trial.
    static std::optional<double> parabola_peak(const Point& a, const Point& b, const Point& c) {
        if (std::isnan(a.propellant_kg) || std::isnan(b.propellant_kg) || std::isnan(c.propellant_kg)) return std::nullopt;
        const double slope_ab = (b.propellant_kg - a.propellant_kg) / (b.kick_rad - a.kick_rad);
        const double slope_bc = (c.propellant_kg - b.propellant_kg) / (c.kick_rad - b.kick_rad);
        const double curvature = (slope_bc - slope_ab) / (c.kick_rad - a.kick_rad);  // half the second derivative
        if (!(curvature < 0.0)) return std::nullopt;
        // f' = slope_ab + 2 curvature (k - (a + b) / 2) = 0
        return 0.5 * (a.kick_rad + b.kick_rad) - slope_ab / (2.0 * curvature);
    }

    // The next kick to fly; none when the optimum is known to kick_tolerance_rad.
    std::optional<double> next_kick() const {
        const auto best = best_index();
        if (!best) return feasibility_probe();
        const size_t b = *best;
        const double kb = points_[b].kick_rad;
        std::optional<double> next;
        if (b == 0 || b + 1 == points_.size()) {
            // the best trial is the outermost on one side: the parabola through it and its two inner neighbours, else
            // a step outwards; never more than `expansion` times the inner spacing beyond it
            const bool low = b == 0;
            const size_t inner = low ? b + 1 : b - 1;
            const double span = std::abs(points_[inner].kick_rad - kb);
            const bool three = low ? b + 2 < points_.size() : b >= 2;
            std::optional<double> peak;
            if (three) {
                peak = low ? parabola_peak(points_[b], points_[b + 1], points_[b + 2])
                           : parabola_peak(points_[b - 2], points_[b - 1], points_[b]);
            }
            next = peak ? std::clamp(*peak, kb - expansion * span, kb + expansion * span)
                        : kb + (low ? -1.0 : 1.0) * expansion * span;
        } else {
            const Point& l = points_[b - 1];
            const Point& r = points_[b + 1];
            // an infeasible neighbour: golden section towards it (the feasible region's edge)
            for (const Point* n : {&l, &r}) {
                if (std::isnan(n->propellant_kg) && std::abs(n->kick_rad - kb) > kick_tolerance_rad) {
                    return kb + golden * (n->kick_rad - kb);
                }
            }
            if (r.kick_rad - l.kick_rad <= 2.0 * kick_tolerance_rad) return std::nullopt;
            next = parabola_peak(l, points_[b], r);
            if (!next || !(*next > l.kick_rad && *next < r.kick_rad)) {
                // no usable parabola: golden section in the larger side of the bracket
                next = r.kick_rad - kb > kb - l.kick_rad ? kb + golden * (r.kick_rad - kb) : kb - golden * (kb - l.kick_rad);
            }
        }
        const double kick = std::clamp(*next, min_kick_angle_rad, max_kick_angle_rad);
        if (std::abs(kick - kb) < kick_tolerance_rad || known(kick)) return std::nullopt;  // converged (or at a bound)
        return kick;
    }

    // No feasible trial yet: the middle of the widest gap the trials have left in the range.
    std::optional<double> feasibility_probe() const {
        std::vector<double> edges{min_kick_angle_rad};
        for (const auto& p : points_) edges.push_back(p.kick_rad);
        edges.push_back(max_kick_angle_rad);
        double widest = 0.0, at = nan;
        for (size_t i = 0; i + 1 < edges.size(); ++i) {
            const double gap = edges[i + 1] - edges[i];
            if (gap > widest) {
                widest = gap;
                at = edges[i] + 0.5 * gap;
            }
        }
        if (widest < initial_step_rad) return std::nullopt;  // the range sampled densely: nothing flies
        return at;
    }

    const FlightSession& like_;
    const AscentProfileRequest& request_;
    std::vector<Point> points_;  // by kick
    std::string last_reason_ = "kein Versuchsflug.";
};

}  // namespace

std::optional<LaunchSite> recovery_landing_site(const std::filesystem::path& config_dir, const RecoveryChoice& recovery) {
    switch (recovery.kind) {
        case RecoveryKind::TowerCatch: return std::nullopt;
        case RecoveryKind::LandingZone:
            return launch_site_from_config(config::load_launch_site(config_dir, recovery.zone_or_ship_id));
        case RecoveryKind::DroneShip:
            return drone_ship_site(config::load_drone_ship(config_dir, recovery.zone_or_ship_id), recovery.latitude_rad,
                                   recovery.longitude_rad, drone_ship_sea_height_m, recovery.heading_rad);
    }
    return std::nullopt;
}


double fly_ascent_trial(const FlightSession& like, const AscentProfileRequest& request, double kick_angle_rad) {
    return fly_trial(like, request, kick_angle_rad).propellant_kg;
}

AscentProfile plan_ascent_profile(const FlightSession& like, const AscentProfileRequest& request) {
    if (request.stack) return KickSearch(like, request).run();  // a changed vehicle: its id is no key
    const auto key = memo_key(like, request);
    {
        std::lock_guard lock(memo_mutex());
        if (const auto it = memo().find(key); it != memo().end()) {
            auto cached = it->second;
            cached.trial_flights = 0;
            return cached;
        }
    }
    const bool disk_cache = plan_cache_dir().has_value();  // tests only (STARSIM_PLAN_CACHE_DIR)
    const auto disk_key = disk_cache ? plan_cache_key(key, request.liftoff) : std::string();
    if (disk_cache) {
        if (auto stored = load_cached_plan(disk_key, like.config_dir())) {  // another test process planned it
            stored->trial_flights = 0;
            std::lock_guard lock(memo_mutex());
            memo()[key] = *stored;
            return *stored;
        }
    }
    const auto profile = KickSearch(like, request).run();
    if (disk_cache) store_cached_plan(disk_key, like.config_dir(), profile);
    std::lock_guard lock(memo_mutex());
    memo()[key] = profile;
    return profile;
}

AscentProfile plan_launch(const FlightSession& like, AscentProfileRequest request) {
    const auto& dir = like.config_dir();
    const auto site = config::load_launch_site(dir, request.site_id);
    // The lowest inclination a launch reaches is the site's latitude (due east); a lower target is raised to it.
    request.target.inclination_rad = std::max(request.target.inclination_rad, std::abs(site.latitude_rad));
    int trial_flights = 0;
    auto planned = [&](const RecoveryChoice& recovery) {
        request.recovery = recovery;
        auto profile = plan_ascent_profile(like, request);
        trial_flights += profile.trial_flights;
        profile.trial_flights = trial_flights;
        profile.recovery = recovery;
        profile.target = request.target;
        return profile;
    };
    auto refused = [&](AscentProfile profile, const std::string& reason) {
        profile.feasible = false;
        profile.reason = profile.reason.empty() ? reason + "." : reason + " (" + profile.reason + ")";
        return profile;
    };
    const char* unreachable = "Ziel mit Booster-Rückkehr nicht erreichbar";

    const std::string method = request_stack(dir, request).stages.front().recovery_method;
    if (method == "tower_catch") {
        AscentProfile none;
        none.target = request.target;
        if (!site.tower) return refused(none, "Kein Fangturm am Startplatz");
        auto profile = planned({RecoveryKind::TowerCatch, site.id, site.latitude_rad, site.longitude_rad, 0.0});
        return profile.feasible ? profile : refused(profile, unreachable);
    }
    if (method != "leg_landing") {
        AscentProfile none;
        none.target = request.target;
        return refused(none, "Die erste Stufe hat keine Rückkehr");
    }
    // Falcon 9: back to the site's landing zone whenever that closes with the RTLS return (SpaceX practice), even where
    // the drone ship would leave the upper stage more.
    if (!site.landing_zone_id.empty()) {
        const auto zone = config::load_launch_site(dir, site.landing_zone_id);
        auto profile = planned({RecoveryKind::LandingZone, zone.id, zone.latitude_rad, zone.longitude_rad, 0.0});
        if (profile.feasible) return profile;
    }
    // The drone ship: the kick searched with the ship at a provisional station downrange (the return without a
    // boostback plans its MECO independent of it), then the ship placed at the stage's predicted impact.
    const double speed_mps = std::sqrt(constants::earth_gm_m3ps2 /
                                       (constants::wgs84_semi_major_axis_m + request.target.periapsis_altitude_m));
    const double azimuth = gnc::launch_azimuth_rad(site.latitude_rad, request.target.inclination_rad, speed_mps,
                                                   request.target.northerly);
    const double arc = provisional_ship_downrange_m / constants::wgs84_semi_major_axis_m;
    const double lat = std::asin(std::sin(site.latitude_rad) * std::cos(arc) +
                                 std::cos(site.latitude_rad) * std::sin(arc) * std::cos(azimuth));
    const double lon = site.longitude_rad + std::atan2(std::sin(azimuth) * std::sin(arc) * std::cos(site.latitude_rad),
                                                       std::cos(arc) - std::sin(site.latitude_rad) * std::sin(lat));
    auto profile = planned({RecoveryKind::DroneShip, default_drone_ship_id, lat, lon, azimuth});
    if (!profile.feasible) return refused(profile, unreachable);
    if (!profile.booster_impact) return refused(profile, "Der Aufschlagpunkt der ersten Stufe ist nicht vorhersagbar");
    const auto& impact = *profile.booster_impact;
    profile.recovery = {RecoveryKind::DroneShip, default_drone_ship_id, impact.latitude_rad, impact.longitude_rad,
                        impact.heading_rad};
    return profile;
}

AscentProfile engage_planned_ascent(FlightSession& session, const AscentProfileRequest& request) {
    const auto profile = plan_launch(session, request);
    if (!profile.feasible) {
        session.schedule_liftoff(std::nullopt);  // the countdown is called off: no T-0 to hold at
        return profile;
    }
    if (const auto zone = recovery_landing_site(session.config_dir(), profile.recovery)) session.set_landing_zone(*zone);
    session.set_booster_return(true);
    gnc::PitchProgram program;
    program.kick_angle_rad = profile.kick_angle_rad;
    session.engage_ascent_autopilot(profile.target, program);
    return profile;
}

std::string engage_pad_ascent(FlightSession& session, const gnc::AscentTarget& target) {
    if (!session.active() || !session.body(session.controlled_index()).clamped()) {
        session.engage_ascent_autopilot(target, gnc::PitchProgram{});
        return {};
    }
    const auto& pad_stack = session.pad_stack();
    if (session.launch_site().id.empty() || !pad_stack) {
        session.schedule_liftoff(std::nullopt);
        return "Startplanung nicht möglich: kein konfigurierter Startplatz.";
    }
    AscentProfileRequest request;
    request.site_id = session.launch_site().id;
    request.heading_rad = session.pad_heading_rad();
    request.liftoff = session.epoch();
    request.target = target;
    if (!session.pad_vehicle_id().empty()) {
        // the vehicle file as configured: build_ascent_stack with its own payload and the fairing kept is the same stack
        request.vehicle_id = session.pad_vehicle_id();
        request.payload_kg = pad_stack->stages.back().payload_mass_kg;
        request.fairing = true;
    } else {
        request.vehicle_id = pad_stack->id;
        request.stack = pad_stack;
    }
    const auto profile = engage_planned_ascent(session, request);
    return profile.feasible ? std::string() : profile.reason;
}

}  // namespace starsim::sim
