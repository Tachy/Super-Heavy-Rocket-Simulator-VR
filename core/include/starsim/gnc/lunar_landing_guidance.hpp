#pragma once

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "starsim/gnc/apollo_braking.hpp"
#include "starsim/gnc/booster_return.hpp"
#include "starsim/gnc/descent_snapshot.hpp"
#include "starsim/gnc/scvx_landing.hpp"

namespace starsim::gnc {

enum class LunarDescentPhase { Braking, Approach, Terminal, Touchdown, Landed, Aborted };
// German: "Bremsphase", "Anflug", "Endanflug", "Aufsetzen", "gelandet", "Abbruch"
const char* to_string(LunarDescentPhase phase);

// Terminal descent on the landing thrusters, after Apollo's P66 (rate of descent mode) [S: Bennett, "Apollo lunar
// descent and ascent trajectories", NASA TM X-58040 (1970)]: the sink rate follows the height, the horizontal velocity
// and the offset from the touchdown point are nulled slowly - the landing thrusters are turned by the RCS alone
// (~1e-3 rad/s^2 on the HLS), so the lateral loop must stay well below the attitude loop's speed. (User 2026-10-15:
// SCvx steering of this set by plan tracking left 1-25 m and up to 1.6 m/s sideways.)
struct TerminalDescentParameters {
    double sink_time_constant_s = 10.0;      // [E] sink setpoint = height above touchdown / this ...
    double max_sink_mps = 10.0;              // [E] ... at most this
    double touchdown_sink_mps = 1.0;         // spec: ~1 m/s at touchdown (also the floor of the setpoint)
    double vertical_time_constant_s = 1.0;   // [E] sink-rate error -> vertical acceleration
    double lateral_frequency_rad_s = 0.2;    // [E] offset/velocity nulling (critically damped)
    double lateral_damping = 1.0;
    double lead_s = 1.0;                     // [E] ~ the time to turn the thrust a few degrees
    double max_tilt_rad = 4.0 * constants::deg_to_rad;  // [E] thrust tilt for the lateral acceleration
};

struct LunarLandingParameters {
    // Braking on the vacuum Raptors (Isp 380 s, fixed: attitude by the RCS - user 2026-10-15), then the gimballed
    // sea_level_1 as the thrust demand falls (as Apollo throttled down): a smaller set takes over when the demand stays
    // below `set_drop_fraction` of its maximum for `set_drop_s`; a set not yet running is lit startup_s ahead.
    // Group ids carry the stage prefix (engine group_id).
    std::vector<std::vector<std::string>> braking_sets{{"hls/vacuum"}, {"hls/sea_level_1"}};
    // A failed engine in a fixed (ungimballed) set: its off-axis thrust is a torque no RCS holds - the set is cut at
    // once and this gimballed set continues the braking (user 2026-10-15); the later sets follow as planned.
    std::vector<std::string> engine_out_set{"hls/sea_level_1", "hls/sea_level_2", "hls/sea_level_3"};
    double set_drop_fraction = 0.9;  // [E]
    double set_drop_s = 2.0;         // [E]
    double demand_trend_s = 4.0;     // [E] the demand's trend over this window predicts a disjoint set's drop
    // High gate (guidance frame: x up, y crossrange, z downrange; origin the touchdown point) [E] - within the SCvx
    // glide cone (30 deg: 2 km over 3 km is 33.7 deg).
    BrakingTarget high_gate{{700.0, 0.0, -800.0}, {-15.0, 0.0, 25.0}, {0.3, 0.0, -0.8}, 0.0};
    double approach_time_to_go_s = 15.0;  // [E] the SCvx approach takes over at this braking t_go
    double braking_time_to_go_guess_s = 400.0;
    ScvxLandingParameters approach;  // phase groups {sea_level_1}, {sea_level_1}, {landing}; HLS limits
    // Once the approach lights its last set (the landing thrusters, planned within pad_final_set_radius_m of the point),
    // the terminal descent flies them to the touchdown.
    TerminalDescentParameters terminal;
};
// The HLS's values (the [E] limits found in Task 3's lunar SCvx test).
LunarLandingParameters hls_landing_parameters();

// Powered descent of a lunar lander: the Apollo braking law from PDI to the high gate, then SCvx plans to the touchdown
// point (Raptors, then the landing thrusters), engines off on leg contact. Commands as the ship landing
// (ReturnCommand) so the session flies them the same way.
class LunarLandingGuidance {
public:
    // `touchdown`: body Moon, the point on the terrain with the pad target from the legs; `approach_azimuth_rad`:
    // clockwise from north at the touchdown point.
    LunarLandingGuidance(LandingSite touchdown, double approach_azimuth_rad, LunarLandingParameters parameters);
    ReturnCommand update(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch,
                         bool leg_contact);
    void mark_landed() { phase_ = LunarDescentPhase::Landed; }
    LunarDescentPhase phase() const { return phase_; }
    double braking_time_to_go_s() const { return braking_t_go_; }
    const std::string& status() const { return status_; }  // German, e.g. why the approach could not be planned
    const ScvxLandingGuidance* approach() const { return approach_.get(); }
    // The braking set running now (index into braking_sets; the engine-out set replaces its groups).
    const std::vector<std::string>& braking_groups() const { return set_groups_; }
    void set_realtime(bool realtime);

private:
    ReturnCommand braking(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    ReturnCommand approach_command(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                   time::Epoch epoch);
    ReturnCommand terminal(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    // Gravity in the plan frame at the vehicle: gravitation less the Moon centre's acceleration, Coriolis and centrifugal
    // terms of the Moon's rotation.
    math::Vec3 effective_gravity_plan(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env,
                                      time::Epoch epoch, const environment::BodyFrame& frame, const DescentState& x) const;
    // Body x (GCRF) that brings the thrust axis of `groups` onto `direction_gcrf` by the shortest rotation.
    static math::Vec3 hull_axis_for(const vehicle::FlightBody& body, const math::Vec3& direction_gcrf,
                                    const std::vector<std::string>& groups);
    void start_approach(const vehicle::FlightBody& body, const vehicle::FlightEnvironment& env, time::Epoch epoch);
    // Set drops and the engine-out switch; the groups to run now.
    std::vector<std::string> braking_groups_now(const vehicle::Vehicle& vehicle, double demand_n, time::Epoch epoch);

    LandingSite site_;
    LunarLandingParameters parameters_;
    PlanFrame plan_;
    math::Mat3 plan_to_guidance_;  // rows: up, crossrange, downrange in plan coordinates
    ApolloBrakingGuidance braking_;
    std::unique_ptr<ScvxLandingGuidance> approach_;
    LunarDescentPhase phase_ = LunarDescentPhase::Braking;
    size_t braking_set_ = 0;
    std::vector<std::string> set_groups_;  // groups of the running braking set
    bool next_lit_ = false;                // the next (disjoint) set started ahead of its drop
    double below_set_s_ = 0.0;
    bool started_ = false;
    time::Epoch last_update_;
    std::deque<std::pair<time::Epoch, double>> demand_history_;  // (epoch, thrust demand N) over demand_trend_s
    double braking_t_go_ = 0.0;
    std::string status_;
    bool debug_ = false;  // STARSIM_DEBUG_LUNAR_DESCENT
    time::Epoch next_debug_;
};

}  // namespace starsim::gnc
