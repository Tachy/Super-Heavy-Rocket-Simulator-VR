#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "starsim/gnc/proximity_geometry.hpp"
#include "starsim/gnc/relative_navigation.hpp"
#include "starsim/gnc/translation_loop.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"
#include "starsim/orbit/clohessy_wiltshire.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::gnc {

// Proximity operations state machine (ISS i4): from Waypoint 0 along the waypoint path (CW transfers outside the
// keep-out sphere, the IDA-3 fly-around, axial approach inside it) to the contact plane, with the corridor and
// speed monitors, commanded hold / retreat / abort (breakout) and resume. Pure guidance: it reads the relative
// state and returns translation fractions in the target port frame; attitude is held by the caller.

// German names (to_string): "Wegpunkt" (holding at path waypoint `waypoint()` - its name is the status), "Annäherung"
// (CW transfer outside the sphere), "Umflug" (CW transfer to a fly-around point), "Anflug" (axial approach Waypoint 1
// -> 2), "Endanflug" (Waypoint 2 -> contact), "Halt", "Rückzug", "Abbruch", "Kontakt".
enum class ProximityPhase { Waypoint, Transfer, Flyaround, Approach, FinalApproach, Hold, Retreat, Abort, Contact };
const char* to_string(ProximityPhase phase);

struct ProximityInput {
    time::Epoch epoch;
    orbit::RelativeState cg_lvlh;       // chaser CG relative to the station CG, station LVLH (velocity in the rotating frame)
    RelativePortState port;             // chaser port vs target port, target port frame
    math::Mat3 port_to_lvlh;            // target port frame axes in the station LVLH frame (columns)
    double mean_motion_radps = 0.0;     // station orbit
    AxisAcceleration authority_port;    // pure translation authority along the target port frame axes
    double propellant_kg = 0.0;
};
struct ProximityOutput {
    math::Vec3 translation_fraction_port;  // to FlightSession::set_translation_command semantics
    bool attitude_hold = true;             // false after a completed breakout: the vehicle drifts (and may coast)
};
struct ProximityEvent {
    time::Epoch epoch;
    ProximityPhase phase;
    std::string status;  // German
};
struct ProximityContact {
    time::Epoch epoch;
    RelativePortState port;
    EnvelopeCheck envelope;
};

class ProximityGuidance {
public:
    ProximityGuidance(DockingApproach approach, ProximityParameters parameters);
    ProximityOutput update(const ProximityInput& in);

    // Commands (false, a German refusal() and no change when refused). Accepted commands take effect in the next update,
    // which supplies the live state; further commands are refused until then.
    bool hold();     // keep the current relative position; refused in Abort / Contact / Hold
    bool retreat();  // back to the last waypoint reached; refused at/before Waypoint 0, in Abort / Contact / Retreat
    bool abort();    // breakout now; refused in Abort / Contact
    bool resume();   // from a commanded Hold, a waiting waypoint after a retreat, or Abort after the breakout burn (back
                     // to Waypoint 0 when the CG range <= resume_max_range_m, outside the keep-out sphere and above
                     // the deorbit reserve); refused otherwise

    // Docking outcomes reported by the session (ISS i5). capture_failed: from Contact back along the axis to the last
    // on-axis waypoint (Waypoint 2), where the guidance waits for resume() ("Rückzug: Fangen fehlgeschlagen (<reason>)").
    // collided: the flight ends passively in Contact with the status "Kollision" (no command accepted).
    void capture_failed(const ProximityInput& in, const std::string& reason);
    void collided(time::Epoch epoch);
    bool collision() const { return collided_; }

    ProximityPhase phase() const { return phase_; }
    // At a waypoint after a retreat (commanded or a failed capture), waiting for resume() (refuelling r5).
    bool waiting_for_go() const { return phase_ == ProximityPhase::Waypoint && waiting_for_go_; }
    size_t waypoint() const { return target_; }  // index into path() of the waypoint flown to / held at
    const std::vector<ProximityWaypoint>& path() const { return path_; }
    const std::string& status() const { return status_; }
    const std::string& refusal() const { return refusal_; }  // German reason of the last refused command
    // Refreshes the live state the command checks use (resume's range / propellant) without flying: the session
    // calls it before a command, since after a breakout it coasts without updates.
    void observe(const ProximityInput& in);
    const std::vector<ProximityEvent>& events() const { return events_; }
    const std::optional<ProximityContact>& contact() const { return contact_; }

private:
    enum class Request { None, Hold, Retreat, Abort, Resume };

    void enter(ProximityPhase phase, std::string status, time::Epoch epoch);
    math::Vec3 target_cg(const ProximityInput& in, size_t index) const;
    bool inside_sphere(const ProximityInput& in) const;
    double corridor_bound_m(double x_m) const;
    void start(const ProximityInput& in);
    void resolve_request(const ProximityInput& in);
    void start_segment(const ProximityInput& in);
    bool plan_transfer(const ProximityInput& in, ProximityPhase phase, double nominal_s, std::string status);
    void start_retreat(const ProximityInput& in, size_t index);
    void begin_abort(std::string status, time::Epoch epoch);
    void arrive(const ProximityInput& in, bool waiting_for_go);
    void auto_hold(const ProximityInput& in, std::string status);
    void transitions(const ProximityInput& in);
    void monitor(const ProximityInput& in);
    ProximityOutput fly(const ProximityInput& in);
    math::Vec3 loop_lvlh(const ProximityInput& in, const math::Vec3& desired_lvlh, const TranslationLoopParameters& lp);
    math::Vec3 loop_port(const ProximityInput& in, const math::Vec3& desired_port, const TranslationLoopParameters& lp);

    DockingApproach approach_;
    ProximityParameters p_;
    std::vector<ProximityWaypoint> path_;
    ProximityPhase phase_ = ProximityPhase::Waypoint;
    size_t target_ = 0, reached_ = 0;
    bool started_ = false, waiting_for_go_ = false, go_ = false, auto_hold_ = false;
    Request request_ = Request::None;
    size_t retreat_index_ = 0;  // with Request::Retreat
    ProximityPhase interrupted_ = ProximityPhase::Waypoint;
    std::string interrupted_status_;
    time::Epoch last_epoch_, arrival_epoch_;
    double last_cg_range_m_ = 0.0, last_propellant_kg_ = 0.0;
    bool refuse(std::string reason);
    bool command_open();  // started and no request pending; sets the refusal otherwise
    std::optional<time::Epoch> waypoint_since_;
    math::Vec3 last_transfer_velocity_lvlh_;
    bool retreat_axial_ = false;
    bool collided_ = false;
    math::Vec3 hold_point_;
    bool hold_in_port_frame_ = false;
    bool breakout_planned_ = false, breakout_done_ = false, breakout_safe_ = false;
    math::Vec3 abort_target_lvlh_;
    TranslationLoop loop_;
    std::string status_, refusal_;
    std::vector<ProximityEvent> events_;
    std::optional<ProximityContact> contact_;
};

}  // namespace starsim::gnc
