#pragma once

#include <optional>
#include <string>
#include <vector>

#include "starsim/contact/docking_interface.hpp"
#include "starsim/contact/probe_drogue_interface.hpp"

namespace starsim::contact {

// IDSS docking sequence of an active soft-capture system against a passive port (ISS i5; IDSS IDD E 3.1.1.1):
// petals -> latches on full ring contact (soft capture) -> attenuation by a damped 6-DOF joint -> retraction ->
// guide pins -> 12 hooks (hard capture). Undocking: hooks open -> separation springs. Passive port frame throughout.
// Too fast a contact runs the joint out of its stroke into the end stop: above the IDSS dynamic compression limit the
// mechanism releases the latches ("Überlast", spec correction 3).
// Refuelling r3: the same sequence with the four-probe interface (probe_drogue): the probes' stroke plays the ring's
// extension; structural latches play the hooks.

enum class DockingPhase { Ready, Contact, SoftCapture, Retraction, HooksClosing, Docked, HooksOpening, Separated, Failed };
const char* to_string(DockingPhase phase);  // German

struct DockingMechanismParameters {
    DockingInterfaceParameters interface;
    // Refuelling r3: the four-probe interface instead of the IDSS ring; its extension is the probe stroke.
    std::optional<ProbeDrogueParameters> probe_drogue;
    double joint_frequency_radps = 0.5;           // [E] attenuation: critically damped, time constant 2 s
    // Rotational joint stiffness from the IDSS loads (Table 3.3.1.4-1): half the bending (2,800 N m) / torsion
    // (1,500 N m) limit at the envelope's 4 deg misalignment / roll [D]; damping from the chaser inertia (ratio below).
    double joint_bending_stiffness_nm_per_rad = 0.5 * 2800.0 / (4.0 * constants::deg_to_rad);
    double joint_torsion_stiffness_nm_per_rad = 0.5 * 1500.0 / (4.0 * constants::deg_to_rad);
    double joint_damping_ratio = 1.0;             // [E]
    // > 0: bending / torsion stiffness = the chaser's inertia * frequency^2 (a heavy chaser: the IDSS values above
    // are load limits of a 12 t pair and would leave a 500 t ship swinging for minutes)
    double joint_rotation_frequency_radps = 0.0;
    double stroke_m = 0.10;                       // [E] axial attenuation stroke beyond the rest extension
    double end_stop_stiffness_n_per_m = 2.0e6;    // [E] beyond the stroke
    double attenuation_min_s = 120.0;             // [E] then retract once settled
    double settled_speed_mps = 0.005;             // [E]
    double settled_rate_radps = 0.05 * constants::deg_to_rad;  // [E]
    double retraction_speed_mps = 0.001;          // [E] 0.30 m in 300 s
    double retraction_timeout_s = 900.0;          // [E]
    double guide_pin_tolerance_m = 0.005;         // [E] fine alignment before the hooks
    double guide_pin_angle_rad = 0.5 * constants::deg_to_rad;  // [E]
    double hooks_close_s = 180.0;  // [E] with attenuation + retraction ~10 min soft -> hard [S: Demo-2 11 min]
    double hooks_open_s = 180.0;   // [E]
    int hook_count = 12;           // [S: IDSS IDD E 3.2.3.4]
    double separation_energy_j = 43.35;  // [E] middle of 39.2-47.5 N m [S: IDSS IDD E 3.2.3.6.2]
    double bounce_confirm_s = 2.0;       // [E] no contact this long ...
    double bounce_gap_m = 0.05;          // [E] ... and the ring this far beyond the petal tips: capture failed
    double rearm_distance_m = 1.0;       // [E] the session re-arms a failed mechanism this far beyond the petal tips
    // IDSS IDD E Table 3.3.1.4-1, SCS maximum interface loads (design limit loads) [S]:
    double dynamic_compression_limit_n = 6500.0;  // up to 0.1 s: above it the mechanism releases (Überlast)
    double static_compression_limit_n = 3500.0;   // held longer than static_after_s: Überlast
    double static_after_s = 0.1;
    double tension_limit_n = 3900.0, shear_limit_n = 3200.0, bending_limit_nm = 2800.0, torsion_limit_nm = 1500.0;
    std::string limit_name = "IDSS-Grenze";  // in the load warnings
};

struct DockingWrench {
    math::Vec3 force_port_n;    // on the chaser, acting at point_port_m (the passive body: -force through its CG)
    math::Vec3 point_port_m;
    math::Vec3 torque_port_nm;  // pure couple on the chaser (joint rotation)
};
struct LoadPeaks {  // at the SCS mating plane centre (the passive ring face centre)
    double compression_n = 0.0, tension_n = 0.0, shear_n = 0.0, bending_nm = 0.0, torsion_nm = 0.0;
};
struct DockingEvent {
    double t_s = 0.0;
    DockingPhase phase = DockingPhase::Ready;
    std::string status;  // German
};

// Relative speed of the separation springs: v = sqrt(2 E / mu), mu the reduced mass.
double separation_relative_speed_mps(const DockingMechanismParameters& p, double chaser_mass_kg, double passive_mass_kg);

class DockingMechanism {
public:
    explicit DockingMechanism(DockingMechanismParameters p = {});
    // One tick: the wrench on the chaser for the next step. `chaser_inertia_kg_m2`: its largest principal moment plus
    // m l^2 about the port (the joint's rotational gain).
    DockingWrench update(double t_s, const DockingPose& pose, double chaser_mass_kg, double chaser_inertia_kg_m2,
                         double dt_s);
    bool undock(double t_s);  // Docked -> HooksOpening; false + refusal() otherwise
    // Ready -> Docked directly (a scenario that starts docked, ISS i6): ring retracted, hooks closed, the event at t_s
    // as after a hard capture. False + refusal() outside Ready.
    bool start_docked(double t_s);
    bool separation_due();    // true once, when Separated is reached
    // While the hooks open: when the springs separate the pair (mechanism clock); nullopt otherwise.
    std::optional<double> separation_t_s() const {
        if (phase_ != DockingPhase::HooksOpening) return std::nullopt;
        return phase_since_s_ + p_.hooks_open_s;
    }
    void rearm();             // Failed -> Ready (a new approach); no-op otherwise
    bool hard_captured() const { return phase_ == DockingPhase::Docked; }
    DockingPhase phase() const { return phase_; }
    const std::string& status() const { return status_; }
    const std::string& failure() const { return failure_; }
    const std::string& refusal() const { return refusal_; }
    const LoadPeaks& peaks() const { return peaks_; }
    const std::vector<DockingEvent>& events() const { return events_; }
    const std::vector<std::string>& warnings() const { return warnings_; }
    double ring_extension_m() const { return extension_m_; }
    double full_extension_m() const;  // the ring / probes fully extended (before capture)
    double first_contact_m() const;   // port separation along the axis at the first touch (petal tips / mouths)
    double reach_m() const;           // CG-free bound: beyond this port distance no part of the interface can touch
    const DockingMechanismParameters& parameters() const { return p_; }

private:
    void enter(DockingPhase phase, std::string status, double t_s);
    void fail(std::string reason, double t_s);
    InterfaceContact contact(const DockingPose& pose, double extension_m) const;
    DockingWrench joint(const DockingPose& pose) const;
    DockingWrench before_capture(double t_s, const DockingPose& pose, double mass_kg, double inertia_kg_m2);
    void record_loads(const DockingWrench& w, double t_s, double dt_s);
    void warn(const std::string& text);

    DockingMechanismParameters p_;
    DockingPhase phase_ = DockingPhase::Ready;
    double extension_m_ = 0.0;
    double phase_since_s_ = 0.0, last_contact_s_ = 0.0, static_over_s_ = 0.0;
    bool ring_touched_ = false, ring_blocked_ = false, tips_only_ = true;
    double k_ = 0.0, c_ = 0.0, cb_ = 0.0, ct_ = 0.0;  // translational stiffness / damping; bending / torsion damping
    double kb_ = 0.0, kt_ = 0.0;                       // bending / torsion stiffness of the joint
    bool separation_pending_ = false;
    LoadPeaks peaks_;
    std::string status_ = "bereit", failure_, refusal_;
    std::vector<DockingEvent> events_;
    std::vector<std::string> warnings_;
};

}  // namespace starsim::contact
