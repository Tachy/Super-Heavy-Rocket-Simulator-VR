#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/frames/body_orientation.hpp"
#include "starsim/gnc/burn_node.hpp"
#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/mission/finite_burn.hpp"
#include "starsim/mission/lunar_outbound.hpp"
#include "starsim/mission/nrho_reference.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Where and how the return should meet the Earth.
struct EarthReturnTarget {
    math::Vec3 site_itrf_m;                  // the landing site (Starbase tower) on the ground
    double interface_height_m = 120e3;       // b2's entry interface
    double flight_path_rad = -5.0 * constants::deg_to_rad;  // relative to the rotating Earth (corridor middle)
    double speed_mps = 0.0;                  // relative, after the braking burn; 0: no braking
    double braking_height_m = 2000e3;        // [E] the braking burn's centre on the inbound leg (spec amendment)
    // Direct entry: the onboard predictor flying `bank_sign` x `nominal_bank_rad` must end at `gate` (b4's deorbit
    // rule); empty: aerocapture - only the plane is aimed (the site in the arrival plane).
    std::optional<gnc::EntryPredictorModel> entry_model;
    gnc::EntryTarget gate;
    double gate_height_m = 800.0;
    // [E] lift down: at ~9.6 km/s a constant 50-80 deg bank (b4's LEO value) skips out beyond the carry limit and
    // 90 deg sits on the skip boundary (the predicted range jumps by 100+ km for a millimetre per second at TEI);
    // 110 deg captures and its range is smooth. The guidance flies the bank it needs (up to 150 deg).
    double nominal_bank_rad = 110.0 * constants::deg_to_rad;
    // [E] Direct entry aims at ONE bank sign's track (l5 Task 8b): at lunar-return speed each sign's track ends
    // ~1,000 km beside the arrival plane, the average of both left the site that far from either and the guidance's
    // one large reversal lost the ship. The guidance then starts on this sign (its track ends closest to the site).
    double bank_sign = 1.0;
    // [E] Direct entry (l5 Task 10): the held sign's track is aimed this far beyond the site, away from the side a
    // reversal moves it to (bank_sign x margin), so that holding and reversing straddle the site and the guidance's
    // reversal timing lands on it. Aimed at the site itself (0), a truth lift 6.5 % high drifted the track 150 km to
    // the reversal side, where no reversal helps (campaign seed 11: -160 km at the gate).
    double lateral_margin_m = 300e3;
    // Direct entry: the aimed track's end beside the arrival plane; the crossrange is measured from it.
    // The gate aiming refines it as a fixed point (warm start: the previous plan's value).
    double lateral_offset_m = 0.0;
};

// The arrival of a return trajectory (after the modelled braking burn).
struct EarthArrival {
    bool reached = false;                 // crossed the interface height inbound
    bool braked = false;                  // the braking burn was applied (speed above the target at its height)
    time::Epoch braking_epoch;
    double braking_delta_v_mps = 0.0;
    time::Epoch interface_epoch;
    math::StateVector interface_gcrf;
    double flight_path_rad = 0.0;         // relative
    double speed_mps = 0.0;               // relative
    double perigee_height_m = 0.0;        // osculating, after the braking (two-body)
    double crossrange_m = 0.0;            // the site's distance from the arrival plane (> 0: right) - lateral offset
    double downrange_error_m = 0.0;       // direct: predictor at the nominal bank (> 0: overshoot); else 0
    double predicted_crossrange_m = 0.0;  // direct: predictor at the nominal bank, from the aim point (lateral_margin_m)
    double lateral_offset_m = 0.0;        // the target's lateral offset this arrival was measured with
};

struct ReturnBurn {
    bool converged = false;
    int iterations = 0;
    gnc::BurnNode burn;
    EarthArrival arrival;
};

struct EarthReturnParameters {
    LunarOrbitKind orbit = LunarOrbitKind::Llo;
    std::shared_ptr<const NrhoReference> nrho;   // Nrho
    EarthReturnTarget target;
    double flight_time_guess_s = 3.5 * 86400.0;  // [E] spec: ~3-4 days (first guess only)
    double search_s = 86400.0;                   // [E] departure revolutions (LLO) / perilunes (NRHO) scanned
    double earliest_departure_s = 1800.0;        // [E] after the plan's epoch
    double max_tei_mps = 1500.0;                 // [E] a TEI above this is refused
    std::vector<double> mcc_after_tei_s{86400.0, 2.5 * 86400.0};  // [E] spec: two corrections
    double mcc_threshold_mps = 0.05;             // [E] as the outbound flight
    double mcc_min_before_braking_s = 6.0 * 3600.0;  // [E] no MCC closer to the braking burn
    double mcc_allowance_mps = 30.0;             // [E] propellant check: mid-course corrections on top of the plan
    // NRHO: a single burn at a perilune meets the return asymptote head-on (~3.4 km/s, the mirror of l3's outbound).
    // The return departs as Artemis plans it: a departure burn (NRD) drops to a lunar flyby, the return powered flyby
    // (RPF) at its periselene leaves for the Earth (user 2026-10-16). Mirror of l3's in-direct transfer [S: Kikuchi,
    // Matsumoto & Ueda, ISSFD 2024: swing-by at 100 km, NRHO insertion ~1.2 d later]; the departure time and the flyby
    // point are searched around each perilune.
    double flyby_altitude_m = 100e3;
};

struct ReturnPlan {
    bool feasible = false;
    std::string reason;  // German when refused
    // NRHO: the departure burn dropping to the return flyby; `tei` is then the powered flyby at its periselene.
    std::optional<gnc::BurnNode> nrho_departure;
    gnc::BurnNode tei;
    EarthArrival arrival;
    double tei_delta_v_mps = 0.0;  // NRHO: departure burn + powered flyby
    double braking_delta_v_mps = 0.0;
};

// Direction (unit, GCRF, Moon-relative) of the hyperbolic excess velocity that leaves the Moon at `departure` on a
// patched-conic return (Earth two-body Lambert arc from the Moon's centre to a vacuum perigee 50 km above the
// equatorial radius, `flight_time_s` later, in the plane through the Moon and the site at the arrival, eastward).
math::Vec3 return_asymptote(const ephemeris::Ephemeris& ephemeris, time::Epoch departure, double flight_time_s,
                            const math::Vec3& site_itrf_m, const frames::BodyOrientation& earth);

// Plans the return from lunar orbit to the Earth in the full force model (the gravity model the flight uses): TEI,
// mid-course corrections and the braking burn before the entry interface (l5). The burns are impulsive nodes (finite
// when a model is set); the session flies them and re-plans at every step.
class EarthReturnPlanner {
public:
    EarthReturnPlanner(const environment::GravityModel& gravity, const ephemeris::Ephemeris& ephemeris,
                       const frames::BodyOrientation& earth_orientation);

    // Coasts `state` to `burn_epoch`, applies `delta_v` (finite with the model set), coasts inbound, applies the
    // braking burn as an impulse at the braking height (retrograde, sized for the target relative speed at the
    // interface), coasts to the interface and evaluates it.
    EarthArrival evaluate(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                          const math::Vec3& delta_v_mps, const EarthReturnTarget& target) const;

    // Newton (minimum-norm, finite differences, backtracking) on the 3 delta-v components: perigee height of the
    // braked conic (the target angle's), crossrange zero, and (direct) the predictor's downrange error zero.
    ReturnBurn solve(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                     const EarthReturnTarget& target, const math::Vec3& initial_delta_v_mps, int max_iterations = 30) const;

    // TEI from the LLO (departure revolutions within search_s, burn at the patched-conic periapsis of each) or from the
    // NRHO (departure burn before each perilune within search_s, powered flyby near that perilune; departure time and
    // flyby point searched); the cheapest converged candidate within max_tei_mps (the sum for the NRHO).
    ReturnPlan plan(const math::StateVector& state, time::Epoch epoch, const EarthReturnParameters& p,
                    double mass_kg, double isp_s, double propellant_kg) const;

    // Mid-course correction at `burn_epoch` (zero first guess) back onto the planned arrival (a nearby gate); empty
    // below the threshold, too close to the braking burn, or not converged (`converged`, if given, false only then).
    std::optional<gnc::BurnNode> mid_course(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                            const EarthReturnParameters& p, bool* converged = nullptr) const;

    // Re-plan from the Earth-bound coast (l5 Task 10, user 2026-10-18: a failed correction must be re-planned from the
    // current flight data): the correction at `burn_epoch` aimed with the gate scan - the site's pass within +-12 h of
    // the arrival, the cheapest crossing - instead of back onto the planned arrival. Not converged: no way to the site.
    ReturnBurn recovery(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                        const EarthReturnTarget& target) const;

    // The braking burn re-planned from the state now as the last targeting burn: burn epoch at the predicted braking
    // height crossing; constraints: relative speed and flight-path angle at the interface, and (direct) the predictor's
    // crossrange (the entry guidance nulls the downrange). Empty if not converged.
    std::optional<gnc::BurnNode> braking(const math::StateVector& state, time::Epoch epoch,
                                         const EarthReturnTarget& target) const;

    // Entry trim after the braking burn (l5 Task 10): the minimum-norm burn at `burn_epoch` that restores the target
    // relative speed, flight-path angle and (direct) predicted crossrange at the interface; empty below
    // entry_trim_threshold_mps or not converged. The burn's execution error moves the angle ~0.1 deg per m/s.
    std::optional<gnc::BurnNode> entry_trim(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                                            const EarthReturnTarget& target) const;
    // [E] below it the trim is not flown (~0.05 deg of entry angle). l5 Task 10: 2 m/s (~0.2 deg) was set while the
    // campaign's entry angle error still came right after the braking burn; the aerocapture corridor (+-0.25 deg)
    // cannot spare 0.2 deg (EI 0.17 deg shallow: exits at 12,000-20,000 km).
    static constexpr double entry_trim_threshold_mps = 0.5;

    // The epoch the inbound coast crosses the braking height (empty: none within 6 days).
    std::optional<time::Epoch> braking_epoch(const math::StateVector& state, time::Epoch epoch,
                                             const EarthReturnTarget& target) const;

    void set_finite_burn(std::optional<FiniteBurnModel> model) { finite_ = model; }

private:
    // patched-conic first guess at the departure: burn epoch and delta-v
    struct Departure { time::Epoch epoch; math::Vec3 delta_v_mps; };
    std::vector<Departure> departures(const math::StateVector& state, time::Epoch epoch,
                                      const EarthReturnParameters& p) const;
    // The Earth-bound burn at `burn_epoch` (TEI or the powered flyby): plane, then (direct, `scan`) the gate.
    ReturnBurn depart(const math::StateVector& state, time::Epoch epoch, time::Epoch burn_epoch,
                      const EarthReturnTarget& target, const math::Vec3& guess, bool scan_gate) const;
    // NRHO: departure burn + powered flyby about `perilune` (departure time, flyby time and point searched on the
    // patched conic, then targeted in the full force model).
    struct NrhoReturn { bool found = false; gnc::BurnNode departure; ReturnBurn flyby; };
    NrhoReturn nrho_return(const math::StateVector& state, time::Epoch epoch, time::Epoch perilune,
                           const EarthReturnParameters& p) const;
    // `state` at `burn_epoch` with the burn applied (finite with the model set)
    math::StateVector burned(const math::StateVector& at_burn, time::Epoch burn_epoch, const math::Vec3& delta_v) const;
    // From a state at or after the braking burn: osculating perigee, interface and predictor.
    void arrive(const math::StateVector& state, time::Epoch epoch, const EarthReturnTarget& target,
                EarthArrival& a) const;
    // Perigee height the braked conic needs for the target interface conditions (exact at the converged arrival).
    double perigee_target_m(const EarthArrival& a, const EarthReturnTarget& target) const;
    // The burn at `burn_epoch` from `at` (the state there) that meets the target relative speed and flight-path angle at
    // the interface and (direct) the predictor's crossrange; none modelled after it. Falls back to speed and angle.
    std::optional<math::Vec3> aim_interface(const math::StateVector& at, time::Epoch burn_epoch,
                                            const EarthReturnTarget& target, const math::Vec3& guess,
                                            const char* what) const;
    // Direct entry, from a plane solution (perigee and crossrange met): the interface epoch moved along the plane
    // solutions (perigee, crossrange and that epoch targeted) until the predictor's downrange error is zero - secant
    // from the arrival (`scan` false: a nearby gate), or scanned +-12 h both ways for the site's pass and the cheapest
    // crossing refined (`scan` true: TEI). The downrange is badly conditioned in the three delta-v components (the
    // site turns ~1,500 km/h under the arrival; perigee and plane are nearly collinear), well in the epoch.
    // The predictor flies one bank sign, whose track ends beside the arrival plane: the plane is aimed that far off
    // the site (EarthReturnTarget::lateral_offset_m), refined by the predicted crossrange until the track ends on the
    // site. Result: the delta-v and the lateral offset it was aimed with.
    struct GateAim { math::Vec3 delta_v_mps; double lateral_offset_m = 0.0; };
    std::optional<GateAim> aim_at_gate(const math::StateVector& at_burn, time::Epoch burn_epoch,
                                       const EarthReturnTarget& target, const math::Vec3& plane_delta_v_mps,
                                       bool scan) const;
    std::optional<math::Vec3> aim_at_gate_once(const math::StateVector& at_burn, time::Epoch burn_epoch,
                                               const EarthReturnTarget& target, const math::Vec3& plane_delta_v_mps,
                                               bool scan) const;

    const environment::GravityModel& gravity_;
    const ephemeris::Ephemeris& ephemeris_;
    const frames::BodyOrientation& earth_;
    PositionTargeter position_;  // the NRHO departure burn onto the flyby's periselene point
    double gm_earth_;
    double gm_moon_;
    std::optional<FiniteBurnModel> finite_;
};

}  // namespace starsim::mission
