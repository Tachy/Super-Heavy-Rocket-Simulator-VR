#pragma once
// Heave, roll and pitch of a drone ship's deck in a long-crested sea (environment::SeaState): linear frequency-domain
// response of a box barge, superposed over the sea's regular wave components.
//
// Per component (unit amplitude, phase referenced to the elevation at the deck centre):
//   1. quasi-static excitation: the least-squares plane fit of the wave over the waterplane rectangle (the
//      Froude-Krylov pressure of a wall-sided box): mean elevation (heave), slope to port (roll) and slope towards the
//      bow (pitch). Separable over the rectangle, so it is analytic: sinc-type factors of k L / 2 and k B / 2.
//   2. dynamics: one damped mode each, driven by the strip-theory relative-motion excitation (Korvin-Kroukovsky): the
//      hydrostatic, added-mass and damping forces act on the motion relative to the wave, the wave reduced by the
//      Smith factor exp(-k d) (pressure decay to the keel):
//        H = exp(-k d) (1 - r^2 ca / (1 + ca) - 2 i zeta r) / (1 - r^2 - 2 i zeta r),  r = omega / omega_n.
//      The plain hydrostatic forcing (numerator 1) would overstate the resonant response 2-3x: the added-mass and
//      damping terms of the wave force (diffraction / Haskind) largely cancel the resonance of a wide shallow barge.
//      exp(-k d) is the usual reduction for heave; for roll / pitch it is a crude approximation [call]. The time factor
//      is exp(-i omega t) (as SeaState: cos(k s - omega t + phase)), so the minus signs make the response lag.
//      omega_n from the box hydrostatics: heave g / ((1 + ca) d); roll / pitch g GM / ((1 + ca) r^2) with
//      GM = KB + BM - KG = d/2 + w^2 / (12 d) - d (w = beam or length). [call] KG at the waterline (a box barge's
//      steel and ballast sit around mid-depth of a 6 m hull loaded to 3 m).
//   Long waves: the deck rides the surface (heave -> elevation, roll / pitch -> wave slopes); short waves average out.
// Non-goals (spec): surge, sway, yaw; coupling between the modes; hydrodynamic memory beyond the constant factors.
//
// Validity: linear up to ~6 deg of roll. The wing extensions (3 m freeboard at ~26 m half-width) reach the water at
// ~6.6 deg, the hull's deck edge at ~11 deg; beyond, restoring and damping turn nonlinear (both grow), so larger
// predicted motions are an upper bound - safe for the sea-state refusal, not a prediction.
// The deck frame composes roll then pitch about the level deck axes (small angles).

#include <complex>
#include <vector>

#include "starsim/config/drone_ship_config.hpp"
#include "starsim/environment/sea_state.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::contact {

struct DeckPose {
    double heave_m = 0.0;   // deck centre above its calm-sea position
    double roll_rad = 0.0;  // about the long axis, starboard down (port up) positive
    double pitch_rad = 0.0;  // about the beam, bow up positive
    double heave_rate_mps = 0.0;
    double roll_rate_radps = 0.0;
    double pitch_rate_radps = 0.0;
};

// The deck frame in the site frame (x east, y north, z up) whose origin is the deck centre at rest. The deck rotates
// about its centre [call] (no surge / sway: spec non-goal).
struct DeckFrame {
    math::Mat3 deck_to_site = math::Mat3::identity();  // columns: long axis (bow), port, deck normal
    math::Vec3 origin_site_m;                          // the deck centre: (0, 0, heave)
    math::Vec3 velocity_site_mps;
    math::Vec3 angular_velocity_site_radps;
};

// Significant values (2 sigma) from the spectrum; the tilt is the combined roll / pitch angle,
// 2 sqrt(var(roll) + var(pitch)).
struct DeckMotionStatistics {
    double significant_heave_m = 0.0;
    double significant_heave_rate_mps = 0.0;
    double significant_tilt_rad = 0.0;
};

struct DeckNaturalPeriods {
    double heave_s = 0.0;
    double roll_s = 0.0;
    double pitch_s = 0.0;
};

// Complex amplitudes per unit wave amplitude, referenced to the wave elevation at the deck centre.
struct DeckResponse {
    std::complex<double> heave_m;
    std::complex<double> roll_rad;
    std::complex<double> pitch_rad;
};

// wave_angle_rad: the wave's travel direction relative to the long axis, clockwise seen from above like a heading
// (0: from stern to bow, pi/2: from port to starboard).
// Quasi-static plane fit of a wave of wavenumber k over the waterplane (no dynamics).
DeckResponse deck_plane_fit(const config::DroneShipHull& hull, double wavenumber_radpm, double wave_angle_rad);
// Plane fit times the modes' oscillator responses; deep water k = omega^2 / g like environment::SeaState.
DeckResponse deck_response(const config::DroneShipHull& hull, double omega_radps, double wave_angle_rad);

class DeckMotion {
public:
    // deck_heading_rad: the bow's direction, clockwise from north. The sea is copied (its components).
    DeckMotion(const environment::SeaState& sea, const config::DroneShipHull& hull, double deck_heading_rad);

    DeckPose pose(double t_s) const;  // t_s: seconds after the sea's time origin
    DeckFrame frame(double t_s) const;
    DeckMotionStatistics statistics() const;
    DeckNaturalPeriods natural_periods() const;

private:
    struct Mode {  // one wave component's deck response: complex amplitudes (wave amplitude and phase included)
        double omega_radps;
        std::complex<double> heave_m, roll_rad, pitch_rad;
    };
    config::DroneShipHull hull_;
    double deck_heading_rad_;
    std::vector<Mode> modes_;
};

}  // namespace starsim::contact
