#pragma once

// Cloud optics (M8 weather and clouds): extinction from water content, the droplet phase function, the delta-Eddington
// two-stream reference, the scattering source the cloud shader uses (TMS), and the CPU cloud light (Sun
// transmittance and sky factor at a point). game/shaders/clouds.gdshaderinc ports the phase function and the source
// terms below: change both together.
//
// Extinction (geometric-optics limit, extinction efficiency 2; Stephens 1978, J. Atmos. Sci. 35, 2123; Fu 1996,
// J. Climate 9, 2058 for ice): sigma = 3 LWC / (2 rho_w r_l) + 3 IWC / (2 rho_ice r_i) [1/m] with the effective radii
// r_l = 10 um, r_i = 30 um (constants.hpp). Single-scattering albedo 1 (visible).
//
// Phase function (per steradian, normalised over the sphere; cos_theta = cosine of the scattering angle, 1 = forward):
// the approximate Mie function of Jendersie & d'Eon 2023 ("An Approximate Mie Scattering Function for Fog and Cloud
// Rendering", SIGGRAPH 2023 Talks, doi 10.1145/3587421.3595409), a Henyey-Greenstein + Draine mixture
//   p(mu) = (1 - w) HG(mu; g_hg) + w Dr(mu; g_d, alpha),
//   HG(mu; g) = (1 - g^2) / (4 pi (1 + g^2 - 2 g mu)^(3/2)),
//   Dr(mu; g, alpha) = HG(mu; g) (1 + alpha mu^2) / (1 + alpha (1 + 2 g^2) / 3)   (Draine 2003, ApJ 598, 1017),
// with their fit for the droplet diameter d in um (5 <= d <= 50):
//   g_hg = exp(-0.0990567 / (d - 1.67154)),  g_d = exp(-2.20679 / (d + 3.91029) - 0.428934),
//   alpha = exp(3.62489 - 8.29288 / (d + 5.52825)),  w = exp(-0.599085 / (d - 0.641583) - 0.665888).
// d = 2 r_l = 20 um: g_hg 0.99461, g_d 0.59379, alpha 27.114, w 0.49816. Ice uses the same function (simplification:
// real ice crystals scatter with g ~0.75-0.8 and show halos).
// Its mean cosine (asymmetry parameter) in closed form, with the Draine moment g_d (1 + alpha (3 + 2 g_d^2) / 5) /
// (1 + alpha (1 + 2 g_d^2) / 3): g = (1 - w) g_hg + w * that = 0.8793.
//
// Two-stream reference: delta-Eddington (Joseph, Wiscombe & Weinman 1976, J. Atmos. Sci. 33, 2452) for conservative
// scattering, closed forms of Meador & Weaver 1980 (J. Atmos. Sci. 37, 630) with omega = 1:
//   f = g^2, g' = g / (1 + g), tau' = (1 - f) tau, gamma1 = 3 (1 - g') / 4, gamma3 = (2 - 3 g' mu0) / 4;
//   beam (Sun at cos zenith mu0): R = [gamma1 tau' + (gamma3 - gamma1 mu0)(1 - exp(-tau'/mu0))] / (1 + gamma1 tau');
//   diffuse (isotropic) incidence: R = gamma1 tau' / (1 + gamma1 tau'), T = 1 / (1 + gamma1 tau'),
//   gamma1 tau' = 3 (1 - g) tau / 4. Black surface below the slab.
// Against a Monte Carlo slab with the phase function above (tools/validation/cloud_slab_monte_carlo.py, 4e5 photons) the
// beam R is within 7 % at mu0 = 1 and 10 % at mu0 = 0.5 for tau 1-50 (within 1 % for tau >= 30), the diffuse T at
// tau 20 0.356 vs 0.346.
//
// Scattering source (what the shader integrates along view rays): the delta-Eddington scaled problem, its radiance by
// the two-stream source-function method (Toon, McKay, Ackerman & Santhanam 1989, J. Geophys. Res. 94, 16287), with the
// TMS single-scattering correction of Nakajima & Tanaka 1988 (JQSRT 40, 51). Independent-column approximation: the
// diffuse field is that of the local plane-parallel column (slab-based multiple scattering as in Bouthors et al. 2008,
// "Interactive multiple anisotropic scattering in clouds", I3D, with the two-stream solution in place of their Monte
// Carlo tables).
// The scaled problem (f = g^2, optical depths t' = (1 - f) t, phase function pE(mu) = (1 + 3 g' mu) / (4 pi),
// single-scattering albedo 1) has, per unit Sun irradiance (normal to the beam) and per unit scaled optical depth, the
// source function
//   J*(t') = I0(t') + g' u_z I1(t') + pE(cos_theta) exp(-tau_s'),
// the Eddington diffuse field I = I0 + I1 u_z (scaled radiance of all orders >= 1) scattered once more (orders >= 2)
// plus the scaled direct beam scattered once (order 1 of the scaled problem). TMS replaces that order-1 term by the
// exact phase function of the truncated problem, P(cos_theta) / (1 - f) (Nakajima & Tanaka 1988: u = u* - u*_1 + u_1,
// u_1 with omega P / (1 - f omega) on the scaled depths, omega = 1):
//   J_TMS(t') = I0(t') + g' u_z I1(t') + P(cos_theta) / (1 - f) exp(-tau_s').
// Per unit UNSCALED optical depth (dt' = (1 - f) dt), which is what cloud_scattering_source() returns:
//   J = (1 - f) [I0(t') + g' u_z I1(t')] + P(cos_theta) exp(-(1 - f) tau_s),
//   L = integral J exp(-(1 - f) tau_v) dtau_v   (every attenuation on the scaled depths: the forward peak keeps the
//                                                scattered light in the Sun's ray and in the view ray).
// Each order is counted once: the diffuse part holds orders >= 2, the TMS term order 1 (and the forward-peak paths
// the scaled problem folds into its direct beam). With
//   I0 = Sigma / (2 pi), I1 = 3 D / (4 pi)   (Eddington: F_up + F_down = 2 pi I0, F_up - F_down = 4 pi I1 / 3),
//   D(t') = mu0 (R + exp(-t'/mu0) - 1)                       (net upward diffuse flux F_up - F_down),
//   Sigma(t') = mu0 [R + 2 gamma1 (R t' + mu0 (1 - exp(-t'/mu0)) - t') + (1 - 2 gamma3)(1 - exp(-t'/mu0))]
//                                                            (F_up + F_down), R = beam R(tau*', mu0) above,
// t' = (1 - f) t the scaled vertical depth below the column top, tau*' = (1 - f) tau* the scaled column, tau_s the
// (unscaled) optical depth along the Sun ray, u_z the cosine of the view-ray propagation direction (toward the camera)
// to the local up. cos_theta is the cosine between the Sun's propagation direction and the view-ray propagation
// direction: 1 = looking into the Sun. Sun below the horizon (mu0 <= 0): no source.
// slab_reflected_radiance() is the closed-form integral of J along an upward view ray out of the top of a slab
// (t-integrals of exp(-t'/mu) times 1, t', exp(-t'/mu0)); slab_reflectance() its hemisphere integral (Gauss-Legendre
// 64 x 64), the model's plane-parallel albedo.
// Known limits (measured against tools/validation/cloud_slab_monte_carlo.py, same phase function, 4e5 photons for R,
// 1e7 for the radiance bins, tests/validation/weather/cloud_slab_radiance_mc.csv):
//   - albedo R vs Monte Carlo: Sun overhead +4 / 0 / -5 / -3 / -3 % (tau 1 / 3 / 10 / 30 / 50); Sun at 60 deg
//     -32 / -32 / -21 / -14 / -12 %;
//   - reflected radiance (view zenith 0 / 30 / 60 deg, relative azimuth 0 / 90 / 180 deg, tau 1 / 10 / 30): Sun
//     overhead within 8 %; Sun at 60 deg: backscatter side (azimuth 180) and nadir view -2..-19 %, azimuth 90
//     -6..-40 %, forward side (azimuth 0) -19..-56 %. The smooth Eddington field misses the forward multiple scattering
//     (several Draine-lobe scatterings toward grazing exit on the Sun's far side): oblique light reflected forward is
//     too dark. Follow-up if that matters: a Monte Carlo transfer table (Bouthors et al. 2008).
//   - semi-infinite conservative slab (true R = 1): 0.976 (mu0 1), 0.901 (mu0 0.5), 1.00004 (mu0 0.2): the source is
//     not strictly conservative, it loses up to 10 % and creates < 0.1 %.
//   - near the forward direction TMS overestimates: the narrow forward peak of P (Henyey-Greenstein lobe, g 0.995) is
//     attenuated on the scaled depths, so the aureole around the Sun seen through a thick cloud is too bright
//     (Nakajima & Tanaka's IMS correction is not modelled). At thin cloud edges (tau << 1) the TMS term is the exact
//     single scattering (silver lining).

#include "starsim/environment/weather_snapshot.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

// Extinction coefficient [1/m] of liquid and ice cloud water contents [kg/m^3], both phases summed.
double cloud_extinction_per_m(double liquid_water_kg_m3, double ice_water_kg_m3);

// Parameters of the Jendersie & d'Eon 2023 phase function (header) for a droplet diameter [m], clamped to 5-50 um.
struct CloudPhaseParameters {
    double hg_g = 0.0, draine_g = 0.0, draine_alpha = 0.0, draine_weight = 0.0;
};
CloudPhaseParameters cloud_phase_parameters(double droplet_diameter_m);

// Phase function [1/sr] of the cloud droplets (diameter 2 r_l), normalised over the sphere.
double cloud_phase_function(double cos_theta);
// Its asymmetry parameter g (mean cosine), closed form.
double cloud_asymmetry_g();

// Delta-Eddington two-stream slab, conservative, black surface below (header formulas).
double two_stream_reflectance(double optical_depth, double asymmetry_g);              // diffuse incidence
double two_stream_diffuse_transmittance(double optical_depth, double asymmetry_g);    // diffuse incidence
double two_stream_beam_reflectance(double optical_depth, double asymmetry_g, double sun_cos_zenith);

// The shader's scattering source (header, TMS): J per unit Sun irradiance and unit (unscaled) optical depth along the
// view ray; the view ray attenuates it with cloud_transport_scale() x its optical depth. view_up_cos is u_z of the
// view-ray propagation direction (toward the camera); depths are unscaled optical depths.
double cloud_transport_scale();  // 1 - f, the scale of every optical depth in the source and along the view ray
double cloud_scattering_source(double cos_theta, double view_up_cos, double sun_cos_zenith, double sun_optical_depth,
                               double depth_from_top, double column_optical_depth);
// Reflected radiance [1/sr per unit Sun irradiance] out of the top of a plane-parallel slab (black surface): the source
// integrated in closed form. relative_azimuth_rad is the angle between the horizontal propagation directions of the
// sunlight and the reflected light (0 = forward side, camera opposite the Sun). 0 for the Sun below the horizon.
double slab_reflected_radiance(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                               double relative_azimuth_rad);
// The model's plane-parallel slab reflectance (albedo) for the Sun at sun_cos_zenith; 0 for the Sun below the horizon.
double slab_reflectance(double optical_depth, double sun_cos_zenith);

// The CPU light. Transmittance of the direct Sun through the grid-mean clouds along the ray from position toward the
// Sun (Beer-Lambert with the unscaled extinction: the Sun disc). No detail noise: a vehicle in a gap between cumuli gets
// its cell's mean. The ray ends where it leaves the cloud volume (above 20 km and climbing) or where it enters the solid
// Earth (2 km below the ellipsoid: the Sun is below the horizon there, the Earth's shadow is not a cloud matter).
double cloud_sun_transmittance(const WeatherSnapshot& snapshot, const math::Vec3& position_itrf_m,
                               const math::Vec3& sun_direction_itrf);
// Diffuse sky factor under the column: the two-stream diffuse transmittance of the column above the position (vertical,
// from the position to 20 km), 1 above the clouds.
double cloud_sky_factor(const WeatherSnapshot& snapshot, const math::Vec3& position_itrf_m);

// Vertical optical depth of the column above a geodetic position (exact for the snapshot's piecewise-linear profile).
double cloud_column_optical_depth_above(const WeatherSnapshot& snapshot, double latitude_rad, double longitude_rad,
                                        double height_ellipsoid_m);

}  // namespace starsim::environment
