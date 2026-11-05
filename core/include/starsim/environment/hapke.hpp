#pragma once

#include <functional>

#include "starsim/config/moon_surface_config.hpp"

namespace starsim::environment {

// Hapke (2012) reflectance of a particulate surface (the lunar regolith; docs/subsystems/moon-surface.md, Hapke): the
// isotropic multiple-scattering approximation with porosity K, the shadow-hiding (SHOE) and coherent-backscatter
// (CBOE) opposition effects, the double Henyey-Greenstein particle phase function and Hapke's (1984) macroscopic
// roughness. Hapke, Theory of Reflectance and Emittance Spectroscopy, 2nd ed. 2012 (ch. 9 opposition effects, ch. 12
// roughness after Hapke 1984, Icarus 59, 41); H function after Hapke 2002 (Icarus 157, 523). The form Sato et al.
// (2014) fitted to LROC WAC data; the reference for the game's Moon shader (game/shaders/moon_hapke.gdshaderinc).
//
// r = K w/4 * mu0e/(mu0e + mue) * [p(g) (1 + B_S0 B_S(g)) + H(mu0e/K) H(mue/K) - 1] * (1 + B_C0 B_C(g)) * S(i, e, g)
// as the radiance factor I/F (pi x Hapke's bidirectional reflectance; a white Lambertian has I/F = mu0).

// Approximations of Chandrasekhar's H function for isotropic scatterers.
enum class HapkeHFunction {
    hapke_2002,  // Hapke 2002 (Icarus 157, 523): < 1 % from the exact H for all w (the default; Hapke 2012's form)
    hapke_1981,  // Hapke 1981: (1 + 2x) / (1 + 2 gamma x), ~4 % (USGS ISIS's photometric Hapke models use it)
};

// H(x) for single-scattering albedo w (0 <= w < 1), x >= 0 (x <= 0 gives 1).
double hapke_h_function(double x, double w, HapkeHFunction form);

// Double Henyey-Greenstein particle phase function at phase angle g (c > 0: backscattering, config/moon_surface.toml):
// p(g) = (1+c)/2 (1-b^2)/(1-2b cos g+b^2)^1.5 + (1-c)/2 (1-b^2)/(1+2b cos g+b^2)^1.5.
double hapke_phase_function(double b, double c, double phase_rad);

// Effective cosines and shadowing of Hapke's rough surface (mean slope angle theta_bar; theta_bar <= 0: smooth, S = 1).
struct HapkeRoughness {
    double mu0_effective = 0.0, mu_effective = 0.0, shadowing = 1.0;
};
HapkeRoughness hapke_roughness(double mu0, double mu, double phase_rad, double theta_bar_rad);

// The radiance factor I/F for cos incidence mu0, cos emission mu and phase angle g (rad) with the mean slope
// theta_bar (scale-dependent: the roughness not drawn as geometry; p.theta_bar_rad is the fit's sub-pixel value).
// 0 for mu0 <= 0 or mu <= 0. Pure function, thread-safe.
double hapke_reflectance(const config::HapkeParameters& p, double mu0, double mu, double phase_rad,
                         double theta_bar_rad, HapkeHFunction form = HapkeHFunction::hapke_2002);

// Disc-integrated reflectance of a uniform Hapke sphere at phase angle g: the mean I/F over its projected disc, the
// "disk-equivalent reflectance" A of Kieffer & Stone (2005, ROLO): the body's irradiance = A x its solid angle x the
// solar irradiance / pi. Gauss-Legendre over the lit and visible lune in photometric coordinates (smooth integrand,
// converged to ~1e-6 at the default order). 0 for g >= pi.
double hapke_disc_reflectance(const config::HapkeParameters& p, double phase_rad, double theta_bar_rad,
                              int quadrature_order = 48);

// The same disc mean for any radiance factor I/F(mu0, mu) at phase angle g (e.g. a Lommel-Seeliger reference).
double disc_integrated_reflectance(const std::function<double(double mu0, double mu)>& radiance_factor,
                                   double phase_rad, int quadrature_order = 48);

}  // namespace starsim::environment
