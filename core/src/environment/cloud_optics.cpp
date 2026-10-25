// Cloud optics and the CPU cloud light (M8 weather and clouds); formulas and citations in cloud_optics.hpp.

#include "starsim/environment/cloud_optics.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <thread>
#include <vector>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

namespace c = constants;

constexpr double four_pi = 4.0 * c::pi;
// The entry-surface cosine (cloud_optics.hpp, "3D Sun light"): regulariser of the ratio at the cloud surface [E: one
// unscaled optical depth = one mean free path] and the smallest cosine (no division by ~0 in the slab solution).
constexpr double entry_regulariser = 1.0;
constexpr double entry_cosine_min = 0.02;

// Fit of Jendersie & d'Eon 2023 (cloud_optics.hpp), droplet diameter in um, valid 5-50 um.
constexpr double fit_min_diameter_um = 5.0, fit_max_diameter_um = 50.0;

double henyey_greenstein(double mu, double g) {
    const double denom = 1.0 + g * g - 2.0 * g * mu;
    return (1.0 - g * g) / (four_pi * denom * std::sqrt(denom));
}

double draine(double mu, double g, double alpha) {
    return henyey_greenstein(mu, g) * (1.0 + alpha * mu * mu) / (1.0 + alpha * (1.0 + 2.0 * g * g) / 3.0);
}

const CloudPhaseParameters& droplet_phase() {
    static const CloudPhaseParameters p = cloud_phase_parameters(2.0 * c::cloud_droplet_effective_radius_m);
    return p;
}

// Delta-Eddington constants of the cloud phase function (conservative scattering).
struct DeltaEddington {
    double f, g_scaled, gamma1;
    explicit DeltaEddington(double g) : f(g * g), g_scaled(g / (1.0 + g)), gamma1(0.75 * (1.0 - g / (1.0 + g))) {}
    double gamma3(double mu0) const { return (2.0 - 3.0 * g_scaled * mu0) / 4.0; }
    // Beam reflectance of a slab of scaled optical depth tau_scaled (Meador & Weaver 1980, omega = 1).
    double beam_reflectance_scaled(double tau_scaled, double mu0) const {
        return (gamma1 * tau_scaled + (gamma3(mu0) - gamma1 * mu0) * (1.0 - std::exp(-tau_scaled / mu0))) /
               (1.0 + gamma1 * tau_scaled);
    }
};

const DeltaEddington& cloud_delta_eddington() {
    static const DeltaEddington d(cloud_asymmetry_g());
    return d;
}

// Diffuse field of the local column per unit Sun irradiance: Sigma = A0 + A1 t' + A2 exp(-t'/mu0) (F_up + F_down),
// D = B0 + B1 exp(-t'/mu0) (F_up - F_down), t' the scaled depth below the top.
struct DiffuseFieldCoefficients {
    double a0 = 0.0, a1 = 0.0, a2 = 0.0, b0 = 0.0, b1 = 0.0;
};

DiffuseFieldCoefficients diffuse_field(const DeltaEddington& de, double column_scaled, double mu0) {
    const double r = de.beam_reflectance_scaled(column_scaled, mu0);
    const double g1 = de.gamma1, g3 = de.gamma3(mu0);
    DiffuseFieldCoefficients k;
    k.a0 = mu0 * (r + 2.0 * g1 * mu0 + 1.0 - 2.0 * g3);
    k.a1 = 2.0 * g1 * mu0 * (r - 1.0);
    k.a2 = -mu0 * (2.0 * g1 * mu0 + 1.0 - 2.0 * g3);
    k.b0 = mu0 * (r - 1.0);
    k.b1 = mu0;
    return k;
}

// Gauss-Legendre nodes / weights on [0, 1].
struct Quadrature {
    std::vector<double> x, w;
};

Quadrature gauss_legendre_unit(int n) {
    Quadrature q;
    q.x.resize(static_cast<std::size_t>(n));
    q.w.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        double z = std::cos(c::pi * (i + 0.75) / (n + 0.5)), dp = 0.0;
        for (int it = 0; it < 100; ++it) {
            double p0 = 1.0, p1 = 0.0;
            for (int j = 1; j <= n; ++j) {
                const double p2 = p1;
                p1 = p0;
                p0 = ((2.0 * j - 1.0) * z * p1 - (j - 1.0) * p2) / j;
            }
            dp = n * (z * p0 - p1) / (z * z - 1.0);
            const double dz = p0 / dp;
            z -= dz;
            if (std::abs(dz) < 1e-15) break;
        }
        q.x[static_cast<std::size_t>(i)] = 0.5 * (1.0 - z);
        q.w[static_cast<std::size_t>(i)] = 1.0 / ((1.0 - z * z) * dp * dp);  // 2 / ((1 - z^2) P'^2) on [-1, 1], halved
    }
    return q;
}

constexpr int slab_mu_nodes = 64, slab_phi_nodes = 64;

// Integration from the ray start toward the Sun: steps of <= 25 m height change and <= 1 km length, midpoint rule
// (the snapshot is linear between layer centres; the vertical rule keeps several samples per 250 m layer).
constexpr double ray_vertical_step_m = 25.0;
constexpr double ray_max_step_m = 1000.0;
constexpr double solid_earth_height_m = -2000.0;
constexpr int ray_max_steps = 200000;

math::Vec3 geodetic_up(double latitude_rad, double longitude_rad) {
    return {std::cos(latitude_rad) * std::cos(longitude_rad), std::cos(latitude_rad) * std::sin(longitude_rad),
            std::sin(latitude_rad)};
}

double extinction_at(const WeatherSnapshot& snapshot, double latitude_rad, double longitude_rad, double height_m) {
    const WeatherCell cell = snapshot.at(latitude_rad, longitude_rad, height_m);
    return cloud_extinction_per_m(cell.liquid_water_kg_m3, cell.ice_water_kg_m3);
}

// Grid-mean extinction and cloud fraction at a point.
CloudColumnDepths extinctions_at(const WeatherSnapshot& snapshot, double latitude_rad, double longitude_rad,
                                 double height_m) {
    const WeatherCell cell = snapshot.at(latitude_rad, longitude_rad, height_m);
    return {cloud_extinction_per_m(cell.liquid_water_kg_m3, cell.ice_water_kg_m3), cell.cloud_fraction};
}

}  // namespace

double cloud_extinction_per_m(double liquid_water_kg_m3, double ice_water_kg_m3) {
    return 3.0 * liquid_water_kg_m3 / (2.0 * c::liquid_water_density_kg_m3 * c::cloud_droplet_effective_radius_m) +
           3.0 * ice_water_kg_m3 / (2.0 * c::ice_density_kg_m3 * c::cloud_ice_effective_radius_m);
}

CloudPhaseParameters cloud_phase_parameters(double droplet_diameter_m) {
    const double d = std::clamp(droplet_diameter_m * 1e6, fit_min_diameter_um, fit_max_diameter_um);
    CloudPhaseParameters p;
    p.hg_g = std::exp(-0.0990567 / (d - 1.67154));
    p.draine_g = std::exp(-2.20679 / (d + 3.91029) - 0.428934);
    p.draine_alpha = std::exp(3.62489 - 8.29288 / (d + 5.52825));
    p.draine_weight = std::exp(-0.599085 / (d - 0.641583) - 0.665888);
    return p;
}

double cloud_phase_function(double cos_theta) {
    const CloudPhaseParameters& p = droplet_phase();
    const double mu = std::clamp(cos_theta, -1.0, 1.0);
    return (1.0 - p.draine_weight) * henyey_greenstein(mu, p.hg_g) +
           p.draine_weight * draine(mu, p.draine_g, p.draine_alpha);
}

double cloud_asymmetry_g() {
    const CloudPhaseParameters& p = droplet_phase();
    const double g2 = p.draine_g * p.draine_g;
    const double draine_mean = p.draine_g * (1.0 + p.draine_alpha * (3.0 + 2.0 * g2) / 5.0) /
                               (1.0 + p.draine_alpha * (1.0 + 2.0 * g2) / 3.0);
    return (1.0 - p.draine_weight) * p.hg_g + p.draine_weight * draine_mean;
}

double two_stream_reflectance(double optical_depth, double asymmetry_g) {
    const double x = 0.75 * (1.0 - asymmetry_g) * std::max(0.0, optical_depth);  // gamma1 tau'
    return x / (1.0 + x);
}

double two_stream_diffuse_transmittance(double optical_depth, double asymmetry_g) {
    return 1.0 / (1.0 + 0.75 * (1.0 - asymmetry_g) * std::max(0.0, optical_depth));
}

double two_stream_beam_reflectance(double optical_depth, double asymmetry_g, double sun_cos_zenith) {
    if (sun_cos_zenith <= 0.0) return 0.0;
    const DeltaEddington de(asymmetry_g);
    return de.beam_reflectance_scaled((1.0 - de.f) * std::max(0.0, optical_depth), std::min(1.0, sun_cos_zenith));
}

double cloud_transport_scale() { return 1.0 - cloud_delta_eddington().f; }

double cloud_scattering_source(double cos_theta, double view_up_cos, double sun_cos_zenith, double sun_optical_depth,
                               double depth_from_top, double column_optical_depth) {
    if (sun_cos_zenith <= 0.0) return 0.0;
    const DeltaEddington& de = cloud_delta_eddington();
    const double k = 1.0 - de.f, mu0 = std::min(1.0, sun_cos_zenith);
    const double column = k * std::max(0.0, column_optical_depth);
    const double t = std::clamp(k * depth_from_top, 0.0, column);
    const DiffuseFieldCoefficients f = diffuse_field(de, column, mu0);
    const double e = std::exp(-t / mu0);
    const double sigma = f.a0 + f.a1 * t + f.a2 * e, net = f.b0 + f.b1 * e;
    const double i0 = sigma / c::two_pi, i1 = 3.0 * net / four_pi;
    return k * (i0 + de.g_scaled * view_up_cos * i1) +
           cloud_phase_function(cos_theta) * std::exp(-k * std::max(0.0, sun_optical_depth));
}

double cloud_sky_scattering_source(double view_up_cos, double depth_from_top, double column_optical_depth) {
    const DeltaEddington& de = cloud_delta_eddington();
    const double k = 1.0 - de.f;
    const double column = k * std::max(0.0, column_optical_depth);
    const double t = std::clamp(k * depth_from_top, 0.0, column);
    const double transmittance = 1.0 / (1.0 + de.gamma1 * column);
    const double sigma = 2.0 - transmittance - 2.0 * de.gamma1 * transmittance * t;  // 1 + R - 2 gamma1 T t'
    const double i0 = sigma / c::two_pi, i1 = 3.0 * -transmittance / four_pi;
    return k * (i0 + de.g_scaled * view_up_cos * i1);
}

double cloud_entry_cosine(double sun_cos_zenith, double sun_optical_depth, double vertical_depth_from_top) {
    const double mu0 = std::clamp(sun_cos_zenith, -1.0, 1.0);
    const double ratio = (std::max(0.0, vertical_depth_from_top) + entry_regulariser * mu0) /
                         (std::max(0.0, sun_optical_depth) + entry_regulariser);
    return std::clamp(ratio, std::max(mu0, entry_cosine_min), 1.0);
}

namespace {

// The entry surface's normal (cloud_optics.hpp "3D Sun light"): up turned toward the Sun by acos(mu0) - acos(mu_e)
// in the plane of up and the Sun.
math::Vec3 entry_normal(const math::Vec3& sun_direction, const math::Vec3& up, double mu0, double mu_e) {
    const math::Vec3 across = sun_direction - up * mu0;
    const double across_norm = math::norm(across);
    if (across_norm <= 1e-9) return up;
    const double turn = std::acos(mu0) - std::acos(mu_e);
    return up * std::cos(turn) + across * (std::sin(turn) / across_norm);
}

}  // namespace

double cloud_scattering_source_3d(const math::Vec3& sun_direction, const math::Vec3& view_propagation,
                                  const math::Vec3& up, double sun_optical_depth, double vertical_depth_from_top,
                                  double column_optical_depth) {
    const double mu0 = std::clamp(math::dot(sun_direction, up), -1.0, 1.0);
    const double mu_e = cloud_entry_cosine(mu0, sun_optical_depth, vertical_depth_from_top);
    const math::Vec3 normal = entry_normal(sun_direction, up, mu0, mu_e);
    const double tau_s = std::max(0.0, sun_optical_depth);
    const double depth = mu_e * tau_s;
    return cloud_scattering_source(-math::dot(sun_direction, view_propagation), math::dot(view_propagation, normal),
                                   mu_e, tau_s, depth, std::max(column_optical_depth, depth));
}

CloudSunFrameAngles cloud_sun_frame_angles(const math::Vec3& sun_propagation, const math::Vec3& view_propagation,
                                           const math::Vec3& axis) {
    CloudSunFrameAngles out;
    const math::Vec3 chord = view_propagation - sun_propagation;  // |chord| = 2 sin(theta / 2): exact near 0
    out.theta_rad = 2.0 * std::asin(std::min(1.0, 0.5 * math::norm(chord)));
    const math::Vec3 e1_raw = axis - sun_propagation * math::dot(axis, sun_propagation);
    const double e1_norm = math::norm(e1_raw);
    if (e1_norm <= 1e-12) return out;  // the Sun along the axis: psi has no meaning (the field is symmetric)
    const math::Vec3 e1 = e1_raw * (1.0 / e1_norm);
    const math::Vec3 normal_part = view_propagation - sun_propagation * math::dot(view_propagation, sun_propagation);
    const double c1 = math::dot(normal_part, e1);
    out.psi_rad = std::atan2(math::norm(normal_part - e1 * c1), c1);
    return out;
}

double cloud_transfer_view_extinction(const CloudTransferTable& table) { return table.draine_weight; }

double cloud_scattering_source_mc(const CloudTransferTable& table, const math::Vec3& sun_direction,
                                  const math::Vec3& view_propagation, const math::Vec3& up, double sun_optical_depth,
                                  double vertical_depth_from_top, double column_optical_depth) {
    const double mu0 = std::clamp(math::dot(sun_direction, up), -1.0, 1.0);
    const double mu_e = cloud_entry_cosine(mu0, sun_optical_depth, vertical_depth_from_top);
    const math::Vec3 normal = entry_normal(sun_direction, up, mu0, mu_e);
    const double tau_s = std::max(0.0, sun_optical_depth);
    const double depth = mu_e * tau_s;
    const double column = std::max(column_optical_depth, depth);
    const math::Vec3 sun_propagation = sun_direction * -1.0;
    const CloudSunFrameAngles angles = cloud_sun_frame_angles(sun_propagation, view_propagation, normal);
    const math::Vec3 chord = view_propagation - sun_propagation;
    const double one_minus_cos = 0.5 * math::dot(chord, chord);
    return cloud_phase_function(1.0 - one_minus_cos) * std::exp(-tau_s) +
           cloud_transfer_lookup(table, depth, column - depth, mu_e, angles.theta_rad, angles.psi_rad);
}

double slab_reflected_radiance_mc(const CloudTransferTable& table, double optical_depth, double sun_cos_zenith,
                                  double view_cos_zenith, double relative_azimuth_rad) {
    if (sun_cos_zenith <= 0.0 || view_cos_zenith <= 0.0 || optical_depth <= 0.0) return 0.0;
    const double mu0 = std::min(1.0, sun_cos_zenith), mu = std::min(1.0, view_cos_zenith);
    const double s0 = std::sqrt(1.0 - mu0 * mu0), sv = std::sqrt(1.0 - mu * mu);
    const math::Vec3 up{0.0, 0.0, 1.0}, sun_propagation{s0, 0.0, -mu0};
    const math::Vec3 view{sv * std::cos(relative_azimuth_rad), sv * std::sin(relative_azimuth_rad), mu};
    const CloudSunFrameAngles angles = cloud_sun_frame_angles(sun_propagation, view, up);
    const math::Vec3 chord = view - sun_propagation;
    const double one_minus_cos = 0.5 * math::dot(chord, chord);
    // The ray's attenuation exp(-w t / mu) (cloud_transfer_view_extinction).
    const double w = cloud_transfer_view_extinction(table);
    const double ray_scale = w / mu;
    // Single scattering: p exp(-t / mu0) along the ray, closed form.
    const double single = cloud_phase_function(1.0 - one_minus_cos) / (mu * (1.0 / mu0 + ray_scale)) *
                          (1.0 - (std::isinf(optical_depth) ? 0.0 : std::exp(-optical_depth * (1.0 / mu0 + ray_scale))));
    // Orders >= 2: panels between the table's depth nodes (from the top and from the bottom), at most 2 mu / w long,
    // 4-point Gauss-Legendre each; beyond 40 mu / w the ray's weight exp(-w t / mu) is < 1e-17.
    static constexpr double gx[4] = {0.0694318442029737, 0.3300094782075719, 0.6699905217924281, 0.9305681557970263};
    static constexpr double gw[4] = {0.1739274225687269, 0.3260725774312731, 0.3260725774312731, 0.1739274225687269};
    const double end = std::min(optical_depth, 40.0 / ray_scale);
    std::vector<double> breaks{0.0, end};
    for (int i = 1; i < table.depth_nodes - 1; ++i) {
        const double node = table.depth_scale * (std::pow(table.depth_ratio, i) - 1.0);
        if (node < end) breaks.push_back(node);
        if (!std::isinf(optical_depth) && optical_depth - node > 0.0 && optical_depth - node < end) {
            breaks.push_back(optical_depth - node);
        }
    }
    std::sort(breaks.begin(), breaks.end());
    double multiple = 0.0;
    for (std::size_t k = 0; k + 1 < breaks.size(); ++k) {
        const double lo = breaks[k], length = breaks[k + 1] - lo;
        if (length <= 0.0) continue;
        const int pieces = std::max(1, static_cast<int>(std::ceil(length * ray_scale / 2.0)));
        const double h = length / pieces;
        for (int piece = 0; piece < pieces; ++piece) {
            for (int q = 0; q < 4; ++q) {
                const double t = lo + (piece + gx[q]) * h;
                multiple += gw[q] * h / mu * std::exp(-t * ray_scale) *
                            cloud_transfer_lookup(table, t, optical_depth - t, mu0, angles.theta_rad, angles.psi_rad);
            }
        }
    }
    return single + multiple;
}

double slab_reflectance_mc(const CloudTransferTable& table, double optical_depth, double sun_cos_zenith) {
    if (sun_cos_zenith <= 0.0 || optical_depth <= 0.0) return 0.0;
    static const Quadrature mu_q = gauss_legendre_unit(slab_mu_nodes), phi_q = gauss_legendre_unit(slab_phi_nodes);
    const double mu0 = std::min(1.0, sun_cos_zenith);
    double flux = 0.0;
    for (std::size_t i = 0; i < mu_q.x.size(); ++i) {
        double ring = 0.0;
        for (std::size_t j = 0; j < phi_q.x.size(); ++j) {
            ring += phi_q.w[j] *
                    slab_reflected_radiance_mc(table, optical_depth, mu0, mu_q.x[i], c::pi * phi_q.x[j]);
        }
        flux += mu_q.w[i] * mu_q.x[i] * c::two_pi * ring;
    }
    return flux / mu0;
}

double slab_total_transmittance_mc(const CloudTransferTable& table, double optical_depth, double sun_cos_zenith,
                                   int mu_nodes, int phi_nodes) {
    if (!(optical_depth > 0.0)) return 1.0;
    if (sun_cos_zenith <= 0.0) return 0.0;
    const Quadrature mu_q = gauss_legendre_unit(mu_nodes), phi_q = gauss_legendre_unit(phi_nodes);
    const double mu0 = std::min(1.0, sun_cos_zenith);
    double flux = 0.0;
    for (std::size_t i = 0; i < mu_q.x.size(); ++i) {
        double ring = 0.0;
        for (std::size_t j = 0; j < phi_q.x.size(); ++j) {
            ring += phi_q.w[j] * slab_reflected_radiance_mc(table, optical_depth, mu0, mu_q.x[i], c::pi * phi_q.x[j]);
        }
        flux += mu_q.w[i] * mu_q.x[i] * c::two_pi * ring;
    }
    return std::clamp(1.0 - flux / mu0, 0.0, 1.0);
}

double two_stream_beam_transmittance(double optical_depth, double asymmetry_g, double sun_cos_zenith) {
    if (!(optical_depth > 0.0)) return 1.0;
    if (sun_cos_zenith <= 0.0) return 0.0;
    return std::clamp(1.0 - two_stream_beam_reflectance(optical_depth, asymmetry_g, sun_cos_zenith), 0.0, 1.0);
}

namespace {

double lut_depth(const CloudSlabTransmittanceLut& lut, int i) {
    return CloudSlabTransmittanceLut::depth_scale * (std::pow(lut.depth_ratio, i) - 1.0);
}

double lut_cosine(int j) {
    using L = CloudSlabTransmittanceLut;
    return L::cosine_min + (1.0 - L::cosine_min) * j / (L::cosine_nodes - 1);
}

// The diffusion asymptote's coefficient k = 3 (1 - g) / 4 (two_stream_diffuse_transmittance).
double diffusion_coefficient() { return 0.75 * (1.0 - cloud_asymmetry_g()); }

}  // namespace

CloudSlabTransmittanceLut make_cloud_slab_transmittance_lut(const CloudTransferTable& table) {
    using L = CloudSlabTransmittanceLut;
    CloudSlabTransmittanceLut lut;
    lut.depth_ratio = std::pow(L::depth_max / L::depth_scale + 1.0, 1.0 / (L::depth_nodes - 1));
    lut.total.assign(static_cast<std::size_t>(L::depth_nodes) * L::cosine_nodes, 1.0);
    // One cosine row per task (~0.35 s each), rows independent.
    std::atomic<int> next{0};
    auto work = [&]() {
        for (int j = next++; j < L::cosine_nodes; j = next++) {
            double previous = 1.0;
            for (int i = 1; i < L::depth_nodes; ++i) {
                // Monotone in tau (the table's Monte Carlo noise must not let a thicker slab pass more light).
                previous = std::min(previous, slab_total_transmittance_mc(table, lut_depth(lut, i), lut_cosine(j)));
                lut.total[static_cast<std::size_t>(j) * L::depth_nodes + i] = previous;
            }
        }
    };
    const int threads = std::clamp(static_cast<int>(std::thread::hardware_concurrency()), 1, L::cosine_nodes);
    std::vector<std::thread> pool;
    for (int t = 1; t < threads; ++t) pool.emplace_back(work);
    work();
    for (std::thread& t : pool) t.join();
    return lut;
}

double cloud_slab_total_transmittance(const CloudSlabTransmittanceLut& lut, double optical_depth,
                                      double sun_cos_zenith) {
    using L = CloudSlabTransmittanceLut;
    if (!(optical_depth > 0.0)) return 1.0;
    if (sun_cos_zenith <= 0.0 || lut.total.empty()) return 0.0;
    const double tau = std::min(optical_depth, L::depth_max);
    const double x = std::clamp(std::log(tau / L::depth_scale + 1.0) / std::log(lut.depth_ratio), 0.0,
                                L::depth_nodes - 1.0);
    const double y = std::clamp((sun_cos_zenith - L::cosine_min) / (1.0 - L::cosine_min) * (L::cosine_nodes - 1), 0.0,
                                L::cosine_nodes - 1.0);
    const int i = std::min(static_cast<int>(x), L::depth_nodes - 2);
    const int j = std::min(static_cast<int>(y), L::cosine_nodes - 2);
    const double fx = x - i, fy = y - j;
    auto at = [&](int jj, int ii) { return lut.total[static_cast<std::size_t>(jj) * L::depth_nodes + ii]; };
    const double t = (1.0 - fy) * ((1.0 - fx) * at(j, i) + fx * at(j, i + 1)) +
                     fy * ((1.0 - fx) * at(j + 1, i) + fx * at(j + 1, i + 1));
    if (optical_depth <= L::depth_max) return t;
    const double k = diffusion_coefficient();
    return t * (1.0 + k * L::depth_max) / (1.0 + k * optical_depth);
}

double cloud_light_march_ratio(const CloudLightMarch& march) {
    const int n = std::max(march.near_samples, 1);
    const double target = march.near_length_m / std::max(march.first_m, 1e-9);  // (r^n - 1) / (r - 1)
    if (target <= n) return 1.0;
    double lo = 1.0, hi = 2.0;
    auto sum = [n](double r) { return (std::pow(r, n) - 1.0) / (r - 1.0); };
    while (sum(hi) < target) hi *= 2.0;
    for (int i = 0; i < 100; ++i) {
        const double mid = 0.5 * (lo + hi);
        (sum(mid) < target ? lo : hi) = mid;  // mid > 1: lo starts at 1, hi >= 2
    }
    return 0.5 * (lo + hi);
}

double cloud_light_optical_depth(const std::function<double(double)>& near_extinction,
                                 const std::function<double(double)>& far_extinction, const CloudLightMarch& march,
                                 double jitter) {
    const double ratio = cloud_light_march_ratio(march);
    const int n = std::max(march.near_samples, 1);
    // First interval so that the n geometric intervals end exactly at near_length.
    const double first = ratio == 1.0 ? march.near_length_m / n
                                      : march.near_length_m * (ratio - 1.0) / (std::pow(ratio, n) - 1.0);
    double tau = 0.0, start = 0.0, length = first;
    for (int i = 0; i < n; ++i) {
        tau += near_extinction(start + jitter * length) * length;
        start += length;
        length *= ratio;
    }
    if (march.far_samples > 0 && march.far_length_m > march.near_length_m) {
        const double step = (march.far_length_m - march.near_length_m) / march.far_samples;
        for (int i = 0; i < march.far_samples; ++i) {
            tau += far_extinction(march.near_length_m + (i + jitter) * step) * step;
        }
    }
    return tau;
}

double slab_reflected_radiance(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                               double relative_azimuth_rad) {
    if (sun_cos_zenith <= 0.0 || view_cos_zenith <= 0.0 || optical_depth <= 0.0) return 0.0;
    const DeltaEddington& de = cloud_delta_eddington();
    const double k = 1.0 - de.f, mu0 = std::min(1.0, sun_cos_zenith), mu = std::min(1.0, view_cos_zenith);
    const double big_t = k * optical_depth;  // scaled column
    const DiffuseFieldCoefficients f = diffuse_field(de, big_t, mu0);
    // Integrals over the scaled depth [0, big_t] of exp(-t/mu) dt / mu times 1, t and exp(-t/mu0).
    const double view_t = std::exp(-big_t / mu);
    const double k0 = 1.0 - view_t, k1 = mu * (1.0 - view_t) - big_t * view_t;
    const double ke = mu0 / (mu0 + mu) * (1.0 - std::exp(-big_t * (1.0 / mu0 + 1.0 / mu)));
    const double cos_theta =
        std::sqrt(1.0 - mu0 * mu0) * std::sqrt(1.0 - mu * mu) * std::cos(relative_azimuth_rad) - mu0 * mu;
    const double diffuse =
        (f.a0 * k0 + f.a1 * k1 + f.a2 * ke) / c::two_pi + 3.0 * de.g_scaled * mu / four_pi * (f.b0 * k0 + f.b1 * ke);
    return diffuse + cloud_phase_function(cos_theta) / k * ke;
}

double slab_reflectance(double optical_depth, double sun_cos_zenith) {
    if (sun_cos_zenith <= 0.0 || optical_depth <= 0.0) return 0.0;
    static const Quadrature mu_q = gauss_legendre_unit(slab_mu_nodes), phi_q = gauss_legendre_unit(slab_phi_nodes);
    const double mu0 = std::min(1.0, sun_cos_zenith);
    double flux = 0.0;
    for (std::size_t i = 0; i < mu_q.x.size(); ++i) {
        double ring = 0.0;  // integral over the azimuth, 0..pi doubled (symmetric about the Sun's plane)
        for (std::size_t j = 0; j < phi_q.x.size(); ++j) {
            ring += phi_q.w[j] * slab_reflected_radiance(optical_depth, mu0, mu_q.x[i], c::pi * phi_q.x[j]);
        }
        flux += mu_q.w[i] * mu_q.x[i] * c::two_pi * ring;
    }
    return flux / mu0;
}

double CloudColumnDepths::cover() const {
    if (!(mean > 0.0)) return 0.0;
    return std::clamp(max_fraction, cloud_min_cover, 1.0);
}

double CloudColumnDepths::in_cloud_depth() const { return mean > 0.0 ? mean / cover() : 0.0; }

double cloud_cover_transmittance(double cover, double in_cloud_transmittance) {
    return 1.0 - cover + cover * in_cloud_transmittance;
}

CloudColumnDepths cloud_column_depths_above(const WeatherSnapshot& snapshot, double latitude_rad,
                                            double longitude_rad, double height_ellipsoid_m) {
    // The profile is linear in height between the breakpoints (layer bottoms and centres): trapezoids are exact for the
    // mean, and the cloud fraction's maximum lies on a breakpoint.
    CloudColumnDepths depth;
    const double start = std::max(height_ellipsoid_m, snapshot.ground_height_m(latitude_rad, longitude_rad));
    if (start >= weather_top_m) return depth;
    double lower = start;
    CloudColumnDepths lower_beta = extinctions_at(snapshot, latitude_rad, longitude_rad, lower);
    depth.max_fraction = lower_beta.max_fraction;
    for (int layer = 0; layer < weather_layer_count; ++layer) {
        const double bottom = weather_layer_bottom_m(layer), top = weather_layer_bottom_m(layer + 1);
        for (double upper : {0.5 * (bottom + top), top}) {
            if (upper <= lower) continue;
            const CloudColumnDepths upper_beta = extinctions_at(snapshot, latitude_rad, longitude_rad, upper);
            depth.mean += 0.5 * (lower_beta.mean + upper_beta.mean) * (upper - lower);
            depth.max_fraction = std::max(depth.max_fraction, upper_beta.max_fraction);
            lower = upper;
            lower_beta = upper_beta;
        }
    }
    return depth;
}

double cloud_column_optical_depth_above(const WeatherSnapshot& snapshot, double latitude_rad, double longitude_rad,
                                        double height_ellipsoid_m) {
    return cloud_column_depths_above(snapshot, latitude_rad, longitude_rad, height_ellipsoid_m).mean;
}

double cloud_sun_transmittance(const WeatherSnapshot& snapshot, const math::Vec3& position_itrf_m,
                               const math::Vec3& sun_direction_itrf) {
    const math::Vec3 dir = math::normalized(sun_direction_itrf);
    math::Vec3 p = position_itrf_m;
    CloudColumnDepths tau;
    for (int step = 0; step < ray_max_steps; ++step) {
        const frames::GeodeticPosition here = frames::fixed_to_geodetic(p);
        if (here.height_m < solid_earth_height_m) break;
        const double climb = math::dot(dir, geodetic_up(here.latitude_rad, here.longitude_rad));
        // The height along a straight line is convex (distance to a convex surface): climbing above the top, it stays.
        if (here.height_m >= weather_top_m && climb >= 0.0) break;
        double length = std::clamp(ray_vertical_step_m / std::max(std::abs(climb), 1e-9), ray_vertical_step_m,
                                   ray_max_step_m);
        if (here.height_m > weather_top_m) length = std::max(length, here.height_m - weather_top_m);
        const frames::GeodeticPosition mid = frames::fixed_to_geodetic(p + dir * (0.5 * length));
        const CloudColumnDepths beta = extinctions_at(snapshot, mid.latitude_rad, mid.longitude_rad, mid.height_m);
        tau.mean += beta.mean * length;
        tau.max_fraction = std::max(tau.max_fraction, beta.max_fraction);
        p += dir * length;
    }
    // Independent columns, maximum overlap: the clear part passes everything, the cloudy part exp(-in-cloud depth).
    return cloud_cover_transmittance(tau.cover(), std::exp(-tau.in_cloud_depth()));
}

double cloud_sky_factor(const WeatherSnapshot& snapshot, const math::Vec3& position_itrf_m) {
    const frames::GeodeticPosition here = frames::fixed_to_geodetic(position_itrf_m);
    const CloudColumnDepths tau =
        cloud_column_depths_above(snapshot, here.latitude_rad, here.longitude_rad, here.height_m);
    return cloud_cover_transmittance(tau.cover(),
                                     two_stream_diffuse_transmittance(tau.in_cloud_depth(), cloud_asymmetry_g()));
}

double cloud_sun_diffuse_transmittance(const WeatherSnapshot& snapshot, const CloudSlabTransmittanceLut* lut,
                                       const math::Vec3& position_itrf_m, const math::Vec3& sun_direction_itrf) {
    const frames::GeodeticPosition here = frames::fixed_to_geodetic(position_itrf_m);
    const double mu0 =
        math::dot(math::normalized(sun_direction_itrf), geodetic_up(here.latitude_rad, here.longitude_rad));
    if (mu0 <= 0.0) return 0.0;
    const CloudColumnDepths column =
        cloud_column_depths_above(snapshot, here.latitude_rad, here.longitude_rad, here.height_m);
    const double c = column.cover();
    const double tau = column.in_cloud_depth();
    if (!(tau > 0.0) || !(c > 0.0)) return 0.0;
    const double total = lut != nullptr ? cloud_slab_total_transmittance(*lut, tau, mu0)
                                        : two_stream_beam_transmittance(tau, cloud_asymmetry_g(), mu0);
    return c * std::max(total - std::exp(-tau / mu0), 0.0);
}

}  // namespace starsim::environment
