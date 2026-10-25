#pragma once

// Monte Carlo transfer table of the multiply scattered light in a cloud (M8 weather and clouds, Task 8b): the source
// function of the scattering orders >= 2 inside a plane-parallel, conservative slab (black surface below) lit by the
// Sun, after Bouthors, Neyret, Max, Bruneton & Crassin 2008 ("Interactive multiple anisotropic scattering in clouds",
// I3D): light transport in a cloud is reduced to that of the locally representative slab, precomputed by Monte Carlo
// with the droplet phase function. Here the tabulated quantity is the source J itself (what the cloud shader
// integrates along view rays), not their surface transfer functions: the view ray's integration stays the shader's.
//
// Generator: tools/validation/cloud_slab_monte_carlo.py --table-runs / --table-build (numpy, fixed seeds), the same
// phase function as cloud_optics.hpp (Jendersie & d'Eon 2023, HG + Draine, 20 um droplets, sampled exactly). Photons
// enter the slab of optical depth tau* at cos zenith mu0; every collision of a photon that has scattered before is
// tallied by depth bin and by its direction in the Sun's frame (128 x 12 bins); the source in a view direction is that
// histogram times the bin mass of the phase function's Draine part around the view direction (sampled exactly, 1e6
// directions per view node), per bin solid angle, times its weight w_D:
//   J(t, Omega) = mu0 / (N dt) sum over collisions w_D p_D(Omega_in . Omega)   [1/sr per unit Sun irradiance normal
//   to the beam and unit (unscaled) optical depth along the view ray].
// HG peak: the phase function is (1 - w_D) HG(g 0.995) + w_D Draine; the HG part is a ~0.3 deg forward peak. Its share
// of the multiple-scattering source, (1 - w_D) (HG * L)(Omega), is (1 - w_D) L(Omega) up to that width, and along the
// view ray dL/dtau = -L + (1 - w_D) L + rest = -w_D L + rest: the view ray integrates single scattering (exact, incl.
// its HG aureole, p(cos theta) exp(-tau_s)) plus this table with the attenuation exp(-w_D tau_v)
// (cloud_optics.hpp cloud_scattering_source_mc). Exact but for the peak's width; tallying the HG part itself would
// need the radiance within 0.3 deg of every view direction (25-50 % noise at 3.4e8 photons deep in thick slabs).
// Statistical error: 8 batches per run, 1-sigma of the mean per table entry (the sigma file).
//
// Parameterisation (the reductions, each documented):
//   - depth: the pair (a, b) = (optical depth below the column top, above the column bottom) along the vertical, so
//     the slab's tau* = a + b and both surfaces' boundary layers are resolved; nodes a_i = s (r^i - 1) (s 0.25, r
//     sqrt 2, i < n - 1: 0, 0.10, 0.25 .. 255.75) and the last node infinity (semi-infinite slab / infinitely deep), interpolated
//     linearly in x = 1 / (1 + K a) between the last finite node and infinity (K = 3 (1 - g) / 4: the diffusion field
//     at a fixed depth is linear in x for thick slabs). Table entries between the simulated slabs (27 runs, tau*
//     0.1 .. 1024, ratio <= 1.5) are interpolated in x(tau*) at the fixed distance to the nearer surface; the
//     semi-infinite column is the least-squares line in x through the runs tau* >= 256 at x = 0; a point infinitely
//     far below the top of a finite slab gets no light; infinitely deep in a semi-infinite one the deepest node's.
//   - Sun: the table holds J / mu0 (the source per unit flux through the top, smooth in mu0; J itself is ~ mu0 and
//     interpolating it between nodes overshot the semi-infinite albedo by 2 % at mu0 0.2), mu0 nodes ((i + 1) / 16)^2
//     = 1/256 .. 1 (fine at grazing light; below 1/256 the first node's J / mu0: the shader's entry cosine is >= 0.02).
//   - view: the scattering angle theta between the Sun's propagation and the view propagation (nodes uniform in
//     u = c1 ln(1 + theta / theta0) + c2 theta: 0.02 deg at the forward peak, 11 deg at the backscatter) and the
//     azimuth psi of the view propagation about the Sun's propagation, folded to [0, pi] (the slab is mirror-
//     symmetric about the Sun's plane), psi = 0 toward the slab's up; 13 nodes. Theta resolves the multiply forward-
//     scattered light near the Sun's direction; psi the up / down anisotropy. No azimuth about
//     the slab normal is needed beyond psi: (mu0, theta, psi) fix the view direction in the slab.
//   - the 3D cloud: the same tilted independent column as cloud_scattering_source_3d (entry cosine mu_e in place of
//     mu0, psi measured from the tilted axis).
// Values: little-endian half floats [a][b][mu0][theta][psi] behind a 256-byte header (magic STCLDTT2, the node counts
// and maps, the Draine weight w_D of the generator's phase function, the photon count, the generator command). The GPU form (cloud_transfer_gpu_texels) packs the two
// neighbouring mu0 nodes and b nodes of a cell into RGBA, so one trilinear fetch (theta, psi, a) gives all 32 corners.

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace starsim::environment {

struct CloudTransferTable {
    int depth_nodes = 0, mu_nodes = 0, theta_nodes = 0, psi_nodes = 0;
    double depth_scale = 0.0, depth_ratio = 0.0, diffusion_k = 0.0;  // a_i = scale (ratio^i - 1); x = 1 / (1 + K a)
    double theta0 = 0.0, theta_log = 0.0, theta_linear = 0.0;        // u = log ln(1 + theta / theta0) + linear theta
    double draine_weight = 0.0;  // w_D of the generator's phase function: the view ray's attenuation exp(-w_D tau)
    std::uint64_t photons = 0;
    std::string command;
    std::vector<std::uint16_t> half_values;  // the file's half floats, [a][b][mu0][theta][psi]
    std::vector<float> values;               // the same as float
};

// Reads a table file (header above); throws std::runtime_error on a missing, short or foreign file.
CloudTransferTable load_cloud_transfer_table(const std::filesystem::path& file);

// Fractional node coordinates (0 .. nodes - 1) of the five table axes; the shader port uses the same maps.
double cloud_transfer_depth_coordinate(const CloudTransferTable& table, double depth);  // infinity -> nodes - 1
double cloud_transfer_mu_coordinate(const CloudTransferTable& table, double sun_cos_zenith);
double cloud_transfer_theta_coordinate(const CloudTransferTable& table, double theta_rad);
double cloud_transfer_psi_coordinate(const CloudTransferTable& table, double psi_rad);

// J (the multiple-scattering source's Draine part, header) [1/sr per unit Sun irradiance and unit optical depth],
// multilinear in the node coordinates.
// depth_from_bottom may be infinite (semi-infinite slab); theta: the scattering angle, psi: the folded azimuth.
double cloud_transfer_lookup(const CloudTransferTable& table, double depth_from_top, double depth_from_bottom,
                             double sun_cos_zenith, double theta_rad, double psi_rad);

// The GPU form: a width x height x depth RGBA half-float volume, x = mu0 cell x theta, y = b cell x psi, z = a;
// RGBA = the entries at (mu0 node, b node), (mu0 + 1, b), (mu0, b + 1), (mu0 + 1, b + 1) of the cell.
struct CloudTransferTexels {
    int width = 0, height = 0, depth = 0;
    std::vector<std::uint16_t> rgba_half;  // x fastest, then y, then z
};
CloudTransferTexels cloud_transfer_gpu_texels(const CloudTransferTable& table);

// IEEE half to float (the table's storage).
float half_to_float(std::uint16_t half);

}  // namespace starsim::environment
