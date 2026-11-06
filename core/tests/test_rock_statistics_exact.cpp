// The statistical rock terms (unresolved_shading: the light tables' cast shadow / lit gain from column tops, the view
// tables' hidden ground / seen radiance from the convex hull) against an exact rasterisation of the template meshes the
// renderer draws (Task 13 of the Moon procedural surface plan, ruling T17-A: the fractured, eroded Task 17 templates are
// not convex, so the hull's regions and the column shadow intervals are approximations).
//
// Per template and burial node: the exposed part (the mesh clipped at the ground cut) is projected along the light and
// along the eye onto the ground (a fine raster of the union of the projected triangles: the regions the rock shadows /
// hides, exact up to the raster), and drawn into a z-buffer seen from the eye (the faces the eye really sees, with
// Lommel-Seeliger per face as the tables use).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "starsim/config/moon_surface_config.hpp"
#include "starsim/environment/hapke.hpp"
#include "starsim/environment/lunar_rocks.hpp"
#include "starsim/environment/rock_templates.hpp"
#include "starsim/environment/unresolved_shading.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using environment::RockTemplates;
using environment::TemplateMesh;
using math::Vec3;

namespace {

constexpr double pi = constants::pi;
constexpr double deg = constants::deg_to_rad;

struct P2 {
    double x, y;
};

// Clips a triangle to z >= cut; vertices relative to the cut (z - cut).
std::vector<Vec3> clip_above(const Vec3 (&tri)[3], double cut) {
    std::vector<Vec3> poly;
    for (int i = 0; i < 3; ++i) {
        const Vec3& p = tri[i];
        const Vec3& q = tri[(i + 1) % 3];
        const double dp = p.z - cut, dq = q.z - cut;
        if (dp >= 0.0) poly.push_back({p.x, p.y, dp});
        if ((dp >= 0.0) != (dq >= 0.0)) {
            const double s = dp / (dp - dq);
            const Vec3 x = p + (q - p) * s;
            poly.push_back({x.x, x.y, 0.0});
        }
    }
    return poly;
}

// Boolean raster of the ground: union of convex polygons.
struct Raster {
    double cell, origin;
    int n;
    std::vector<unsigned char> bits;
    Raster(double half_extent, double cell_size)
        : cell(cell_size), origin(-half_extent), n(static_cast<int>(std::ceil(2.0 * half_extent / cell_size))),
          bits(static_cast<size_t>(n) * static_cast<size_t>(n), 0) {}
    void fill(const std::vector<P2>& poly) {
        if (poly.size() < 3) return;
        double x0 = 1e30, x1 = -1e30, y0 = 1e30, y1 = -1e30;
        for (const P2& p : poly) {
            x0 = std::min(x0, p.x), x1 = std::max(x1, p.x), y0 = std::min(y0, p.y), y1 = std::max(y1, p.y);
        }
        // Orientation of the polygon (clipped triangles keep the triangle's winding in projection; either sign).
        double area2 = 0.0;
        for (size_t i = 0; i < poly.size(); ++i) {
            const P2& a = poly[i];
            const P2& b = poly[(i + 1) % poly.size()];
            area2 += a.x * b.y - b.x * a.y;
        }
        if (area2 == 0.0) return;
        const double sign = area2 > 0.0 ? 1.0 : -1.0;
        const int i0 = std::max(0, static_cast<int>(std::floor((x0 - origin) / cell - 0.5)));
        const int i1 = std::min(n - 1, static_cast<int>(std::ceil((x1 - origin) / cell - 0.5)));
        const int j0 = std::max(0, static_cast<int>(std::floor((y0 - origin) / cell - 0.5)));
        const int j1 = std::min(n - 1, static_cast<int>(std::ceil((y1 - origin) / cell - 0.5)));
        for (int i = i0; i <= i1; ++i) {
            const double x = origin + (i + 0.5) * cell;
            for (int j = j0; j <= j1; ++j) {
                const double y = origin + (j + 0.5) * cell;
                bool inside = true;
                for (size_t k = 0; k < poly.size() && inside; ++k) {
                    const P2& a = poly[k];
                    const P2& b = poly[(k + 1) % poly.size()];
                    inside = sign * ((b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x)) >= 0.0;
                }
                if (inside) bits[static_cast<size_t>(i) * static_cast<size_t>(n) + static_cast<size_t>(j)] = 1;
            }
        }
    }
    double area() const {
        double c = 0.0;
        for (unsigned char b : bits) c += b;
        return c * cell * cell;
    }
    void unite(const Raster& other) {
        for (size_t i = 0; i < bits.size(); ++i) bits[i] = bits[i] | other.bits[i];
    }
    double area_and_not(const Raster& other) const {
        double c = 0.0;
        for (size_t i = 0; i < bits.size(); ++i) c += (bits[i] && !other.bits[i]) ? 1.0 : 0.0;
        return c * cell * cell;
    }
};

// The rock's Hapke law (the renderer's: rock w, the globe's theta_bar) for the Hapke flux variant; null: none.
const config::HapkeParameters* rock_hapke = nullptr;

struct Exact {
    double hidden = 0.0;      // |P_v| / A - 1
    double excess = 0.0;      // |P_v \ P_l| / A
    double shadow = 0.0;      // |P_l| / A - 1
    double flux = 0.0;        // seen Lommel-Seeliger radiance per plan area over the flat ground's (face normals)
    double flux_vertex = 0.0; // the same with the mesh's interpolated vertex normals (the renderer's normals)
    double flux_nadir = 0.0;  // the zenith view's flux (face normals)
};

// Exact terms of one template's exposed part for light (e_l, azimuth psi) and eye (e_v, azimuth psi + dphi).
Exact exact_terms(const TemplateMesh& mesh, double cut, double e_l, double e_v, double dphi, double psi,
                  bool with_flux = true) {
    constexpr double ground_cell = 0.01;  // [D] raster of the ground regions (unit rock size)
    constexpr double view_cell = 0.004;   // [D] z-buffer pixel
    const double cot_l = std::cos(e_l) / std::sin(e_l), cot_v = std::cos(e_v) / std::sin(e_v);
    const double phi = psi + dphi;
    const double reach = 1.2 + 1.2 * std::max(cot_l, cot_v);
    Raster plan(reach, ground_cell), light(reach, ground_cell), view(reach, ground_cell);
    std::vector<std::vector<Vec3>> polys;
    std::vector<std::array<Vec3, 3>> tri_normals;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const size_t a = static_cast<size_t>(mesh.indices[t]), b = static_cast<size_t>(mesh.indices[t + 1]),
                     c = static_cast<size_t>(mesh.indices[t + 2]);
        const Vec3 tri[3] = {mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]};
        std::vector<Vec3> poly = clip_above(tri, cut);
        if (poly.size() < 3) continue;
        const auto project = [&poly](double cot, double az) {
            std::vector<P2> p;
            for (const Vec3& v : poly) p.push_back({v.x - v.z * cot * std::cos(az), v.y - v.z * cot * std::sin(az)});
            return p;
        };
        plan.fill(project(0.0, 0.0));
        light.fill(project(cot_l, psi));
        view.fill(project(cot_v, phi));
        polys.push_back(std::move(poly));
        tri_normals.push_back({mesh.normals[a], mesh.normals[b], mesh.normals[c]});
    }
    Exact out;
    const double area = plan.area();
    if (!(area > 0.0)) return out;
    // The regions with the plan view (the ground under a bulge above the cut is lit at a low Sun but hidden from the
    // zenith): sigma = |P u plan| / A - 1, X = |P_v u plan \ P_l u plan| / A; the ground lost = 1 + sigma_l + X.
    light.unite(plan);
    view.unite(plan);
    out.hidden = view.area() / area - 1.0;
    out.shadow = light.area() / area - 1.0;
    out.excess = view.area_and_not(light) / area;
    const Vec3 sun{std::cos(e_l) * std::cos(psi), std::cos(e_l) * std::sin(psi), std::sin(e_l)};
    // z-buffer from an eye direction: flux per plan area over the flat ground's Lommel-Seeliger.
    const auto seen_flux = [&](double ev, double az, bool vertex_normals) {
        const Vec3 v{std::cos(ev) * std::cos(az), std::cos(ev) * std::sin(az), std::sin(ev)};
        const Vec3 u1 = math::normalized(math::cross(Vec3{0.0, 0.0, 1.0}, v).x == 0.0 &&
                                                 math::cross(Vec3{0.0, 0.0, 1.0}, v).y == 0.0
                                             ? Vec3{1.0, 0.0, 0.0}
                                             : math::cross(Vec3{0.0, 0.0, 1.0}, v));
        const Vec3 u2 = math::cross(v, u1);
        const double half = 1.3;
        const int n = static_cast<int>(std::ceil(2.0 * half / view_cell));
        std::vector<double> depth(static_cast<size_t>(n) * static_cast<size_t>(n), -1e30);
        std::vector<double> value(static_cast<size_t>(n) * static_cast<size_t>(n), 0.0);
        const double se = std::sin(e_l), sev = std::sin(ev);
        const double cos_g = math::dot(sun, v);
        const double phase = std::acos(std::clamp(cos_g, -1.0, 1.0));
        const bool hapke = vertex_normals && rock_hapke != nullptr;  // the variant: Hapke per face over the flat rock
        const double flat = hapke ? environment::hapke_reflectance(*rock_hapke, se, sev, phase, rock_hapke->theta_bar_rad)
                                  : se / (se + sev);
        for (size_t f = 0; f < polys.size(); ++f) {
            const std::vector<Vec3>& poly = polys[f];
            for (size_t k = 1; k + 1 < poly.size(); ++k) {
                const Vec3 q[3] = {poly[0], poly[k], poly[k + 1]};
                Vec3 nrm = math::cross(q[1] - q[0], q[2] - q[0]);
                const double len = std::sqrt(math::dot(nrm, nrm));
                if (!(len > 0.0)) continue;
                nrm = nrm / len;
                if (math::dot(nrm, v) <= 0.0) continue;  // back face
                double sx[3], sy[3], sz[3];
                for (int i = 0; i < 3; ++i) sx[i] = math::dot(q[i], u1), sy[i] = math::dot(q[i], u2), sz[i] = math::dot(q[i], v);
                const double det = (sx[1] - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (sy[1] - sy[0]);
                if (det == 0.0) continue;
                const int i0 = std::max(0, static_cast<int>(std::floor((std::min({sx[0], sx[1], sx[2]}) + half) / view_cell)));
                const int i1 = std::min(n - 1, static_cast<int>(std::ceil((std::max({sx[0], sx[1], sx[2]}) + half) / view_cell)));
                const int j0 = std::max(0, static_cast<int>(std::floor((std::min({sy[0], sy[1], sy[2]}) + half) / view_cell)));
                const int j1 = std::min(n - 1, static_cast<int>(std::ceil((std::max({sy[0], sy[1], sy[2]}) + half) / view_cell)));
                // Shading normal: the face's, or the triangle's vertex normals interpolated (approximate for clipped
                // polygons: the original triangle's three normals by the sub-triangle's barycentrics).
                const std::array<Vec3, 3>& vn = tri_normals[f];
                for (int i = i0; i <= i1; ++i) {
                    const double x = -half + (i + 0.5) * view_cell;
                    for (int j = j0; j <= j1; ++j) {
                        const double y = -half + (j + 0.5) * view_cell;
                        const double w1 = ((x - sx[0]) * (sy[2] - sy[0]) - (sx[2] - sx[0]) * (y - sy[0])) / det;
                        const double w2 = ((sx[1] - sx[0]) * (y - sy[0]) - (x - sx[0]) * (sy[1] - sy[0])) / det;
                        const double w0 = 1.0 - w1 - w2;
                        if (w0 < 0.0 || w1 < 0.0 || w2 < 0.0) continue;
                        const double z = w0 * sz[0] + w1 * sz[1] + w2 * sz[2];
                        const size_t idx = static_cast<size_t>(i) * static_cast<size_t>(n) + static_cast<size_t>(j);
                        if (z <= depth[idx]) continue;
                        depth[idx] = z;
                        Vec3 shade = nrm;
                        if (vertex_normals && !hapke) {
                            const Vec3 s = vn[0] * w0 + vn[1] * w1 + vn[2] * w2;  // approximate (see above)
                            shade = math::normalized(s);
                        }
                        const double mu0 = math::dot(shade, sun), mu = std::max(math::dot(shade, v), 1e-6);
                        value[idx] = mu0 > 0.0 ? (hapke ? environment::hapke_reflectance(*rock_hapke, mu0, mu, phase,
                                                                                         rock_hapke->theta_bar_rad)
                                                        : mu0 / (mu0 + mu))
                                               : 0.0;
                    }
                }
            }
        }
        double sum = 0.0;
        for (size_t i = 0; i < value.size(); ++i) {
            if (depth[i] > -1e29) sum += value[i];
        }
        return sum * view_cell * view_cell / flat / (area * sev);
    };
    if (!with_flux) return out;
    out.flux = seen_flux(e_v, phi, false);
    out.flux_vertex = seen_flux(e_v, phi, true);
    out.flux_nadir = seen_flux(0.5 * pi, phi, false);
    return out;
}

}  // namespace

TEST_CASE("Rock statistics against the exact drawn templates (diagnostic)", "[moon_surface][unresolved_shading][.diag]") {
    using M = environment::UnresolvedShadingModel;
    const RockTemplates& templates = RockTemplates::instance();
    auto cfg = config::load_moon_surface_config(std::filesystem::path(STARSIM_CONFIG_DIR) / "moon_surface.toml");
    const double mean = cfg.rocks.burial_fraction_mean, sd = cfg.rocks.burial_fraction_sd;
    config::HapkeParameters rock_law = cfg.hapke;
    rock_law.w = std::min(cfg.hapke.w * cfg.rocks.albedo_to_regolith, 0.999);
    rock_hapke = &rock_law;
    const auto lt = environment::template_rock_light_tables(mean, sd, &rock_law);
    const auto vt = environment::template_rock_view_tables(mean, sd, {}, nullptr, &rock_law);
    struct Geo {
        double el, ev, dphi;
    };
    // The rock patch's failing geometry (i 83, e 30, g 60: Sun 7 deg, eye 60 deg, azimuth difference 37.4 deg) on the
    // tables' grid points around it, and two others.
    const Geo geos[] = {{8.0 * deg, 60.0 * deg, 30.0 * deg}, {6.0 * deg, 60.0 * deg, 30.0 * deg},
                        {16.0 * deg, 90.0 * deg, 0.0}, {30.0 * deg, 44.0 * deg, 30.0 * deg}};
    for (int lod : {1, 3}) {
        for (const Geo& g : geos) {
            double w_sum = 0.0;
            Exact ex{};
            for (int q = 0; q < 5; ++q) {
                const double z = q - 2.0;
                const double w = std::exp(-0.5 * z * z);
                const double f = std::clamp(mean + sd * z, 0.0, environment::LunarRocks::max_burial_fraction);
                for (int id = 0; id < RockTemplates::count; ++id) {
                    const double cut = templates.bottom_z(id) + f * (templates.top_z(id) - templates.bottom_z(id));
                    const TemplateMesh& mesh = templates.mesh(id, lod);
                    for (int yaw = 0; yaw < 2; ++yaw) {
                        const double psi = 2.0 * pi * (yaw + 0.5) / 2.0;
                        const Exact e = exact_terms(mesh, cut, g.el, g.ev, g.dphi, psi);
                        ex.hidden += 0.5 * w * e.hidden, ex.excess += 0.5 * w * e.excess, ex.shadow += 0.5 * w * e.shadow;
                        ex.flux += 0.5 * w * e.flux, ex.flux_vertex += 0.5 * w * e.flux_vertex;
                        ex.flux_nadir += 0.5 * w * e.flux_nadir;
                    }
                    w_sum += w;
                }
            }
            const int kv = static_cast<int>(std::lround(g.ev / M::view_elevation_step_rad));
            const int ka = static_cast<int>(std::lround(g.dphi / M::view_azimuth_step_rad));
            const int kl = static_cast<int>(std::lround(g.el / M::lit_gain_step_rad));
            const size_t idx = (static_cast<size_t>(kv) * M::view_azimuth_samples + static_cast<size_t>(ka)) *
                                   M::lit_gain_samples + static_cast<size_t>(kl);
            const double sigma_v = kv >= M::view_elevation_samples - 1 ? 0.0 : lt.shadow[static_cast<size_t>(kv)];
            const double table_flux = (1.0 + sigma_v) * lt.gain[static_cast<size_t>(kl)] * vt.gain_ratio[idx];
            std::printf("LOD %d e_l %4.1f e_v %4.1f dphi %4.1f | 1+hidden exact %.4f table %.4f | excess exact %.4f table %.4f"
                        " | shadow exact %.4f table %.4f | rock flux exact %.4f (Hapke %.4f) table %.4f | nadir flux exact"
                        " %.4f columns G %.4f\n",
                        lod, g.el / deg, g.ev / deg, g.dphi / deg, 1.0 + ex.hidden / w_sum, 1.0 + sigma_v,
                        ex.excess / w_sum, vt.excess[idx], ex.shadow / w_sum, lt.shadow[static_cast<size_t>(kl)],
                        ex.flux / w_sum, ex.flux_vertex / w_sum, table_flux, ex.flux_nadir / w_sum,
                        lt.gain[static_cast<size_t>(kl)]);
        }
    }
    // The light tables' cast shadow (8 azimuths, 9 burial nodes) against the exact LOD-1 regions with the same
    // sampling, light 8 and 30 deg.
    for (int kl : {4, 15}) {
        double exact = 0.0, weights = 0.0;
        for (int q = 0; q < 9; ++q) {
            const double z = q - 4.0;
            const double w = std::exp(-0.5 * z * z);
            const double f = std::clamp(mean + sd * z, 0.0, environment::LunarRocks::max_burial_fraction);
            for (int id = 0; id < RockTemplates::count; ++id) {
                const double cut = templates.bottom_z(id) + f * (templates.top_z(id) - templates.bottom_z(id));
                for (int a = 0; a < environment::rock_light_azimuths; ++a) {
                    const double psi = 2.0 * pi * (a + 0.5) / environment::rock_light_azimuths;
                    exact += w / environment::rock_light_azimuths *
                             exact_terms(templates.mesh(id, 1), cut, kl * 2.0 * deg, 0.5 * pi, 0.0, psi, false).shadow;
                }
                weights += w;
            }
        }
        std::printf("light tables: sigma(%d deg) table %.4f exact LOD1 %.4f (%+.2f %%)\n", 2 * kl,
                    lt.shadow[static_cast<size_t>(kl)], exact / weights,
                    100.0 * (lt.shadow[static_cast<size_t>(kl)] / (exact / weights) - 1.0));
    }
    SUCCEED();
}

TEST_CASE("Rock mesh shadow: a half-spheroid's closed form and a raster of the templates", "[moon_surface][unresolved_shading]") {
    // UV spheroid, plan radius 1, height H = 2 eta (D = 2), cut at z = 0.
    const double eta = 0.444, h = 2.0 * eta;
    TemplateMesh mesh;
    constexpr int nu = 192, nv = 96;
    for (int j = 0; j <= nv; ++j) {
        const double th = pi * j / nv;
        for (int i = 0; i < nu; ++i) {
            const double ph = 2.0 * pi * i / nu;
            mesh.vertices.push_back({std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), h * std::cos(th)});
            mesh.normals.push_back({0.0, 0.0, 1.0});
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            const int a = j * nu + i, b = j * nu + (i + 1) % nu, c = (j + 1) * nu + i, d = (j + 1) * nu + (i + 1) % nu;
            mesh.indices.insert(mesh.indices.end(), {a, c, b, b, c, d});
        }
    }
    for (double e : {8.0, 30.0}) {
        const double closed = environment::rock_shadow_per_cover(eta, std::sin(e * deg));
        const double lines = environment::rock_mesh_shadow(mesh, 0.0, e * deg, 0.3);
        const double raster = exact_terms(mesh, 0.0, e * deg, 0.5 * pi, 0.0, 0.3, false).shadow;
        std::printf("half-spheroid e %4.1f: closed %.4f lines %.4f raster %.4f\n", e, closed, lines, raster);
        CHECK(std::abs(lines - closed) < 1e-3 * (1.0 + closed));
        CHECK(std::abs(raster - closed) < 2e-3 * (1.0 + closed));
    }
    const RockTemplates& templates = RockTemplates::instance();
    for (int id : {0, 12, 22, 24}) {
        const double cut = templates.bottom_z(id) + 0.3 * (templates.top_z(id) - templates.bottom_z(id));
        for (int lod : {1, 3}) {
            const double lines = environment::rock_mesh_shadow(templates.mesh(id, lod), cut, 8.0 * deg, 0.4);
            const double raster = exact_terms(templates.mesh(id, lod), cut, 8.0 * deg, 0.5 * pi, 0.0, 0.4, false).shadow;
            std::printf("template %d LOD %d: lines %.4f raster %.4f\n", id, lod, lines, raster);
            CHECK(std::abs(lines - raster) < 5e-3 * (1.0 + raster));  // the raster cell (0.01) and the line spacing
        }
    }
}
