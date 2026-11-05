// Touchdown point selection for the lunar descent (declared in lunar_descent.hpp).
//
// Design decisions:
// - The LOLA checks (footprint plane, roughness, glide cone) run first for every candidate, nearest first, exactly as
//   without the detailed surface: they are the cheap pre-check, and detail off is that selection bit for bit.
// - The hazard check of the detailed surface runs only on candidates that pass them, and stops at the first admissible
//   one. Feet use DetailedLunarTerrain::support_under_disc (the pivoting pad the leg contact feels) at the same frame
//   convention as contact::TerrainField (a vertical through the candidate's tangent-plane point; heights along the
//   candidate's up). The roll at touchdown is not controlled: every roll is checked (feet sampled on the footprint
//   circle, the base disc's samples once per candidate).
// - The hull base (bells): the contact ground on a grid plus the contact tops of the rocks that can reach above the
//   plane by more than the ground does (an upper bound of each rock's top decides whether its footprint is sampled).

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "starsim/mission/lunar_descent.hpp"

namespace starsim::mission {

namespace {
constexpr int footprint_samples = 12;  // points on the leg circle (independent of the vehicle's leg count)
// Hull base disc: the contact ground on a square grid of this spacing ([E]: the relief left between the nodes at half
// a metre is a few cm, small against the 0.9 m HLS clearance) and a rock's footprint on (2k + 1)^2 points over its
// bounding square (+- a) with k = rock_samples_per_semi_axis ([E]).
constexpr double base_grid_m = 0.5;
constexpr int rock_samples_per_semi_axis = 6;

struct LatLon {
    double latitude_rad;
    double longitude_rad;
};

// Point `east_m` / `north_m` away from (lat, lon) in the local horizontal plane on the sphere of `radius_m`
// (local ENU basis, back to latitude/longitude with atan2).
LatLon offset_position(const LatLon& at, double east_m, double north_m, double radius_m) {
    const double sl = std::sin(at.latitude_rad), cl = std::cos(at.latitude_rad);
    const double so = std::sin(at.longitude_rad), co = std::cos(at.longitude_rad);
    const math::Vec3 up{cl * co, cl * so, sl};
    const math::Vec3 east{-so, co, 0.0};
    const math::Vec3 north{-sl * co, -sl * so, cl};
    const math::Vec3 p = up * radius_m + east * east_m + north * north_m;
    return {std::atan2(p.z, std::hypot(p.x, p.y)), std::atan2(p.y, p.x)};
}

struct FootprintFit {
    double slope_rad;
    double roughness_m;
    double centre_height_m;
};

// Least-squares plane z = a e + b n + c through the centre and the circle samples. The layout is symmetric
// (sum e = sum n = sum e n = 0, sum e^2 = sum n^2), so the normal equations decouple.
FootprintFit fit_footprint(const environment::LunarTerrain& terrain, const LatLon& centre, double radius_m,
                           double sphere_m) {
    std::vector<math::Vec3> points;  // (east, north, height)
    points.push_back({0.0, 0.0, terrain.height_m(centre.latitude_rad, centre.longitude_rad)});
    for (int k = 0; k < footprint_samples; ++k) {
        const double az = constants::two_pi * k / footprint_samples;
        const double e = radius_m * std::sin(az), n = radius_m * std::cos(az);
        const auto q = offset_position(centre, e, n, sphere_m + points[0].z);
        points.push_back({e, n, terrain.height_m(q.latitude_rad, q.longitude_rad)});
    }
    double sez = 0.0, snz = 0.0, see = 0.0, snn = 0.0, sz = 0.0;
    for (const auto& p : points) {
        sez += p.x * p.z;
        snz += p.y * p.z;
        see += p.x * p.x;
        snn += p.y * p.y;
        sz += p.z;
    }
    const double a = sez / see, b = snz / snn, c = sz / static_cast<double>(points.size());
    double roughness = 0.0;
    for (const auto& p : points) roughness = std::max(roughness, std::abs(p.z - (a * p.x + b * p.y + c)));
    return {std::atan(std::hypot(a, b)), roughness, points[0].z};
}

// Terrain uprange of the point (against the approach azimuth) stays below the glide cone by the margin.
bool glide_cone_clear(const environment::LunarTerrain& terrain, const LatLon& centre, double centre_height_m,
                      double approach_azimuth_rad, double sphere_m, const LunarDescentParameters& p) {
    constexpr double first_m = 100.0, step_m = 20.0;  // [E] from beyond the footprint, one pixel apart
    const double tan_slope = std::tan(p.glide_slope_rad);
    for (double d = first_m; d <= p.glide_check_range_m; d += step_m) {
        const auto q = offset_position(centre, -d * std::sin(approach_azimuth_rad), -d * std::cos(approach_azimuth_rad),
                                       sphere_m + centre_height_m);
        if (terrain.height_m(q.latitude_rad, q.longitude_rad) - centre_height_m > d * tan_slope - p.glide_margin_m) {
            return false;
        }
    }
    return true;
}

// The candidate's local frame: origin on its LOLA height, east / north / up (as contact::TerrainField's site frame).
struct LocalFrame {
    math::Vec3 origin_m, east, north, up;
    double sphere_m;
    math::Vec3 vertical(double e, double n) const { return math::normalized(origin_m + east * e + north * n); }
    // Height along up of the point height_m above the sphere on the vertical `unit`.
    double local_height(const math::Vec3& unit, double height_m) const {
        return dot(unit * (sphere_m + height_m) - origin_m, up);
    }
};

LocalFrame local_frame(const LatLon& c, double height_m, double sphere_m) {
    const double sl = std::sin(c.latitude_rad), cl = std::cos(c.latitude_rad);
    const double so = std::sin(c.longitude_rad), co = std::cos(c.longitude_rad);
    const math::Vec3 up{cl * co, cl * so, sl};
    return {up * (sphere_m + height_m), {-so, co, 0.0}, {-sl * co, -sl * so, cl}, up, sphere_m};
}

struct Plane {
    double a, b, c;  // z = a e + b n + c
    double at(double e, double n) const { return a * e + b * n + c; }
};

// Least-squares plane through (e, n, z) points (>= 3, not collinear).
Plane fit_plane(const std::vector<math::Vec3>& pts) {
    double s1 = 0, se = 0, sn = 0, see = 0, snn = 0, sen = 0, sz = 0, sez = 0, snz = 0;
    for (const auto& p : pts) {
        s1 += 1.0;
        se += p.x;
        sn += p.y;
        see += p.x * p.x;
        snn += p.y * p.y;
        sen += p.x * p.y;
        sz += p.z;
        sez += p.x * p.z;
        snz += p.y * p.z;
    }
    // [see sen se; sen snn sn; se sn s1] (a b c) = (sez snz sz), Cramer's rule.
    const auto det3 = [](double a11, double a12, double a13, double a21, double a22, double a23, double a31, double a32,
                         double a33) {
        return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
    };
    const double d = det3(see, sen, se, sen, snn, sn, se, sn, s1);
    return {det3(sez, sen, se, snz, snn, sn, sz, sn, s1) / d, det3(see, sez, se, sen, snz, sn, se, sz, s1) / d,
            det3(see, sen, sez, sen, snn, snz, se, sn, sz) / d};
}

struct HazardResult {
    bool admissible = false;
    double height_m = 0.0, slope_rad = 0.0, max_step_m = 0.0, min_clearance_m = 0.0;
};

// The hazard check of one candidate (centre on its LOLA height) for every roll; stops at the first failed roll.
HazardResult check_hazards(const environment::DetailedLunarTerrain& surface, const LatLon& centre, double lola_height_m,
                           double footprint_radius_m, const LunarDescentParameters& p, const TouchdownHazardLimits& h) {
    const LocalFrame f = local_frame(centre, lola_height_m, surface.lola().sphere_radius_m());
    const int legs = static_cast<int>(h.leg_azimuths_rad.size());
    // Roll samples: at most hazard_roll_spacing_per_pad_radius x the pad radius apart on the footprint circle, a
    // multiple of the leg count (evenly spaced legs then reuse every foot sample).
    int rolls = static_cast<int>(std::ceil(constants::two_pi * footprint_radius_m /
                                           (hazard_roll_spacing_per_pad_radius * h.foot_pad_radius_m)));
    rolls = ((rolls + legs - 1) / legs) * legs;
    const double step = constants::two_pi / rolls;

    std::unordered_map<long long, math::Vec3> feet;  // quantised azimuth -> (e, n, z)
    const auto foot = [&](double azimuth) -> const math::Vec3& {
        double az = std::fmod(azimuth, constants::two_pi);
        if (az < 0.0) az += constants::two_pi;
        const long long key = std::llround(az / step * 1.0e6);
        const auto it = feet.find(key);
        if (it != feet.end()) return it->second;
        const double e = footprint_radius_m * std::sin(az), n = footprint_radius_m * std::cos(az);
        const math::Vec3 u = f.vertical(e, n);
        const auto s = surface.support_under_disc(u, f.up, h.foot_pad_radius_m);
        return feet.emplace(key, math::Vec3{e, n, f.local_height(u, s.height_m)}).first->second;
    };

    HazardResult r;
    std::vector<Plane> planes;
    std::vector<math::Vec3> pts(static_cast<size_t>(legs));
    double height_sum = 0.0;
    for (int i = 0; i < rolls; ++i) {
        for (int k = 0; k < legs; ++k) pts[static_cast<size_t>(k)] = foot(i * step + h.leg_azimuths_rad[static_cast<size_t>(k)]);
        const Plane pl = fit_plane(pts);
        double lo = std::numeric_limits<double>::infinity(), hi = -lo;
        for (const auto& q : pts) {
            const double d = q.z - pl.at(q.x, q.y);
            lo = std::min(lo, d);
            hi = std::max(hi, d);
        }
        const double slope = std::atan(std::hypot(pl.a, pl.b));
        r.slope_rad = std::max(r.slope_rad, slope);
        r.max_step_m = std::max(r.max_step_m, hi - lo);
        if (slope > p.slope_limit_rad || hi - lo > h.max_foot_step_m) return r;
        planes.push_back(pl);
        height_sum += pl.c;
    }

    // Hull base: the ground's highest rise above each roll's plane, then the rocks that can top it.
    std::vector<double> rise(planes.size(), -std::numeric_limits<double>::infinity());
    const auto add_sample = [&](double e, double n, double z) {
        for (size_t i = 0; i < planes.size(); ++i) rise[i] = std::max(rise[i], z - planes[i].at(e, n));
    };
    const int m = static_cast<int>(std::floor(h.base_radius_m / base_grid_m));
    for (int i = -m; i <= m; ++i) {
        for (int j = -m; j <= m; ++j) {
            const double e = i * base_grid_m, n = j * base_grid_m;
            if (std::hypot(e, n) > h.base_radius_m) continue;
            const math::Vec3 u = f.vertical(e, n);
            add_sample(e, n, f.local_height(u, surface.contact_ground_m(u)));
        }
    }
    if (surface.rocks_on()) {
        const auto rocks = surface.rocks().rocks_touching(f.up, h.base_radius_m,
                                                          environment::DetailedLunarTerrain::contact_min_rock_diameter_m);
        for (const auto& rock : rocks) {
            const double a = rock.axes_m.x;
            const math::Vec3 base = rock.base_unit * (f.sphere_m + lola_height_m) - f.origin_m;
            const double eb = dot(base, f.east), nb = dot(base, f.north);
            // Upper bound of the rock's top (its centre + the c semi-axis bounds it) on the base vertical.
            const double top_z = f.local_height(rock.base_unit, rock.base_height_m - rock.burial_m + rock.axes_m.z);
            bool may_top = false;
            for (size_t i = 0; i < planes.size() && !may_top; ++i) {
                const double reach = std::hypot(planes[i].a, planes[i].b) * a;
                may_top = top_z - planes[i].at(eb, nb) + reach > rise[i];
            }
            if (!may_top) continue;
            const double d = a / rock_samples_per_semi_axis;
            for (int i = -rock_samples_per_semi_axis; i <= rock_samples_per_semi_axis; ++i) {
                for (int j = -rock_samples_per_semi_axis; j <= rock_samples_per_semi_axis; ++j) {
                    const double e = eb + i * d, n = nb + j * d;
                    if (std::hypot(e, n) > h.base_radius_m) continue;
                    const math::Vec3 u = f.vertical(e, n);
                    if (const auto t = surface.contact_rock_top(rock, u)) add_sample(e, n, f.local_height(u, t->first));
                }
            }
        }
    }
    r.min_clearance_m = std::numeric_limits<double>::infinity();
    for (const double x : rise) r.min_clearance_m = std::min(r.min_clearance_m, h.bell_clearance_m - x);
    if (r.min_clearance_m < 0.0) return r;
    r.admissible = true;
    r.height_m = lola_height_m + height_sum / static_cast<double>(planes.size());
    return r;
}

// Candidates on the grid, nearest first; the first that passes the LOLA checks and `accept` (filling the point's
// remaining fields) is returned.
template <typename Accept>
std::optional<TouchdownPoint> search(const environment::LunarTerrain& terrain, double latitude_rad,
                                     double longitude_rad, double approach_azimuth_rad, double footprint_radius_m,
                                     const LunarDescentParameters& p, Accept&& accept) {
    struct Candidate {
        double offset_m;
        double east_m;
        double north_m;
    };
    std::vector<Candidate> candidates;
    const int n = static_cast<int>(std::floor(p.touchdown_search_radius_m / p.touchdown_grid_m));
    for (int i = -n; i <= n; ++i) {
        for (int j = -n; j <= n; ++j) {
            const double e = i * p.touchdown_grid_m, nn = j * p.touchdown_grid_m;
            const double d = std::hypot(e, nn);
            if (d <= p.touchdown_search_radius_m) candidates.push_back({d, e, nn});
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.offset_m < b.offset_m; });

    const LatLon site{latitude_rad, longitude_rad};
    const double sphere_m = terrain.sphere_radius_m();
    const double site_height_m = terrain.height_m(latitude_rad, longitude_rad);
    for (const auto& c : candidates) {
        const auto centre = offset_position(site, c.east_m, c.north_m, sphere_m + site_height_m);
        const auto fit = fit_footprint(terrain, centre, footprint_radius_m, sphere_m);
        if (fit.slope_rad > p.slope_limit_rad || fit.roughness_m > p.roughness_limit_m) continue;
        if (!glide_cone_clear(terrain, centre, fit.centre_height_m, approach_azimuth_rad, sphere_m, p)) continue;
        TouchdownPoint point{centre.latitude_rad, centre.longitude_rad, fit.centre_height_m, fit.slope_rad,
                             c.offset_m};
        if (accept(centre, point)) return point;
    }
    return std::nullopt;
}
}  // namespace

std::optional<TouchdownPoint> select_touchdown_point(const environment::LunarTerrain& terrain, double latitude_rad,
                                                     double longitude_rad, double approach_azimuth_rad,
                                                     double footprint_radius_m, const LunarDescentParameters& p) {
    return search(terrain, latitude_rad, longitude_rad, approach_azimuth_rad, footprint_radius_m, p,
                  [](const LatLon&, TouchdownPoint&) { return true; });
}

std::optional<TouchdownPoint> select_touchdown_point(const environment::DetailedLunarTerrain& surface,
                                                     double latitude_rad, double longitude_rad,
                                                     double approach_azimuth_rad, double footprint_radius_m,
                                                     const LunarDescentParameters& p,
                                                     const TouchdownHazardLimits& hazards) {
    if (!surface.detail_on()) {
        return select_touchdown_point(surface.lola(), latitude_rad, longitude_rad, approach_azimuth_rad,
                                      footprint_radius_m, p);
    }
    if (hazards.leg_azimuths_rad.size() < 3 || !(hazards.foot_pad_radius_m > 0.0) || !(footprint_radius_m > 0.0)) {
        throw std::invalid_argument("select_touchdown_point: Gefahrenprüfung braucht mindestens drei Beine und Fußteller");
    }
    return search(surface.lola(), latitude_rad, longitude_rad, approach_azimuth_rad, footprint_radius_m, p,
                  [&](const LatLon& centre, TouchdownPoint& point) {
                      const auto r = check_hazards(surface, centre, point.height_m, footprint_radius_m, p, hazards);
                      if (!r.admissible) return false;
                      point.height_m = r.height_m;
                      point.slope_rad = r.slope_rad;
                      point.max_foot_step_m = r.max_step_m;
                      point.min_bell_clearance_m = r.min_clearance_m;
                      return true;
                  });
}

TouchdownHazardLimits touchdown_hazard_limits(const config::StageConfig& stage) {
    const auto& legs = stage.landing_legs;
    if (legs.azimuths_rad.empty() || !(legs.foot_pad_radius_m > 0.0) || stage.engine_groups.empty()) {
        throw std::invalid_argument("touchdown_hazard_limits: Stufe ohne Landebeine, Fußteller oder Triebwerke");
    }
    double lowest_exit_station_m = std::numeric_limits<double>::infinity();
    for (const auto& g : stage.engine_groups) lowest_exit_station_m = std::min(lowest_exit_station_m, g.station_m);
    TouchdownHazardLimits h;
    h.foot_pad_radius_m = legs.foot_pad_radius_m;
    h.max_foot_step_m = legs.stroke_m;
    h.bell_clearance_m = (lowest_exit_station_m - legs.foot_station_m) - legs.stroke_m;
    h.base_radius_m = 0.5 * stage.diameter_m;
    h.leg_azimuths_rad = legs.azimuths_rad;
    return h;
}

}  // namespace starsim::mission
