// Touchdown point selection for the lunar descent (declared in lunar_descent.hpp).
//
// Design decisions:
// - The LOLA checks (footprint plane, roughness, glide cone) run first for every candidate, nearest first, exactly as
//   without the detailed surface: they are the cheap pre-check, and detail off is that selection bit for bit.
// - The hazard check of the detailed surface runs only on candidates that pass them. With detail on the candidates are
//   checked ring by ring in parallel (a refused site checks every candidate: minutes serially) and the first admissible
//   one in the serial order wins, so the point is the serial one bit for bit; check_hazards is pure and the surface's
//   caches are thread-safe. Feet use DetailedLunarTerrain::support_under_disc (the pivoting pad the leg contact feels)
//   at the same frame convention as contact::TerrainField (a vertical through the candidate's tangent-plane point;
//   heights along the candidate's up). The roll at touchdown is not controlled: every roll is checked (feet sampled on the footprint
//   circle, the base disc's samples once per candidate).
// - The hull base (bells): the contact ground on a grid plus the contact tops of the rocks that can reach above the
//   plane by more than the ground does (an upper bound of each rock's top decides whether its footprint is sampled).

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>
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
// Parallel candidate search: cores left to the 200 Hz flight simulation and the render thread while it runs (as the
// in-game weather builds), and a cap on the pool.
constexpr int search_cores_left_free = 2;
constexpr int max_search_threads = 32;

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
    const auto key_of = [&](double azimuth) {
        double az = std::fmod(azimuth, constants::two_pi);
        if (az < 0.0) az += constants::two_pi;
        return std::llround(az / step * 1.0e6);
    };
    const auto foot_az = [&](int roll, int leg) { return roll * step + h.leg_azimuths_rad[static_cast<size_t>(leg)]; };

    std::unordered_map<long long, math::Vec3> feet;  // quantised azimuth -> (e, n, z)
    const auto foot = [&](double azimuth) -> const math::Vec3& {
        const long long key = key_of(azimuth);
        const auto it = feet.find(key);
        if (it != feet.end()) return it->second;
        const double e = footprint_radius_m * std::sin(azimuth), n = footprint_radius_m * std::cos(azimuth);
        const math::Vec3 u = f.vertical(e, n);
        const auto s = surface.support_under_disc(u, f.up, h.foot_pad_radius_m);
        return feet.emplace(key, math::Vec3{e, n, f.local_height(u, s.height_m)}).first->second;
    };

    HazardResult r;
    std::vector<TouchdownRestPlane> planes;  // every roll's facets
    std::vector<math::Vec3> pts(static_cast<size_t>(legs));
    double height_sum = 0.0;
    int roll_count = 0;
    for (int i = 0; i < rolls; ++i) {
        for (int k = 0; k < legs; ++k) pts[static_cast<size_t>(k)] = foot(foot_az(i, k));
        const auto facets = touchdown_rest_planes(pts);
        if (facets.empty()) return r;  // no foot triangle under the axis: the hull would tip
        for (const auto& rest : facets) {  // balanced on an edge: it may tip onto either side
            r.slope_rad = std::max(r.slope_rad, rest.slope_rad());
            r.max_step_m = std::max(r.max_step_m, rest.hang_m);
            if (rest.slope_rad() > p.slope_limit_rad || rest.hang_m > h.max_foot_step_m) return r;
            planes.push_back(rest);
        }
        height_sum += facets.front().c;
        ++roll_count;
    }

    // Hull base: the ground's highest rise above each roll's rest plane, then the rocks that can top it.
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
    r.height_m = lola_height_m + height_sum / static_cast<double>(roll_count);
    return r;
}

// Memo of the hazard results (they do not depend on the approach azimuth, only the glide cone does): the mission's
// reachability check (both azimuths) and the descent planner ask for the same candidates. One context at a time (the
// surface's parts by ownership - a freed and reallocated part never matches -, its options, the footprint, the slope
// limit and the hazard limits); per candidate centre (latitude / longitude bits). Thread-safe (the lock is not held
// while a candidate is checked).
class HazardMemo {
public:
    std::optional<HazardResult> find(const environment::DetailedLunarTerrain& s, double footprint_m,
                                     const LunarDescentParameters& p, const TouchdownHazardLimits& h,
                                     const LatLon& c) {
        const std::lock_guard lock(mutex_);
        if (!same_context(s, footprint_m, p, h)) return std::nullopt;
        const auto it = results_.find(key(c));
        if (it == results_.end()) return std::nullopt;
        return it->second;
    }
    void store(const environment::DetailedLunarTerrain& s, double footprint_m, const LunarDescentParameters& p,
               const TouchdownHazardLimits& h, const LatLon& c, const HazardResult& r) {
        const std::lock_guard lock(mutex_);
        if (!same_context(s, footprint_m, p, h) || results_.size() >= max_entries) {
            results_.clear();
            lola_ = s.lola_shared();
            detail_ = s.detail_shared();
            rocks_ = s.rocks_shared();
            options_ = s.options();
            footprint_m_ = footprint_m;
            slope_limit_rad_ = p.slope_limit_rad;
            limits_ = h;
        }
        results_[key(c)] = r;
    }

private:
    static constexpr size_t max_entries = 50'000;  // [E] > the ~7,850 candidates of a 1 km / 20 m search, ~3 MB
    static std::pair<std::uint64_t, std::uint64_t> key(const LatLon& c) {
        return {std::bit_cast<std::uint64_t>(c.latitude_rad), std::bit_cast<std::uint64_t>(c.longitude_rad)};
    }
    template <typename T>
    static bool same_owner(const std::weak_ptr<const T>& w, const std::shared_ptr<const T>& s) {
        return !w.owner_before(s) && !s.owner_before(w) && (s != nullptr || w.expired());
    }
    bool same_context(const environment::DetailedLunarTerrain& s, double footprint_m, const LunarDescentParameters& p,
                      const TouchdownHazardLimits& h) const {
        return !lola_.expired() && same_owner(lola_, s.lola_shared()) && same_owner(detail_, s.detail_shared()) &&
               same_owner(rocks_, s.rocks_shared()) && options_ == s.options() && footprint_m_ == footprint_m &&
               slope_limit_rad_ == p.slope_limit_rad && limits_.foot_pad_radius_m == h.foot_pad_radius_m &&
               limits_.max_foot_step_m == h.max_foot_step_m && limits_.bell_clearance_m == h.bell_clearance_m &&
               limits_.base_radius_m == h.base_radius_m && limits_.leg_azimuths_rad == h.leg_azimuths_rad;
    }
    std::mutex mutex_;
    std::weak_ptr<const environment::LunarTerrain> lola_;
    std::weak_ptr<const environment::LunarSurfaceDetail> detail_;
    std::weak_ptr<const environment::LunarRocks> rocks_;
    environment::SurfaceDetailOptions options_;
    double footprint_m_ = 0.0, slope_limit_rad_ = 0.0;
    TouchdownHazardLimits limits_;
    std::map<std::pair<std::uint64_t, std::uint64_t>, HazardResult> results_;
};

HazardMemo& hazard_memo() {
    static HazardMemo memo;
    return memo;
}

struct Candidate {
    double offset_m;
    double east_m;
    double north_m;
};

// The candidate grid in the site's east/north plane within the search radius, nearest first (ties in grid order).
std::vector<Candidate> sorted_candidates(const LunarDescentParameters& p) {
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
    return candidates;
}

// Worker threads of the parallel search: `requested` if positive, else the hardware's threads less the cores left to
// the flight simulation and render threads (at least 1, at most max_search_threads).
int search_thread_count(int requested) {
    if (requested > 0) return std::min(requested, max_search_threads);
    const int hardware = static_cast<int>(std::thread::hardware_concurrency());
    return std::clamp(hardware - search_cores_left_free, 1, max_search_threads);
}

// Runs fn(i) for i in [0, count) on `threads` threads (the calling thread is one; dynamic: each takes the next index).
// The first exception is rethrown on the calling thread after all have joined. If starting a worker fails
// (std::system_error from std::thread, final review m2), no further workers are started: the calling thread and the
// workers started so far take the remaining indices (dynamic), so the result is the same.
template <typename Fn>
void parallel_for(int count, int threads, Fn&& fn) {
    threads = std::min(threads, count);
    if (threads <= 1) {
        for (int i = 0; i < count; ++i) fn(i);
        return;
    }
    std::atomic<int> next{0};
    std::vector<std::exception_ptr> errors(static_cast<size_t>(threads));
    const auto work = [&](int t) {
        try {
            for (int i = next.fetch_add(1); i < count; i = next.fetch_add(1)) fn(i);
        } catch (...) {
            errors[static_cast<size_t>(t)] = std::current_exception();
            next.store(count);  // the others stop at their next index
        }
    };
    std::vector<std::thread> pool;
    pool.reserve(static_cast<size_t>(threads - 1));
    try {
        for (int t = 1; t < threads; ++t) pool.emplace_back(work, t);
    } catch (...) {
        // Thread creation failed: fewer workers; work(0) below drains the rest.
    }
    work(0);
    for (auto& th : pool) th.join();
    for (const auto& e : errors) {
        if (e) std::rethrow_exception(e);
    }
}

// The nearest candidate for which `evaluate` (a pure, thread-safe function of the candidate returning the point or
// nothing) gives a point. Serial (threads 1): candidates in order, stops at the first point. Parallel: ring by ring of
// touchdown_grid_m width (nearest ring first), every candidate of a ring evaluated, the first point in candidate order
// taken - the same candidate as the serial order, so the same point bit for bit.
template <typename Evaluate>
std::optional<TouchdownPoint> nearest_admissible(const std::vector<Candidate>& candidates, double ring_m, int threads,
                                                 Evaluate&& evaluate) {
    const int count = static_cast<int>(candidates.size());
    if (threads <= 1) {
        for (int i = 0; i < count; ++i) {
            if (auto point = evaluate(candidates[static_cast<size_t>(i)])) return point;
        }
        return std::nullopt;
    }
    std::vector<std::optional<TouchdownPoint>> results;
    std::vector<int> order;  // evaluation order within the ring: by azimuth
    for (int first = 0; first < count;) {
        const double ring = std::floor(candidates[static_cast<size_t>(first)].offset_m / ring_m);
        int end = first + 1;
        while (end < count && std::floor(candidates[static_cast<size_t>(end)].offset_m / ring_m) == ring) ++end;
        // The threads take the candidates in azimuth order, so the ones in flight are neighbours whose footprints
        // share surface cells (the grid spacing is below the footprint's diameter): the cells stay in the surface's
        // contact cache. Only the evaluation order; the choice below is by candidate index.
        order.resize(static_cast<size_t>(end - first));
        for (int i = 0; i < end - first; ++i) order[static_cast<size_t>(i)] = i;
        const auto azimuth = [&](int i) {
            const Candidate& c = candidates[static_cast<size_t>(first + i)];
            return std::atan2(c.east_m, c.north_m);
        };
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            const double za = azimuth(a), zb = azimuth(b);
            return za < zb || (za == zb && a < b);
        });
        results.assign(static_cast<size_t>(end - first), std::nullopt);
        parallel_for(end - first, threads, [&](int k) {
            const int i = order[static_cast<size_t>(k)];
            results[static_cast<size_t>(i)] = evaluate(candidates[static_cast<size_t>(first + i)]);
        });
        for (auto& r : results) {
            if (r) return r;
        }
        first = end;
    }
    return std::nullopt;
}

// Candidates on the grid, nearest first; the nearest that passes the LOLA checks and `accept` (filling the point's
// remaining fields; pure and thread-safe when threads > 1) is returned.
template <typename Accept>
std::optional<TouchdownPoint> search(const environment::LunarTerrain& terrain, double latitude_rad,
                                     double longitude_rad, double approach_azimuth_rad, double footprint_radius_m,
                                     const LunarDescentParameters& p, int threads, Accept&& accept) {
    const LatLon site{latitude_rad, longitude_rad};
    const double sphere_m = terrain.sphere_radius_m();
    const double site_height_m = terrain.height_m(latitude_rad, longitude_rad);
    return nearest_admissible(sorted_candidates(p), p.touchdown_grid_m, threads,
                              [&](const Candidate& c) -> std::optional<TouchdownPoint> {
        const auto centre = offset_position(site, c.east_m, c.north_m, sphere_m + site_height_m);
        const auto fit = fit_footprint(terrain, centre, footprint_radius_m, sphere_m);
        if (fit.slope_rad > p.slope_limit_rad || fit.roughness_m > p.roughness_limit_m) return std::nullopt;
        if (!glide_cone_clear(terrain, centre, fit.centre_height_m, approach_azimuth_rad, sphere_m, p)) {
            return std::nullopt;
        }
        TouchdownPoint point{centre.latitude_rad, centre.longitude_rad, fit.centre_height_m, fit.slope_rad,
                             c.offset_m};
        if (!accept(centre, point)) return std::nullopt;
        return point;
    });
}
}  // namespace

std::optional<TouchdownPoint> select_touchdown_point(const environment::LunarTerrain& terrain, double latitude_rad,
                                                     double longitude_rad, double approach_azimuth_rad,
                                                     double footprint_radius_m, const LunarDescentParameters& p) {
    return search(terrain, latitude_rad, longitude_rad, approach_azimuth_rad, footprint_radius_m, p, 1,
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
        throw std::invalid_argument("select_touchdown_point: the hazard check needs at least three legs and foot pads");
    }
    static const bool debug = std::getenv("STARSIM_DEBUG_TOUCHDOWN") != nullptr;  // diagnostics
    std::atomic<int> checked{0}, remembered{0};
    std::atomic<std::int64_t> checked_us{0};
    const int threads = search_thread_count(p.touchdown_search_threads);
    const auto t_start = std::chrono::steady_clock::now();
    auto result = search(surface.lola(), latitude_rad, longitude_rad, approach_azimuth_rad, footprint_radius_m, p,
                         threads, [&](const LatLon& centre, TouchdownPoint& point) {
                             auto& memo = hazard_memo();
                             auto known = memo.find(surface, footprint_radius_m, p, hazards, centre);
                             if (known) {
                                 ++remembered;
                             } else {
                                 const auto t0 = std::chrono::steady_clock::now();
                                 known = check_hazards(surface, centre, point.height_m, footprint_radius_m, p, hazards);
                                 checked_us += std::chrono::duration_cast<std::chrono::microseconds>(
                                                   std::chrono::steady_clock::now() - t0).count();
                                 ++checked;
                                 memo.store(surface, footprint_radius_m, p, hazards, centre, *known);
                             }
                             const HazardResult& r = *known;
                             if (!r.admissible) return false;
                             point.height_m = r.height_m;
                             point.slope_rad = r.slope_rad;
                             point.max_foot_step_m = r.max_step_m;
                             point.min_bell_clearance_m = r.min_clearance_m;
                             return true;
                         });
    if (debug) {
        const double wall_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_start).count();
        std::printf("[touchdown] %d threads, %.1f ms: hazard checks %d (%.1f ms summed), %d from the memo: %s\n",
                    threads, wall_ms, checked.load(), checked_us.load() * 1e-3, remembered.load(),
                    result ? "found" : "refused");
    }
    return result;
}

double TouchdownRestPlane::slope_rad() const { return std::atan(std::hypot(a, b)); }

std::vector<TouchdownRestPlane> touchdown_rest_planes(const std::vector<math::Vec3>& feet_enz) {
    // Barycentric tolerance for the centre on a triangle's edge (between two opposite feet).
    constexpr double edge_tolerance = 1e-12;
    constexpr double tie_m = 1e-9;  // [E] plane heights at the centre this close are a tie; a foot this far above a
                                    // plane still counts as on it
    std::vector<TouchdownRestPlane> candidates;
    double highest = -std::numeric_limits<double>::infinity();
    const size_t count = feet_enz.size();
    for (size_t i = 0; i < count; ++i) {
        for (size_t j = i + 1; j < count; ++j) {
            for (size_t k = j + 1; k < count; ++k) {
                const auto& A = feet_enz[i];
                const auto& B = feet_enz[j];
                const auto& C = feet_enz[k];
                const double det = (B.x - A.x) * (C.y - A.y) - (C.x - A.x) * (B.y - A.y);
                if (std::abs(det) < 1e-12) continue;  // collinear in plan view
                // barycentric coordinates of the centre (0, 0)
                const double wb = ((-A.x) * (C.y - A.y) - (C.x - A.x) * (-A.y)) / det;
                const double wc = ((B.x - A.x) * (-A.y) - (-A.x) * (B.y - A.y)) / det;
                const double wa = 1.0 - wb - wc;
                if (wa < -edge_tolerance || wb < -edge_tolerance || wc < -edge_tolerance) continue;
                const math::Vec3 normal = cross(B - A, C - A);  // plane z = a e + b n + c through A, B, C
                const double a = -normal.x / normal.z, b = -normal.y / normal.z;
                candidates.push_back({a, b, A.z - a * A.x - b * A.y, 0.0});
                highest = std::max(highest, candidates.back().c);
            }
        }
    }
    // The envelope's facets at the centre: the highest planes there that support every foot (two when the centre lies
    // on the upper hull's edge between two opposite feet - one on each side of it).
    std::vector<TouchdownRestPlane> facets, tied;
    for (auto& plane : candidates) {
        if (plane.c < highest - tie_m) continue;
        bool supports = true;
        for (const auto& q : feet_enz) {
            const double below = plane.at(q.x, q.y) - q.z;
            supports = supports && below > -tie_m;
            plane.hang_m = std::max(plane.hang_m, below);
        }
        auto& into = supports ? facets : tied;
        const bool known = std::any_of(into.begin(), into.end(), [&](const TouchdownRestPlane& q) {
            return std::abs(q.a - plane.a) < tie_m && std::abs(q.b - plane.b) < tie_m && std::abs(q.c - plane.c) < tie_m;
        });  // coplanar feet give the same facet from several triangles
        if (!known) into.push_back(plane);
    }
    return facets.empty() ? tied : facets;  // rounding: the tied planes when none supports within tie_m
}

TouchdownHazardLimits touchdown_hazard_limits(const config::StageConfig& stage) {
    const auto& legs = stage.landing_legs;
    if (legs.azimuths_rad.empty() || !(legs.foot_pad_radius_m > 0.0) || stage.engine_groups.empty()) {
        throw std::invalid_argument("touchdown_hazard_limits: stage without landing legs, foot pads or engines");
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
