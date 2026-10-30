#include "starsim/environment/terrain_horizon.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/math/constants.hpp"
#include "starsim/math/vec3.hpp"

namespace starsim::environment {

namespace {

using math::Vec3;

// [E] Growth of the march step per sample (2 % of the distance). Chosen so the march stays within 0.1 deg of a
// brute-force march with half-pixel steps on the LDEM_16 and the 20 m site tile (test_terrain_horizon.cpp).
constexpr double step_growth = 1.02;
// [E] Largest step: half the 1.9 km spacing of LDEM_16, so that far ridges are not stepped over.
constexpr double max_step_m = 1000.0;

// Local frame of a point on the unit sphere: up, east, north.
struct LocalFrame {
    Vec3 up, east, north;
};

LocalFrame local_frame(double latitude_rad, double longitude_rad) {
    const double cl = std::cos(latitude_rad), sl = std::sin(latitude_rad);
    const double co = std::cos(longitude_rad), so = std::sin(longitude_rad);
    return {Vec3{cl * co, cl * so, sl}, Vec3{-so, co, 0.0}, Vec3{-sl * co, -sl * so, cl}};
}

// The sample sequence of a march along a great circle, the same for every ray of a march: distance d_k from the
// observer (first at min_distance_m, steps from first_step_m growing by step_growth), cos / sin of the angular distance
// t_k = d_k / R, and c / s, 1 / s for the relief bound. Generated on demand (a ray stops early), by rotating
// (cos t, sin t) - no cos / sin per sample. Only samples with t < pi (and d <= max_distance_m when set) exist.
class MarchSequence {
public:
    struct Sample {
        double distance_m;
        double c, s;  // cos t, sin t
        double cot;   // c / s
        double csc;   // 1 / s
    };

    MarchSequence(double sphere_radius_m, const HorizonMarch& march)
        : inv_radius_(1.0 / sphere_radius_m),
          end_m_(march.max_distance_m > 0.0 ? march.max_distance_m : std::numeric_limits<double>::infinity()),
          distance_m_(march.min_distance_m > 0.0 ? march.min_distance_m : march.first_step_m),
          step_m_(march.first_step_m),
          c_(std::cos(distance_m_ * inv_radius_)),
          s_(std::sin(distance_m_ * inv_radius_)) {
        valid_ = is_valid();
        while (ensure(first_) && samples_[first_].distance_m < march.sample_from_m) ++first_;  // the band's start
    }

    size_t first() const { return first_; }
    // Makes sample k available; false when the sequence ends before it.
    bool ensure(size_t k) {
        while (samples_.size() <= k && valid_) generate();
        return k < samples_.size();
    }
    const Sample& operator[](size_t k) const { return samples_[k]; }

private:
    bool is_valid() const { return distance_m_ <= end_m_ && distance_m_ * inv_radius_ < constants::pi; }

    void generate() {
        samples_.push_back({distance_m_, c_, s_, c_ / s_, 1.0 / s_});
        distance_m_ += step_m_;
        const double dt = step_m_ * inv_radius_;
        double cd, sd;
        if (dt < 1e-3) {  // Taylor to dt^6 / dt^5: below 1e-17 for dt < 1e-3
            const double d2 = dt * dt;
            cd = 1.0 - d2 * (0.5 - d2 * (1.0 / 24.0 - d2 / 720.0));
            sd = dt * (1.0 - d2 * (1.0 / 6.0 - d2 / 120.0));
        } else {
            cd = std::cos(dt);
            sd = std::sin(dt);
        }
        const double c_next = c_ * cd - s_ * sd;
        s_ = s_ * cd + c_ * sd;
        c_ = c_next;
        step_m_ = std::min(step_m_ * step_growth, max_step_m);
        valid_ = is_valid();
    }

    double inv_radius_, end_m_, distance_m_, step_m_, c_, s_;
    bool valid_ = false;
    size_t first_ = 0;
    std::vector<Sample> samples_;
};

// The highest tan(elevation) over one ray's samples of `sequence` (the elevation of a sample at angular distance
// t < pi is atan2(r1 c - r0, r1 s) with s > 0, so the maximum is the maximum of (r1 c - r0) / (r1 s), kept as a
// fraction: no division per sample). Stops early when the relief bound (no terrain from the next sample on can be
// higher than r1_max allows) cannot beat the running maximum. -infinity without samples.
double march_ray(const Terrain& terrain, double sphere_radius_m, const Vec3& up, const Vec3& direction,
                 double height_m, MarchSequence& sequence) {
    const double r0 = sphere_radius_m + height_m;
    const double r1_max = sphere_radius_m + std::max(terrain.max_height_m(), 0.0);
    const double k_max = r0 / r1_max;
    // An observer above the relief bound sees the bound's sphere first rise to its tangent point: cos t > 1 / k_max.
    const double tangent_cos = k_max > 1.0 ? 1.0 / k_max : 2.0;
    const double tangent_tan = k_max > 1.0 ? -std::sqrt(k_max * k_max - 1.0) : 0.0;
    double best_n = -1.0, best_d = 0.0;  // -infinity
    for (size_t k = sequence.first(); sequence.ensure(k); ++k) {
        const MarchSequence::Sample& sample = sequence[k];
        if (best_d > 0.0) {
            // tan of the elevation of the bound's sphere at this sample: (r1 c - r0) / (r1 s) = c / s - k_max / s
            const double bound = sample.c > tangent_cos ? tangent_tan : sample.cot - k_max * sample.csc;
            if (bound * best_d <= best_n) break;
        }
        const double r1 = sphere_radius_m + terrain.height_at_direction_m(up * sample.c + direction * sample.s);
        const double n = r1 * sample.c - r0, d = r1 * sample.s;
        if (n * best_d > best_n * d) {
            best_n = n;
            best_d = d;
        }
    }
    return best_d > 0.0 ? best_n / best_d : -std::numeric_limits<double>::infinity();
}

double elevation_rad(double tan_elevation) { return std::atan(tan_elevation); }

}  // namespace

double horizon_elevation_rad(const Terrain& terrain, double sphere_radius_m, double latitude_rad, double longitude_rad,
                             double height_m, double azimuth_rad, const HorizonMarch& march) {
    const LocalFrame frame = local_frame(latitude_rad, longitude_rad);
    const Vec3 direction = frame.north * std::cos(azimuth_rad) + frame.east * std::sin(azimuth_rad);
    MarchSequence sequence(sphere_radius_m, march);
    return elevation_rad(march_ray(terrain, sphere_radius_m, frame.up, direction, height_m, sequence));
}

std::vector<float> horizon_elevations(const Terrain& terrain, double sphere_radius_m,
                                      const std::vector<double>& latitude_rad, const std::vector<double>& longitude_rad,
                                      const std::vector<double>& height_m, int azimuth_count,
                                      const HorizonMarch& march) {
    if (latitude_rad.size() != longitude_rad.size() || latitude_rad.size() != height_m.size() || azimuth_count < 1) {
        throw std::invalid_argument("horizon_elevations: point arrays differ in length or azimuth_count < 1");
    }
    std::vector<float> result(latitude_rad.size() * static_cast<size_t>(azimuth_count));
    MarchSequence sequence(sphere_radius_m, march);
    for (size_t i = 0; i < latitude_rad.size(); ++i) {
        const LocalFrame frame = local_frame(latitude_rad[i], longitude_rad[i]);
        for (int k = 0; k < azimuth_count; ++k) {
            const double azimuth = k * 2.0 * constants::pi / azimuth_count;
            const Vec3 direction = frame.north * std::cos(azimuth) + frame.east * std::sin(azimuth);
            result[i * static_cast<size_t>(azimuth_count) + static_cast<size_t>(k)] = static_cast<float>(
                elevation_rad(march_ray(terrain, sphere_radius_m, frame.up, direction, height_m[i], sequence)));
        }
    }
    return result;
}

namespace {

// [E] Azimuth shifts below this (rad, 0.006 deg) between a vertex and a stride vertex are not resampled.
constexpr double min_resampled_shift_rad = 1e-4;

// [E] Linear resampling of the azimuth profile (22.5 deg apart) stands in for the rotation of the local north between a
// vertex and a stride vertex; beyond this shift (rad, 1.1 deg) it loses too much, so the stride shrinks (near the poles
// the norths of neighbouring vertices turn quickly: up to 1.4 deg per vertex at 373 m spacing and 89.5 deg latitude).
constexpr double max_resampled_shift_rad = 0.05;

// [E] The distance bands of a lattice bake: from start_spacings (x the lattice spacing) on, every stride-th vertex is
// marched and the result interpolated to the others. An interpolation over a stride s at distance >= d errs by
// about e (s / d)^2 / 4 for an elevation e (more at a switch of the occluding ridge), so every band starts at d = 8 s.
struct BandSpec {
    int stride;
    double start_spacings;
};
constexpr BandSpec band_specs[] = {{1, 0.0}, {2, 16.0}, {4, 32.0}, {8, 64.0}, {16, 128.0}};
constexpr int max_stride = 16;  // n - 1 must be divisible by it

struct Lattice {
    int n = 0;
    double spacing_m = 0.0;
    int azimuth_count = 0;
    const std::vector<double>& latitude_rad;
    const std::vector<double>& longitude_rad;
    const std::vector<double>& height_m;
    std::vector<LocalFrame> frames;
    std::vector<double> azimuth_cos, azimuth_sin;
};

// Marches every stride-th vertex over the distance band [start_m, end_m] (end_m <= 0: to the relief bound) of its
// sample sequence (from one spacing, first step lattice_first_step_m, as horizon_elevations with that HorizonMarch);
// result row-major over the stride lattice, azimuth-minor, as elevations (rad).
std::vector<double> band_elevations(const Terrain& terrain, double sphere_radius_m, const Lattice& lat, int stride,
                                    double start_m, double end_m) {
    const int m = (lat.n - 1) / stride + 1;
    std::vector<double> band(static_cast<size_t>(m) * static_cast<size_t>(m) * static_cast<size_t>(lat.azimuth_count));
    size_t out = 0;
    // One sample sequence per first step (a vertex's step depends on the data resolution there, so a few differ).
    std::vector<std::pair<double, MarchSequence>> sequences;
    for (int I = 0; I < m; ++I) {
        for (int J = 0; J < m; ++J) {
            const size_t v = static_cast<size_t>(I * stride) * static_cast<size_t>(lat.n) +
                             static_cast<size_t>(J * stride);
            const LocalFrame& frame = lat.frames[v];
            const double first_step_m = lattice_first_step_m(terrain, lat.latitude_rad[v], lat.longitude_rad[v],
                                                             lat.spacing_m);
            auto found = std::find_if(sequences.begin(), sequences.end(),
                                      [&](const auto& entry) { return entry.first == first_step_m; });
            if (found == sequences.end()) {
                sequences.emplace_back(first_step_m, MarchSequence(sphere_radius_m,
                    HorizonMarch{lat.spacing_m, first_step_m, end_m, start_m}));
                found = sequences.end() - 1;
            }
            MarchSequence& sequence = found->second;
            for (int k = 0; k < lat.azimuth_count; ++k) {
                const Vec3 direction = frame.north * lat.azimuth_cos[static_cast<size_t>(k)] +
                                       frame.east * lat.azimuth_sin[static_cast<size_t>(k)];
                band[out++] = elevation_rad(
                    march_ray(terrain, sphere_radius_m, frame.up, direction, lat.height_m[v], sequence));
            }
        }
    }
    return band;
}

// The azimuth (rad) at vertex `to` of the direction that has azimuth 0 at vertex `from`, the direction carried along
// the short great circle between the two (the rotation of the local norths against each other).
double north_transport_angle(const LocalFrame& from, const LocalFrame& to) {
    const double cos_theta = std::clamp(dot(from.up, to.up), -1.0, 1.0);
    const Vec3 axis = cross(from.up, to.up);
    const double sin_theta = norm(axis);
    if (sin_theta < 1e-15) return 0.0;
    const Vec3 a = axis / sin_theta;
    const Vec3 north = from.north * cos_theta + cross(a, from.north) * sin_theta +
                       a * (dot(a, from.north) * (1.0 - cos_theta));
    return std::atan2(dot(north, to.east), dot(north, to.north));
}

// The azimuth profile (azimuth_count samples at k * 2 pi / azimuth_count, periodic) at azimuth_rad, linear.
double profile_at(const double* profile, int azimuth_count, double azimuth_rad) {
    const double x = azimuth_rad * azimuth_count / (2.0 * constants::pi);
    const double fl = std::floor(x);
    const double f = x - fl;
    const int i0 = ((static_cast<int>(fl) % azimuth_count) + azimuth_count) % azimuth_count;
    const int i1 = (i0 + 1) % azimuth_count;
    return profile[i0] * (1.0 - f) + profile[i1] * f;
}

// Bilinear interpolation of a stride-lattice band to every vertex, merged into result by maximum.
void merge_band(const Lattice& lat, const std::vector<double>& band, int stride, std::vector<double>& result) {
    const int n = lat.n, m = (n - 1) / stride + 1, az = lat.azimuth_count;
    std::vector<double> value(static_cast<size_t>(az));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const int I0 = std::min(i / stride, m - 2), J0 = std::min(j / stride, m - 2);
            const double fi = static_cast<double>(i - I0 * stride) / stride;
            const double fj = static_cast<double>(j - J0 * stride) / stride;
            const size_t v = static_cast<size_t>(i) * static_cast<size_t>(n) + static_cast<size_t>(j);
            std::fill(value.begin(), value.end(), 0.0);
            for (int di = 0; di < 2; ++di) {
                for (int dj = 0; dj < 2; ++dj) {
                    const double w = (di ? fi : 1.0 - fi) * (dj ? fj : 1.0 - fj);
                    if (w == 0.0) continue;  // also keeps an edge's values independent of the lattice's interior
                    const int I = I0 + di, J = J0 + dj;
                    const double* profile = &band[(static_cast<size_t>(I) * static_cast<size_t>(m) +
                                                   static_cast<size_t>(J)) * static_cast<size_t>(az)];
                    const size_t sv = static_cast<size_t>(I * stride) * static_cast<size_t>(n) +
                                      static_cast<size_t>(J * stride);
                    if (sv == v) {
                        for (int k = 0; k < az; ++k) value[static_cast<size_t>(k)] += w * profile[k];
                        continue;
                    }
                    const double alpha = north_transport_angle(lat.frames[v], lat.frames[sv]);
                    if (std::abs(alpha) < min_resampled_shift_rad) {
                        for (int k = 0; k < az; ++k) value[static_cast<size_t>(k)] += w * profile[k];
                        continue;
                    }
                    for (int k = 0; k < az; ++k) {
                        value[static_cast<size_t>(k)] +=
                            w * profile_at(profile, az, k * 2.0 * constants::pi / az + alpha);
                    }
                }
            }
            for (int k = 0; k < az; ++k) {
                double& r = result[v * static_cast<size_t>(az) + static_cast<size_t>(k)];
                r = std::max(r, value[static_cast<size_t>(k)]);
            }
        }
    }
}

}  // namespace

std::vector<float> horizon_elevations_lattice(const Terrain& terrain, double sphere_radius_m, int n,
                                              const std::vector<double>& latitude_rad,
                                              const std::vector<double>& longitude_rad,
                                              const std::vector<double>& height_m, double spacing_m,
                                              int azimuth_count) {
    const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n);
    if (n < 2 || (n - 1) % max_stride != 0 || latitude_rad.size() != count || longitude_rad.size() != count ||
        height_m.size() != count || azimuth_count < 1 || spacing_m <= 0.0) {
        throw std::invalid_argument("horizon_elevations_lattice: n - 1 not divisible by 16, array sizes differ from "
                                    "n * n, azimuth_count < 1 or spacing_m <= 0");
    }
    Lattice lat{n, spacing_m, azimuth_count, latitude_rad, longitude_rad, height_m, {}, {}, {}};
    lat.frames.reserve(count);
    for (size_t v = 0; v < count; ++v) lat.frames.push_back(local_frame(latitude_rad[v], longitude_rad[v]));
    for (int k = 0; k < azimuth_count; ++k) {
        lat.azimuth_cos.push_back(std::cos(k * 2.0 * constants::pi / azimuth_count));
        lat.azimuth_sin.push_back(std::sin(k * 2.0 * constants::pi / azimuth_count));
    }
    double north_turn_per_vertex_rad = 0.0;  // the largest turn of the local north between neighbouring vertices
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            const size_t v = static_cast<size_t>(i) * static_cast<size_t>(n) + static_cast<size_t>(j);
            if (j + 1 < n) {
                north_turn_per_vertex_rad = std::max(north_turn_per_vertex_rad,
                                                     std::abs(north_transport_angle(lat.frames[v], lat.frames[v + 1])));
            }
            if (i + 1 < n) {
                north_turn_per_vertex_rad = std::max(
                    north_turn_per_vertex_rad,
                    std::abs(north_transport_angle(lat.frames[v], lat.frames[v + static_cast<size_t>(n)])));
            }
        }
    }
    std::vector<double> result(count * static_cast<size_t>(azimuth_count), -std::numeric_limits<double>::infinity());
    const int band_count = static_cast<int>(sizeof(band_specs) / sizeof(band_specs[0]));
    for (int b = 0; b < band_count; ++b) {
        const double start_m = band_specs[b].start_spacings * spacing_m;
        const double end_m = b + 1 < band_count ? band_specs[b + 1].start_spacings * spacing_m : 0.0;
        int stride = band_specs[b].stride;
        while (stride > 1 && stride * north_turn_per_vertex_rad > max_resampled_shift_rad) stride /= 2;
        merge_band(lat, band_elevations(terrain, sphere_radius_m, lat, stride, start_m, end_m), stride, result);
    }
    return std::vector<float>(result.begin(), result.end());
}

}  // namespace starsim::environment
