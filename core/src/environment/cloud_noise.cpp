// Tileable cloud noise textures (M8 weather and clouds); channels and the coverage rule in cloud_noise.hpp.

#include "starsim/environment/cloud_noise.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "parallel_rows.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr int channels = 4;  // RGBA8

// Seeds of the Worley channels (any fixed values: the textures are deterministic).
constexpr std::uint32_t shape_worley_seed = 0x1u;
constexpr std::uint32_t erosion_worley_seed = 0x2u;
constexpr std::uint32_t detail_worley_seed = 0x3u;

// Perlin fBm of the shape: lattice periods 8, 16, 32, 64 over the texture (Hillaire's TileableVolumeNoise), the gain
// 2^-H of the fractal shapes (header).
constexpr int perlin_base_period = 8;
constexpr int perlin_octaves = 4;
// The edge-width noise (shape A, header): lattice periods 16, 32, 64, gain 0.5, its own gradient seed.
constexpr int width_noise_base_period = 16;
constexpr int width_noise_octaves = 3;
constexpr double width_noise_gain = 0.5;
constexpr std::uint32_t width_noise_seed = 0x6u;

// The composite tables (header): jittered samples per shape texel cell, fine bins per unit value (thresholds), joint
// bins per unit value x width-noise bins (edge mean factors), the sample seed and the chunks of the parallel pass.
constexpr int composite_samples_per_cell = 2;
constexpr double composite_bins_per_unit = 4096.0;
constexpr double joint_bins_per_unit = 2048.0;
constexpr int joint_width_bins = 16;
constexpr std::uint32_t composite_sample_seed = 0x7u;
constexpr int composite_chunks = 16;
// The shape prefilter's tables (header): uniform random positions per row (sampling error of a fraction <= 3.5e-4)
// and their seed.
constexpr int prefilter_samples = 2000000;
constexpr std::uint32_t prefilter_sample_seed = 0x9u;
// Increment statistics (detail amplitude, edge widths): sample pairs and seed.
constexpr int increment_pairs = 200000;
constexpr std::uint32_t increment_seed = 0x8u;
// The finest Perlin period of the shape [m] (the lag where its spectrum is joined, header).
constexpr double shape_finest_period_m = cloud_shape_tile_horizontal_m / (perlin_base_period << (perlin_octaves - 1));
// Worley fBm weights of the three octaves c, 2c, 4c (Schneider 2015 / TileableVolumeNoise).
constexpr double worley_weights[3] = {0.625, 0.25, 0.125};

// Bins of the rank equalisation of the Perlin-Worley channel.
constexpr int equalisation_bins = 1 << 16;

// Coverage table from the trilinearly filtered shape (cloud_noise.hpp): samples per texel cell (8 x 128^3 = 1.7e7),
// histogram bins per byte code (the codes k / 255 are bin edges; quantile resolution 1 / 4080), seed of the jittered
// sample positions, and the offset of a threshold from a code's point mass (above float rounding of the filter).
constexpr int filtered_samples_per_cell = 8;
constexpr std::size_t filtered_bins_per_code = 16;
constexpr std::uint32_t filtered_sample_seed = 0x4u;
constexpr double point_mass_offset = 1e-5;

// Integer hash (Wellons' lowbias32): platform-independent, so the textures are the same everywhere.
std::uint32_t hash32(std::uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

std::uint32_t hash_cell(int x, int y, int z, std::uint32_t seed) {
    return hash32(static_cast<std::uint32_t>(x) + hash32(static_cast<std::uint32_t>(y) +
                                                         hash32(static_cast<std::uint32_t>(z) + hash32(seed))));
}

int wrap(int i, int period) {
    const int m = i % period;
    return m < 0 ? m + period : m;
}

double unit_from_hash(std::uint32_t h) { return (h >> 8) * (1.0 / 16777216.0); }  // [0, 1), 24 bits

double fade(double t) { return t * t * t * (t * (t * 6.0 - 15.0) + 10.0); }

// Gradient of Perlin's improved noise (the 12 cube-edge directions) dotted with the offset.
double gradient_dot(std::uint32_t h, double x, double y, double z) {
    switch (h % 12u) {
        case 0: return x + y;
        case 1: return -x + y;
        case 2: return x - y;
        case 3: return -x - y;
        case 4: return x + z;
        case 5: return -x + z;
        case 6: return x - z;
        case 7: return -x - z;
        case 8: return y + z;
        case 9: return -y + z;
        case 10: return y - z;
        default: return -y - z;
    }
}

double lerp(double a, double b, double t) { return a + (b - a) * t; }

double worley_fbm(const math::Vec3& uv, int cells, std::uint32_t seed) {
    double sum = 0.0;
    for (int o = 0; o < 3; ++o) {
        const int c = cells << o;
        sum += worley_weights[o] * tileable_worley(uv * static_cast<double>(c), c, seed);
    }
    return sum;
}

// Perlin fBm of `octaves` lattice periods from base_period up (x2 each) with the amplitude gain per octave, in [0, 1].
double perlin_fbm01(const math::Vec3& uv, int base_period, int octaves, double gain, std::uint32_t seed) {
    double sum = 0.0, amplitude = 1.0, total = 0.0;
    for (int o = 0; o < octaves; ++o) {
        const int period = base_period << o;
        sum += amplitude * tileable_perlin(uv * static_cast<double>(period), period, seed);
        total += amplitude;
        amplitude *= gain;
    }
    return std::clamp(0.5 * (sum / total + 1.0), 0.0, 1.0);
}

std::uint8_t to_byte(double v) { return static_cast<std::uint8_t>(std::lround(std::clamp(v, 0.0, 1.0) * 255.0)); }

math::Vec3 texel_uv(int x, int y, int z, int size) {
    return {(x + 0.5) / size, (y + 0.5) / size, (z + 0.5) / size};
}

// Monotone remap of values to a uniform byte distribution (through a fine histogram's mid-bin CDF).
std::vector<std::uint8_t> rank_equalise(const std::vector<float>& values) {
    const auto [lo_it, hi_it] = std::minmax_element(values.begin(), values.end());
    const double lo = *lo_it, span = std::max(1e-12, static_cast<double>(*hi_it) - lo);
    auto bin_of = [&](float v) {
        return std::min(equalisation_bins - 1, static_cast<int>((v - lo) / span * equalisation_bins));
    };
    std::vector<std::size_t> counts(equalisation_bins, 0);
    for (float v : values) ++counts[static_cast<std::size_t>(bin_of(v))];
    std::vector<std::uint8_t> code_of_bin(equalisation_bins);
    std::size_t below = 0;
    const double n = static_cast<double>(values.size());
    for (int b = 0; b < equalisation_bins; ++b) {
        const double cdf = (below + 0.5 * counts[static_cast<std::size_t>(b)]) / n;
        code_of_bin[static_cast<std::size_t>(b)] = static_cast<std::uint8_t>(std::min(255.0, std::floor(cdf * 256.0)));
        below += counts[static_cast<std::size_t>(b)];
    }
    std::vector<std::uint8_t> codes(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) codes[i] = code_of_bin[static_cast<std::size_t>(bin_of(values[i]))];
    return codes;
}

// The distribution of the trilinearly filtered R channel: filtered_samples_per_cell jittered positions in every cell
// between texel centres (uniform over the torus). A cell whose 8 corners share one code filters to exactly that code:
// a point mass at k / 255, counted apart (counts[bins + k]). The other samples go into fine bins on [0, 1]
// (counts[0 .. bins)) and are taken as uniform inside a bin.
constexpr std::size_t filtered_bins = 255 * filtered_bins_per_code;

std::vector<double> filtered_shape_histogram(const std::vector<std::uint8_t>& rgba, int size) {
    const std::size_t bins = filtered_bins;
    // Per z row: bins continuous counts, then 256 point masses.
    std::vector<std::vector<std::uint32_t>> row_counts(static_cast<std::size_t>(size));
    detail::for_each_row(size, [&](int z) {
        std::vector<std::uint32_t>& counts = row_counts[static_cast<std::size_t>(z)];
        counts.assign(bins + 256, 0);
        auto at = [&](int x, int y, int zz) {
            const std::size_t i =
                (static_cast<std::size_t>(wrap(zz, size)) * size + wrap(y, size)) * size + wrap(x, size);
            return rgba[channels * i] / 255.0;
        };
        for (int y = 0; y < size; ++y) {
            for (int x = 0; x < size; ++x) {
                double corner[8];
                bool uniform = true;
                for (int c = 0; c < 8; ++c) {
                    corner[c] = at(x + (c & 1), y + ((c >> 1) & 1), z + (c >> 2));
                    uniform = uniform && corner[c] == corner[0];
                }
                if (uniform) {
                    counts[bins + static_cast<std::size_t>(std::lround(corner[0] * 255.0))] +=
                        filtered_samples_per_cell;
                    continue;
                }
                std::uint32_t h = hash_cell(x, y, z, filtered_sample_seed);
                for (int s = 0; s < filtered_samples_per_cell; ++s) {
                    h = hash32(h);
                    const double wx = unit_from_hash(h);
                    h = hash32(h);
                    const double wy = unit_from_hash(h);
                    h = hash32(h);
                    const double wz = unit_from_hash(h);
                    const double x00 = lerp(corner[0], corner[1], wx), x10 = lerp(corner[2], corner[3], wx);
                    const double x01 = lerp(corner[4], corner[5], wx), x11 = lerp(corner[6], corner[7], wx);
                    const double v = lerp(lerp(x00, x10, wy), lerp(x01, x11, wy), wz);
                    ++counts[std::min(bins - 1, static_cast<std::size_t>(v * bins))];
                }
            }
        }
    });
    std::vector<double> counts(bins + 256, 0.0);
    for (const auto& row : row_counts) {
        for (std::size_t b = 0; b < row.size(); ++b) counts[b] += row[b];
    }
    return counts;
}

// coverage_threshold[i]: the quantile of the filtered R channel with fraction i / 255 above it (header).
std::array<float, 256> coverage_thresholds(const std::vector<double>& counts) {
    const std::size_t bins = filtered_bins;
    double total = 0.0;
    for (double c : counts) total += c;
    std::array<float, 256> t{};
    t[0] = static_cast<float>(255.5 / 255.0);   // nothing above (filtered values are <= 1)
    t[255] = static_cast<float>(-0.5 / 255.0);  // everything above
    // Walk down from the top: above = samples > the current position. At a bin edge that is a code, its point mass
    // is passed first (a threshold just below the code includes it), then the bin below.
    std::size_t b = bins;
    bool edge_mass_passed = false;
    double above = 0.0;
    for (int i = 1; i < 255; ++i) {
        const double target = i / 255.0 * total;
        float threshold = 0.0f;
        while (true) {
            const double edge = static_cast<double>(b) / bins;
            if (b % filtered_bins_per_code == 0 && !edge_mass_passed) {
                const double mass = counts[bins + b / filtered_bins_per_code];
                if (above + mass >= target && mass > 0.0) {
                    // The target is inside the jump: exclude (just above the code) or include (just below), nearest.
                    threshold = static_cast<float>(target - above < above + mass - target ? edge + point_mass_offset
                                                                                         : edge - point_mass_offset);
                    break;
                }
                above += mass;
                edge_mass_passed = true;
            }
            if (b == 0) break;  // threshold 0: everything above but the zero mass
            const double in_bin = counts[b - 1];
            if (above + in_bin >= target && in_bin > 0.0) {
                threshold = static_cast<float>(edge - (target - above) / in_bin / bins);
                break;
            }
            above += in_bin;
            --b;
            edge_mass_passed = false;
        }
        // Monotone (a jump threshold sits point_mass_offset above its code, a bin quantile may sit closer).
        t[static_cast<std::size_t>(i)] = std::min(threshold, t[static_cast<std::size_t>(i - 1)]);
    }
    return t;
}

// Gaussian energy of a binary pattern on the torus (void-and-cluster): energy[p] = sum over set q of g(p - q).
class BlueNoiseEnergy {
public:
    explicit BlueNoiseEnergy(int size) : size_(size), kernel_(static_cast<std::size_t>(size) * size),
                                         energy_(static_cast<std::size_t>(size) * size, 0.0) {
        constexpr double sigma = 1.5;
        for (int dy = 0; dy < size; ++dy) {
            for (int dx = 0; dx < size; ++dx) {
                const int wx = std::min(dx, size - dx), wy = std::min(dy, size - dy);
                kernel_[static_cast<std::size_t>(dy) * size + dx] =
                    std::exp(-(wx * wx + wy * wy) / (2.0 * sigma * sigma));
            }
        }
    }
    void toggle(int p, double sign) {
        const int px = p % size_, py = p / size_;
        for (int y = 0; y < size_; ++y) {
            for (int x = 0; x < size_; ++x) {
                const int dx = wrap(x - px, size_), dy = wrap(y - py, size_);
                energy_[static_cast<std::size_t>(y) * size_ + x] += sign * kernel_[static_cast<std::size_t>(dy) * size_ + dx];
            }
        }
    }
    // The set pixel of the highest energy (tightest cluster) or the unset one of the lowest (largest void).
    int extreme(const std::vector<std::uint8_t>& set, bool tightest_cluster) const {
        int best = -1;
        for (int p = 0; p < size_ * size_; ++p) {
            if ((set[static_cast<std::size_t>(p)] != 0) != tightest_cluster) continue;
            const double e = energy_[static_cast<std::size_t>(p)];
            if (best < 0 || (tightest_cluster ? e > energy_[static_cast<std::size_t>(best)]
                                              : e < energy_[static_cast<std::size_t>(best)])) {
                best = p;
            }
        }
        return best;
    }

private:
    int size_;
    std::vector<double> kernel_, energy_;
};

}  // namespace

double cloud_edge_ramp(double x) {
    const double t = std::clamp(x, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

std::vector<std::uint8_t> generate_blue_noise(int size) {
    const int count = size * size;
    // Initial pattern: 10 % of the pixels from the hash, then relaxed (move the tightest cluster to the largest void
    // until that is the same pixel).
    std::vector<std::uint8_t> set(static_cast<std::size_t>(count), 0);
    BlueNoiseEnergy energy(size);
    const int initial = count / 10;
    std::uint32_t h = 0x5u;
    for (int placed = 0; placed < initial;) {
        h = hash32(h);
        const int p = static_cast<int>(h % static_cast<std::uint32_t>(count));
        if (set[static_cast<std::size_t>(p)]) continue;
        set[static_cast<std::size_t>(p)] = 1;
        energy.toggle(p, 1.0);
        ++placed;
    }
    for (int iteration = 0; iteration < count; ++iteration) {
        const int cluster = energy.extreme(set, true);
        set[static_cast<std::size_t>(cluster)] = 0;
        energy.toggle(cluster, -1.0);
        const int void_pixel = energy.extreme(set, false);
        set[static_cast<std::size_t>(void_pixel)] = 1;
        energy.toggle(void_pixel, 1.0);
        if (void_pixel == cluster) break;
    }
    std::vector<int> rank(static_cast<std::size_t>(count), 0);
    // Phase 1: the initial pattern's pixels ranked by removing the tightest cluster (rank initial - 1 down to 0).
    {
        std::vector<std::uint8_t> pattern = set;
        BlueNoiseEnergy e = energy;
        for (int r = initial - 1; r >= 0; --r) {
            const int cluster = e.extreme(pattern, true);
            pattern[static_cast<std::size_t>(cluster)] = 0;
            e.toggle(cluster, -1.0);
            rank[static_cast<std::size_t>(cluster)] = r;
        }
    }
    // Phases 2 and 3: fill the largest void, ranks initial .. count - 1.
    for (int r = initial; r < count; ++r) {
        const int void_pixel = energy.extreme(set, false);
        set[static_cast<std::size_t>(void_pixel)] = 1;
        energy.toggle(void_pixel, 1.0);
        rank[static_cast<std::size_t>(void_pixel)] = r;
    }
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(count));
    for (int p = 0; p < count; ++p) {
        mask[static_cast<std::size_t>(p)] = static_cast<std::uint8_t>(static_cast<long>(rank[static_cast<std::size_t>(p)]) * 256 / count);
    }
    return mask;
}

double tileable_perlin(const math::Vec3& p, int period, std::uint32_t seed) {
    const double fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy), iz = static_cast<int>(fz);
    const double x = p.x - fx, y = p.y - fy, z = p.z - fz;
    auto corner = [&](int dx, int dy, int dz) {
        const std::uint32_t h = hash_cell(wrap(ix + dx, period), wrap(iy + dy, period), wrap(iz + dz, period), seed);
        return gradient_dot(h, x - dx, y - dy, z - dz);
    };
    const double u = fade(x), v = fade(y), w = fade(z);
    const double x00 = lerp(corner(0, 0, 0), corner(1, 0, 0), u), x10 = lerp(corner(0, 1, 0), corner(1, 1, 0), u);
    const double x01 = lerp(corner(0, 0, 1), corner(1, 0, 1), u), x11 = lerp(corner(0, 1, 1), corner(1, 1, 1), u);
    return lerp(lerp(x00, x10, v), lerp(x01, x11, v), w);
}

double tileable_worley(const math::Vec3& p, int period, std::uint32_t seed) {
    const double fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy), iz = static_cast<int>(fz);
    const double x = p.x - fx, y = p.y - fy, z = p.z - fz;  // position inside the cell
    double nearest_sq = 3.0 * 4.0;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                std::uint32_t h = hash_cell(wrap(ix + dx, period), wrap(iy + dy, period), wrap(iz + dz, period), seed);
                const double px = dx + unit_from_hash(h);
                h = hash32(h);
                const double py = dy + unit_from_hash(h);
                h = hash32(h);
                const double pz = dz + unit_from_hash(h);
                const double ddx = px - x, ddy = py - y, ddz = pz - z;
                nearest_sq = std::min(nearest_sq, ddx * ddx + ddy * ddy + ddz * ddz);
            }
        }
    }
    return 1.0 - std::min(1.0, std::sqrt(nearest_sq));
}

namespace {

// All four channels of a GPU-style trilinear sample (sample_noise_trilinear).
std::array<double, 4> trilinear4(const std::vector<std::uint8_t>& rgba, int size, const math::Vec3& texel) {
    const double gx = texel.x - 0.5, gy = texel.y - 0.5, gz = texel.z - 0.5;
    const double fx = std::floor(gx), fy = std::floor(gy), fz = std::floor(gz);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy), iz = static_cast<int>(fz);
    const double w[3] = {gx - fx, gy - fy, gz - fz};
    std::array<double, 4> v{};
    for (int c = 0; c < 8; ++c) {
        const int dx = c & 1, dy = (c >> 1) & 1, dz = c >> 2;
        const double weight = (dx ? w[0] : 1.0 - w[0]) * (dy ? w[1] : 1.0 - w[1]) * (dz ? w[2] : 1.0 - w[2]);
        const std::size_t i =
            (static_cast<std::size_t>(wrap(iz + dz, size)) * size + wrap(iy + dy, size)) * size + wrap(ix + dx, size);
        for (int ch = 0; ch < 4; ++ch) v[static_cast<std::size_t>(ch)] += weight * rgba[channels * i + ch];
    }
    for (double& x : v) x /= 255.0;
    return v;
}

// Shape texel coordinates of an isotropic position [m] (the tiles' scales; the shader's anisotropic coordinates).
math::Vec3 shape_texel_of(const CloudNoise& noise, const math::Vec3& position_m) {
    const double n = noise.shape_size;
    return {position_m.x * n / cloud_shape_tile_horizontal_m, position_m.y * n / cloud_shape_tile_horizontal_m,
            position_m.z * n / cloud_shape_tile_vertical_m};
}

double detail_tile_of_level(int level) { return cloud_detail_tile_m / std::pow(cloud_detail_level_ratio, level); }

// Deterministic uniform numbers in [0, 1) from a hash chain.
struct HashSequence {
    std::uint32_t h;
    double next() {
        h = hash32(h);
        return unit_from_hash(h);
    }
};

// A random position in the shape tile [m] and a random horizontal unit direction (increment statistics).
struct IncrementPair {
    math::Vec3 position_m, direction;
};

IncrementPair increment_pair(int index) {
    HashSequence r{hash32(static_cast<std::uint32_t>(index) ^ hash32(increment_seed))};
    IncrementPair p;
    p.position_m = {r.next() * cloud_shape_tile_horizontal_m, r.next() * cloud_shape_tile_horizontal_m,
                    r.next() * cloud_shape_tile_vertical_m};
    const double angle = r.next() * constants::two_pi;
    p.direction = {std::cos(angle), std::sin(angle), 0.0};
    return p;
}

// Mean of f(pair) over the increment pairs, in parallel.
template <class F>
double increment_mean(F f) {
    std::vector<double> sums(static_cast<std::size_t>(composite_chunks), 0.0);
    detail::for_each_row(composite_chunks, [&](int chunk) {
        for (int i = chunk; i < increment_pairs; i += composite_chunks) sums[static_cast<std::size_t>(chunk)] += f(increment_pair(i));
    });
    double sum = 0.0;
    for (double s : sums) sum += s;
    return sum / increment_pairs;
}

// Horizontal increment variance (structure function) of the shape value at the lag [m].
double shape_structure_function(const CloudNoise& noise, double lag_m) {
    return increment_mean([&](const IncrementPair& p) {
        const double a = trilinear4(noise.shape_rgba, noise.shape_size, shape_texel_of(noise, p.position_m))[0];
        const double b = trilinear4(noise.shape_rgba, noise.shape_size,
                                    shape_texel_of(noise, p.position_m + p.direction * lag_m))[0];
        return (b - a) * (b - a);
    });
}

// Horizontal increment variance of the detail value at the lag [tiles].
double detail_structure_function(const CloudNoise& noise, double lag_tiles) {
    return increment_mean([&](const IncrementPair& p) {
        const math::Vec3 q = p.position_m / cloud_detail_tile_m;
        const double a = cloud_detail_value(noise, q), b = cloud_detail_value(noise, q + p.direction * lag_tiles);
        return (b - a) * (b - a);
    });
}

// The fractal shape value of all levels at a position [m].
double full_shape_value(const CloudNoise& noise, const math::Vec3& position_m) {
    const double s = trilinear4(noise.shape_rgba, noise.shape_size, shape_texel_of(noise, position_m))[0];
    return cloud_shape_value(noise, s, position_m, cloud_detail_levels);
}

// Median |v(p + lag e) - v(p)| of the fractal value of all levels (the edge widths, header).
double median_increment(const CloudNoise& noise, double lag_m) {
    std::vector<float> values(static_cast<std::size_t>(increment_pairs));
    detail::for_each_row(composite_chunks, [&](int chunk) {
        for (int i = chunk; i < increment_pairs; i += composite_chunks) {
            const IncrementPair p = increment_pair(i);
            values[static_cast<std::size_t>(i)] = static_cast<float>(
                std::abs(full_shape_value(noise, p.position_m + p.direction * lag_m) - full_shape_value(noise, p.position_m)));
        }
    });
    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());
    return *middle;
}

// Histograms of the fractal value per detail state (header, coverage rule and soft edges): fine (thresholds) and joint
// with the width noise (edge mean factors), over [lo, hi].
struct CompositeHistograms {
    double lo = 0.0, hi = 1.0;
    std::size_t fine_bins = 0, joint_bins = 0;
    std::vector<double> fine;   // [state][fine bin]
    std::vector<double> joint;  // [state][joint bin][width bin]
};

CompositeHistograms composite_histograms(const CloudNoise& noise) {
    CompositeHistograms h;
    double reach = 0.0;  // largest |perturbation|
    for (float a : noise.level_amplitude) reach += a * std::max(noise.detail_mean, 1.0f - noise.detail_mean);
    h.lo = -reach;
    h.hi = 1.0 + reach;
    h.fine_bins = static_cast<std::size_t>(std::ceil((h.hi - h.lo) * composite_bins_per_unit));
    h.joint_bins = static_cast<std::size_t>(std::ceil((h.hi - h.lo) * joint_bins_per_unit));
    const std::size_t fine_size = cloud_detail_states * h.fine_bins;
    const std::size_t joint_size = cloud_detail_states * h.joint_bins * joint_width_bins;
    std::vector<std::vector<std::uint32_t>> fine(composite_chunks), joint(composite_chunks);
    const int n = noise.shape_size;
    const double cell_m[3] = {cloud_shape_tile_horizontal_m / n, cloud_shape_tile_horizontal_m / n,
                              cloud_shape_tile_vertical_m / n};
    detail::for_each_row(composite_chunks, [&](int chunk) {
        std::vector<std::uint32_t>& f = fine[static_cast<std::size_t>(chunk)];
        std::vector<std::uint32_t>& j = joint[static_cast<std::size_t>(chunk)];
        f.assign(fine_size, 0);
        j.assign(joint_size, 0);
        for (int z = chunk; z < n; z += composite_chunks) {
            for (int y = 0; y < n; ++y) {
                for (int x = 0; x < n; ++x) {
                    HashSequence r{hash_cell(x, y, z, composite_sample_seed)};
                    for (int s = 0; s < composite_samples_per_cell; ++s) {
                        // A uniform position between texel centres (the cell of the trilinear filter).
                        const math::Vec3 position_m{(x + 0.5 + r.next()) * cell_m[0], (y + 0.5 + r.next()) * cell_m[1],
                                                    (z + 0.5 + r.next()) * cell_m[2]};
                        const std::array<double, 4> shape =
                            trilinear4(noise.shape_rgba, n, shape_texel_of(noise, position_m));
                        const std::size_t width_bin = std::min<std::size_t>(
                            joint_width_bins - 1, static_cast<std::size_t>(shape[3] * joint_width_bins));
                        double v = shape[0];
                        for (int state = 0; state < cloud_detail_states; ++state) {
                            if (state > 0) {
                                const int k = state - 1;
                                v += noise.level_amplitude[static_cast<std::size_t>(k)] *
                                     (cloud_detail_value(noise, position_m / detail_tile_of_level(k)) - noise.detail_mean);
                            }
                            const double u = v - h.lo;
                            const std::size_t fb = std::min(h.fine_bins - 1, static_cast<std::size_t>(u * composite_bins_per_unit));
                            const std::size_t jb = std::min(h.joint_bins - 1, static_cast<std::size_t>(u * joint_bins_per_unit));
                            ++f[static_cast<std::size_t>(state) * h.fine_bins + fb];
                            ++j[(static_cast<std::size_t>(state) * h.joint_bins + jb) * joint_width_bins + width_bin];
                        }
                    }
                }
            }
        }
    });
    h.fine.assign(fine_size, 0.0);
    h.joint.assign(joint_size, 0.0);
    for (int c = 0; c < composite_chunks; ++c) {
        for (std::size_t i = 0; i < fine_size; ++i) h.fine[i] += fine[static_cast<std::size_t>(c)][i];
        for (std::size_t i = 0; i < joint_size; ++i) h.joint[i] += joint[static_cast<std::size_t>(c)][i];
    }
    return h;
}

// Thresholds of a state from its fine histogram (continuous bins, quantiles interpolated inside a bin).
std::array<float, 256> quantile_thresholds(const CompositeHistograms& h, int state) {
    const double* counts = h.fine.data() + static_cast<std::size_t>(state) * h.fine_bins;
    double total = 0.0;
    for (std::size_t b = 0; b < h.fine_bins; ++b) total += counts[b];
    std::array<float, 256> t{};
    t[0] = static_cast<float>(h.hi + 0.5 / 255.0);
    t[255] = static_cast<float>(h.lo - 0.5 / 255.0);
    std::size_t b = h.fine_bins;
    double above = 0.0;
    for (int i = 1; i < 255; ++i) {
        const double target = i / 255.0 * total;
        while (b > 1 && above + counts[b - 1] < target) above += counts[--b];
        const double in_bin = std::max(counts[b - 1], 1e-30);
        const double top = h.lo + static_cast<double>(b) / composite_bins_per_unit;
        const double threshold = top - std::clamp((target - above) / in_bin, 0.0, 1.0) / composite_bins_per_unit;
        t[static_cast<std::size_t>(i)] = std::min(static_cast<float>(threshold), t[static_cast<std::size_t>(i - 1)]);
    }
    return t;
}

// Edge mean factors of a row (detail state or prefilter row) from its joint histogram and thresholds t (header): bins
// at their centres.
std::array<float, 256> joint_edge_mean_factors(const CloudNoise& noise, const CompositeHistograms& h, int state,
                                               const std::array<float, 256>& t) {
    const double* counts = h.joint.data() + static_cast<std::size_t>(state) * h.joint_bins * joint_width_bins;
    double total = 0.0;
    for (std::size_t i = 0; i < h.joint_bins * joint_width_bins; ++i) total += counts[i];
    std::array<double, joint_width_bins> width{};
    for (int k = 0; k < joint_width_bins; ++k) width[static_cast<std::size_t>(k)] = cloud_edge_width_value(noise, (k + 0.5) / joint_width_bins);
    std::array<float, 256> m{};
    detail::for_each_row(256, [&](int i) {
        const double threshold = t[static_cast<std::size_t>(i)];
        double sum = 0.0;
        for (std::size_t b = 0; b < h.joint_bins; ++b) {
            const double v = h.lo + (b + 0.5) / joint_bins_per_unit;
            if (v <= threshold) continue;
            for (int k = 0; k < joint_width_bins; ++k) {
                const double c = counts[b * joint_width_bins + static_cast<std::size_t>(k)];
                if (c > 0.0) sum += c * cloud_edge_ramp((v - threshold) / width[static_cast<std::size_t>(k)]);
            }
        }
        m[static_cast<std::size_t>(i)] = static_cast<float>(std::min(sum / total, i / 255.0));
    });
    return m;
}

double table_at(const std::array<std::array<float, 256>, cloud_detail_states>& table, double detail_state, int code) {
    const double s = std::clamp(detail_state, 0.0, static_cast<double>(cloud_detail_levels));
    const int lo = static_cast<int>(std::floor(s));
    const int hi = std::min(lo + 1, cloud_detail_levels);
    const std::size_t c = static_cast<std::size_t>(std::clamp(code, 0, 255));
    return lerp(table[static_cast<std::size_t>(lo)][c], table[static_cast<std::size_t>(hi)][c], s - lo);
}

// Box-filtered mip chain of a size^3 RGBA8 texture (header, shape prefilter): levels 1 .. log2(size), each texel the
// rounded mean of the 2 x 2 x 2 texels below it (GPU mip convention: level l texel centres at level-0 coordinates
// / 2^l).
std::vector<std::vector<std::uint8_t>> mip_chain(const std::vector<std::uint8_t>& rgba, int size) {
    std::vector<std::vector<std::uint8_t>> mips;
    for (int s = size / 2; s >= 1; s /= 2) {
        const std::vector<std::uint8_t>& source = mips.empty() ? rgba : mips.back();
        const int from = 2 * s;
        std::vector<std::uint8_t> level(static_cast<std::size_t>(s) * s * s * channels);
        for (int z = 0; z < s; ++z) {
            for (int y = 0; y < s; ++y) {
                for (int x = 0; x < s; ++x) {
                    for (int ch = 0; ch < channels; ++ch) {
                        int sum = 0;
                        for (int c = 0; c < 8; ++c) {
                            const std::size_t i = (static_cast<std::size_t>(2 * z + (c >> 2)) * from + 2 * y + ((c >> 1) & 1)) *
                                                      from + 2 * x + (c & 1);
                            sum += source[channels * i + static_cast<std::size_t>(ch)];
                        }
                        level[channels * ((static_cast<std::size_t>(z) * s + y) * s + x) + static_cast<std::size_t>(ch)] =
                            static_cast<std::uint8_t>((sum + 4) / 8);
                    }
                }
            }
        }
        mips.push_back(std::move(level));
    }
    return mips;
}

// All four channels of the shape at level-0 texel coordinates and the prefilter level (sample_shape_prefiltered).
std::array<double, 4> prefiltered4(const CloudNoise& noise, const math::Vec3& texel, double level) {
    const auto at = [&](int l) {
        if (l == 0) return trilinear4(noise.shape_rgba, noise.shape_size, texel);
        return trilinear4(noise.shape_mips[static_cast<std::size_t>(l - 1)], noise.shape_size >> l,
                          texel / static_cast<double>(1 << l));
    };
    const double lv = std::clamp(level, 0.0, static_cast<double>(cloud_shape_prefilter_levels));
    const int lo = static_cast<int>(std::floor(lv));
    if (lv == lo) return at(lo);
    const std::array<double, 4> a = at(lo), b = at(lo + 1);
    std::array<double, 4> v{};
    for (std::size_t ch = 0; ch < 4; ++ch) v[ch] = lerp(a[ch], b[ch], lv - lo);
    return v;
}

// Histograms of the prefiltered shape value per row (header; row 0 unused: state 0's tables): fine and joint with the
// width noise, over [0, 1], at uniform random positions.
CompositeHistograms prefilter_histograms(const CloudNoise& noise) {
    CompositeHistograms h;
    h.lo = 0.0;
    h.hi = 1.0;
    h.fine_bins = static_cast<std::size_t>(composite_bins_per_unit);
    h.joint_bins = static_cast<std::size_t>(joint_bins_per_unit);
    const std::size_t rows = cloud_shape_prefilter_rows;
    const std::size_t fine_size = rows * h.fine_bins, joint_size = rows * h.joint_bins * joint_width_bins;
    std::vector<std::vector<std::uint32_t>> fine(composite_chunks), joint(composite_chunks);
    const double n = noise.shape_size;
    detail::for_each_row(composite_chunks, [&](int chunk) {
        std::vector<std::uint32_t>& f = fine[static_cast<std::size_t>(chunk)];
        std::vector<std::uint32_t>& j = joint[static_cast<std::size_t>(chunk)];
        f.assign(fine_size, 0);
        j.assign(joint_size, 0);
        for (int i = chunk; i < prefilter_samples; i += composite_chunks) {
            HashSequence r{hash32(static_cast<std::uint32_t>(i) ^ hash32(prefilter_sample_seed))};
            const math::Vec3 texel{r.next() * n, r.next() * n, r.next() * n};
            for (std::size_t row = 1; row < rows; ++row) {
                const std::array<double, 4> v =
                    prefiltered4(noise, texel, static_cast<double>(row) / cloud_shape_prefilter_rows_per_level);
                const std::size_t width_bin =
                    std::min<std::size_t>(joint_width_bins - 1, static_cast<std::size_t>(v[3] * joint_width_bins));
                const std::size_t fb = std::min(h.fine_bins - 1, static_cast<std::size_t>(v[0] * composite_bins_per_unit));
                const std::size_t jb = std::min(h.joint_bins - 1, static_cast<std::size_t>(v[0] * joint_bins_per_unit));
                ++f[row * h.fine_bins + fb];
                ++j[(row * h.joint_bins + jb) * joint_width_bins + width_bin];
            }
        }
    });
    h.fine.assign(fine_size, 0.0);
    h.joint.assign(joint_size, 0.0);
    for (int c = 0; c < composite_chunks; ++c) {
        for (std::size_t i = 0; i < fine_size; ++i) h.fine[i] += fine[static_cast<std::size_t>(c)][i];
        for (std::size_t i = 0; i < joint_size; ++i) h.joint[i] += joint[static_cast<std::size_t>(c)][i];
    }
    return h;
}

double prefilter_table_at(const std::array<std::array<float, 256>, cloud_shape_prefilter_rows>& table, double level,
                          int code) {
    const double s = std::clamp(level * cloud_shape_prefilter_rows_per_level, 0.0,
                                static_cast<double>(cloud_shape_prefilter_rows - 1));
    const int lo = static_cast<int>(std::floor(s));
    const int hi = std::min(lo + 1, cloud_shape_prefilter_rows - 1);
    const std::size_t c = static_cast<std::size_t>(std::clamp(code, 0, 255));
    return lerp(table[static_cast<std::size_t>(lo)][c], table[static_cast<std::size_t>(hi)][c], s - lo);
}

}  // namespace

double cloud_shape_prefilter_level(double footprint_m, int shape_size) {
    if (!(footprint_m > cloud_shape_prefilter_start_m)) return 0.0;
    const double onset = std::min(std::log2(footprint_m / cloud_shape_prefilter_start_m), 1.0);
    const double texel_m = cloud_shape_tile_horizontal_m / shape_size;
    return std::clamp(onset * std::log2(footprint_m / texel_m), 0.0, static_cast<double>(cloud_shape_prefilter_levels));
}

double sample_shape_prefiltered(const CloudNoise& noise, int channel, const math::Vec3& texel, double level) {
    return prefiltered4(noise, texel, level)[static_cast<std::size_t>(std::clamp(channel, 0, 3))];
}

double cloud_prefilter_threshold(const CloudNoise& noise, double level, int code) {
    return prefilter_table_at(noise.prefilter_threshold, level, code);
}

double cloud_prefilter_edge_mean(const CloudNoise& noise, double level, int code) {
    return prefilter_table_at(noise.prefilter_edge_mean, level, code);
}

double cloud_shape_hurst_exponent() { return 2.0 - constants::cloud_perimeter_fractal_dimension; }

double cloud_edge_width_value(const CloudNoise& noise, double width_noise) {
    const double n = std::clamp(width_noise, 0.0, 1.0);
    return noise.edge_width_min_value * std::pow(noise.edge_width_max_value / noise.edge_width_min_value, n);
}

double cloud_detail_level_weight(int level, double footprint_m) {
    if (!(footprint_m > 0.0)) return 1.0;
    const double middle_cell_m = detail_tile_of_level(level) / 4.0;
    return std::clamp(std::log2(middle_cell_m / footprint_m) - 1.0, 0.0, 1.0);
}

double cloud_detail_state(double footprint_m, int max_levels) {
    double state = 0.0;
    for (int k = 0; k < std::min(max_levels, cloud_detail_levels); ++k) state += cloud_detail_level_weight(k, footprint_m);
    return state;
}

double sample_noise_trilinear(const std::vector<std::uint8_t>& rgba, int size, int channel, const math::Vec3& texel) {
    return trilinear4(rgba, size, texel)[static_cast<std::size_t>(std::clamp(channel, 0, 3))];
}

double cloud_detail_value(const CloudNoise& noise, const math::Vec3& tile) {
    const std::array<double, 4> d = trilinear4(noise.detail_rgba, noise.detail_size, tile * static_cast<double>(noise.detail_size));
    return noise.detail_weights[0] * d[0] + noise.detail_weights[1] * d[1] + noise.detail_weights[2] * d[2];
}

double cloud_shape_value(const CloudNoise& noise, double shape_value, const math::Vec3& position_m,
                         double detail_state) {
    double v = shape_value;
    for (int k = 0; k < cloud_detail_levels; ++k) {
        const double w = std::clamp(detail_state - k, 0.0, 1.0);
        if (w <= 0.0) break;
        v += w * noise.level_amplitude[static_cast<std::size_t>(k)] *
             (cloud_detail_value(noise, position_m / detail_tile_of_level(k)) - noise.detail_mean);
    }
    return v;
}

double cloud_coverage_threshold(const CloudNoise& noise, double detail_state, int code) {
    return table_at(noise.coverage_threshold, detail_state, code);
}

double cloud_edge_mean_factor(const CloudNoise& noise, double detail_state, int code) {
    return table_at(noise.edge_mean_factor, detail_state, code);
}

CloudNoise generate_cloud_noise() {
    CloudNoise noise;
    const int n = noise.shape_size, d = noise.detail_size;
    const double hurst = cloud_shape_hurst_exponent();
    const std::size_t shape_texels = static_cast<std::size_t>(n) * n * n;
    noise.shape_rgba.assign(shape_texels * channels, 0);
    std::vector<float> perlin_worley(shape_texels), width_noise(shape_texels);
    detail::for_each_row(n, [&](int z) {
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const std::size_t i = (static_cast<std::size_t>(z) * n + y) * n + x;
                const math::Vec3 uv = texel_uv(x, y, z, n);
                const double wf = worley_fbm(uv, 4, shape_worley_seed);
                const double perlin = perlin_fbm01(uv, perlin_base_period, perlin_octaves, std::pow(2.0, -hurst), 0u);
                perlin_worley[i] = static_cast<float>(wf + perlin * (1.0 - wf));
                width_noise[i] = static_cast<float>(
                    perlin_fbm01(uv, width_noise_base_period, width_noise_octaves, width_noise_gain, width_noise_seed));
                noise.shape_rgba[channels * i + 1] = to_byte(worley_fbm(uv, 4, erosion_worley_seed));
                noise.shape_rgba[channels * i + 2] = to_byte(worley_fbm(uv, 8, erosion_worley_seed));
            }
        }
    });
    const std::vector<std::uint8_t> shape = rank_equalise(perlin_worley);
    const std::vector<std::uint8_t> width = rank_equalise(width_noise);
    for (std::size_t i = 0; i < shape_texels; ++i) {
        noise.shape_rgba[channels * i] = shape[i];
        noise.shape_rgba[channels * i + 3] = width[i];
    }
    noise.blue_noise = generate_blue_noise(noise.blue_noise_size);

    noise.detail_rgba.assign(static_cast<std::size_t>(d) * d * d * channels, 0);
    detail::for_each_row(d, [&](int z) {
        for (int y = 0; y < d; ++y) {
            for (int x = 0; x < d; ++x) {
                const std::size_t i = (static_cast<std::size_t>(z) * d + y) * d + x;
                const math::Vec3 uv = texel_uv(x, y, z, d);
                noise.detail_rgba[channels * i] = to_byte(tileable_worley(uv * 2.0, 2, detail_worley_seed));
                noise.detail_rgba[channels * i + 1] = to_byte(tileable_worley(uv * 4.0, 4, detail_worley_seed));
                noise.detail_rgba[channels * i + 2] = to_byte(tileable_worley(uv * 8.0, 8, detail_worley_seed));
            }
        }
    });
    // The detail value's octaves (cells T / 2, T / 4, T / 8) with the fractal gain 2^-H, normalised.
    const double gain = std::pow(2.0, -hurst), norm = 1.0 + gain + gain * gain;
    noise.detail_weights = {static_cast<float>(1.0 / norm), static_cast<float>(gain / norm),
                            static_cast<float>(gain * gain / norm)};
    double detail_sum = 0.0;
    for (std::size_t i = 0; i < noise.detail_rgba.size(); i += channels) {
        for (int c = 0; c < 3; ++c) detail_sum += noise.detail_weights[static_cast<std::size_t>(c)] * noise.detail_rgba[i + static_cast<std::size_t>(c)];
    }
    noise.detail_mean = static_cast<float>(detail_sum / 255.0 / (noise.detail_rgba.size() / channels));

    // The detail amplitude: the shape's spectrum continued as fBm (header).
    const double r0 = shape_finest_period_m, r = cloud_detail_join_lag_m;
    const double target = shape_structure_function(noise, r0) * std::pow(r / r0, 2.0 * hurst);
    double per_unit = 0.0;  // the levels' increment variance per A^2
    for (int k = 0; k < cloud_detail_levels; ++k) {
        per_unit += std::pow(cloud_detail_level_ratio, -2.0 * hurst * k) *
                    detail_structure_function(noise, r / detail_tile_of_level(k));
    }
    const double amplitude = cloud_detail_amplitude_calibration *
                             std::sqrt(std::max(target - shape_structure_function(noise, r), 0.0) / per_unit);
    for (int k = 0; k < cloud_detail_levels; ++k) {
        noise.level_amplitude[static_cast<std::size_t>(k)] =
            static_cast<float>(amplitude * std::pow(cloud_detail_level_ratio, -hurst * k));
    }
    noise.edge_width_min_value = static_cast<float>(median_increment(noise, constants::cloud_edge_width_min_m));
    noise.edge_width_max_value = static_cast<float>(median_increment(noise, constants::cloud_edge_width_max_m));

    // The tables per detail state: state 0 exactly from the filtered shape (point masses), the others from the
    // composite's samples; the edge mean factors of all states from the joint samples.
    noise.coverage_threshold[0] = coverage_thresholds(filtered_shape_histogram(noise.shape_rgba, n));
    const CompositeHistograms histograms = composite_histograms(noise);
    for (int state = 1; state < cloud_detail_states; ++state) {
        noise.coverage_threshold[static_cast<std::size_t>(state)] = quantile_thresholds(histograms, state);
    }
    for (int state = 0; state < cloud_detail_states; ++state) {
        noise.edge_mean_factor[static_cast<std::size_t>(state)] =
            joint_edge_mean_factors(noise, histograms, state, noise.coverage_threshold[static_cast<std::size_t>(state)]);
    }

    // The shape prefilter (header): the mip chain, row 0 = state 0, the other rows from their blended values.
    noise.shape_mips = mip_chain(noise.shape_rgba, n);
    noise.prefilter_threshold[0] = noise.coverage_threshold[0];
    noise.prefilter_edge_mean[0] = noise.edge_mean_factor[0];
    const CompositeHistograms prefiltered = prefilter_histograms(noise);
    for (int row = 1; row < cloud_shape_prefilter_rows; ++row) {
        const std::size_t r = static_cast<std::size_t>(row);
        noise.prefilter_threshold[r] = quantile_thresholds(prefiltered, row);
        noise.prefilter_edge_mean[r] = joint_edge_mean_factors(noise, prefiltered, row, noise.prefilter_threshold[r]);
    }
    return noise;
}

}  // namespace starsim::environment
