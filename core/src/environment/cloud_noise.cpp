// Tileable cloud noise textures (M8 weather and clouds); channels and the coverage rule in cloud_noise.hpp.

#include "starsim/environment/cloud_noise.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include "parallel_rows.hpp"

namespace starsim::environment {

namespace {

constexpr int channels = 4;  // RGBA8

// Seeds of the Worley channels (any fixed values: the textures are deterministic).
constexpr std::uint32_t shape_worley_seed = 0x1u;
constexpr std::uint32_t erosion_worley_seed = 0x2u;
constexpr std::uint32_t detail_worley_seed = 0x3u;

// Perlin fBm of the shape: lattice periods 8, 16, 32, 64 over the texture, gain 0.5 (Hillaire's TileableVolumeNoise).
constexpr int perlin_base_period = 8;
constexpr int perlin_octaves = 4;
// Worley fBm weights of the three octaves c, 2c, 4c (Schneider 2015 / TileableVolumeNoise).
constexpr double worley_weights[3] = {0.625, 0.25, 0.125};

// Bins of the rank equalisation of the Perlin-Worley channel.
constexpr int equalisation_bins = 1 << 16;

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

double perlin_fbm01(const math::Vec3& uv) {
    double sum = 0.0, amplitude = 1.0, total = 0.0;
    for (int o = 0; o < perlin_octaves; ++o) {
        const int period = perlin_base_period << o;
        sum += amplitude * tileable_perlin(uv * static_cast<double>(period), period);
        total += amplitude;
        amplitude *= 0.5;
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

// coverage_threshold[i]: t = (k + 0.5) / 255 with the code k whose count of texels above it is nearest to i / 255.
std::array<float, 256> coverage_thresholds(const std::vector<std::uint8_t>& rgba) {
    std::array<std::size_t, 256> counts{};
    for (std::size_t i = 0; i < rgba.size(); i += channels) ++counts[rgba[i]];
    std::array<std::size_t, 257> above{};  // above[k + 1] = texels with a code > k, k = -1..255
    for (int k = 254; k >= -1; --k) above[k + 1] = above[k + 2] + counts[k + 1];  // above[256] = 0
    const double n = static_cast<double>(rgba.size() / channels);
    std::array<float, 256> t{};
    for (int i = 0; i < 256; ++i) {
        const double target = i / 255.0;
        int best = 255;
        for (int k = 255; k >= -1; --k) {
            if (std::abs(above[k + 1] / n - target) < std::abs(above[best + 1] / n - target)) best = k;
        }
        t[static_cast<std::size_t>(i)] = static_cast<float>((best + 0.5) / 255.0);
    }
    return t;
}

}  // namespace

double tileable_perlin(const math::Vec3& p, int period) {
    const double fx = std::floor(p.x), fy = std::floor(p.y), fz = std::floor(p.z);
    const int ix = static_cast<int>(fx), iy = static_cast<int>(fy), iz = static_cast<int>(fz);
    const double x = p.x - fx, y = p.y - fy, z = p.z - fz;
    auto corner = [&](int dx, int dy, int dz) {
        const std::uint32_t h = hash_cell(wrap(ix + dx, period), wrap(iy + dy, period), wrap(iz + dz, period), 0u);
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

CloudNoise generate_cloud_noise() {
    CloudNoise noise;
    const int n = noise.shape_size, d = noise.detail_size;
    const std::size_t shape_texels = static_cast<std::size_t>(n) * n * n;
    noise.shape_rgba.assign(shape_texels * channels, 0);
    std::vector<float> perlin_worley(shape_texels);
    detail::for_each_row(n, [&](int z) {
        for (int y = 0; y < n; ++y) {
            for (int x = 0; x < n; ++x) {
                const std::size_t i = (static_cast<std::size_t>(z) * n + y) * n + x;
                const math::Vec3 uv = texel_uv(x, y, z, n);
                const double wf = worley_fbm(uv, 4, shape_worley_seed);
                perlin_worley[i] = static_cast<float>(wf + perlin_fbm01(uv) * (1.0 - wf));
                noise.shape_rgba[channels * i + 1] = to_byte(worley_fbm(uv, 4, erosion_worley_seed));
                noise.shape_rgba[channels * i + 2] = to_byte(worley_fbm(uv, 8, erosion_worley_seed));
                noise.shape_rgba[channels * i + 3] = to_byte(worley_fbm(uv, 16, erosion_worley_seed));
            }
        }
    });
    const std::vector<std::uint8_t> shape = rank_equalise(perlin_worley);
    for (std::size_t i = 0; i < shape_texels; ++i) noise.shape_rgba[channels * i] = shape[i];
    noise.coverage_threshold = coverage_thresholds(noise.shape_rgba);

    noise.detail_rgba.assign(static_cast<std::size_t>(d) * d * d * channels, 0);
    detail::for_each_row(d, [&](int z) {
        for (int y = 0; y < d; ++y) {
            for (int x = 0; x < d; ++x) {
                const std::size_t i = (static_cast<std::size_t>(z) * d + y) * d + x;
                const math::Vec3 uv = texel_uv(x, y, z, d);
                noise.detail_rgba[channels * i] = to_byte(worley_fbm(uv, 2, detail_worley_seed));
                noise.detail_rgba[channels * i + 1] = to_byte(worley_fbm(uv, 4, detail_worley_seed));
                noise.detail_rgba[channels * i + 2] = to_byte(worley_fbm(uv, 8, detail_worley_seed));
            }
        }
    });
    return noise;
}

}  // namespace starsim::environment
