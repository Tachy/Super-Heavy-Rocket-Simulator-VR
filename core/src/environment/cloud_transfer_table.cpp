// Monte Carlo transfer table of the multiply scattered cloud light; layout and derivation in cloud_transfer_table.hpp.

#include "starsim/environment/cloud_transfer_table.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>

#include "starsim/math/constants.hpp"

namespace starsim::environment {

namespace {

constexpr char table_magic[8] = {'S', 'T', 'C', 'L', 'D', 'T', 'T', '2'};
constexpr std::size_t header_bytes = 256;

template <class T>
T read_le(const unsigned char* p) {
    T value;
    std::memcpy(&value, p, sizeof(T));  // the generator writes little endian; so do the supported targets
    return value;
}

// Fractional coordinate split into the lower node (clamped so that node + 1 exists) and the weight of the upper one.
struct Bracket {
    int index;
    double weight;
};

Bracket bracket(double coordinate, int nodes) {
    const double u = std::clamp(coordinate, 0.0, static_cast<double>(nodes - 1));
    const int i = std::min(static_cast<int>(u), nodes - 2);
    return {i, u - i};
}

}  // namespace

float half_to_float(std::uint16_t half) {
    const std::uint32_t sign = static_cast<std::uint32_t>(half & 0x8000u) << 16;
    const int exponent = (half >> 10) & 0x1f;
    const std::uint32_t mantissa = half & 0x3ffu;
    if (exponent == 0) {  // zero or subnormal: mantissa x 2^-24
        const float v = std::ldexp(static_cast<float>(mantissa), -24);
        return sign ? -v : v;
    }
    std::uint32_t bits = 0;
    if (exponent == 31) {
        bits = sign | 0x7f800000u | (mantissa << 13);
    } else {
        bits = sign | (static_cast<std::uint32_t>(exponent - 15 + 127) << 23) | (mantissa << 13);
    }
    float out;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

CloudTransferTable load_cloud_transfer_table(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) throw std::runtime_error("cloud transfer table not found: " + file.string());
    std::vector<unsigned char> head(header_bytes);
    in.read(reinterpret_cast<char*>(head.data()), static_cast<std::streamsize>(head.size()));
    if (in.gcount() != static_cast<std::streamsize>(header_bytes) || std::memcmp(head.data(), table_magic, 8) != 0) {
        throw std::runtime_error("not a cloud transfer table: " + file.string());
    }
    CloudTransferTable t;
    const unsigned char* p = head.data() + 8;
    t.depth_nodes = static_cast<int>(read_le<std::uint32_t>(p));
    t.mu_nodes = static_cast<int>(read_le<std::uint32_t>(p + 4));
    t.theta_nodes = static_cast<int>(read_le<std::uint32_t>(p + 8));
    t.psi_nodes = static_cast<int>(read_le<std::uint32_t>(p + 12));
    p += 16;
    t.depth_scale = read_le<double>(p);
    t.depth_ratio = read_le<double>(p + 8);
    t.diffusion_k = read_le<double>(p + 16);
    t.theta0 = read_le<double>(p + 24);
    t.theta_log = read_le<double>(p + 32);
    t.theta_linear = read_le<double>(p + 40);
    t.draine_weight = read_le<double>(p + 48);
    p += 56;
    t.photons = read_le<std::uint64_t>(p);
    p += 8;
    const unsigned char* head_end = head.data() + header_bytes;
    const unsigned char* text_end = std::find(p, head_end, static_cast<unsigned char>(0));
    t.command.assign(reinterpret_cast<const char*>(p), static_cast<std::size_t>(text_end - p));
    if (t.depth_nodes < 3 || t.mu_nodes < 2 || t.theta_nodes < 2 || t.psi_nodes < 2 ||
        !(t.draine_weight > 0.0 && t.draine_weight <= 1.0)) {
        throw std::runtime_error("cloud transfer table has too few nodes or no Draine weight: " + file.string());
    }
    const std::size_t count = static_cast<std::size_t>(t.depth_nodes) * t.depth_nodes * t.mu_nodes * t.theta_nodes *
                              t.psi_nodes;
    t.half_values.resize(count);
    in.read(reinterpret_cast<char*>(t.half_values.data()), static_cast<std::streamsize>(count * 2));
    if (in.gcount() != static_cast<std::streamsize>(count * 2)) {
        throw std::runtime_error("cloud transfer table is truncated: " + file.string());
    }
    t.values.resize(count);
    std::transform(t.half_values.begin(), t.half_values.end(), t.values.begin(), half_to_float);
    return t;
}

double cloud_transfer_depth_coordinate(const CloudTransferTable& table, double depth) {
    const double a = std::max(0.0, depth);
    const int last_finite = table.depth_nodes - 2;
    const double a_last = table.depth_scale * (std::pow(table.depth_ratio, last_finite) - 1.0);
    if (a <= a_last) return std::log1p(a / table.depth_scale) / std::log(table.depth_ratio);
    if (std::isinf(a)) return table.depth_nodes - 1.0;
    // Linear in x = 1 / (1 + K a) from the last finite node (x_last) to infinity (x = 0).
    const double x = 1.0 / (1.0 + table.diffusion_k * a), x_last = 1.0 / (1.0 + table.diffusion_k * a_last);
    return last_finite + 1.0 - x / x_last;
}

double cloud_transfer_mu_coordinate(const CloudTransferTable& table, double sun_cos_zenith) {
    return std::sqrt(std::clamp(sun_cos_zenith, 0.0, 1.0)) * table.mu_nodes - 1.0;  // node i: ((i + 1) / n)^2
}

double cloud_transfer_theta_coordinate(const CloudTransferTable& table, double theta_rad) {
    const double theta = std::clamp(theta_rad, 0.0, constants::pi);
    return table.theta_log * std::log1p(theta / table.theta0) + table.theta_linear * theta;
}

double cloud_transfer_psi_coordinate(const CloudTransferTable& table, double psi_rad) {
    return std::clamp(std::abs(psi_rad), 0.0, constants::pi) / constants::pi * (table.psi_nodes - 1);
}

double cloud_transfer_lookup(const CloudTransferTable& table, double depth_from_top, double depth_from_bottom,
                             double sun_cos_zenith, double theta_rad, double psi_rad) {
    const Bracket a = bracket(cloud_transfer_depth_coordinate(table, depth_from_top), table.depth_nodes);
    const Bracket b = bracket(cloud_transfer_depth_coordinate(table, depth_from_bottom), table.depth_nodes);
    const Bracket m = bracket(cloud_transfer_mu_coordinate(table, sun_cos_zenith), table.mu_nodes);
    const Bracket t = bracket(cloud_transfer_theta_coordinate(table, theta_rad), table.theta_nodes);
    const Bracket s = bracket(cloud_transfer_psi_coordinate(table, psi_rad), table.psi_nodes);
    double sum = 0.0;
    for (int corner = 0; corner < 32; ++corner) {
        const int da = corner & 1, db = (corner >> 1) & 1, dm = (corner >> 2) & 1, dt = (corner >> 3) & 1,
                  ds = (corner >> 4) & 1;
        const double w = (da ? a.weight : 1.0 - a.weight) * (db ? b.weight : 1.0 - b.weight) *
                         (dm ? m.weight : 1.0 - m.weight) * (dt ? t.weight : 1.0 - t.weight) *
                         (ds ? s.weight : 1.0 - s.weight);
        if (w == 0.0) continue;
        const std::size_t index =
            ((((static_cast<std::size_t>(a.index + da) * table.depth_nodes + (b.index + db)) * table.mu_nodes +
               (m.index + dm)) * table.theta_nodes + (t.index + dt)) * table.psi_nodes) + (s.index + ds);
        sum += w * table.values[index];
    }
    return std::clamp(sun_cos_zenith, 0.0, 1.0) * sum;  // the table holds J / mu0
}

CloudTransferTexels cloud_transfer_gpu_texels(const CloudTransferTable& table) {
    CloudTransferTexels out;
    const int mu_cells = table.mu_nodes - 1, b_cells = table.depth_nodes - 1;
    out.width = mu_cells * table.theta_nodes;
    out.height = b_cells * table.psi_nodes;
    out.depth = table.depth_nodes;
    out.rgba_half.resize(static_cast<std::size_t>(out.width) * out.height * out.depth * 4);
    const auto at = [&table](int a, int b, int m, int th, int ps) {
        return table.half_values[((((static_cast<std::size_t>(a) * table.depth_nodes + b) * table.mu_nodes + m) *
                                       table.theta_nodes + th) * table.psi_nodes) + ps];
    };
    std::size_t k = 0;
    for (int a = 0; a < out.depth; ++a) {
        for (int y = 0; y < out.height; ++y) {
            const int b = y / table.psi_nodes, ps = y % table.psi_nodes;
            for (int x = 0; x < out.width; ++x) {
                const int m = x / table.theta_nodes, th = x % table.theta_nodes;
                out.rgba_half[k++] = at(a, b, m, th, ps);
                out.rgba_half[k++] = at(a, b, m + 1, th, ps);
                out.rgba_half[k++] = at(a, b + 1, m, th, ps);
                out.rgba_half[k++] = at(a, b + 1, m + 1, th, ps);
            }
        }
    }
    return out;
}

}  // namespace starsim::environment
