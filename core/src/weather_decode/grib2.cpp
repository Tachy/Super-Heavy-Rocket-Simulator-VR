// GFS GRIB2 message decoding (M8 weather and clouds), after the WMO Manual on Codes FM 92 GRIB edition 2 and its
// templates 3.0, 5.0, 5.2, 5.3 (octet numbers below are the manual's, 1-based within a section). All multi-octet values
// are big-endian; signed ones are sign-magnitude (top bit = sign). A decoded value is Y = (R + X 2^E) 10^-D, with
// R the float32 reference value, E the binary and D the decimal scale factor of section 5.
// Complex packing (5.2, 5.3; same algorithm as NCEPLIBS-g2c comunpack.c): section 7 holds, each padded to whole
// octets, [5.3: the first `order` original values and the minimum of the differences, sign-magnitude in `extra` octets
// each], the NG group reference values (nbits each), the NG group widths (reference + value) and the NG group lengths
// (reference + value * increment, the last one given in section 5); then every group's values back to back, `width`
// bits each (width 0: the group is its reference). Spatial differencing of order 1 / 2 is undone with the minimum.

#include "starsim/weather_decode/weather_decode.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

namespace starsim::weather_decode {

namespace {

constexpr std::size_t indicator_size = 16;  // section 0
constexpr std::size_t end_section_size = 4;  // section 8, "7777"
constexpr int regular_lat_lon_template = 0;
constexpr int simple_packing_template = 0;
constexpr int complex_packing_template = 2;
constexpr int spatial_differencing_template = 3;
constexpr int no_bitmap = 255;
constexpr double micro_degrees_per_degree = 1e6;  // template 3.0 angle unit when the basic angle is 0 / missing
constexpr std::uint32_t missing_u32 = 0xFFFFFFFFu;
// Damaged-header guards (downloaded input): no grid beyond 2^24 points (GFS 0.25 deg: 1.04e6; 0.1 deg global: 6.5e6)
// is allocated, and no packed field is wider than 32 bits.
constexpr std::uint64_t max_grid_points = std::uint64_t{1} << 24;
constexpr int max_field_bits = 32;

[[noreturn]] void refuse(const std::string& why) { throw DecodeError("GRIB2 message: " + why); }

// One section: its octets, indexable by the manual's 1-based octet numbers.
struct Section {
    std::span<const std::byte> bytes;

    std::uint64_t unsigned_at(std::size_t octet, std::size_t width) const {
        if (octet < 1 || octet - 1 + width > bytes.size()) refuse("section " + std::to_string(number()) + " too short");
        std::uint64_t value = 0;
        for (std::size_t i = 0; i < width; ++i) value = (value << 8) | std::to_integer<std::uint64_t>(bytes[octet - 1 + i]);
        return value;
    }
    std::int64_t signed_at(std::size_t octet, std::size_t width) const {
        const std::uint64_t raw = unsigned_at(octet, width);
        const std::uint64_t sign = std::uint64_t{1} << (8 * width - 1);
        const auto magnitude = static_cast<std::int64_t>(raw & (sign - 1));
        return (raw & sign) ? -magnitude : magnitude;
    }
    float float_at(std::size_t octet) const {
        return std::bit_cast<float>(static_cast<std::uint32_t>(unsigned_at(octet, 4)));
    }
    int number() const { return bytes.size() > 4 ? std::to_integer<int>(bytes[4]) : -1; }
};

// MSB-first bit stream over section 7's data octets.
class BitReader {
public:
    explicit BitReader(std::span<const std::byte> data) : data_(data) {}

    std::uint64_t read(int bits) {
        if (bits < 0 || bits > 63) refuse("field width " + std::to_string(bits) + " bits");
        if (position_ + static_cast<std::uint64_t>(bits) > 8 * static_cast<std::uint64_t>(data_.size())) {
            refuse("data section ends inside the packed values");
        }
        std::uint64_t value = 0;
        for (int i = 0; i < bits; ++i, ++position_) {
            const auto octet = std::to_integer<unsigned>(data_[position_ >> 3]);
            value = (value << 1) | ((octet >> (7 - (position_ & 7))) & 1u);
        }
        return value;
    }
    std::int64_t read_sign_magnitude(int bits) {
        const bool negative = read(1) != 0;
        const auto magnitude = static_cast<std::int64_t>(read(bits - 1));
        return negative ? -magnitude : magnitude;
    }
    void align() { position_ = (position_ + 7) & ~std::uint64_t{7}; }

private:
    std::span<const std::byte> data_;
    std::uint64_t position_ = 0;
};

struct Packing {
    int template_number = 0;
    double reference = 0.0, binary_scale = 1.0, decimal_scale = 1.0;
    int bits = 0;
};

// Integer field X of complex packing (5.2, 5.3): groups, then the spatial differencing undone.
std::vector<std::int64_t> unpack_complex(const Section& s5, std::span<const std::byte> data, std::size_t count,
                                         int bits) {
    if (s5.unsigned_at(23, 1) != 0) refuse("missing-value management is not supported");
    const std::size_t groups = s5.unsigned_at(32, 4);
    const auto width_reference = static_cast<std::int64_t>(s5.unsigned_at(36, 1));
    const int width_bits = static_cast<int>(s5.unsigned_at(37, 1));
    const auto length_reference = static_cast<std::int64_t>(s5.unsigned_at(38, 4));
    const auto length_increment = static_cast<std::int64_t>(s5.unsigned_at(42, 1));
    const auto last_length = static_cast<std::int64_t>(s5.unsigned_at(43, 4));
    const int length_bits = static_cast<int>(s5.unsigned_at(47, 1));
    const bool differenced = s5.unsigned_at(10, 2) == spatial_differencing_template;
    const int order = differenced ? static_cast<int>(s5.unsigned_at(48, 1)) : 0;
    const int extra_bits = differenced ? 8 * static_cast<int>(s5.unsigned_at(49, 1)) : 0;
    if (order < 0 || order > 2) refuse("spatial differencing of order " + std::to_string(order));
    if (groups == 0 || groups > count) refuse("invalid group count");
    if (width_bits > max_field_bits || length_bits > max_field_bits || extra_bits > max_field_bits) {
        refuse("group descriptor wider than 32 bits");
    }
    // Every group has a reference, a width and a length in the data section: check before sizing the group tables.
    if (static_cast<std::uint64_t>(groups) * static_cast<std::uint64_t>(bits + width_bits + length_bits) >
        8 * static_cast<std::uint64_t>(data.size())) {
        refuse("group descriptors exceed the data section");
    }

    BitReader reader(data);
    std::int64_t first[2] = {0, 0};
    std::int64_t minimum = 0;
    if (order > 0) {
        if (extra_bits == 0) refuse("spatial differencing without its descriptors");
        for (int i = 0; i < order; ++i) first[i] = reader.read_sign_magnitude(extra_bits);
        minimum = reader.read_sign_magnitude(extra_bits);
    }
    reader.align();
    std::vector<std::int64_t> group_reference(groups), group_width(groups), group_length(groups);
    for (auto& v : group_reference) v = static_cast<std::int64_t>(reader.read(bits));
    reader.align();
    for (auto& v : group_width) v = width_reference + static_cast<std::int64_t>(reader.read(width_bits));
    reader.align();
    for (auto& v : group_length) v = length_reference + length_increment * static_cast<std::int64_t>(reader.read(length_bits));
    reader.align();
    group_length.back() = last_length;

    std::vector<std::int64_t> x;
    x.reserve(count);
    for (std::size_t g = 0; g < groups; ++g) {
        if (group_length[g] < 0 || x.size() + static_cast<std::size_t>(group_length[g]) > count) {
            refuse("group lengths exceed the number of values");
        }
        if (group_width[g] > max_field_bits) refuse("group width above 32 bits");
        for (std::int64_t i = 0; i < group_length[g]; ++i) {
            x.push_back(group_reference[g] + static_cast<std::int64_t>(reader.read(static_cast<int>(group_width[g]))));
        }
    }
    if (x.size() != count) refuse("group lengths do not add up to the number of values");

    // Undone in modular uint64 arithmetic: a damaged field may wrap but never overflows a signed integer (UB); valid
    // fields stay far inside the range.
    const auto u = [](std::int64_t v) { return static_cast<std::uint64_t>(v); };
    if (order == 1) {
        x[0] = first[0];
        for (std::size_t i = 1; i < count; ++i) x[i] = static_cast<std::int64_t>(u(x[i]) + u(minimum) + u(x[i - 1]));
    } else if (order == 2) {
        if (count < 2) refuse("second-order differencing of fewer than two values");
        x[0] = first[0];
        x[1] = first[1];
        for (std::size_t i = 2; i < count; ++i) {
            x[i] = static_cast<std::int64_t>(u(x[i]) + u(minimum) + 2 * u(x[i - 1]) - u(x[i - 2]));
        }
    }
    return x;
}

}  // namespace

Grib2Field decode_grib2_message(std::span<const std::byte> message) {
    if (message.size() < indicator_size + end_section_size) refuse("shorter than sections 0 and 8");
    if (std::memcmp(message.data(), "GRIB", 4) != 0) refuse("does not start with \"GRIB\"");
    if (std::to_integer<int>(message[7]) != 2) refuse("GRIB edition " + std::to_string(std::to_integer<int>(message[7])));
    const std::uint64_t total = Section{message.first(indicator_size)}.unsigned_at(9, 8);
    if (total != message.size()) refuse("length " + std::to_string(total) + " but " + std::to_string(message.size()) +
                                        " bytes given");
    if (std::memcmp(message.data() + total - end_section_size, "7777", 4) != 0) refuse("no end section \"7777\"");

    std::optional<Section> s3, s5, s6, s7;
    for (std::size_t pos = indicator_size; pos < total - end_section_size;) {
        if (pos + 5 > total - end_section_size) refuse("section header runs into the end section");
        const Section header{message.subspan(pos, 5)};
        const std::uint64_t length = header.unsigned_at(1, 4);
        if (length < 5 || length > total - end_section_size - pos) refuse("section length out of the message");
        const Section section{message.subspan(pos, static_cast<std::size_t>(length))};
        std::optional<Section>* slot = nullptr;
        switch (section.number()) {
            case 3: slot = &s3; break;
            case 5: slot = &s5; break;
            case 6: slot = &s6; break;
            case 7: slot = &s7; break;
            default: break;
        }
        if (slot != nullptr) {
            if (slot->has_value()) refuse("more than one field in the message");
            *slot = section;
        }
        pos += static_cast<std::size_t>(length);
    }
    if (!s3 || !s5 || !s6 || !s7) refuse("sections 3, 5, 6 and 7 are required");

    // Section 3, template 3.0: regular latitude / longitude grid.
    if (s3->unsigned_at(13, 2) != regular_lat_lon_template) refuse("grid template is not 3.0 (regular lat-lon)");
    const std::size_t count = s3->unsigned_at(7, 4);
    const auto columns = static_cast<std::size_t>(s3->unsigned_at(31, 4));
    const auto rows = static_cast<std::size_t>(s3->unsigned_at(35, 4));
    const std::uint64_t basic_angle = s3->unsigned_at(39, 4);
    if (basic_angle != 0 && basic_angle != missing_u32) refuse("grid angles in a basic angle other than degrees");
    if (s3->unsigned_at(72, 1) != 0) refuse("scanning mode other than north -> south, west -> east");
    const double first_latitude_deg = static_cast<double>(s3->signed_at(47, 4)) / micro_degrees_per_degree;
    const double first_longitude_deg = static_cast<double>(s3->signed_at(51, 4)) / micro_degrees_per_degree;
    const std::uint64_t di = s3->unsigned_at(64, 4);
    const std::uint64_t dj = s3->unsigned_at(68, 4);
    if (di != dj) refuse("longitude and latitude steps differ");
    if (count > max_grid_points || rows > max_grid_points || columns > max_grid_points) {
        refuse("grid larger than 2^24 points");  // also keeps rows and columns inside int
    }
    if (rows == 0 || columns == 0 || rows * columns != count) refuse("grid size does not match the number of points");

    // Section 6: no bitmap.
    if (s6->unsigned_at(6, 1) != no_bitmap) refuse("bitmaps are not supported");

    // Section 5: data representation.
    if (s5->unsigned_at(6, 4) != count) refuse("number of packed values differs from the grid");
    Packing packing;
    packing.template_number = static_cast<int>(s5->unsigned_at(10, 2));
    packing.reference = s5->float_at(12);
    packing.binary_scale = std::ldexp(1.0, static_cast<int>(s5->signed_at(16, 2)));
    packing.decimal_scale = std::pow(10.0, -static_cast<double>(s5->signed_at(18, 2)));
    packing.bits = static_cast<int>(s5->unsigned_at(20, 1));
    if (packing.bits > max_field_bits) refuse("more than 32 bits per value");

    const std::span<const std::byte> data = s7->bytes.subspan(5);
    if (packing.template_number == simple_packing_template &&
        static_cast<std::uint64_t>(count) * static_cast<std::uint64_t>(packing.bits) >
            8 * static_cast<std::uint64_t>(data.size())) {
        refuse("data section shorter than its packed values");
    }
    std::vector<std::int64_t> x;
    switch (packing.template_number) {
        case simple_packing_template: {
            x.resize(count, 0);
            BitReader reader(data);
            if (packing.bits > 0) {
                for (auto& v : x) v = static_cast<std::int64_t>(reader.read(packing.bits));
            }
            break;
        }
        case complex_packing_template:
        case spatial_differencing_template:
            x = unpack_complex(*s5, data, count, packing.bits);
            break;
        default:
            refuse("packing template 5." + std::to_string(packing.template_number) + " is not supported");
    }

    Grib2Field field;
    field.grid.levels = 1;
    field.grid.rows = static_cast<int>(rows);
    field.grid.columns = static_cast<int>(columns);
    field.grid.values.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        field.grid.values[i] = static_cast<float>(
            (packing.reference + static_cast<double>(x[i]) * packing.binary_scale) * packing.decimal_scale);
    }
    field.first_latitude_deg = first_latitude_deg;
    field.first_longitude_deg = first_longitude_deg;
    field.step_deg = static_cast<double>(di) / micro_degrees_per_degree;
    return field;
}

}  // namespace starsim::weather_decode
