// Weather decoders (M8 weather and clouds, Task 1): ERA5 Zarr blosc chunks decode bit-exact, GFS GRIB2 messages to
// cfgrib's values. Fixtures and references: tools/validation/gen_weather_fixtures.py -> tests/validation/weather/.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <toml++/toml.hpp>

#include "starsim/weather_decode/weather_decode.hpp"

using namespace starsim;
namespace fs = std::filesystem;

namespace {

const fs::path fixture_dir = STARSIM_WEATHER_FIXTURE_DIR;

std::vector<std::byte> read_bytes(const fs::path& file) {
    std::ifstream in(file, std::ios::binary);
    REQUIRE(in);
    std::vector<char> chars((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<std::byte> bytes(chars.size());
    std::memcpy(bytes.data(), chars.data(), chars.size());
    return bytes;
}

std::vector<float> read_floats(const fs::path& file) {
    const std::vector<std::byte> bytes = read_bytes(file);
    std::vector<float> values(bytes.size() / sizeof(float));
    std::memcpy(values.data(), bytes.data(), values.size() * sizeof(float));
    return values;
}

bool same_bits(const std::vector<float>& a, const std::vector<float>& b) {
    return a.size() == b.size() && std::memcmp(a.data(), b.data(), a.size() * sizeof(float)) == 0;
}

// Byte offset of GRIB2 section `number` in a single-field message (walks the section lengths after section 0).
std::size_t section_offset(const std::vector<std::byte>& message, int number) {
    std::size_t pos = 16;
    while (pos + 5 <= message.size()) {
        std::uint32_t length = 0;
        for (int i = 0; i < 4; ++i) length = (length << 8) | std::to_integer<std::uint32_t>(message[pos + i]);
        if (std::to_integer<int>(message[pos + 4]) == number) return pos;
        pos += length;
    }
    FAIL("section " << number << " not found");
    return 0;
}

// Writes `value` big-endian into `width` octets starting at the manual's 1-based octet of the section at `section`.
void put_octets(std::vector<std::byte>& message, std::size_t section, std::size_t octet, std::size_t width,
                std::uint64_t value) {
    for (std::size_t i = 0; i < width; ++i) {
        message[section + octet - 1 + i] = static_cast<std::byte>((value >> (8 * (width - 1 - i))) & 0xFF);
    }
}

void put_u32_le(std::vector<std::byte>& frame, std::size_t offset, std::uint32_t value) {
    for (std::size_t i = 0; i < 4; ++i) frame[offset + i] = static_cast<std::byte>((value >> (8 * i)) & 0xFF);
}

}  // namespace

TEST_CASE("Zarr blosc chunk decodes bit-exact", "[weather]") {
    const toml::table reference = toml::parse_file((fixture_dir / "era5_cc_reference.toml").string());
    const int levels = reference["levels"].value_or(0);

    SECTION("one split block, byte shuffle (the store's codec) and without shuffle") {
        const int rows = reference["cutout"]["rows"].value_or(0);
        const int columns = reference["cutout"]["columns"].value_or(0);
        const std::vector<float> expected = read_floats(fixture_dir / "era5_cc_cutout_values.bin");
        for (const char* name : {"era5_cc_cutout.bin", "era5_cc_cutout_noshuffle.bin"}) {
            INFO(name);
            const weather_decode::FloatGrid grid =
                weather_decode::decode_zarr_blosc_chunk(read_bytes(fixture_dir / name), levels, rows, columns);
            CHECK(grid.levels == levels);
            CHECK(grid.rows == rows);
            CHECK(grid.columns == columns);
            CHECK(same_bits(grid.values, expected));
            const std::size_t last = expected.size() - 1;
            CHECK(grid.at(levels - 1, rows - 1, columns - 1) == expected[last]);
            CHECK(grid.at(1, 2, 3) == expected[(1 * rows + 2) * columns + 3]);
        }
    }
    SECTION("block table, a split block and an unsplit leftover block") {
        const int rows = reference["blocks"]["rows"].value_or(0);
        const int columns = reference["blocks"]["columns"].value_or(0);
        const weather_decode::FloatGrid grid = weather_decode::decode_zarr_blosc_chunk(
            read_bytes(fixture_dir / "era5_cc_blocks.bin"), levels, rows, columns);
        CHECK(same_bits(grid.values, read_floats(fixture_dir / "era5_cc_blocks_values.bin")));
    }
}

TEST_CASE("Damaged blosc chunks are refused", "[weather]") {
    const std::vector<std::byte> good = read_bytes(fixture_dir / "era5_cc_blocks.bin");
    const int levels = 4, rows = 64, columns = 96;
    REQUIRE_NOTHROW(weather_decode::decode_zarr_blosc_chunk(good, levels, rows, columns));
    using weather_decode::DecodeError;

    SECTION("truncated") {
        const std::span<const std::byte> all(good);
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(all.first(10), levels, rows, columns), DecodeError);
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(all.first(good.size() / 2), levels, rows, columns),
                        DecodeError);
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(all.first(good.size() - 1), levels, rows, columns),
                        DecodeError);
    }
    SECTION("wrong typesize") {
        std::vector<std::byte> bad = good;
        bad[3] = std::byte{8};
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns), DecodeError);
    }
    SECTION("unknown codec byte") {
        std::vector<std::byte> bad = good;
        bad[2] = (bad[2] & std::byte{0x1F}) | std::byte{3 << 5};  // zlib
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns), DecodeError);
        bad[2] = (bad[2] & std::byte{0x1F}) | std::byte{7 << 5};  // not assigned
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns), DecodeError);
    }
    SECTION("bit shuffle") {
        std::vector<std::byte> bad = good;
        bad[2] |= std::byte{0x04};
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns), DecodeError);
    }
    SECTION("damaged lz4 stream") {
        std::vector<std::byte> bad = good;
        for (std::size_t i = 40; i < 140; ++i) bad[i] = std::byte{0xFF};
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns), DecodeError);
    }
    SECTION("block size larger than the data") {
        std::vector<std::byte> bad = good;
        put_u32_le(bad, 8, 4u * levels * rows * columns + 4u);  // nbytes + one float
        CHECK_THROWS_WITH(weather_decode::decode_zarr_blosc_chunk(bad, levels, rows, columns),
                          Catch::Matchers::ContainsSubstring("invalid block size"));
    }
    SECTION("shape does not match the frame") {
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(good, levels, rows, columns + 1), DecodeError);
        CHECK_THROWS_AS(weather_decode::decode_zarr_blosc_chunk(good, 0, rows, columns), DecodeError);
    }
}

TEST_CASE("GRIB2 records decode to cfgrib's values", "[weather]") {
    // ecCodes (under cfgrib) unpacks in double and cfgrib hands out float32: our float32 must be within one float32 ulp
    // of it. The packing's own precision is far coarser (step 2^E 10^-D): TCDC 300 mb 5.3 E 0 D 1 -> 0.1 %, CLMR 850 mb
    // 5.3 E 1 D 8 -> 2e-8 kg/kg, CLMR 300 mb 5.3 E 4 D 9 -> 1.6e-8 kg/kg, CLMR 50 mb 5.0 with 0 bits (constant field).
    for (const char* name : {"gfs_clmr_300mb", "gfs_clmr_850mb", "gfs_clmr_50mb", "gfs_tcdc_300mb"}) {
        INFO(name);
        const toml::table reference = toml::parse_file((fixture_dir / (std::string(name) + "_reference.toml")).string());
        const weather_decode::Grib2Field field =
            weather_decode::decode_grib2_message(read_bytes(fixture_dir / (std::string(name) + ".grb2")));
        const toml::array* points = reference["point"].as_array();
        REQUIRE(points != nullptr);
        CHECK(points->size() == 20);
        for (const toml::node& node : *points) {
            const toml::table& point = *node.as_table();
            const int row = point["row"].value_or(-1);
            const int column = point["column"].value_or(-1);
            const float expected = static_cast<float>(point["value"].value_or(std::nan("")));
            const float decoded = field.grid.at(0, row, column);
            INFO("row " << row << " column " << column << ": " << decoded << " vs " << expected);
            CHECK(std::abs(decoded - expected) <= std::abs(expected) * std::numeric_limits<float>::epsilon());
        }
    }
}

TEST_CASE("GRIB2 grid metadata", "[weather]") {
    for (const char* name : {"gfs_clmr_850mb", "gfs_clmr_50mb", "gfs_tcdc_300mb"}) {
        INFO(name);
        const weather_decode::Grib2Field field =
            weather_decode::decode_grib2_message(read_bytes(fixture_dir / (std::string(name) + ".grb2")));
        CHECK(field.grid.levels == 1);
        CHECK(field.grid.rows == 721);
        CHECK(field.grid.columns == 1440);
        CHECK(field.grid.values.size() == 721u * 1440u);
        CHECK(field.first_latitude_deg == 90.0);
        CHECK(field.first_longitude_deg == 0.0);
        CHECK(field.step_deg == 0.25);
    }
}

TEST_CASE("Damaged GRIB2 messages are refused", "[weather]") {
    const std::vector<std::byte> good = read_bytes(fixture_dir / "gfs_tcdc_300mb.grb2");
    const std::span<const std::byte> all(good);
    using weather_decode::DecodeError;
    CHECK_THROWS_AS(weather_decode::decode_grib2_message(all.first(8)), DecodeError);
    CHECK_THROWS_AS(weather_decode::decode_grib2_message(all.first(good.size() / 2)), DecodeError);
    std::vector<std::byte> bad = good;
    bad[0] = std::byte{'X'};  // not "GRIB"
    CHECK_THROWS_AS(weather_decode::decode_grib2_message(bad), DecodeError);
    bad = good;
    bad[7] = std::byte{1};  // GRIB edition 1
    CHECK_THROWS_AS(weather_decode::decode_grib2_message(bad), DecodeError);
    bad = good;
    bad[bad.size() - 1] = std::byte{'0'};  // no end section "7777"
    CHECK_THROWS_AS(weather_decode::decode_grib2_message(bad), DecodeError);
}

TEST_CASE("Damaged GRIB2 headers are refused before any allocation", "[weather]") {
    // Section lengths stay intact, so the outer message checks pass and each header guard is reached.
    using Catch::Matchers::ContainsSubstring;
    const std::vector<std::byte> complex = read_bytes(fixture_dir / "gfs_tcdc_300mb.grb2");
    const std::vector<std::byte> simple = read_bytes(fixture_dir / "gfs_clmr_50mb.grb2");

    SECTION("grid above 2^24 points (rows * columns = count = 2^32 - 1)") {
        std::vector<std::byte> bad = complex;
        const std::size_t s3 = section_offset(bad, 3), s5 = section_offset(bad, 5);
        put_octets(bad, s3, 7, 4, 0xFFFFFFFFu);
        put_octets(bad, s3, 31, 4, 65537);
        put_octets(bad, s3, 35, 4, 65535);
        put_octets(bad, s5, 6, 4, 0xFFFFFFFFu);
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("larger than 2^24"));
    }
    SECTION("rows beyond int (2^31 x 1)") {
        std::vector<std::byte> bad = complex;
        const std::size_t s3 = section_offset(bad, 3), s5 = section_offset(bad, 5);
        put_octets(bad, s3, 7, 4, 0x80000000u);
        put_octets(bad, s3, 31, 4, 1);
        put_octets(bad, s3, 35, 4, 0x80000000u);
        put_octets(bad, s5, 6, 4, 0x80000000u);
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("larger than 2^24"));
    }
    SECTION("simple packing: more packed bits than the data section holds") {
        std::vector<std::byte> bad = simple;
        put_octets(bad, section_offset(bad, 5), 20, 1, 8);  // 8 bits per value, no data octets
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("shorter than its packed values"));
    }
    SECTION("complex packing: more group descriptors than the data section holds") {
        std::vector<std::byte> bad = complex;
        put_octets(bad, section_offset(bad, 5), 32, 4, 721u * 1440u);  // NG = one group per point
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("exceed the data section"));
    }
    SECTION("complex packing: group width / length fields wider than 32 bits") {
        std::vector<std::byte> bad = complex;
        put_octets(bad, section_offset(bad, 5), 37, 1, 40);  // bits of the group widths
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("wider than 32 bits"));
        bad = complex;
        put_octets(bad, section_offset(bad, 5), 47, 1, 63);  // bits of the group lengths
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("wider than 32 bits"));
        bad = complex;
        put_octets(bad, section_offset(bad, 5), 36, 1, 200);  // reference of the group widths
        CHECK_THROWS_WITH(weather_decode::decode_grib2_message(bad), ContainsSubstring("group width above 32 bits"));
    }
}
