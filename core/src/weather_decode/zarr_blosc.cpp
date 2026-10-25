// ERA5 Zarr chunk decoding (M8 weather and clouds): the blosc1 frame around the lz4 blocks, after c-blosc's
// README_HEADER.rst (format version 2, as c-blosc 1.x writes it through numcodecs):
//   octet 0      format version (<= 2)          octet 1   codec format version (lz4: 1)
//   octet 2      flags: 0x01 byte shuffle, 0x02 stored without compression (memcpyed), 0x04 bit shuffle,
//                0x10 blocks not split, bits 5-7 the codec (0 blosclz, 1 lz4/lz4hc, 2 snappy, 3 zlib, 4 zstd)
//   octet 3      typesize                       octets 4-7  nbytes (uncompressed), uint32 LE
//   octets 8-11  blocksize, uint32 LE           octets 12-15 ctbytes (the whole frame), uint32 LE
// memcpyed: the nbytes raw bytes follow the header. Otherwise int32 LE bstarts[nblocks] (block offsets from the frame
// start, nblocks = ceil(nbytes / blocksize), the last block may be shorter: the leftover); each block is `nsplits`
// streams of (int32 LE cbytes, cbytes of lz4 data), nsplits = typesize unless the frame says "not split" or the block
// is the leftover; a stream whose cbytes equals its uncompressed size is stored raw. Byte shuffle is undone per block:
// stream j of the block holds byte j of every element.

#include "starsim/weather_decode/weather_decode.hpp"

#include <bit>
#include <cstdint>
#include <cstring>
#include <string>

#include <lz4.h>

namespace starsim::weather_decode {

namespace {

static_assert(std::endian::native == std::endian::little, "float32 LE values are copied as they are");

constexpr std::size_t header_size = 16;
constexpr int max_format_version = 2;  // BLOSC_VERSION_FORMAT of c-blosc 1.x
constexpr int lz4_codec = 1;           // BLOSC_LZ4_FORMAT
constexpr int lz4_format_version = 1;  // BLOSC_LZ4_VERSION_FORMAT
constexpr std::uint8_t flag_byte_shuffle = 0x01;
constexpr std::uint8_t flag_memcpyed = 0x02;
constexpr std::uint8_t flag_bit_shuffle = 0x04;
constexpr std::uint8_t flag_dont_split = 0x10;

std::uint32_t read_u32_le(std::span<const std::byte> bytes, std::size_t offset) {
    std::uint32_t value = 0;
    for (int i = 3; i >= 0; --i) value = (value << 8) | std::to_integer<std::uint32_t>(bytes[offset + i]);
    return value;
}

[[noreturn]] void refuse(const std::string& why) { throw DecodeError("blosc chunk: " + why); }

// Undoes the byte shuffle of one block: element i, byte j comes from src[j * elements + i].
void unshuffle(const std::byte* src, std::byte* dst, std::size_t block_bytes, std::size_t typesize) {
    const std::size_t elements = block_bytes / typesize;
    for (std::size_t j = 0; j < typesize; ++j) {
        for (std::size_t i = 0; i < elements; ++i) dst[i * typesize + j] = src[j * elements + i];
    }
    const std::size_t tail = elements * typesize;  // bytes beyond whole elements are not shuffled
    std::memcpy(dst + tail, src + tail, block_bytes - tail);
}

// Decompresses one block of `block_bytes` starting at `offset` into `out` (unshuffled afterwards by the caller).
void decode_block(std::span<const std::byte> frame, std::size_t offset, std::size_t block_bytes, std::size_t nsplits,
                  std::byte* out) {
    const std::size_t split_bytes = block_bytes / nsplits;
    for (std::size_t s = 0; s < nsplits; ++s) {
        if (offset + 4 > frame.size()) refuse("block runs past the end of the frame");
        const std::uint32_t cbytes = read_u32_le(frame, offset);
        offset += 4;
        if (cbytes == 0 || cbytes > frame.size() - offset) refuse("block stream runs past the end of the frame");
        const std::byte* src = frame.data() + offset;
        std::byte* dst = out + s * split_bytes;
        if (cbytes == split_bytes) {
            std::memcpy(dst, src, split_bytes);
        } else {
            const int written = LZ4_decompress_safe(reinterpret_cast<const char*>(src), reinterpret_cast<char*>(dst),
                                                    static_cast<int>(cbytes), static_cast<int>(split_bytes));
            if (written != static_cast<int>(split_bytes)) refuse("damaged lz4 stream");
        }
        offset += cbytes;
    }
}

}  // namespace

FloatGrid decode_zarr_blosc_chunk(std::span<const std::byte> chunk, int levels, int rows, int columns) {
    if (levels <= 0 || rows <= 0 || columns <= 0) refuse("the requested shape is empty");
    const std::uint64_t count =
        static_cast<std::uint64_t>(levels) * static_cast<std::uint64_t>(rows) * static_cast<std::uint64_t>(columns);
    if (chunk.size() < header_size) refuse("shorter than its header");

    const int version = std::to_integer<int>(chunk[0]);
    const int codec_version = std::to_integer<int>(chunk[1]);
    const std::uint8_t flags = std::to_integer<std::uint8_t>(chunk[2]);
    const std::size_t typesize = std::to_integer<std::size_t>(chunk[3]);
    const std::uint32_t nbytes = read_u32_le(chunk, 4);
    const std::uint32_t blocksize = read_u32_le(chunk, 8);
    const std::uint32_t ctbytes = read_u32_le(chunk, 12);

    if (version == 0 || version > max_format_version) refuse("unknown format version " + std::to_string(version));
    if ((flags >> 5) != lz4_codec) refuse("codec " + std::to_string(flags >> 5) + " is not lz4");
    if (codec_version != lz4_format_version) refuse("unknown lz4 format version " + std::to_string(codec_version));
    if (flags & flag_bit_shuffle) refuse("bit shuffle is not supported");
    if (typesize != sizeof(float)) refuse("typesize " + std::to_string(typesize) + " is not float32");
    if (nbytes != count * sizeof(float)) refuse("holds " + std::to_string(nbytes) + " bytes, the shape needs " +
                                                std::to_string(count * sizeof(float)));
    if (ctbytes != chunk.size()) refuse("frame size " + std::to_string(ctbytes) + " but " +
                                        std::to_string(chunk.size()) + " bytes given");

    FloatGrid grid;
    grid.levels = levels;
    grid.rows = rows;
    grid.columns = columns;
    grid.values.resize(count);
    std::byte* out = reinterpret_cast<std::byte*>(grid.values.data());

    if (flags & flag_memcpyed) {
        if (header_size + nbytes != ctbytes) refuse("stored frame of the wrong size");
        std::memcpy(out, chunk.data() + header_size, nbytes);
        return grid;
    }
    // c-blosc clamps the block size to nbytes: a larger one is a damaged header (and would size the scratch block).
    if (blocksize == 0 || blocksize % typesize != 0 || blocksize > nbytes) {
        refuse("invalid block size " + std::to_string(blocksize));
    }
    const std::size_t nblocks = (nbytes + blocksize - 1) / blocksize;
    if (header_size + 4 * nblocks > chunk.size()) refuse("block table runs past the end of the frame");

    const bool shuffled = (flags & flag_byte_shuffle) != 0;
    std::vector<std::byte> scratch(shuffled ? blocksize : 0);
    for (std::size_t b = 0; b < nblocks; ++b) {
        const std::size_t start = read_u32_le(chunk, header_size + 4 * b);
        if (start < header_size + 4 * nblocks || start >= chunk.size()) refuse("block offset outside the frame");
        const bool leftover = b + 1 == nblocks && nbytes % blocksize != 0;
        const std::size_t block_bytes = leftover ? nbytes % blocksize : blocksize;
        const std::size_t nsplits = (flags & flag_dont_split) || leftover ? 1 : typesize;
        std::byte* block_out = out + b * blocksize;
        if (shuffled) {
            decode_block(chunk, start, block_bytes, nsplits, scratch.data());
            unshuffle(scratch.data(), block_out, block_bytes, typesize);
        } else {
            decode_block(chunk, start, block_bytes, nsplits, block_out);
        }
    }
    return grid;
}

}  // namespace starsim::weather_decode
