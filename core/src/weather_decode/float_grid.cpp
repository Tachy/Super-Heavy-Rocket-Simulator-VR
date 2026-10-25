// The decoded value grid of the weather decoders (M8 weather and clouds).

#include "starsim/weather_decode/weather_decode.hpp"

#include <cstddef>

namespace starsim::weather_decode {

float FloatGrid::at(int level, int row, int column) const {
    const std::size_t index =
        (static_cast<std::size_t>(level) * static_cast<std::size_t>(rows) + static_cast<std::size_t>(row)) *
            static_cast<std::size_t>(columns) +
        static_cast<std::size_t>(column);
    return values.at(index);
}

}  // namespace starsim::weather_decode
