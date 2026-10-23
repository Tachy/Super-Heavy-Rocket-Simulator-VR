#include "starsim/environment/geoid.hpp"

namespace starsim::environment {

double Geoid::undulation_m(double latitude_rad, double longitude_rad) const {
    return grid_.value(latitude_rad, longitude_rad);
}

}  // namespace starsim::environment
