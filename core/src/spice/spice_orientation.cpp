#include "spice_check.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::spice {

frames::FixedToInertial SpiceOrientation::fixed_to_gcrf(time::Epoch epoch) const {
    // 6x6 state transformation: [[R, 0], [dR/dt, R]].
    SpiceDouble xform[6][6];
    sxform_c(frame_name_.c_str(), "J2000", epoch.tdb_seconds_past_j2000(), xform);
    detail::throw_if_failed("sxform_c");

    frames::FixedToInertial result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.rotation(i, j) = xform[i][j];
            result.rotation_rate_per_s(i, j) = xform[i + 3][j];
        }
    }
    return result;
}

}  // namespace starsim::spice
