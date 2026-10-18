#pragma once

#include <memory>
#include <vector>

namespace starsim::vehicle {

// Settled liquid in a cylindrical tank whose free surface stands perpendicular to the apparent gravity ("down" =
// minus the specific force). Normalised geometry: radius 1, length `aspect` (= L / r), centred at the origin, axis x.
// Lateral u: along down's component perpendicular to the axis (the liquid's side); transverse w: the third axis.
// Quasi-static: no slosh dynamics.
struct LiquidMoments {
    double axial = 0.0;           // centroid along x (units of r)
    double lateral = 0.0;         // centroid along u (units of r)
    double axial_var = 0.0;       // variances about the centroid (units of r^2)
    double lateral_var = 0.0;
    double transverse_var = 0.0;
};

// Moments over a grid of tilts (angle between down and -x, 1 deg steps) and plane offsets; queries interpolate.
class SettlingTable {
public:
    explicit SettlingTable(double aspect);
    // fill: liquid volume over tank volume (clamped to 0..1); tilt_rad: angle between down and -x (0..pi).
    LiquidMoments query(double fill, double tilt_rad) const;
    double aspect() const { return aspect_; }

private:
    struct Raw {
        double v = 0.0, mx = 0.0, mu = 0.0, sxx = 0.0, suu = 0.0, sww = 0.0;
    };
    Raw row_at(size_t row, double volume) const;

    double aspect_;
    size_t offsets_;
    std::vector<Raw> raw_;  // [row * offsets_ + offset], volume rising with the offset
};

// The table for `aspect`, built once per process (thread-safe).
std::shared_ptr<const SettlingTable> settling_table(double aspect);

// Direct solution (plane offset by bisection): the reference for the table, slow.
LiquidMoments settle_direct(double aspect, double fill, double tilt_rad);

}  // namespace starsim::vehicle
