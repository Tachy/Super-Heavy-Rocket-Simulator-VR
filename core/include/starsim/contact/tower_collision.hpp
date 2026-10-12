#pragma once

#include "starsim/contact/tower_geometry.hpp"

namespace starsim::contact {

// True if a vehicle hull (capsule from `bottom_m` to `top_m`, site frame) touches the tower structure or the
// launch mount. The arms are not part of this test: they act through the contact model.
bool hits_tower(const TowerGeometry& tower, const math::Vec3& bottom_m, const math::Vec3& top_m, double hull_radius_m);

}  // namespace starsim::contact
