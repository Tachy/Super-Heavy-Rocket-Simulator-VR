#include "starsim/contact/tower_collision.hpp"

namespace starsim::contact {

bool hits_tower(const TowerGeometry& tower, const math::Vec3& bottom_m, const math::Vec3& top_m, double hull_radius_m) {
    return capsule_overlaps_box(bottom_m, top_m, hull_radius_m, tower.tower_box()) ||
           capsule_overlaps_annulus(bottom_m, top_m, hull_radius_m, tower.mount());
}

}  // namespace starsim::contact
