#include "starsim/aero/parachute.hpp"

namespace starsim::aero {

double parachute_drag_area_m2(const config::ParachuteConfig& p, double time_since_deploy_s) {
    if (time_since_deploy_s <= 0.0) return 0.0;
    double t = time_since_deploy_s;
    double from = 0.0;
    const auto filling = [&](double to) {
        const double x = t / p.fill_time_s;
        return p.drag_area_m2 * (from + (to - from) * x * x);
    };
    for (const auto& stage : p.reefing) {
        if (t < p.fill_time_s) return filling(stage.area_fraction);
        t -= p.fill_time_s;
        if (t < stage.hold_s) return p.drag_area_m2 * stage.area_fraction;
        t -= stage.hold_s;
        from = stage.area_fraction;
    }
    if (t < p.fill_time_s) return filling(1.0);
    return p.drag_area_m2;
}

math::Vec3 parachute_force_body_n(double drag_area_m2, const math::Vec3& v, double density_kg_m3) {
    return v * (-0.5 * density_kg_m3 * norm(v) * drag_area_m2);
}

}  // namespace starsim::aero
