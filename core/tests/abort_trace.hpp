// Diagnostics for the hidden abort clearance cases: the hull axis in the site frame per sample, written as CSV for
// tools/validation/abort_clearance.py, which measures it against the drawn tower foot, mount and ramps.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "starsim/contact/primitives.hpp"
#include "starsim/sim/catch_tower_sim.hpp"
#include "starsim/sim/flight_session.hpp"

namespace starsim::test {

// Smallest distance from the hull surface to an oriented box, with the hull axis sampled every metre (negative: inside).
inline double hull_to_box_m(const math::Vec3& bottom_m, const math::Vec3& top_m, double radius_m, const contact::OrientedBox& box) {
    double best = 1e12;
    const double length = norm(top_m - bottom_m);
    const int samples = static_cast<int>(std::ceil(length)) + 1;
    for (int i = 0; i <= samples; ++i) {
        const math::Vec3 p = bottom_m + (top_m - bottom_m) * (static_cast<double>(i) / samples);
        const math::Vec3 local = transpose(box.axes) * (p - box.center);
        const double dx = std::max(std::abs(local.x) - box.half_extents.x, 0.0);
        const double dy = std::max(std::abs(local.y) - box.half_extents.y, 0.0);
        const double dz = std::max(std::abs(local.z) - box.half_extents.z, 0.0);
        best = std::min(best, std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return best - radius_m;
}

class AbortTrace {
public:
    explicit AbortTrace(const std::string& name) {
        const char* dir = std::getenv("STARSIM_ABORT_TRACE_DIR");
        if (dir) file_ = std::fopen((std::string(dir) + "/" + name + ".csv").c_str(), "w");
        if (file_) std::fprintf(file_, "t_s,bx,by,bz,tx,ty,tz,radius_m,tower_m,arm1_m,arm2_m,phase\n");
    }
    ~AbortTrace() {
        if (file_) std::fclose(file_);
    }
    AbortTrace(const AbortTrace&) = delete;
    AbortTrace& operator=(const AbortTrace&) = delete;

    // Records body `index` (site frame: east, north, up from the pad centre).
    void sample(const sim::FlightSession& session, const frames::BodyOrientation& earth, size_t index, double t_s) {
        if (!file_ || !session.tower()) return;
        const auto* tower = session.tower();
        const auto& body = session.body(index);
        const auto& g = tower->geometry();
        const auto tr = earth.fixed_to_gcrf(session.epoch());
        const math::Mat3 to_site = transpose(g.site_to_itrf()) * transpose(tr.rotation);
        const auto& s = body.state();
        const math::Vec3 cg = g.to_site(transpose(tr.rotation) * s.position_gcrf_m);
        const math::Mat3 body_to_site = to_site * s.attitude.to_matrix();
        const double cg_station = body.vehicle().mass_properties().cg_station_m;
        const math::Vec3 bottom = cg + body_to_site * math::Vec3{-cg_station, 0.0, 0.0};
        const math::Vec3 top = cg + body_to_site * math::Vec3{body.vehicle().length_m() - cg_station, 0.0, 0.0};
        const double radius = 0.5 * body.vehicle().diameter_m();
        const auto& arms = tower->arms();
        std::fprintf(file_, "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n", t_s, bottom.x, bottom.y, bottom.z, top.x, top.y,
                     top.z, radius, hull_to_box_m(bottom, top, radius, g.tower_box()),
                     hull_to_box_m(bottom, top, radius, g.arm_box(1, arms.opening_rad(), arms.carriage_offset_m())),
                     hull_to_box_m(bottom, top, radius, g.arm_box(-1, arms.opening_rad(), arms.carriage_offset_m())),
                     sim::to_string(tower->phase()));
    }

private:
    std::FILE* file_ = nullptr;
};

}  // namespace starsim::test
