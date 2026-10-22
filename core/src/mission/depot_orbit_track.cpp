#include "starsim/mission/depot_orbit_track.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace starsim::mission {

DepotOrbitTrack::DepotOrbitTrack(const orbit::StationMeanElements& start, double decay_mps, double gm_m3ps2,
                                 double j2, double radius_m, std::optional<DepotKeeping> keeping)
    : segments_{{start, decay_mps}}, gm_m3ps2_(gm_m3ps2), j2_(j2), radius_m_(radius_m), keeping_(keeping) {}

const DepotOrbitTrack::Segment& DepotOrbitTrack::segment_at(time::Epoch epoch) const {
    // the last segment starting at or before `epoch` (the first one before the start)
    auto it = std::upper_bound(segments_.begin(), segments_.end(), epoch,
                               [](time::Epoch t, const Segment& s) { return t < s.start.epoch; });
    return it == segments_.begin() ? segments_.front() : *std::prev(it);
}

orbit::StationMeanElements DepotOrbitTrack::advance(const Segment& s, time::Epoch epoch,
                                                    std::vector<time::Epoch>* reboost_epochs) const {
    // pieces from the segment start, each from one a to the floor (or to `epoch`)
    orbit::StationMeanElements piece = s.start;
    double latitude = s.start.argument_of_latitude_rad;
    while (true) {
        const double a0 = piece.semi_major_axis_m;
        double dt_s = epoch - piece.epoch;
        bool reboost = false;
        if (keeping_ && s.decay_mps < 0.0 && dt_s > 0.0) {
            const double to_floor_s = (keeping_->floor_semi_major_axis_m - a0) / s.decay_mps;  // >= 0 above the floor
            if (to_floor_s < dt_s) {
                dt_s = std::max(0.0, to_floor_s);
                reboost = true;
            }
        }
        const auto rates = orbit::j2_secular_rates(piece, gm_m3ps2_, j2_, radius_m_);
        const double n0 = std::sqrt(gm_m3ps2_ / (a0 * a0 * a0));
        latitude += rates.latitude_radps * dt_s - 0.75 * n0 / a0 * s.decay_mps * dt_s * dt_s;
        const time::Epoch end = piece.epoch + dt_s;
        auto next = orbit::advance_mean_elements(piece, end, gm_m3ps2_, j2_, radius_m_, latitude);
        next.semi_major_axis_m = a0 + s.decay_mps * dt_s;
        if (!reboost) return next;
        if (reboost_epochs) reboost_epochs->push_back(end);
        next.semi_major_axis_m = keeping_->target_semi_major_axis_m;
        piece = next;
        if (!(keeping_->target_semi_major_axis_m > keeping_->floor_semi_major_axis_m)) return piece;  // no cycle
    }
}

orbit::StationMeanElements DepotOrbitTrack::at(time::Epoch epoch) const {
    return advance(segment_at(epoch), epoch, nullptr);
}

std::vector<time::Epoch> DepotOrbitTrack::reboosts(time::Epoch from, time::Epoch to) const {
    std::vector<time::Epoch> all, out;
    for (size_t i = 0; i < segments_.size(); ++i) {
        const time::Epoch end = i + 1 < segments_.size() ? segments_[i + 1].start.epoch : to;
        if (end <= from || segments_[i].start.epoch >= to) continue;
        advance(segments_[i], std::min(end, to), &all);
    }
    for (const auto t : all) {
        if (t > from && t <= to) out.push_back(t);
    }
    return out;
}

void DepotOrbitTrack::change(time::Epoch epoch, double delta_a_m, double decay_mps) {
    if (epoch < last_change()) throw std::invalid_argument("DepotOrbitTrack: change before the last one");
    auto start = at(epoch);
    start.semi_major_axis_m += delta_a_m;
    segments_.push_back({start, decay_mps});
}

double along_track_delta_a_m(double semi_major_axis_m, double delta_v_mps, double gm_m3ps2) {
    return 2.0 * semi_major_axis_m * delta_v_mps / std::sqrt(gm_m3ps2 / semi_major_axis_m);
}

double drag_decay_mps(double density_kg_m3, double drag_area_m2, double mass_kg, double semi_major_axis_m,
                      double gm_m3ps2) {
    return -density_kg_m3 * drag_area_m2 * std::sqrt(gm_m3ps2 * semi_major_axis_m) / mass_kg;
}

}  // namespace starsim::mission
