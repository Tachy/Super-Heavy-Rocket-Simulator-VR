#include "starsim/frames/interpolated_orientation.hpp"

#include <cmath>

namespace starsim::frames {

namespace {

math::Mat3 skew(const math::Vec3& w) {
    math::Mat3 k;
    k.m[0] = {0.0, -w.z, w.y};
    k.m[1] = {w.z, 0.0, -w.x};
    k.m[2] = {-w.y, w.x, 0.0};
    return k;
}

math::Mat3 plus(const math::Mat3& a, const math::Mat3& b, double scale) {
    math::Mat3 r;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[i][j] + scale * b.m[i][j];
    }
    return r;
}

// exp([w]x dt) by Rodrigues' formula.
math::Mat3 turned(const math::Vec3& w, double dt) {
    const double rate = norm(w);
    const double angle = rate * dt;
    if (!(rate > 0.0) || angle == 0.0) return math::Mat3::identity();
    const math::Mat3 k = skew(w / rate);
    return plus(plus(math::Mat3::identity(), k, std::sin(angle)), k * k, 1.0 - std::cos(angle));
}

}  // namespace

InterpolatedOrientation::InterpolatedOrientation(const BodyOrientation& source, double node_spacing_s)
    : source_(source), spacing_s_(node_spacing_s) {}

auto InterpolatedOrientation::node(std::int64_t index) const -> const Node& {
    auto it = nodes_.find(index);
    if (it == nodes_.end()) {
        const auto f = source_.fixed_to_gcrf(time::Epoch::from_tdb_seconds_past_j2000(static_cast<double>(index) * spacing_s_));
        const math::Mat3 omega = transpose(f.rotation) * f.rotation_rate_per_s;
        // the skew part (R^T dR/dt is skew-symmetric up to rounding)
        const math::Vec3 w{0.5 * (omega(2, 1) - omega(1, 2)), 0.5 * (omega(0, 2) - omega(2, 0)),
                           0.5 * (omega(1, 0) - omega(0, 1))};
        it = nodes_.emplace(index, Node{f.rotation, w}).first;
    }
    return it->second;
}

FixedToInertial InterpolatedOrientation::fixed_to_gcrf(time::Epoch epoch) const {
    if (!(spacing_s_ > 0.0)) return source_.fixed_to_gcrf(epoch);
    const double t = epoch.tdb_seconds_past_j2000();
    const auto index = static_cast<std::int64_t>(std::llround(t / spacing_s_));
    if (!last_ || last_index_ != index) {
        last_ = &node(index);  // unordered_map nodes keep their address
        last_index_ = index;
    }
    FixedToInertial out;
    out.rotation = last_->rotation * turned(last_->rate_body_radps, t - static_cast<double>(index) * spacing_s_);
    out.rotation_rate_per_s = out.rotation * skew(last_->rate_body_radps);
    return out;
}

}  // namespace starsim::frames
