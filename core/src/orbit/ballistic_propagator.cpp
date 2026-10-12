#include "starsim/orbit/ballistic_propagator.hpp"

namespace starsim::orbit {

integrators::OdeFunction<6> ballistic_dynamics(const environment::GravityModel& gravity,
                                               time::Epoch reference_epoch) {
    return [&gravity, reference_epoch](double t_s, const integrators::State<6>& y) {
        const math::Vec3 position_m{y[0], y[1], y[2]};
        const math::Vec3 a = gravity.acceleration_mps2(position_m, reference_epoch + t_s);
        return integrators::State<6>{{y[3], y[4], y[5], a.x, a.y, a.z}};
    };
}

integrators::Dop853Options<6> default_coast_options() {
    integrators::Dop853Options<6> options;
    options.relative_tolerance = 1e-12;
    options.absolute_tolerance = integrators::State<6>{{1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6}};
    return options;
}

CoastResult propagate_coast(const environment::GravityModel& gravity, const math::StateVector& state0,
                            time::Epoch epoch0, time::Epoch epoch_end,
                            std::span<const integrators::Event<6>> events,
                            const integrators::Dop853Options<6>& options) {
    const auto result = integrators::integrate<6>(ballistic_dynamics(gravity, epoch0), 0.0, math::pack(state0),
                                                  epoch_end - epoch0, options, events);
    return {math::unpack(result.y), epoch0 + result.t_s, result.event_index, result.evaluations};
}

}  // namespace starsim::orbit
