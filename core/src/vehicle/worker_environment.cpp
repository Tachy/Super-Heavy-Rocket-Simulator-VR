#include "starsim/vehicle/worker_environment.hpp"

#include <stdexcept>

namespace starsim::vehicle {

namespace {

const EnvironmentSources& checked(const EnvironmentSources& s) {
    if (s.ephemeris == nullptr || s.earth == nullptr) {
        throw std::invalid_argument("WorkerEnvironment: ephemeris and Earth orientation are required");
    }
    return s;
}

}  // namespace

WorkerEnvironment::WorkerEnvironment(EnvironmentSources sources)
    : sources_(std::move(checked(sources))),
      ephemeris_(*sources_.ephemeris, sources_.ephemeris_node_spacing_s),
      earth_(*sources_.earth, sources_.orientation_node_spacing_s),
      moon_orientation_(sources_.moon ? std::optional<frames::InterpolatedOrientation>(
                                            std::in_place, *sources_.moon, sources_.orientation_node_spacing_s)
                                      : std::nullopt),
      gravity_(ephemeris_, earth_, sources_.gravity),
      moon_(moon_orientation_ ? std::optional<environment::ReferenceBody>(
                                    environment::ReferenceBody::moon(ephemeris_, *moon_orientation_))
                              : std::nullopt) {
    env_.emplace(FlightEnvironment{gravity_, atmosphere_, earth_, &ephemeris_, true, moon_ ? &*moon_ : nullptr,
                                   sources_.radiative_heating.get()});
    env_->earth_terrain = sources_.earth_terrain.get();
}

void WorkerEnvironment::copy_flags(const FlightEnvironment& from) {
    auto& e = *env_;
    e.ephemeris = from.ephemeris ? &ephemeris_ : nullptr;
    e.solar_radiation_pressure = from.solar_radiation_pressure;
    e.moon = from.moon && moon_ ? &*moon_ : nullptr;
    e.radiative_heating = from.radiative_heating ? sources_.radiative_heating.get() : nullptr;
    e.air_density_factor = from.air_density_factor;
    e.propellant_settling = from.propellant_settling;
    e.ignition_settling = from.ignition_settling;
    e.actuator_moments_about_cg_point = from.actuator_moments_about_cg_point;
    e.boil_off = from.boil_off;
    e.grid_fin_drag_couple = from.grid_fin_drag_couple;
    // the session's terrain may have been replaced since (set_earth_terrain): the worker holds the one it was given
    e.earth_terrain = from.earth_terrain == sources_.earth_terrain.get() ? sources_.earth_terrain.get() : nullptr;
    if (from.earth_terrain && e.earth_terrain == nullptr) {
        throw std::logic_error("WorkerEnvironment: the session's Earth terrain changed - build a new worker environment");
    }
}

}  // namespace starsim::vehicle
