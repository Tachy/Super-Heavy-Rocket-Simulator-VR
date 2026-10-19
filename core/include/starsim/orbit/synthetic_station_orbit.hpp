#pragma once

#include <optional>

#include "starsim/frames/body_orientation.hpp"
#include "starsim/math/mat3.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/orbit/oem.hpp"
#include "starsim/time/epoch.hpp"

// Synthetic station orbit for mission dates without OEM coverage: mean elements taken from the latest OEM and advanced
// by the secular J2 rates; the phase (argument of latitude) is chosen by the scenario.
namespace starsim::orbit {

// Rotation from the true equator frame of `epoch` to GCRF: z = the Earth's true pole (ITRF z-axis, the J2 axis of
// the gravity model), x = the GCRF x axis projected onto that equator (close to the usual GCRF RAAN origin, defined
// for any pole tilt; the node on the GCRF equator would lie near GCRF +y in 2026). Station elements refer to this
// frame (the ISS's 51.64 deg are measured from the true equator; in GCRF the 2026 OEM shows ~51.77 deg). [E] the
// frame's own precession rate (~1e-12 rad/s) is ignored when states are rotated.
math::Mat3 true_equator_to_gcrf(const frames::BodyOrientation& earth, time::Epoch epoch);

// Mean elements of a station orbit at `epoch` in the true equator frame of that epoch (J2 short-period terms
// averaged out over one revolution).
struct StationMeanElements {
    time::Epoch epoch;
    double semi_major_axis_m = 0.0;
    double eccentricity = 0.0;
    double inclination_rad = 0.0;
    double raan_rad = 0.0;
    double argument_of_perigee_rad = 0.0;
    double argument_of_latitude_rad = 0.0;
};

// Secular J2 rates (rad/s) of the RAAN, the argument of perigee and the mean argument of latitude.
struct J2Rates {
    double raan_radps, perigee_radps, latitude_radps;
};

J2Rates j2_secular_rates(const StationMeanElements& mean, double gm_m3ps2, double j2, double radius_m);

// Averages the osculating elements of the OEM rows over one revolution from its first row (RAAN and argument of
// latitude unwrapped and taken at the revolution's middle row). Throws OemError for fewer than 10 rows.
StationMeanElements mean_elements_from_oem(const Oem& oem, double gm_m3ps2, const frames::BodyOrientation& earth);

// Mean elements advanced to `epoch` by the J2 rates; the argument of latitude is set by the scenario when given.
StationMeanElements advance_mean_elements(const StationMeanElements& mean, time::Epoch epoch, double gm_m3ps2,
                                          double j2, double radius_m,
                                          std::optional<double> argument_of_latitude_rad = {});

// The state of mean elements used as osculating ones [E: the J2 short-period terms, ~+-10 km in radius, are not
// restored; mean anomaly taken as true anomaly at e < 0.003], rotated to GCRF.
math::StateVector synthetic_station_state(const StationMeanElements& mean, double gm_m3ps2,
                                         const frames::BodyOrientation& earth);

}  // namespace starsim::orbit
