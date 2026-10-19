#include "starsim/orbit/synthetic_station_orbit.hpp"

#include <cmath>
#include <vector>

#include "starsim/math/constants.hpp"
#include "starsim/orbit/kepler.hpp"

namespace starsim::orbit {

namespace {

constexpr size_t min_rows_per_revolution = 10;

double wrap_two_pi(double angle_rad) {
    const double a = std::fmod(angle_rad, constants::two_pi);
    return a < 0.0 ? a + constants::two_pi : a;
}

// Continues `angle_rad` from `previous_rad` without 2 pi jumps.
double unwrap(double angle_rad, double previous_rad) {
    return previous_rad + std::remainder(angle_rad - previous_rad, constants::two_pi);
}

math::StateVector rotated(const math::Mat3& r, const math::StateVector& s) {
    return {r * s.position_m, r * s.velocity_mps};
}

}  // namespace

math::Mat3 true_equator_to_gcrf(const frames::BodyOrientation& earth, time::Epoch epoch) {
    const math::Vec3 pole = math::normalized(earth.fixed_to_gcrf(epoch).rotation * math::Vec3{0.0, 0.0, 1.0});
    // x: the GCRF x axis projected onto the true equator (well defined for any pole tilt, close to the usual GCRF
    // RAAN origin); the node of the true equator on the GCRF equator would sit near GCRF +y in 2026 and be undefined
    // at J2000.
    const math::Vec3 gcrf_x{1.0, 0.0, 0.0};
    const math::Vec3 x = math::normalized(gcrf_x - dot(gcrf_x, pole) * pole);
    return math::Mat3::from_columns(x, math::cross(pole, x), pole);
}

// Vallado, Fundamentals of Astrodynamics and Applications, 4th ed., eq. 9-41 [S]: secular J2 rates of the node,
// the perigee and the mean anomaly; the mean argument of latitude advances by the sum of the last two.
J2Rates j2_secular_rates(const StationMeanElements& mean, double gm_m3ps2, double j2, double radius_m) {
    const double a = mean.semi_major_axis_m;
    const double e2 = mean.eccentricity * mean.eccentricity;
    const double n = std::sqrt(gm_m3ps2 / (a * a * a));
    const double p = a * (1.0 - e2);
    const double k = 1.5 * j2 * (radius_m / p) * (radius_m / p) * n;
    const double sin2_i = std::sin(mean.inclination_rad) * std::sin(mean.inclination_rad);
    const double perigee = k * (2.0 - 2.5 * sin2_i);
    const double mean_anomaly = n + k * std::sqrt(1.0 - e2) * (1.0 - 1.5 * sin2_i);
    return {-k * std::cos(mean.inclination_rad), perigee, mean_anomaly + perigee};
}

StationMeanElements mean_elements_from_oem(const Oem& oem, double gm_m3ps2, const frames::BodyOrientation& earth) {
    if (oem.row_count() < min_rows_per_revolution) throw OemError("OEM too short for mean elements");
    const auto equator_state = [&](size_t i) {
        return rotated(transpose(true_equator_to_gcrf(earth, oem.row_epoch(i))), oem.row_state(i));
    };
    const auto first = elements_from_state(equator_state(0), gm_m3ps2);
    const double period_s = constants::two_pi * std::sqrt(std::pow(first.semi_major_axis_m, 3) / gm_m3ps2);
    const time::Epoch t0 = oem.row_epoch(0);

    std::vector<size_t> rows;
    for (size_t i = 0; i < oem.row_count() && oem.row_epoch(i) - t0 < period_s; ++i) rows.push_back(i);
    if (rows.size() < min_rows_per_revolution) throw OemError("OEM too short for mean elements");

    double sum_a = 0.0, sum_e = 0.0, sum_i = 0.0, sum_raan = 0.0, sum_perigee = 0.0;
    double raan = first.raan_rad, perigee = first.argument_of_periapsis_rad;
    double latitude = first.argument_of_periapsis_rad + first.true_anomaly_rad;
    const size_t middle = rows.size() / 2;
    double latitude_middle = latitude;
    for (size_t k = 0; k < rows.size(); ++k) {
        const auto el = elements_from_state(equator_state(rows[k]), gm_m3ps2);
        raan = unwrap(el.raan_rad, raan);
        perigee = unwrap(el.argument_of_periapsis_rad, perigee);
        latitude = unwrap(el.argument_of_periapsis_rad + el.true_anomaly_rad, latitude);
        sum_a += el.semi_major_axis_m;
        sum_e += el.eccentricity;
        sum_i += el.inclination_rad;
        sum_raan += raan;
        sum_perigee += perigee;
        if (k == middle) latitude_middle = latitude;
    }
    const double count = static_cast<double>(rows.size());
    StationMeanElements mean;
    mean.epoch = oem.row_epoch(rows[middle]);
    mean.semi_major_axis_m = sum_a / count;
    mean.eccentricity = sum_e / count;
    mean.inclination_rad = sum_i / count;
    mean.raan_rad = wrap_two_pi(sum_raan / count);
    mean.argument_of_perigee_rad = wrap_two_pi(sum_perigee / count);
    mean.argument_of_latitude_rad = wrap_two_pi(latitude_middle);
    return mean;
}

StationMeanElements advance_mean_elements(const StationMeanElements& mean, time::Epoch epoch, double gm_m3ps2,
                                          double j2, double radius_m, std::optional<double> argument_of_latitude_rad) {
    const auto rates = j2_secular_rates(mean, gm_m3ps2, j2, radius_m);
    const double dt_s = epoch - mean.epoch;
    StationMeanElements out = mean;
    out.epoch = epoch;
    out.raan_rad = wrap_two_pi(mean.raan_rad + rates.raan_radps * dt_s);
    out.argument_of_perigee_rad = wrap_two_pi(mean.argument_of_perigee_rad + rates.perigee_radps * dt_s);
    out.argument_of_latitude_rad = argument_of_latitude_rad
                                       ? wrap_two_pi(*argument_of_latitude_rad)
                                       : wrap_two_pi(mean.argument_of_latitude_rad + rates.latitude_radps * dt_s);
    return out;
}

math::StateVector synthetic_station_state(const StationMeanElements& mean, double gm_m3ps2,
                                         const frames::BodyOrientation& earth) {
    // [E] mean anomaly ~ true anomaly at e < 0.003 (difference 2 e sin M <= 0.35 deg along track)
    const KeplerianElements el{mean.semi_major_axis_m,
                               mean.eccentricity,
                               mean.inclination_rad,
                               mean.raan_rad,
                               mean.argument_of_perigee_rad,
                               mean.argument_of_latitude_rad - mean.argument_of_perigee_rad};
    return rotated(true_equator_to_gcrf(earth, mean.epoch), state_from_elements(el, gm_m3ps2));
}

}  // namespace starsim::orbit
