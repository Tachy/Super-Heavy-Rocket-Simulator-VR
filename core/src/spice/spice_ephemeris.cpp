#include <cstdio>

#include "spice_check.hpp"
#include "starsim/spice/spice_adapters.hpp"

namespace starsim::spice {

namespace {

constexpr double km_to_m = 1000.0;
constexpr SpiceInt earth_naif_id = 399;

SpiceInt naif_id(ephemeris::Body body) {
    switch (body) {
        case ephemeris::Body::Sun: return 10;
        case ephemeris::Body::Earth: return earth_naif_id;
        case ephemeris::Body::Moon: return 301;
        case ephemeris::Body::EarthMoonBarycenter: return 3;
    }
    throw SpiceError("unknown body");
}

}  // namespace

ephemeris::StateVector SpiceEphemeris::state_gcrf(ephemeris::Body target, time::Epoch epoch) const {
    if (target == ephemeris::Body::Earth) {
        return {};
    }
    SpiceDouble state_km[6];
    SpiceDouble light_time_s = 0.0;
    spkgeo_c(naif_id(target), epoch.tdb_seconds_past_j2000(), "J2000", earth_naif_id, state_km, &light_time_s);
    detail::throw_if_failed("spkgeo_c");
    return {{state_km[0] * km_to_m, state_km[1] * km_to_m, state_km[2] * km_to_m},
            {state_km[3] * km_to_m, state_km[4] * km_to_m, state_km[5] * km_to_m}};
}

math::Vec3 SpiceEphemeris::apparent_position_gcrf(ephemeris::Body target, time::Epoch epoch,
                                                  const math::StateVector& observer_gcrf) const {
    const double et = epoch.tdb_seconds_past_j2000();
    SpiceDouble observer_km[6] = {
        observer_gcrf.position_m.x / km_to_m,   observer_gcrf.position_m.y / km_to_m,
        observer_gcrf.position_m.z / km_to_m,   observer_gcrf.velocity_mps.x / km_to_m,
        observer_gcrf.velocity_mps.y / km_to_m, observer_gcrf.velocity_mps.z / km_to_m};
    SpiceChar target_id[16];
    std::snprintf(target_id, sizeof(target_id), "%d", static_cast<int>(naif_id(target)));
    SpiceDouble state_km[6];
    SpiceDouble light_time_s = 0.0;
    // Constant-velocity observer about the Earth's centre; converged light time + stellar aberration.
    spkcvo_c(target_id, et, "J2000", "OBSERVER", "CN+S", observer_km, et, "EARTH", "J2000", state_km,
             &light_time_s);
    detail::throw_if_failed("spkcvo_c");
    return {state_km[0] * km_to_m, state_km[1] * km_to_m, state_km[2] * km_to_m};
}

double SpiceEphemeris::gm_m3ps2(ephemeris::Body body) const {
    SpiceInt count = 0;
    SpiceDouble gm_km3ps2 = 0.0;
    bodvcd_c(naif_id(body), "GM", 1, &count, &gm_km3ps2);
    detail::throw_if_failed("bodvcd_c GM");
    return gm_km3ps2 * km_to_m * km_to_m * km_to_m;
}

}  // namespace starsim::spice
