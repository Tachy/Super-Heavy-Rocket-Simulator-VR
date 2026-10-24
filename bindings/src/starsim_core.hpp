#pragma once

#include <memory>
#include <mutex>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "starsim/environment/atmosphere_scattering.hpp"
#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/lunar_terrain.hpp"

#include "starsim/spice/spice_adapters.hpp"
#include "starsim/spice/spice_session.hpp"

namespace starsim::bindings {

// Godot-facing facade of the simulation core. Adapter only: converts types and forwards calls.
// Godot is built with precision=double, so Vector3/Basis carry full float64 values.
class StarsimCore : public godot::RefCounted {
    GDCLASS(StarsimCore, godot::RefCounted)

public:
    // Loads the SPICE kernel set from `kernel_dir`. Returns false (and logs) on error.
    bool open(const godot::String& kernel_dir);
    bool is_open() const { return session_ != nullptr; }

    // UTC ISO 8601 string -> TDB seconds past J2000.
    double utc_to_tdb_s(const godot::String& utc_iso) const;
    godot::String tdb_s_to_utc(double tdb_s) const;

    // Geocentric GCRF position of "sun" / "moon" in metres.
    godot::Vector3 body_position_gcrf_m(const godot::String& body, double tdb_s) const;

    // Apparent (light-time + aberration corrected) position of a body as seen by an observer whose
    // state is given relative to the Earth's centre in GCRF. Returns the vector observer -> body.
    godot::Vector3 apparent_body_position_gcrf_m(const godot::String& body, double tdb_s,
                                                 const godot::Vector3& observer_position_m,
                                                 const godot::Vector3& observer_velocity_mps) const;

    // Rotations body-fixed -> GCRF (rows of the rotation matrix in Basis rows).
    // Earth: ITRF93. Moon: MOON_ME, the frame lunar textures are registered in.
    godot::Basis earth_fixed_to_gcrf(double tdb_s) const;
    godot::Basis moon_fixed_to_gcrf(double tdb_s) const;

    // WGS84 geodetic position (degrees, metres) -> GCRF metres.
    godot::Vector3 geodetic_to_gcrf_m(double latitude_deg, double longitude_deg, double height_m, double tdb_s) const;

    // Fraction of the solar disk hidden by the Moon for a ground observer (0 = none, 1 = total).
    double solar_eclipse_fraction(double latitude_deg, double longitude_deg, double height_m, double tdb_s) const;

    // Loads the Earth and Moon terrain through the process-wide cache (config_dir/../data). Returns
    // {earth: bool, moon: bool}; prints one German line per missing or unloadable terrain. Call before the
    // surface queries below, from one thread.
    godot::Dictionary load_terrain(const godot::String& config_dir);

    // Terrain heights and water flags at latitude / longitude points (degrees; body "earth" or "moon"):
    // {present, height_m: PackedFloat64Array above the WGS84 ellipsoid / lunar sphere, water: PackedByteArray}.
    // Thread-safe (immutable terrains, no SPICE): callable from WorkerThreadPool threads after load_terrain.
    godot::Dictionary surface_samples(const godot::String& body, const godot::PackedFloat64Array& latitudes_deg,
                                      const godot::PackedFloat64Array& longitudes_deg) const;
    // Spacing in metres of the finest terrain grid at a point; INF without the terrain.
    double surface_resolution_m(const godot::String& body, double latitude_deg, double longitude_deg) const;
    // The finest of those spacings over latitude / longitude points (degrees; a renderer node's lattice); INF without
    // the terrain or points. Thread-safe like surface_samples.
    double surface_finest_resolution_m(const godot::String& body, const godot::PackedFloat64Array& latitudes_deg,
                                       const godot::PackedFloat64Array& longitudes_deg) const;

    // Atmosphere (M8): loads <cache_dir>/earth_atmosphere.bin or precomputes it (config_dir/atmosphere/earth_optics.toml)
    // and keeps the lookups for the calls below. {ok, error, transmittance / scattering / single_mie / irradiance:
    // PackedFloat32Array RGBA, *_size, parameters: Dictionary of the AtmosphereParameters fields}. Callable from a
    // worker thread. Texel order (AtmosphereTables): 4 floats per texel, x fastest, then y, then z. transmittance:
    // x = mu, y = r (transmittance_size); irradiance: x = mu_s, y = r; scattering / single_mie: x packs (nu, mu_s) as
    // x = nu_index * scattering_mu_s + mu_s_index (mu_s fastest), y = mu, z = r (scattering_size = width, height, depth;
    // scattering_cells: Vector4i(nu, mu_s, mu, r) cell counts).
    godot::Dictionary load_atmosphere(const godot::String& config_dir, const godot::String& cache_dir);
    // The lookups take ITRF metres and unit directions; before a successful load they return zeros.
    godot::Vector3 atmosphere_sun_transmittance(const godot::Vector3& position_itrf_m,
                                                const godot::Vector3& sun_direction_itrf) const;
    // {luminance: Vector3 cd/m^2, transmittance: Vector3}
    godot::Dictionary atmosphere_sky_luminance(const godot::Vector3& camera_itrf_m,
                                               const godot::Vector3& view_direction_itrf,
                                               const godot::Vector3& sun_direction_itrf) const;
    godot::Dictionary atmosphere_sky_luminance_to_point(const godot::Vector3& camera_itrf_m,
                                                        const godot::Vector3& point_itrf_m,
                                                        const godot::Vector3& sun_direction_itrf) const;
    // {sun_lux: Vector3, sky_lux: Vector3}
    godot::Dictionary atmosphere_illuminance(const godot::Vector3& point_itrf_m, const godot::Vector3& normal_itrf,
                                             const godot::Vector3& sun_direction_itrf) const;

protected:
    static void _bind_methods();

private:
    const environment::Terrain* terrain_of(const godot::String& body) const;
    std::shared_ptr<const environment::AtmosphereScattering> atmosphere() const;

    std::shared_ptr<const environment::EarthTerrain> earth_terrain_;  // load_terrain(); nullptr when absent
    std::shared_ptr<const environment::LunarTerrain> moon_terrain_;
    mutable std::mutex atmosphere_mutex_;  // guards atmosphere_ (swapped by load_atmosphere on any thread)
    std::shared_ptr<const environment::AtmosphereScattering> atmosphere_;
    std::unique_ptr<spice::SpiceSession> session_;
    spice::SpiceTimeConverter time_;
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_ = spice::SpiceOrientation::moon_mean_earth();  // map frame
};

}  // namespace starsim::bindings
