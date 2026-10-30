#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/packed_int64_array.hpp>
#include <godot_cpp/variant/packed_vector2_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "starsim/environment/atmospheric_refraction.hpp"
#include "starsim/environment/atmosphere_scattering.hpp"
#include "starsim/environment/earth_terrain.hpp"
#include "starsim/environment/lunar_terrain.hpp"
#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_render_field.hpp"
#include "starsim/environment/cloud_transfer_table.hpp"
#include "starsim/environment/weather_snapshot.hpp"

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
    // GCRF metres (Earth-centred) -> WGS84 geodetic Vector3(latitude_deg, longitude_deg, height_m above the
    // ellipsoid), e.g. the camera's local time zone.
    godot::Vector3 gcrf_to_geodetic(const godot::Vector3& position_gcrf_m, double tdb_s) const;
    // Height in metres of a GCRF point above mean sea level (the EGM2008 geoid: the Earth terrain's when loaded, else
    // the committed 1-degree grid); NAN without either (load_terrain not called / no config).
    double sea_level_height_m(const godot::Vector3& position_gcrf_m, double tdb_s) const;

    // Fraction of the solar disk hidden by the Moon for a ground observer (0 = none, 1 = total).
    double solar_eclipse_fraction(double latitude_deg, double longitude_deg, double height_m, double tdb_s) const;
    // The same for an observer anywhere (GCRF metres, at rest in GCRF: the aberration of its own motion is ignored,
    // < 0.01 % of the disc): the sunlight on a vehicle or on the camera (game: Sun light, eye adaptation).
    double solar_eclipse_fraction_gcrf(const godot::Vector3& position_gcrf_m, double tdb_s) const;
    // Astronomical refraction for a camera at an Earth-fixed position (ITRF metres): {min_rad, span_rad, lift_rad:
    // PackedFloat32Array; empty lift: none}, the lift (apparent - true elevation) over the true elevation
    // on the grid min + span (i/(n-1))^2 (environment::refraction_lift_table, US76 density, the sphere of the local
    // ellipsoid radius). Renderer: Sun, Moon and stars drawn at their refracted directions.
    godot::Dictionary atmospheric_refraction(const godot::Vector3& camera_fixed_m) const;

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

    // Terrain horizon (terrain shadows; the Moon's sphere only, "earth" and unknown bodies have none): horizon
    // elevations (rad, + above the local horizontal) of an n x n vertex lattice of a terrain node (row-major,
    // n = sqrt(size), n - 1 divisible by 16; latitudes / longitudes in degrees, heights in metres above the sphere,
    // spacing_m the vertex spacing) at azimuth_count azimuths k 2 pi / azimuth_count (clockwise from each vertex's
    // north): vertex-major, azimuth-minor, banded bake (environment::horizon_elevations_lattice). layer_major
    // (azimuth_count divisible by 4): the same values reordered (azimuth / 4, vertex, azimuth % 4), so each group of
    // four azimuths is one contiguous RGBA image. Empty without the terrain or on invalid sizes. Thread-safe like
    // surface_samples.
    godot::PackedFloat32Array horizon_elevations(const godot::String& body,
                                                 const godot::PackedFloat64Array& latitudes_deg,
                                                 const godot::PackedFloat64Array& longitudes_deg,
                                                 const godot::PackedFloat64Array& heights_m, int azimuth_count,
                                                 double spacing_m, bool layer_major = false) const;
    // One full march (environment::horizon_elevation_rad, from the data resolution there); NAN without the terrain.
    double horizon_elevation_rad(const godot::String& body, double latitude_deg, double longitude_deg, double height_m,
                                 double azimuth_rad) const;
    // The terrain's highest point above its reference sphere (Terrain::max_height_m); NAN without the terrain.
    double terrain_max_height_m(const godot::String& body) const;

    // Atmosphere (M8): loads <cache_dir>/earth_atmosphere.bin or precomputes it (config_dir/atmosphere/earth_optics.toml)
    // and keeps the lookups for the calls below. {ok, error, transmittance / scattering / single_mie /
    // single_rayleigh / irradiance: PackedFloat32Array RGBA, *_size, parameters: Dictionary of the
    // AtmosphereParameters fields}. Callable from a worker thread. Texel order (AtmosphereTables): 4 floats per
    // texel, x fastest, then y, then z. transmittance:
    // x = mu, y = r (transmittance_size); irradiance: x = mu_s, y = r; scattering / single_mie: x packs (nu, mu_s) as
    // x = nu_index * scattering_mu_s + mu_s_index (mu_s fastest), y = mu, z = r (scattering_size = width, height, depth;
    // scattering_cells: Vector4i(nu, mu_s, mu, r) cell counts).
    godot::Dictionary load_atmosphere(const godot::String& config_dir, const godot::String& cache_dir);
    // The lookups take ITRF metres and unit directions; before a successful load they return zeros.
    godot::Vector3 atmosphere_sun_transmittance(const godot::Vector3& position_itrf_m,
                                                const godot::Vector3& sun_direction_itrf) const;
    // Checks only (cloud world volume, core cloud_volume_bake.hpp): the CPU reference of the cell bake of ONE snapshot
    // file - (rho, sigma_c) in 1/m. Positions are Earth-local render axes (polar +Y).
    // shape_origins_earth_m: 3 values (the anchor point; its noise origins as CloudLayer computes them) or 12 (anchor,
    // then the cumulus, stratiform and ice origins the GPU got). The snapshot's render field and the noise are loaded
    // once and kept for the next call. Empty / (-1, -1) on an unreadable file or malformed input (error pushed).
    // Single cell: cell_m = (horizontal, vertical) m, the box east / north / up at the centre.
    godot::Vector2 weather_cloud_cell_bake(const godot::String& snapshot_path, const godot::Vector3& centre_earth_m,
                                           const godot::Vector2& cell_m,
                                           const godot::PackedFloat64Array& shape_origins_earth_m);
    // Batched: cells with 12 values each (centre, then the full edge vectors e_x, e_y, e_z - the grid's own axes);
    // baked in parallel, one (rho, sigma_c) per cell.
    godot::PackedVector2Array weather_cloud_cell_bakes(const godot::String& snapshot_path,
                                                       const godot::PackedFloat64Array& cells,
                                                       const godot::PackedFloat64Array& shape_origins_earth_m);
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

    // Weather (M8 weather and clouds). Snapshot dictionaries: {kind: "era5" | "gfs" | "analog" | "off", valid_hour,
    // cycle_hour, lead_h, analog_hour: int, cache_key: String}; valid hours are hours since 1900-01-01 00:00 UTC. All of
    // these are callable from a worker thread; errors are German push_error lines and `ok = false` (or an empty result).
    // The two snapshots bracketing utc_hour_f for the warp step (1, 3, 6 h) and the blend between them:
    // {ok, error, before, after, blend}. Empty era5_zattrs and latest_gfs_cycle_hour < 0 mean unknown (offline).
    godot::Dictionary weather_bracket(double utc_hour_f, int64_t step_h, const godot::String& era5_zattrs,
                                      int64_t latest_gfs_cycle_hour, int64_t analog_year) const;
    // The downloads of a snapshot: [{url, first_byte, last_byte (-1: whole object / open end), file_name}]; idx_text
    // is the GFS index file's text (ignored for ERA5 / analog). Empty with an error when a record is missing.
    godot::Array weather_requests(const godot::Dictionary& snapshot, const godot::String& idx_text) const;
    godot::String weather_gfs_index_url(const godot::Dictionary& snapshot) const;
    // Cycle hours (hours since 1900) to probe, newest first, from the cycle at or before the wall clock.
    godot::PackedInt64Array weather_gfs_cycle_candidates(double wall_utc_hour_f, int64_t count) const;
    // 1, 3 or 6 (hours between snapshots) or 0 = hold, for the warp factor and the wall seconds per snapshot download.
    int64_t weather_snapshot_step(double warp, double seconds_per_download) const;
    // Fractional hours since 1900-01-01 00:00 UTC of a TDB epoch (leap seconds via the time converter).
    double weather_utc_hour(double epoch_tdb_s) const;
    // Decodes the raw files of `snapshot` in raw_dir into the cache file out_file (the geoid from config_dir).
    godot::Dictionary weather_build_snapshot(const godot::Dictionary& snapshot, const godot::String& raw_dir,
                                             const godot::String& out_file, const godot::String& config_dir) const;
    // Reads a cache file: {ok, error, rows, columns, layers, cloud_texels: PackedByteArray (layer-major RGBA8, the GPU
    // form environment::cloud_render_texels: cloud fraction, 16-bit linear extinction, in-cloud depth code),
    // cloud_ice_texels: PackedByteArray (layer-major R8, environment::cloud_render_ice_texels: the ice share of the
    // extinction), cloud_column_texels: PackedByteArray (layer-major RG8, environment::cloud_render_column_texels: the
    // grid-mean column depth above each layer centre (R, log code) and the largest cloud fraction above it (G)), cloud_base_texels: PackedByteArray (rows x columns RG32F as bytes, environment::cloud_render_base_texels: the flat cloud base and its layer's extinction scale, 0 = none), cloud_top_m: float (environment::cloud_render_top_m: no cloud above it), layer_bottoms_m:PackedFloat64Array (layers + 1), kind, valid_hour, cycle_hour, lead_h, analog_hour, cache_key, file (the path as given)}.
    // The game builds the label from the id fields. The snapshot stays in a small cache (the last three files) for
    // weather_set_active. Call it on a worker: the GPU form of a 0.25-deg snapshot is 133 MB.
    godot::Dictionary weather_load_snapshot(const godot::String& file);
    // The pair the CPU light queries blend: cache files ("" = none: the empty snapshot, so a fade from / to no weather
    // blends toward zero depths), blend 0..1. Cache only, never reads: a file that is not in the 3-entry load cache
    // gives {ok = false, error "Wetterdatei nicht geladen"} and leaves the active pair as it was - load it on a worker
    // with weather_load_snapshot() and call again. A cache hit becomes the newest entry.
    godot::Dictionary weather_set_active(const godot::String& before_file, const godot::String& after_file, double blend);
    // The files in the load cache, oldest first (the cache keeps the last three).
    godot::PackedStringArray weather_cached_files() const;
    // {sun_transmittance, sky_factor, sun_diffuse} of the active pair, blended linearly (cloud_optics.hpp: the direct
    // Sun, the sky's diffuse transmittance, the Sun's light diffused through the column per unit of its horizontal
    // irradiance - with the Monte Carlo table once cloud_transfer_table() ran, else delta-Eddington); {1, 1, 0}
    // without an active snapshot.
    godot::Dictionary weather_cloud_light(const godot::Vector3& position_itrf_m,
                                          const godot::Vector3& sun_direction_itrf) const;
    // The tileable cloud noise: {shape: PackedByteArray RGBA8, shape_size, detail, detail_size, detail_states,
    // coverage_threshold and edge_mean_factor: PackedFloat32Array (256 x detail_states, one row of 256 per state),
    // level_amplitude and level_lift: PackedFloat32Array (detail_states - 1), type_constants: {shader constant name:
    // value} (cloud_noise.hpp cloud types), detail_mean, detail_tile_m, detail_level_ratio,
    // edge_width_min_value, edge_width_max_value, blue_noise: PackedByteArray R8, blue_noise_size, the shape prefilter:
    // shape_mips: Array of PackedByteArray RGBA8 (levels 1 .. log2(shape_size)), prefilter_threshold and
    // prefilter_edge_mean: PackedFloat32Array (256 x prefilter_rows), prefilter_rows, prefilter_rows_per_level,
    // prefilter_levels, prefilter_start_m} (cloud_noise.hpp).
    godot::Dictionary cloud_noise() const;
    // The cloud optics the shader needs as numbers (cloud_optics.hpp, cloud_render_texels.hpp): {hg_g, draine_g,
    // draine_alpha, draine_weight, asymmetry_g, transport_scale, extinction_step_per_m, depth_code_min, depth_code_max}.
    godot::Dictionary cloud_optics_parameters() const;
    // Checks only: environment::slab_reflected_radiance (1/sr per unit Sun irradiance).
    double cloud_slab_radiance(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                               double relative_azimuth_rad) const;
    // Loads the Monte Carlo transfer table (environment::load_cloud_transfer_table) and keeps it for
    // cloud_slab_radiance_mc: {ok, error, texels: PackedByteArray (RGBA half floats, cloud_transfer_gpu_texels), size:
    // Vector3i, nodes: Vector4i (depth, mu0, theta, psi), depth_map: Vector3 (scale, ln ratio, K), theta_map: Vector3
    // (theta0, log, linear), transmittance_lut: PackedFloat32Array (the slab's total beam transmittance,
    // CloudSlabTransmittanceLut: cosine rows of depth nodes), transmittance_lut_size: Vector2i (depth, cosine nodes),
    // transmittance_lut_map: Vector4 (depth scale, ln depth ratio, depth max, cosine min)}. Call it on a worker (reads
    // ~4.5 MB, builds the transmittance table on all cores in ~1 s).
    godot::Dictionary cloud_transfer_table(const godot::String& file);
    // Checks only: environment::slab_reflected_radiance_mc with the loaded table (-1 without one).
    double cloud_slab_radiance_mc(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                                  double relative_azimuth_rad) const;
    // Checks only: writes WeatherSnapshot::uniform_slab(181, 360, ...) to `file`.
    bool weather_write_slab_snapshot(const godot::String& file, double bottom_m, double top_m,
                                     double liquid_water_kg_m3, double cloud_fraction) const;
    // Checks only: writes WeatherSnapshot::gfs_test_column(181, 360, ...) (a snapshot built from fields, the LCL raise
    // included) to `file`; {ok, optical_depth (environment::cloud_render_column_optical_depth), ground_m, lcl_m} of the
    // cell nearest to (latitude_deg, longitude_deg).
    godot::Dictionary weather_write_column_snapshot(const godot::String& file, double latitude_deg,
                                                    double longitude_deg, double cloud_top_m, double liquid_kg_kg, double cloud_percent,
                                                    double t2m_k, double td2m_k) const;

protected:
    static void _bind_methods();

private:
    const environment::Terrain* terrain_of(const godot::String& body) const;
    std::shared_ptr<const environment::AtmosphereScattering> atmosphere() const;

    std::shared_ptr<const environment::EarthTerrain> earth_terrain_;  // load_terrain(); nullptr when absent
    std::shared_ptr<const environment::LunarTerrain> moon_terrain_;
    std::shared_ptr<const environment::Geoid> coarse_geoid_;  // load_terrain(): config's 1-degree grid; nullptr if unreadable
    mutable std::mutex atmosphere_mutex_;  // guards atmosphere_ (swapped by load_atmosphere on any thread)
    std::shared_ptr<const environment::AtmosphereScattering> atmosphere_;
    using SnapshotPtr = std::shared_ptr<const environment::WeatherSnapshot>;
    SnapshotPtr weather_snapshot_cached(const std::string& file);  // load cache only (a hit becomes the newest); nullptr if absent
    SnapshotPtr weather_snapshot_of(const std::string& file);  // load cache, else reads off-lock; nullptr if unreadable
    void weather_evict(const std::string& file) const;  // a rewritten file must not be served from the load cache
    mutable std::mutex weather_mutex_;  // guards the three below (the game calls from the main thread, loads on a worker)
    mutable std::vector<std::pair<std::string, SnapshotPtr>> weather_cache_;  // last loaded files, newest last
    SnapshotPtr weather_before_, weather_after_;
    double weather_blend_ = 0.0;
    // The cell bake reference's inputs (weather_cloud_cell_bakes): the noise and the last file's render field.
    struct BakeInputs {
        std::shared_ptr<const environment::CloudNoise> noise;
        std::shared_ptr<const environment::CloudRenderField> field;
    };
    BakeInputs bake_inputs(const std::string& file);
    std::mutex bake_mutex_;  // guards the three below
    std::shared_ptr<const environment::CloudNoise> bake_noise_;
    SnapshotPtr bake_snapshot_;  // the snapshot bake_field_ was built from (a rewritten file loads a new one)
    std::shared_ptr<const environment::CloudRenderField> bake_field_;
    mutable std::mutex transfer_mutex_;  // guards transfer_table_ (loaded on a worker, read by the checks)
    std::shared_ptr<const environment::CloudTransferTable> transfer_table_;
    std::shared_ptr<const environment::CloudSlabTransmittanceLut> transmittance_lut_;  // guarded by transfer_mutex_
    environment::RefractionProfile refraction_{environment::Us76Atmosphere{}};
    std::unique_ptr<spice::SpiceSession> session_;
    spice::SpiceTimeConverter time_;
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
    spice::SpiceOrientation moon_ = spice::SpiceOrientation::moon_mean_earth();  // map frame
};

}  // namespace starsim::bindings
