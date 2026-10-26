#include "starsim_core.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int64_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3i.hpp>
#include <godot_cpp/variant/vector4i.hpp>

#include <array>
#include <cstring>
#include <filesystem>
#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere_table_file.hpp"
#include "starsim/environment/cloud_noise.hpp"
#include "starsim/environment/cloud_optics.hpp"
#include "starsim/environment/cloud_render_texels.hpp"
#include "starsim/environment/cloud_transfer_table.hpp"
#include "starsim/environment/eclipse.hpp"
#include "starsim/environment/weather_snapshot_build.hpp"
#include "starsim/environment/weather_snapshot_file.hpp"
#include "starsim/environment/weather_source.hpp"
#include "starsim/environment/surface_samples.hpp"
#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/sim/earth_terrain_loader.hpp"
#include "starsim/sim/terrain_cache.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }
math::Vec3 to_core(const godot::Vector3& v) { return {v.x, v.y, v.z}; }

godot::Basis to_godot(const math::Mat3& r) {
    return godot::Basis(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
}

time::Epoch epoch(double tdb_s) { return time::Epoch::from_tdb_seconds_past_j2000(tdb_s); }

ephemeris::Body parse_body(const godot::String& name) {
    const auto lower = name.to_lower();
    if (lower == "sun") return ephemeris::Body::Sun;
    if (lower == "moon") return ephemeris::Body::Moon;
    if (lower == "earth") return ephemeris::Body::Earth;
    throw std::invalid_argument("unknown body (expected sun, moon or earth)");
}

// Runs `fn`, converting core exceptions into a Godot error and a fallback value.
template <typename Fn, typename T>
T guarded(const StarsimCore& core, Fn&& fn, T fallback) {
    if (!core.is_open()) {
        godot::UtilityFunctions::push_error("StarsimCore: call open() first");
        return fallback;
    }
    try {
        return fn();
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("StarsimCore: ") + e.what());
        return fallback;
    }
}

godot::PackedFloat32Array to_packed(const std::vector<float>& values) {
    godot::PackedFloat32Array out;
    out.resize(static_cast<int64_t>(values.size()));
    if (!values.empty()) std::memcpy(out.ptrw(), values.data(), values.size() * sizeof(float));
    return out;
}

godot::Vector3 to_godot(const std::array<double, 3>& v) { return {v[0], v[1], v[2]}; }

godot::Dictionary atmosphere_parameters_dictionary(const environment::AtmosphereParameters& p) {
    godot::Dictionary d;
    d["bottom_radius_m"] = p.bottom_radius_m;
    d["top_radius_m"] = p.top_radius_m;
    d["mu_s_min"] = p.mu_s_min;
    d["mie_phase_g"] = p.mie_phase_g;
    d["sun_angular_radius_rad"] = p.sun_angular_radius_rad;
    d["solar_illuminance_rgb"] = to_godot(p.solar_illuminance_rgb);
    d["sky_radiance_to_luminance"] = to_godot(p.sky_radiance_to_luminance);
    d["sun_radiance_to_luminance"] = to_godot(p.sun_radiance_to_luminance);
    d["toa_solar_illuminance_lux"] = p.toa_solar_illuminance_lux;
    d["ground_albedo"] = p.ground_albedo;
    return d;
}

std::string utf8_of(const godot::String& s) { return s.utf8().get_data(); }

const char* weather_kind_name(environment::WeatherSourceKind kind) {
    switch (kind) {
        case environment::WeatherSourceKind::era5: return "era5";
        case environment::WeatherSourceKind::gfs: return "gfs";
        case environment::WeatherSourceKind::analog: return "analog";
        case environment::WeatherSourceKind::off: break;
    }
    return "off";
}

environment::WeatherSourceKind weather_kind_from_name(const godot::String& name) {
    if (name == "era5") return environment::WeatherSourceKind::era5;
    if (name == "gfs") return environment::WeatherSourceKind::gfs;
    if (name == "analog") return environment::WeatherSourceKind::analog;
    if (name == "off") return environment::WeatherSourceKind::off;
    throw std::invalid_argument("unknown weather source kind '" + utf8_of(name) + "'");
}

void put_weather_id(godot::Dictionary& d, const environment::WeatherSnapshotId& id) {
    d["kind"] = godot::String(weather_kind_name(id.kind));
    d["valid_hour"] = static_cast<int64_t>(id.valid_hour);
    d["cycle_hour"] = static_cast<int64_t>(id.cycle_hour);
    d["lead_h"] = static_cast<int64_t>(id.lead_h);
    d["analog_hour"] = static_cast<int64_t>(id.analog_hour);
    d["cache_key"] = id.kind == environment::WeatherSourceKind::off ? godot::String()
                                                                    : godot::String::utf8(id.cache_key().c_str());
}

godot::Dictionary weather_id_dictionary(const environment::WeatherSnapshotId& id) {
    godot::Dictionary d;
    put_weather_id(d, id);
    return d;
}

environment::WeatherSnapshotId weather_id_from(const godot::Dictionary& d) {
    environment::WeatherSnapshotId id;
    id.kind = weather_kind_from_name(d.get("kind", godot::String("off")));
    id.valid_hour = static_cast<int64_t>(d.get("valid_hour", 0));
    id.cycle_hour = static_cast<int64_t>(d.get("cycle_hour", 0));
    id.lead_h = static_cast<int>(static_cast<int64_t>(d.get("lead_h", 0)));
    id.analog_hour = static_cast<int64_t>(d.get("analog_hour", 0));
    return id;
}

godot::Dictionary weather_failure(const char* german_prefix, const std::exception& e) {
    const godot::String message = godot::String::utf8(german_prefix) + godot::String::utf8(e.what());
    godot::UtilityFunctions::push_error(message);
    godot::Dictionary result;
    result["ok"] = false;
    result["error"] = message;
    return result;
}

constexpr std::size_t weather_cache_size = 3;  // before, after and the one being prepared

}  // namespace

bool StarsimCore::open(const godot::String& kernel_dir) {
    try {
        session_ = std::make_unique<spice::SpiceSession>(
            spice::default_kernel_files(std::filesystem::path(kernel_dir.utf8().get_data())));
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String("StarsimCore.open: ") + e.what());
        return false;
    }
}

double StarsimCore::utc_to_tdb_s(const godot::String& utc_iso) const {
    return guarded(*this, [&] {
        const auto utc = time::UtcDateTime::parse_iso(utc_iso.utf8().get_data());
        if (!utc) {
            throw std::invalid_argument("invalid UTC ISO string");
        }
        return time_.from_utc(*utc).tdb_seconds_past_j2000();
    }, 0.0);
}

godot::String StarsimCore::tdb_s_to_utc(double tdb_s) const {
    return guarded(*this, [&] { return godot::String(time_.to_utc(epoch(tdb_s)).to_iso_string().c_str()); },
                   godot::String());
}

godot::Vector3 StarsimCore::body_position_gcrf_m(const godot::String& body, double tdb_s) const {
    return guarded(*this, [&] { return to_godot(ephemeris_.state_gcrf(parse_body(body), epoch(tdb_s)).position_m); },
                   godot::Vector3());
}

godot::Vector3 StarsimCore::apparent_body_position_gcrf_m(const godot::String& body, double tdb_s,
                                                          const godot::Vector3& observer_position_m,
                                                          const godot::Vector3& observer_velocity_mps) const {
    return guarded(*this, [&] {
        const math::StateVector observer{to_core(observer_position_m), to_core(observer_velocity_mps)};
        return to_godot(ephemeris_.apparent_position_gcrf(parse_body(body), epoch(tdb_s), observer));
    }, godot::Vector3());
}

godot::Basis StarsimCore::earth_fixed_to_gcrf(double tdb_s) const {
    return guarded(*this, [&] { return to_godot(earth_.fixed_to_gcrf(epoch(tdb_s)).rotation); }, godot::Basis());
}

godot::Basis StarsimCore::moon_fixed_to_gcrf(double tdb_s) const {
    return guarded(*this, [&] { return to_godot(moon_.fixed_to_gcrf(epoch(tdb_s)).rotation); }, godot::Basis());
}

godot::Vector3 StarsimCore::geodetic_to_gcrf_m(double latitude_deg, double longitude_deg, double height_m,
                                              double tdb_s) const {
    return guarded(*this, [&] {
        const frames::GeodeticPosition site{latitude_deg * constants::deg_to_rad,
                                            longitude_deg * constants::deg_to_rad, height_m};
        return to_godot(earth_.fixed_to_gcrf(epoch(tdb_s)).position(frames::geodetic_to_fixed(site)));
    }, godot::Vector3());
}

godot::Vector3 StarsimCore::gcrf_to_geodetic(const godot::Vector3& position_gcrf_m, double tdb_s) const {
    return guarded(*this, [&] {
        const math::Vec3 fixed = transpose(earth_.fixed_to_gcrf(epoch(tdb_s)).rotation) * to_core(position_gcrf_m);
        const frames::GeodeticPosition site = frames::fixed_to_geodetic(fixed);
        return godot::Vector3(site.latitude_rad * constants::rad_to_deg, site.longitude_rad * constants::rad_to_deg,
                              site.height_m);
    }, godot::Vector3());
}

double StarsimCore::solar_eclipse_fraction(double latitude_deg, double longitude_deg, double height_m,
                                           double tdb_s) const {
    return guarded(*this, [&] {
        const auto t = epoch(tdb_s);
        const auto transform = earth_.fixed_to_gcrf(t);
        const math::Vec3 fixed = frames::geodetic_to_fixed(
            {latitude_deg * constants::deg_to_rad, longitude_deg * constants::deg_to_rad, height_m});
        const math::StateVector observer{transform.position(fixed), transform.velocity(fixed, {})};
        return environment::occulted_fraction(
            ephemeris_.apparent_position_gcrf(ephemeris::Body::Sun, t, observer), constants::sun_radius_m,
            ephemeris_.apparent_position_gcrf(ephemeris::Body::Moon, t, observer), constants::moon_mean_radius_m);
    }, 0.0);
}

double StarsimCore::solar_eclipse_fraction_gcrf(const godot::Vector3& position_gcrf_m, double tdb_s) const {
    return guarded(*this, [&] {
        const auto t = epoch(tdb_s);
        const math::StateVector observer{to_core(position_gcrf_m), {}};
        return environment::occulted_fraction(
            ephemeris_.apparent_position_gcrf(ephemeris::Body::Sun, t, observer), constants::sun_radius_m,
            ephemeris_.apparent_position_gcrf(ephemeris::Body::Moon, t, observer), constants::moon_mean_radius_m);
    }, 0.0);
}

godot::Dictionary StarsimCore::atmospheric_refraction(const godot::Vector3& camera_fixed_m) const {
    return guarded(*this, [&] {
        const math::Vec3 fixed = to_core(camera_fixed_m);
        const double altitude_m = frames::fixed_to_geodetic(fixed).height_m;
        const auto table = environment::refraction_lift_table(refraction_, norm(fixed) - altitude_m, altitude_m);
        godot::PackedFloat32Array lift;
        lift.resize(static_cast<int64_t>(table.lift_rad.size()));
        for (std::size_t i = 0; i < table.lift_rad.size(); ++i) {
            lift.set(static_cast<int64_t>(i), static_cast<float>(table.lift_rad[i]));
        }
        godot::Dictionary result;
        result["min_rad"] = table.true_elevation_min_rad;
        result["span_rad"] = table.true_elevation_span_rad;
        result["lift_rad"] = lift;
        return result;
    }, godot::Dictionary());
}

godot::Dictionary StarsimCore::load_terrain(const godot::String& config_dir) {
    const std::filesystem::path config(config_dir.utf8().get_data());
    const auto data_dir = config.parent_path() / "data";
    const auto say = [](const std::string& message) { godot::UtilityFunctions::print(godot::String::utf8(message.c_str())); };
    earth_terrain_.reset();
    moon_terrain_.reset();
    if (!sim::earth_terrain_available(data_dir)) {
        say("Erdgelände fehlt - flacher Boden (python tools/data_pipeline/fetch.py)");
    } else {
        try {
            earth_terrain_ = sim::shared_earth_terrain(config, data_dir);
        } catch (const std::exception& e) {
            say(std::string("Erdgelände nicht ladbar - flacher Boden: ") + e.what());
        }
    }
    if (!std::filesystem::exists(data_dir / "terrain" / "ldem_16.toml")) {
        say("Mondgelände fehlt - glatte Kugel (python tools/data_pipeline/fetch.py)");
    } else {
        try {
            moon_terrain_ = sim::shared_lunar_terrain(config, data_dir);
        } catch (const std::exception& e) {
            say(std::string("Mondgelände nicht ladbar - glatte Kugel: ") + e.what());
        }
    }
    godot::Dictionary result;
    result["earth"] = earth_terrain_ != nullptr;
    result["moon"] = moon_terrain_ != nullptr;
    return result;
}

const environment::Terrain* StarsimCore::terrain_of(const godot::String& body) const {
    const auto lower = body.to_lower();
    if (lower == "earth") return earth_terrain_.get();
    if (lower == "moon") return moon_terrain_.get();
    return nullptr;
}

godot::Dictionary StarsimCore::surface_samples(const godot::String& body, const godot::PackedFloat64Array& latitudes_deg,
                                               const godot::PackedFloat64Array& longitudes_deg) const {
    godot::Dictionary result;
    const environment::Terrain* terrain = terrain_of(body);
    godot::PackedFloat64Array heights;
    godot::PackedByteArray water;
    const bool present = terrain != nullptr && latitudes_deg.size() == longitudes_deg.size();
    if (terrain != nullptr && !present) {
        godot::UtilityFunctions::push_error("StarsimCore.surface_samples: Breiten und Längen müssen gleich viele sein");
    }
    if (present) {
        const auto n = static_cast<size_t>(latitudes_deg.size());
        std::vector<double> lat(n), lon(n);
        for (size_t i = 0; i < n; ++i) {
            lat[i] = latitudes_deg[static_cast<int64_t>(i)] * constants::deg_to_rad;
            lon[i] = longitudes_deg[static_cast<int64_t>(i)] * constants::deg_to_rad;
        }
        const auto samples = environment::sample_surface(*terrain, lat, lon);
        heights.resize(static_cast<int64_t>(n));
        water.resize(static_cast<int64_t>(n));
        for (size_t i = 0; i < n; ++i) {
            heights[static_cast<int64_t>(i)] = samples.height_m[i];
            water[static_cast<int64_t>(i)] = samples.water[i];
        }
    }
    result["present"] = present;
    result["height_m"] = heights;
    result["water"] = water;
    return result;
}

double StarsimCore::surface_resolution_m(const godot::String& body, double latitude_deg, double longitude_deg) const {
    const environment::Terrain* terrain = terrain_of(body);
    if (terrain == nullptr) return std::numeric_limits<double>::infinity();
    return terrain->resolution_m(latitude_deg * constants::deg_to_rad, longitude_deg * constants::deg_to_rad);
}

double StarsimCore::surface_finest_resolution_m(const godot::String& body,
                                                const godot::PackedFloat64Array& latitudes_deg,
                                                const godot::PackedFloat64Array& longitudes_deg) const {
    const environment::Terrain* terrain = terrain_of(body);
    if (terrain == nullptr) return std::numeric_limits<double>::infinity();
    if (latitudes_deg.size() != longitudes_deg.size()) {
        godot::UtilityFunctions::push_error(
            "StarsimCore.surface_finest_resolution_m: Breiten und Längen müssen gleich viele sein");
        return std::numeric_limits<double>::infinity();
    }
    const auto n = static_cast<size_t>(latitudes_deg.size());
    std::vector<double> lat(n), lon(n);
    const double* lat_deg = latitudes_deg.ptr();
    const double* lon_deg = longitudes_deg.ptr();
    for (size_t i = 0; i < n; ++i) {
        lat[i] = lat_deg[i] * constants::deg_to_rad;
        lon[i] = lon_deg[i] * constants::deg_to_rad;
    }
    return environment::finest_resolution_m(*terrain, lat, lon);
}

std::shared_ptr<const environment::AtmosphereScattering> StarsimCore::atmosphere() const {
    const std::lock_guard<std::mutex> lock(atmosphere_mutex_);
    return atmosphere_;
}

godot::Dictionary StarsimCore::load_atmosphere(const godot::String& config_dir, const godot::String& cache_dir) {
    godot::Dictionary result;
    try {
        // The slow part (cache read or precompute) runs off-lock; only the swap is locked.
        const auto tables = environment::load_or_compute_atmosphere(
            std::filesystem::path(config_dir.utf8().get_data()), std::filesystem::path(cache_dir.utf8().get_data()));
        auto scattering = std::make_shared<const environment::AtmosphereScattering>(tables);
        {
            const std::lock_guard<std::mutex> lock(atmosphere_mutex_);
            atmosphere_ = std::move(scattering);
        }
        const auto& s = tables->sizes;
        result["ok"] = true;
        result["error"] = godot::String();
        result["transmittance"] = to_packed(tables->transmittance);
        result["transmittance_size"] = godot::Vector2i(s.transmittance_mu, s.transmittance_r);
        result["scattering"] = to_packed(tables->scattering);
        result["single_mie"] = to_packed(tables->single_mie);
        result["scattering_size"] = godot::Vector3i(s.scattering_width(), s.scattering_height(), s.scattering_depth());
        result["scattering_cells"] = godot::Vector4i(s.scattering_nu, s.scattering_mu_s, s.scattering_mu, s.scattering_r);
        result["irradiance"] = to_packed(tables->irradiance);
        result["irradiance_size"] = godot::Vector2i(s.irradiance_mu_s, s.irradiance_r);
        result["parameters"] = atmosphere_parameters_dictionary(tables->parameters);
    } catch (const std::exception& e) {
        const godot::String message = godot::String::utf8("Atmosphäre konnte nicht geladen werden: ") + godot::String::utf8(e.what());
        godot::UtilityFunctions::push_error(message);
        result["ok"] = false;
        result["error"] = message;
    }
    return result;
}

godot::Vector3 StarsimCore::atmosphere_sun_transmittance(const godot::Vector3& position_itrf_m,
                                                         const godot::Vector3& sun_direction_itrf) const {
    const auto a = atmosphere();
    if (!a) return {};
    return to_godot(a->sun_transmittance(to_core(position_itrf_m), to_core(sun_direction_itrf)));
}

godot::Dictionary StarsimCore::atmosphere_sky_luminance(const godot::Vector3& camera_itrf_m,
                                                        const godot::Vector3& view_direction_itrf,
                                                        const godot::Vector3& sun_direction_itrf) const {
    godot::Dictionary d;
    math::Vec3 transmittance{};
    math::Vec3 luminance{};
    if (const auto a = atmosphere()) {
        luminance = a->sky_luminance(to_core(camera_itrf_m), to_core(view_direction_itrf),
                                     to_core(sun_direction_itrf), &transmittance);
    }
    d["luminance"] = to_godot(luminance);
    d["transmittance"] = to_godot(transmittance);
    return d;
}

godot::Dictionary StarsimCore::atmosphere_sky_luminance_to_point(const godot::Vector3& camera_itrf_m,
                                                                 const godot::Vector3& point_itrf_m,
                                                                 const godot::Vector3& sun_direction_itrf) const {
    godot::Dictionary d;
    math::Vec3 transmittance{};
    math::Vec3 luminance{};
    if (const auto a = atmosphere()) {
        luminance = a->sky_luminance_to_point(to_core(camera_itrf_m), to_core(point_itrf_m),
                                              to_core(sun_direction_itrf), &transmittance);
    }
    d["luminance"] = to_godot(luminance);
    d["transmittance"] = to_godot(transmittance);
    return d;
}

godot::Dictionary StarsimCore::atmosphere_illuminance(const godot::Vector3& point_itrf_m,
                                                      const godot::Vector3& normal_itrf,
                                                      const godot::Vector3& sun_direction_itrf) const {
    godot::Dictionary d;
    environment::AtmosphereScattering::Illuminance lux{};
    if (const auto a = atmosphere()) {
        lux = a->sun_and_sky_illuminance(to_core(point_itrf_m), to_core(normal_itrf), to_core(sun_direction_itrf));
    }
    d["sun_lux"] = to_godot(lux.sun_lux);
    d["sky_lux"] = to_godot(lux.sky_lux);
    return d;
}

godot::Dictionary StarsimCore::weather_bracket(double utc_hour_f, int64_t step_h, const godot::String& era5_zattrs,
                                               int64_t latest_gfs_cycle_hour, int64_t analog_year) const {
    try {
        std::optional<environment::Era5Availability> era5;
        if (!era5_zattrs.is_empty()) era5 = environment::parse_era5_zattrs(utf8_of(era5_zattrs));
        std::optional<environment::GfsCycle> gfs;
        if (latest_gfs_cycle_hour >= 0) gfs = environment::GfsCycle{latest_gfs_cycle_hour};
        const auto bracket = environment::weather_bracket(utc_hour_f, static_cast<int>(step_h), era5, gfs,
                                                          static_cast<int>(analog_year));
        godot::Dictionary result;
        result["ok"] = true;
        result["error"] = godot::String();
        result["before"] = weather_id_dictionary(bracket.before);
        result["after"] = weather_id_dictionary(bracket.after);
        result["blend"] = bracket.blend;
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Wetterquelle konnte nicht gewählt werden: ", e);
    }
}

godot::Array StarsimCore::weather_requests(const godot::Dictionary& snapshot, const godot::String& idx_text) const {
    godot::Array out;
    try {
        const auto id = weather_id_from(snapshot);
        std::vector<environment::WeatherRequest> requests;
        if (id.kind == environment::WeatherSourceKind::gfs) {
            requests = environment::gfs_requests(id, utf8_of(idx_text));
        } else if (id.kind != environment::WeatherSourceKind::off) {
            requests = environment::era5_requests(id);
        }
        for (const auto& r : requests) {
            godot::Dictionary d;
            d["url"] = godot::String::utf8(r.url.c_str());
            d["first_byte"] = static_cast<int64_t>(r.first_byte);
            d["last_byte"] = static_cast<int64_t>(r.last_byte);
            d["file_name"] = godot::String::utf8(r.file_name.c_str());
            out.push_back(d);
        }
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String::utf8("Wetter-Downloads konnten nicht geplant werden: ") +
                                            godot::String::utf8(e.what()));
        out.clear();
    }
    return out;
}

godot::String StarsimCore::weather_gfs_index_url(const godot::Dictionary& snapshot) const {
    try {
        return godot::String::utf8(environment::gfs_index_url(weather_id_from(snapshot)).c_str());
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String::utf8("GFS-Index-Adresse ungültig: ") +
                                            godot::String::utf8(e.what()));
        return {};
    }
}

godot::PackedInt64Array StarsimCore::weather_gfs_cycle_candidates(double wall_utc_hour_f, int64_t count) const {
    godot::PackedInt64Array out;
    try {
        for (const auto& c : environment::gfs_cycle_candidates(wall_utc_hour_f, static_cast<int>(count))) {
            out.push_back(c.cycle_hour);
        }
    } catch (const std::exception& e) {
        weather_failure("GFS-Zyklen konnten nicht bestimmt werden: ", e);
        out.clear();
    }
    return out;
}

int64_t StarsimCore::weather_snapshot_step(double warp, double seconds_per_download) const {
    try {
        return environment::weather_snapshot_step(warp, seconds_per_download);
    } catch (const std::exception& e) {
        weather_failure("Schrittweite der Wetter-Momentaufnahmen ungültig: ", e);
        return 0;  // hold
    }
}

double StarsimCore::weather_utc_hour(double epoch_tdb_s) const {
    return guarded(*this, [&] {
        const time::UtcDateTime utc = time_.to_utc(epoch(epoch_tdb_s));
        return static_cast<double>(environment::hours_since_1900(utc)) + utc.minute / 60.0 + utc.second / 3600.0;
    }, 0.0);
}

godot::Dictionary StarsimCore::weather_build_snapshot(const godot::Dictionary& snapshot, const godot::String& raw_dir,
                                                      const godot::String& out_file,
                                                      const godot::String& config_dir) const {
    try {
        const auto built = environment::build_weather_snapshot_from_raw(
            weather_id_from(snapshot), std::filesystem::path(utf8_of(raw_dir)), std::filesystem::path(utf8_of(config_dir)));
        const std::filesystem::path out(utf8_of(out_file));
        if (out.has_parent_path()) std::filesystem::create_directories(out.parent_path());
        environment::write_weather_snapshot(out, built);
        {  // a rebuilt file must not be served from the load cache
            const std::lock_guard<std::mutex> lock(weather_mutex_);
            std::erase_if(weather_cache_, [&](const auto& entry) { return entry.first == utf8_of(out_file); });
        }
        godot::Dictionary result;
        result["ok"] = true;
        result["error"] = godot::String();
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Wetter-Momentaufnahme konnte nicht berechnet werden: ", e);
    }
}

StarsimCore::SnapshotPtr StarsimCore::weather_snapshot_of(const std::string& file) {
    {
        const std::lock_guard<std::mutex> lock(weather_mutex_);
        for (const auto& [name, ptr] : weather_cache_) {
            if (name == file) return ptr;
        }
    }
    // The slow part (reading 133 MB and its checksum) runs off-lock.
    auto read = environment::read_weather_snapshot(std::filesystem::path(file));
    if (!read) return nullptr;
    auto ptr = std::make_shared<const environment::WeatherSnapshot>(std::move(*read));
    const std::lock_guard<std::mutex> lock(weather_mutex_);
    std::erase_if(weather_cache_, [&](const auto& entry) { return entry.first == file; });
    weather_cache_.emplace_back(file, ptr);
    while (weather_cache_.size() > weather_cache_size) weather_cache_.erase(weather_cache_.begin());
    return ptr;
}

godot::Dictionary StarsimCore::weather_load_snapshot(const godot::String& file) {
    try {
        const auto snapshot = weather_snapshot_of(utf8_of(file));
        if (!snapshot) {
            throw std::runtime_error("Datei fehlt, ist beschädigt oder hat ein anderes Format: " + utf8_of(file));
        }
        const std::vector<std::uint8_t> texels = environment::cloud_render_texels(*snapshot);
        godot::Dictionary result;
        result["ok"] = true;
        result["error"] = godot::String();
        result["rows"] = static_cast<int64_t>(snapshot->rows());
        result["columns"] = static_cast<int64_t>(snapshot->columns());
        result["layers"] = static_cast<int64_t>(environment::weather_layer_count);
        godot::PackedByteArray bytes;
        bytes.resize(static_cast<int64_t>(texels.size()));
        if (!texels.empty()) std::memcpy(bytes.ptrw(), texels.data(), texels.size());
        result["cloud_texels"] = bytes;
        const std::vector<std::uint8_t> ice = environment::cloud_render_ice_texels(*snapshot);
        godot::PackedByteArray ice_bytes;
        ice_bytes.resize(static_cast<int64_t>(ice.size()));
        if (!ice.empty()) std::memcpy(ice_bytes.ptrw(), ice.data(), ice.size());
        result["cloud_ice_texels"] = ice_bytes;
        const std::vector<std::uint8_t> column = environment::cloud_render_column_texels(*snapshot);
        godot::PackedByteArray column_bytes;
        column_bytes.resize(static_cast<int64_t>(column.size()));
        if (!column.empty()) std::memcpy(column_bytes.ptrw(), column.data(), column.size());
        result["cloud_column_texels"] = column_bytes;
        godot::PackedFloat64Array bottoms;
        for (int i = 0; i <= environment::weather_layer_count; ++i) bottoms.push_back(environment::weather_layer_bottom_m(i));
        result["layer_bottoms_m"] = bottoms;
        put_weather_id(result, snapshot->id());
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Wetter-Momentaufnahme konnte nicht geladen werden: ", e);
    }
}

godot::Dictionary StarsimCore::weather_set_active(const godot::String& before_file, const godot::String& after_file,
                                                  double blend) {
    try {
        SnapshotPtr before, after;
        if (!before_file.is_empty() && !(before = weather_snapshot_of(utf8_of(before_file)))) {
            throw std::runtime_error("Datei nicht lesbar: " + utf8_of(before_file));
        }
        if (!after_file.is_empty() && !(after = weather_snapshot_of(utf8_of(after_file)))) {
            throw std::runtime_error("Datei nicht lesbar: " + utf8_of(after_file));
        }
        {
            const std::lock_guard<std::mutex> lock(weather_mutex_);
            weather_before_ = std::move(before);
            weather_after_ = std::move(after);
            weather_blend_ = std::clamp(blend, 0.0, 1.0);
        }
        godot::Dictionary result;
        result["ok"] = true;
        result["error"] = godot::String();
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Aktives Wetter konnte nicht gesetzt werden: ", e);
    }
}

godot::Dictionary StarsimCore::weather_cloud_light(const godot::Vector3& position_itrf_m,
                                                   const godot::Vector3& sun_direction_itrf) const {
    double sun = 1.0, sky = 1.0, diffuse = 0.0;
    try {
    SnapshotPtr before, after;
    double blend = 0.0;
    {
        const std::lock_guard<std::mutex> lock(weather_mutex_);
        before = weather_before_;
        after = weather_after_;
        blend = weather_blend_;
    }
    std::shared_ptr<const environment::CloudSlabTransmittanceLut> lut;
    {
        const std::lock_guard<std::mutex> lock(transfer_mutex_);
        lut = transmittance_lut_;
    }
    if (before || after) {
        const math::Vec3 p = to_core(position_itrf_m), s = to_core(sun_direction_itrf);
        // The renderer's order: blend the column depth and cover of both snapshots, then the transmittances.
        const double mu0 = environment::cloud_sun_cosine_to_vertical(p, s);
        const auto column = [&](const SnapshotPtr& snap) { return environment::cloud_column_depths_at(*snap, p); };
        const auto ray = [&](const SnapshotPtr& snap) { return environment::cloud_sun_ray_depths(*snap, p, s); };
        environment::CloudColumnDepths ray_depths, column_depths;
        if (before && after) {
            ray_depths = environment::blend_cloud_column_depths(ray(before), ray(after), blend);
            column_depths = environment::blend_cloud_column_depths(column(before), column(after), blend);
        } else {
            ray_depths = ray(before ? before : after);
            column_depths = column(before ? before : after);
        }
        sun = environment::cloud_sun_transmittance(ray_depths, mu0);
        sky = environment::cloud_sky_factor(column_depths);
        diffuse = environment::cloud_sun_diffuse_transmittance(column_depths, lut.get(), mu0);
    }
    } catch (const std::exception& e) {
        weather_failure("Wolkenlicht konnte nicht berechnet werden: ", e);
        sun = 1.0;
        sky = 1.0;
        diffuse = 0.0;
    }
    godot::Dictionary result;
    result["sun_transmittance"] = sun;
    result["sky_factor"] = sky;
    result["sun_diffuse"] = diffuse;
    return result;
}

godot::Dictionary StarsimCore::cloud_noise() const {
    try {
        const environment::CloudNoise noise = environment::generate_cloud_noise();
        const auto bytes = [](const std::vector<std::uint8_t>& v) {
            godot::PackedByteArray out;
            out.resize(static_cast<int64_t>(v.size()));
            if (!v.empty()) std::memcpy(out.ptrw(), v.data(), v.size());
            return out;
        };
        const auto rows = [](const auto& table) {
            godot::PackedFloat32Array out;
            for (const auto& row : table) {
                for (float v : row) out.push_back(v);
            }
            return out;
        };
        godot::Dictionary result;
        result["shape"] = bytes(noise.shape_rgba);
        result["shape_size"] = static_cast<int64_t>(noise.shape_size);
        result["detail"] = bytes(noise.detail_rgba);
        result["detail_size"] = static_cast<int64_t>(noise.detail_size);
        result["detail_states"] = static_cast<int64_t>(environment::cloud_detail_states);
        result["coverage_threshold"] = rows(noise.coverage_threshold);
        result["edge_mean_factor"] = rows(noise.edge_mean_factor);
        godot::PackedFloat32Array amplitudes;
        for (float a : noise.level_amplitude) amplitudes.push_back(a);
        result["level_amplitude"] = amplitudes;
        result["detail_mean"] = noise.detail_mean;
        result["detail_weights"] =
            godot::Vector3(noise.detail_weights[0], noise.detail_weights[1], noise.detail_weights[2]);
        result["detail_tile_m"] = environment::cloud_detail_tile_m;
        result["detail_level_ratio"] = environment::cloud_detail_level_ratio;
        result["edge_width_min_value"] = noise.edge_width_min_value;
        result["edge_width_max_value"] = noise.edge_width_max_value;
        result["blue_noise"] = bytes(noise.blue_noise);
        result["blue_noise_size"] = static_cast<int64_t>(noise.blue_noise_size);
        godot::Array mips;
        for (const auto& level : noise.shape_mips) mips.push_back(bytes(level));
        result["shape_mips"] = mips;
        result["prefilter_threshold"] = rows(noise.prefilter_threshold);
        result["prefilter_edge_mean"] = rows(noise.prefilter_edge_mean);
        result["prefilter_rows"] = static_cast<int64_t>(environment::cloud_shape_prefilter_rows);
        result["prefilter_rows_per_level"] = static_cast<int64_t>(environment::cloud_shape_prefilter_rows_per_level);
        result["prefilter_levels"] = static_cast<int64_t>(environment::cloud_shape_prefilter_levels);
        result["prefilter_start_m"] = environment::cloud_shape_prefilter_start_m;
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Wolkenrauschen konnte nicht erzeugt werden: ", e);
    }
}

godot::Dictionary StarsimCore::cloud_optics_parameters() const {
    const environment::CloudPhaseParameters phase =
        environment::cloud_phase_parameters(2.0 * constants::cloud_droplet_effective_radius_m);
    godot::Dictionary result;
    result["hg_g"] = phase.hg_g;
    result["draine_g"] = phase.draine_g;
    result["draine_alpha"] = phase.draine_alpha;
    result["draine_weight"] = phase.draine_weight;
    result["asymmetry_g"] = environment::cloud_asymmetry_g();
    result["transport_scale"] = environment::cloud_transport_scale();
    result["extinction_step_per_m"] = environment::cloud_texel_extinction_step_per_m();
    result["depth_code_min"] = environment::cloud_depth_code_min;
    result["depth_code_max"] = environment::cloud_depth_code_max;
    return result;
}

double StarsimCore::cloud_slab_radiance(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                                        double relative_azimuth_rad) const {
    return environment::slab_reflected_radiance(optical_depth, sun_cos_zenith, view_cos_zenith, relative_azimuth_rad);
}

godot::Dictionary StarsimCore::cloud_transfer_table(const godot::String& file) {
    try {
        auto table = std::make_shared<const environment::CloudTransferTable>(
            environment::load_cloud_transfer_table(std::filesystem::path(utf8_of(file))));
        const environment::CloudTransferTexels gpu = environment::cloud_transfer_gpu_texels(*table);
        godot::PackedByteArray texels;
        texels.resize(static_cast<int64_t>(gpu.rgba_half.size() * sizeof(std::uint16_t)));
        std::memcpy(texels.ptrw(), gpu.rgba_half.data(), gpu.rgba_half.size() * sizeof(std::uint16_t));
        godot::Dictionary result;
        result["ok"] = true;
        result["texels"] = texels;
        result["size"] = godot::Vector3i(gpu.width, gpu.height, gpu.depth);
        result["nodes"] = godot::Vector4i(table->depth_nodes, table->mu_nodes, table->theta_nodes, table->psi_nodes);
        result["depth_map"] = godot::Vector3(table->depth_scale, std::log(table->depth_ratio), table->diffusion_k);
        result["theta_map"] = godot::Vector3(table->theta0, table->theta_log, table->theta_linear);
        // The slab's total transmittance (light under the clouds, Task 9): ~1 s on all cores.
        auto lut = std::make_shared<const environment::CloudSlabTransmittanceLut>(
            environment::make_cloud_slab_transmittance_lut(*table));
        using L = environment::CloudSlabTransmittanceLut;
        godot::PackedFloat32Array lut_values;
        for (double v : lut->total) lut_values.push_back(static_cast<float>(v));
        result["transmittance_lut"] = lut_values;
        result["transmittance_lut_size"] = godot::Vector2i(L::depth_nodes, L::cosine_nodes);
        result["transmittance_lut_map"] =
            godot::Vector4(L::depth_scale, std::log(lut->depth_ratio), L::depth_max, L::cosine_min);
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        transfer_table_ = std::move(table);
        transmittance_lut_ = std::move(lut);
        return result;
    } catch (const std::exception& e) {
        return weather_failure("Wolken-Transfertabelle konnte nicht gelesen werden: ", e);
    }
}

double StarsimCore::cloud_slab_radiance_mc(double optical_depth, double sun_cos_zenith, double view_cos_zenith,
                                           double relative_azimuth_rad) const {
    std::shared_ptr<const environment::CloudTransferTable> table;
    {
        std::lock_guard<std::mutex> lock(transfer_mutex_);
        table = transfer_table_;
    }
    if (!table) return -1.0;
    return environment::slab_reflected_radiance_mc(*table, optical_depth, sun_cos_zenith, view_cos_zenith,
                                                   relative_azimuth_rad);
}

bool StarsimCore::weather_write_slab_snapshot(const godot::String& file, double bottom_m, double top_m,
                                              double liquid_water_kg_m3, double cloud_fraction) const {
    try {
        environment::write_weather_snapshot(
            std::filesystem::path(utf8_of(file)),
            environment::WeatherSnapshot::uniform_slab(181, 360, bottom_m, top_m, liquid_water_kg_m3, cloud_fraction));
        return true;
    } catch (const std::exception& e) {
        godot::UtilityFunctions::push_error(godot::String::utf8("Wolkenplatte konnte nicht geschrieben werden: ") +
                                            godot::String::utf8(e.what()));
        return false;
    }
}

void StarsimCore::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("open", "kernel_dir"), &StarsimCore::open);
    ClassDB::bind_method(D_METHOD("is_open"), &StarsimCore::is_open);
    ClassDB::bind_method(D_METHOD("utc_to_tdb_s", "utc_iso"), &StarsimCore::utc_to_tdb_s);
    ClassDB::bind_method(D_METHOD("tdb_s_to_utc", "tdb_s"), &StarsimCore::tdb_s_to_utc);
    ClassDB::bind_method(D_METHOD("body_position_gcrf_m", "body", "tdb_s"), &StarsimCore::body_position_gcrf_m);
    ClassDB::bind_method(D_METHOD("apparent_body_position_gcrf_m", "body", "tdb_s", "observer_position_m",
                                  "observer_velocity_mps"),
                         &StarsimCore::apparent_body_position_gcrf_m);
    ClassDB::bind_method(D_METHOD("earth_fixed_to_gcrf", "tdb_s"), &StarsimCore::earth_fixed_to_gcrf);
    ClassDB::bind_method(D_METHOD("moon_fixed_to_gcrf", "tdb_s"), &StarsimCore::moon_fixed_to_gcrf);
    ClassDB::bind_method(D_METHOD("geodetic_to_gcrf_m", "latitude_deg", "longitude_deg", "height_m", "tdb_s"),
                         &StarsimCore::geodetic_to_gcrf_m);
    ClassDB::bind_method(D_METHOD("gcrf_to_geodetic", "position_gcrf_m", "tdb_s"), &StarsimCore::gcrf_to_geodetic);
    ClassDB::bind_method(D_METHOD("solar_eclipse_fraction", "latitude_deg", "longitude_deg", "height_m", "tdb_s"),
                         &StarsimCore::solar_eclipse_fraction);
    ClassDB::bind_method(D_METHOD("solar_eclipse_fraction_gcrf", "position_gcrf_m", "tdb_s"),
                         &StarsimCore::solar_eclipse_fraction_gcrf);
    ClassDB::bind_method(D_METHOD("atmospheric_refraction", "camera_fixed_m"), &StarsimCore::atmospheric_refraction);
    ClassDB::bind_method(D_METHOD("load_terrain", "config_dir"), &StarsimCore::load_terrain);
    ClassDB::bind_method(D_METHOD("surface_samples", "body", "latitudes_deg", "longitudes_deg"),
                         &StarsimCore::surface_samples);
    ClassDB::bind_method(D_METHOD("surface_resolution_m", "body", "latitude_deg", "longitude_deg"),
                         &StarsimCore::surface_resolution_m);
    ClassDB::bind_method(D_METHOD("surface_finest_resolution_m", "body", "latitudes_deg", "longitudes_deg"),
                         &StarsimCore::surface_finest_resolution_m);
    ClassDB::bind_method(D_METHOD("load_atmosphere", "config_dir", "cache_dir"), &StarsimCore::load_atmosphere);
    ClassDB::bind_method(D_METHOD("atmosphere_sun_transmittance", "position_itrf_m", "sun_direction_itrf"),
                         &StarsimCore::atmosphere_sun_transmittance);
    ClassDB::bind_method(D_METHOD("atmosphere_sky_luminance", "camera_itrf_m", "view_direction_itrf",
                                  "sun_direction_itrf"),
                         &StarsimCore::atmosphere_sky_luminance);
    ClassDB::bind_method(D_METHOD("atmosphere_sky_luminance_to_point", "camera_itrf_m", "point_itrf_m",
                                  "sun_direction_itrf"),
                         &StarsimCore::atmosphere_sky_luminance_to_point);
    ClassDB::bind_method(D_METHOD("atmosphere_illuminance", "point_itrf_m", "normal_itrf", "sun_direction_itrf"),
                         &StarsimCore::atmosphere_illuminance);
    ClassDB::bind_method(D_METHOD("weather_bracket", "utc_hour_f", "step_h", "era5_zattrs", "latest_gfs_cycle_hour",
                                  "analog_year"), &StarsimCore::weather_bracket);
    ClassDB::bind_method(D_METHOD("weather_requests", "snapshot", "idx_text"), &StarsimCore::weather_requests);
    ClassDB::bind_method(D_METHOD("weather_gfs_index_url", "snapshot"), &StarsimCore::weather_gfs_index_url);
    ClassDB::bind_method(D_METHOD("weather_gfs_cycle_candidates", "wall_utc_hour_f", "count"),
                         &StarsimCore::weather_gfs_cycle_candidates);
    ClassDB::bind_method(D_METHOD("weather_snapshot_step", "warp", "seconds_per_download"),
                         &StarsimCore::weather_snapshot_step);
    ClassDB::bind_method(D_METHOD("weather_utc_hour", "epoch_tdb_s"), &StarsimCore::weather_utc_hour);
    ClassDB::bind_method(D_METHOD("weather_build_snapshot", "snapshot", "raw_dir", "out_file", "config_dir"),
                         &StarsimCore::weather_build_snapshot);
    ClassDB::bind_method(D_METHOD("weather_load_snapshot", "file"), &StarsimCore::weather_load_snapshot);
    ClassDB::bind_method(D_METHOD("weather_set_active", "before_file", "after_file", "blend"),
                         &StarsimCore::weather_set_active);
    ClassDB::bind_method(D_METHOD("weather_cloud_light", "position_itrf_m", "sun_direction_itrf"),
                         &StarsimCore::weather_cloud_light);
    ClassDB::bind_method(D_METHOD("cloud_noise"), &StarsimCore::cloud_noise);
    ClassDB::bind_method(D_METHOD("cloud_optics_parameters"), &StarsimCore::cloud_optics_parameters);
    ClassDB::bind_method(D_METHOD("cloud_slab_radiance", "optical_depth", "sun_cos_zenith", "view_cos_zenith",
                                  "relative_azimuth_rad"), &StarsimCore::cloud_slab_radiance);
    ClassDB::bind_method(D_METHOD("cloud_transfer_table", "file"), &StarsimCore::cloud_transfer_table);
    ClassDB::bind_method(D_METHOD("cloud_slab_radiance_mc", "optical_depth", "sun_cos_zenith", "view_cos_zenith",
                                  "relative_azimuth_rad"), &StarsimCore::cloud_slab_radiance_mc);
    ClassDB::bind_method(D_METHOD("weather_write_slab_snapshot", "file", "bottom_m", "top_m", "liquid_water_kg_m3",
                                  "cloud_fraction"), &StarsimCore::weather_write_slab_snapshot);
}

}  // namespace starsim::bindings
