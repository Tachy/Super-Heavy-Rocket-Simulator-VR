#include "starsim_core.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3i.hpp>
#include <godot_cpp/variant/vector4i.hpp>

#include <array>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "starsim/environment/atmosphere_table_file.hpp"
#include "starsim/environment/eclipse.hpp"
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
    ClassDB::bind_method(D_METHOD("solar_eclipse_fraction", "latitude_deg", "longitude_deg", "height_m", "tdb_s"),
                         &StarsimCore::solar_eclipse_fraction);
    ClassDB::bind_method(D_METHOD("solar_eclipse_fraction_gcrf", "position_gcrf_m", "tdb_s"),
                         &StarsimCore::solar_eclipse_fraction_gcrf);
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
}

}  // namespace starsim::bindings
