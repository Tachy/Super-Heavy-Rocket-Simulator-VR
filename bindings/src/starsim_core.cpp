#include "starsim_core.hpp"

#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "starsim/frames/geodetic.hpp"
#include "starsim/math/constants.hpp"

namespace starsim::bindings {

namespace {

godot::Vector3 to_godot(const math::Vec3& v) { return {v.x, v.y, v.z}; }

time::Epoch epoch(double tdb_s) { return time::Epoch::from_tdb_seconds_past_j2000(tdb_s); }

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
    return guarded(*this, [&] {
        const auto lower = body.to_lower();
        ephemeris::Body target;
        if (lower == "sun") {
            target = ephemeris::Body::Sun;
        } else if (lower == "moon") {
            target = ephemeris::Body::Moon;
        } else if (lower == "earth") {
            target = ephemeris::Body::Earth;
        } else {
            throw std::invalid_argument("unknown body");
        }
        return to_godot(ephemeris_.state_gcrf(target, epoch(tdb_s)).position_m);
    }, godot::Vector3());
}

godot::Basis StarsimCore::earth_fixed_to_gcrf(double tdb_s) const {
    return guarded(*this, [&] {
        const auto r = earth_.fixed_to_gcrf(epoch(tdb_s)).rotation;
        return godot::Basis(r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2));
    }, godot::Basis());
}

godot::Vector3 StarsimCore::geodetic_to_gcrf_m(double latitude_deg, double longitude_deg, double height_m,
                                              double tdb_s) const {
    return guarded(*this, [&] {
        const frames::GeodeticPosition site{latitude_deg * constants::deg_to_rad,
                                            longitude_deg * constants::deg_to_rad, height_m};
        return to_godot(earth_.fixed_to_gcrf(epoch(tdb_s)).position(frames::geodetic_to_fixed(site)));
    }, godot::Vector3());
}

void StarsimCore::_bind_methods() {
    using godot::ClassDB;
    using godot::D_METHOD;
    ClassDB::bind_method(D_METHOD("open", "kernel_dir"), &StarsimCore::open);
    ClassDB::bind_method(D_METHOD("is_open"), &StarsimCore::is_open);
    ClassDB::bind_method(D_METHOD("utc_to_tdb_s", "utc_iso"), &StarsimCore::utc_to_tdb_s);
    ClassDB::bind_method(D_METHOD("tdb_s_to_utc", "tdb_s"), &StarsimCore::tdb_s_to_utc);
    ClassDB::bind_method(D_METHOD("body_position_gcrf_m", "body", "tdb_s"), &StarsimCore::body_position_gcrf_m);
    ClassDB::bind_method(D_METHOD("earth_fixed_to_gcrf", "tdb_s"), &StarsimCore::earth_fixed_to_gcrf);
    ClassDB::bind_method(D_METHOD("geodetic_to_gcrf_m", "latitude_deg", "longitude_deg", "height_m", "tdb_s"),
                         &StarsimCore::geodetic_to_gcrf_m);
}

}  // namespace starsim::bindings
