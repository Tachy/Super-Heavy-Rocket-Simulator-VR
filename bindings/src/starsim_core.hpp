#pragma once

#include <memory>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

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

    // Rotation ITRF -> GCRF (rows of the rotation matrix in Basis rows).
    godot::Basis earth_fixed_to_gcrf(double tdb_s) const;

    // WGS84 geodetic position (degrees, metres) -> GCRF metres.
    godot::Vector3 geodetic_to_gcrf_m(double latitude_deg, double longitude_deg, double height_m, double tdb_s) const;

protected:
    static void _bind_methods();

private:
    std::unique_ptr<spice::SpiceSession> session_;
    spice::SpiceTimeConverter time_;
    spice::SpiceEphemeris ephemeris_;
    spice::SpiceOrientation earth_ = spice::SpiceOrientation::earth();
};

}  // namespace starsim::bindings
