#include "starsim/config/atmosphere_optics_config.hpp"

#include "toml_document.hpp"

namespace starsim::config {

AtmosphereOpticsConfig load_atmosphere_optics(const std::filesystem::path& toml_path) {
    const detail::Document doc(toml_path);
    const toml::table& t = doc.root;
    AtmosphereOpticsConfig c;
    c.top_altitude_m = doc.positive(t, "top_altitude_m");
    c.ground_altitude_m = doc.number(t, "ground_altitude_m");
    if (!(c.ground_altitude_m < c.top_altitude_m)) doc.fail("ground_altitude_m must be below top_altitude_m");
    c.aerosol_optical_depth_550nm = doc.number(t, "aerosol_optical_depth_550nm");
    if (c.aerosol_optical_depth_550nm < 0.0) doc.fail("'aerosol_optical_depth_550nm' must not be negative");
    c.aerosol_angstrom_exponent = doc.number(t, "aerosol_angstrom_exponent");
    c.aerosol_single_scattering_albedo = doc.number(t, "aerosol_single_scattering_albedo");
    if (!(c.aerosol_single_scattering_albedo > 0.0 && c.aerosol_single_scattering_albedo <= 1.0)) {
        doc.fail("'aerosol_single_scattering_albedo' must be in (0, 1]");
    }
    c.aerosol_scale_height_m = doc.positive(t, "aerosol_scale_height_m");
    c.mie_phase_g = doc.number(t, "mie_phase_g");
    if (!(c.mie_phase_g > -1.0 && c.mie_phase_g < 1.0)) doc.fail("'mie_phase_g' must be in (-1, 1)");
    c.ozone_column_du = doc.number(t, "ozone_column_du");
    if (c.ozone_column_du < 0.0) doc.fail("'ozone_column_du' must not be negative");
    c.ground_albedo = doc.number(t, "ground_albedo");
    if (!(c.ground_albedo >= 0.0 && c.ground_albedo <= 1.0)) doc.fail("'ground_albedo' must be in [0, 1]");
    const double orders = doc.number(t, "scattering_orders");
    if (!(orders >= 1.0 && orders <= 16.0) || orders != static_cast<int>(orders)) {
        doc.fail("'scattering_orders' must be an integer in [1, 16]");
    }
    c.scattering_orders = static_cast<int>(orders);
    c.multiple_scattering_tail = doc.boolean(t, "multiple_scattering_tail");
    return c;
}

}  // namespace starsim::config
