#include "starsim/config/vehicle_config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

#include <toml++/toml.hpp>

#include "starsim/math/constants.hpp"
#include "toml_document.hpp"

namespace starsim::config {

namespace {

constexpr double landing_propellant_tolerance_kg = 1.0;  // landing_propellant_kg vs the header contents

using detail::Document;

math::Vec3 vec3(const Document& doc, const toml::table& t, std::string_view key) {
    const auto* list = t[key].as_array();
    if (!list || list->size() != 3) doc.fail(std::string(key) + " must be [x, y, z]");
    double v[3];
    for (int k = 0; k < 3; ++k) {
        const auto value = (*list)[static_cast<std::size_t>(k)].value<double>();
        if (!value) doc.fail(std::string(key) + " must be [x, y, z]");
        v[k] = *value;
    }
    return {v[0], v[1], v[2]};
}

std::vector<double> number_list(const Document& doc, const toml::table& t, std::string_view key) {
    const auto* list = t[key].as_array();
    if (!list || list->empty()) doc.fail("missing list '" + std::string(key) + "'");
    std::vector<double> out;
    for (const auto& node : *list) {
        const auto value = node.value<double>();
        if (!value) doc.fail("'" + std::string(key) + "' must hold numbers");
        out.push_back(*value);
    }
    return out;
}

std::vector<std::string> text_list(const Document& doc, const toml::table& t, std::string_view key) {
    const auto* list = t[key].as_array();
    if (!list || list->empty()) doc.fail("missing list '" + std::string(key) + "'");
    std::vector<std::string> out;
    for (const auto& node : *list) {
        const auto value = node.value<std::string>();
        if (!value) doc.fail("'" + std::string(key) + "' must hold strings");
        out.push_back(*value);
    }
    return out;
}

// [[stage.rcs.pod]] / [[stage.rcs_pod]] entries: a set of directions at each azimuth of a ring -> thrusters.
void append_rcs_pods(const Document& doc, const toml::array& pods, std::vector<RcsThrusterConfig>& out) {
    for (const auto& node : pods) {
        const auto& p = *node.as_table();
        const std::string id = doc.text(p, "id");
        const double station = doc.number(p, "station_m");
        const double radius = doc.positive(p, "radius_m");
        const double thrust = doc.positive(p, "thrust_n");
        const auto azimuths = number_list(doc, p, "azimuths_deg");
        const auto directions = text_list(doc, p, "directions");
        const bool behind = p["behind_nose_cone"].value_or(false);
        for (size_t k = 0; k < azimuths.size(); ++k) {
            const double a = azimuths[k] * constants::deg_to_rad;
            const math::Vec3 radial{0.0, std::cos(a), std::sin(a)}, tangential{0.0, -std::sin(a), std::cos(a)};
            for (const auto& d : directions) {
                math::Vec3 direction;
                if (d == "tangential_plus") direction = tangential;
                else if (d == "tangential_minus") direction = -tangential;
                else if (d == "inward") direction = -radial;
                else if (d == "forward") direction = {1.0, 0.0, 0.0};
                else if (d == "aft") direction = {-1.0, 0.0, 0.0};
                else doc.fail("stage.rcs.pod.directions: unknown '" + d + "'");
                out.push_back({id + "_" + std::to_string(k + 1) + "_" + d,
                           math::Vec3{station, 0.0, 0.0} + radial * radius, direction, thrust, behind});
            }
        }
    }
}

// [stage.rcs]: thrusters from pods (a set of directions at each azimuth of a ring) and single [[stage.rcs.thruster]]s.
RcsConfig load_rcs(const Document& doc, const toml::table& rcs, const std::vector<TankConfig>& tanks) {
    RcsConfig out;
    out.isp_s = doc.positive(rcs, "isp_s");
    out.propellant = doc.text(rcs, "propellant");
    out.minimum_on_time_s = doc.positive(rcs, "minimum_on_time_s");
    if (out.propellant != "main" && std::none_of(tanks.begin(), tanks.end(), [&](const TankConfig& k) {
            return k.id == out.propellant && k.contents == TankContents::Rcs;
        })) {
        doc.fail("stage.rcs.propellant must be \"main\" or the id of a tank with contents = \"rcs\"");
    }
    if (auto pods = rcs["pod"].as_array()) append_rcs_pods(doc, *pods, out.thrusters);
    if (auto list = rcs["thruster"].as_array()) {
        for (const auto& node : *list) {
            const auto& q = *node.as_table();
            const auto position = number_list(doc, q, "position_m");
            const auto direction = number_list(doc, q, "direction");
            if (position.size() != 3 || direction.size() != 3) {
                doc.fail("stage.rcs.thruster position_m and direction need 3 numbers");
            }
            const math::Vec3 d{direction[0], direction[1], direction[2]};
            if (!(math::norm(d) > 0.0)) doc.fail("stage.rcs.thruster direction must not be zero");
            out.thrusters.push_back({doc.text(q, "id"), {position[0], position[1], position[2]}, math::normalized(d),
                                     doc.positive(q, "thrust_n"), q["behind_nose_cone"].value_or(false)});
        }
    }
    if (out.thrusters.empty()) doc.fail("stage.rcs needs [[stage.rcs.pod]] or [[stage.rcs.thruster]] entries");
    return out;
}

PropellantConfig load_propellant(const std::filesystem::path& config_dir, const std::string& id) {
    const Document doc(config_dir / "propellants.toml");
    if (auto list = doc.root["propellant"].as_array()) {
        for (const auto& node : *list) {
            const auto& t = *node.as_table();
            if (doc.text(t, "id") == id) {
                PropellantConfig p{id, doc.number(t, "mixture_ratio"), doc.number(t, "oxidizer_density_kg_m3"),
                                   doc.number(t, "fuel_density_kg_m3")};
                // All or none of boiling point, latent heat and surface tension, positive.
                auto boiling = [&](const char* side, double& point_k, double& latent_j_kg, double& tension_n_m) {
                    const std::string point_key = std::string(side) + "_boiling_point_k";
                    const std::string latent_key = std::string(side) + "_latent_heat_j_kg";
                    const std::string tension_key = std::string(side) + "_surface_tension_n_m";
                    if (!t.contains(point_key) && !t.contains(latent_key) && !t.contains(tension_key)) return;
                    point_k = doc.positive(t, point_key);
                    latent_j_kg = doc.positive(t, latent_key);
                    tension_n_m = doc.positive(t, tension_key);
                };
                boiling("oxidizer", p.oxidizer_boiling_point_k, p.oxidizer_latent_heat_j_kg,
                        p.oxidizer_surface_tension_n_m);
                boiling("fuel", p.fuel_boiling_point_k, p.fuel_latent_heat_j_kg, p.fuel_surface_tension_n_m);
                return p;
            }
        }
    }
    doc.fail("propellant '" + id + "' not defined");
}

EngineConfig load_engine(const std::filesystem::path& config_dir, const std::string& id) {
    const Document doc(config_dir / "engines" / (id + ".toml"));
    const auto& engine = doc.table(doc.root, "engine");
    const auto& performance = doc.table(doc.root, "performance");
    const auto& throttle = doc.table(doc.root, "throttle");
    const auto& gimbal = doc.table(doc.root, "gimbal");
    const auto& mass = doc.table(doc.root, "mass");
    const auto& timing = doc.table(doc.root, "timing");

    EngineConfig e;
    e.id = id;
    e.name = doc.text(engine, "name");
    e.mixture_ratio = doc.number(engine, "mixture_ratio");
    e.reference_pressure_pa = doc.number(performance, "reference_pressure_pa");
    e.thrust_ref_n = doc.number(performance, "thrust_ref_n");
    e.isp_ref_s = doc.number(performance, "isp_ref_s");
    const double exit_diameter = doc.number(performance, "nozzle_exit_diameter_m");
    e.nozzle_exit_area_m2 = constants::pi * 0.25 * exit_diameter * exit_diameter;
    e.max_ambient_pressure_pa = doc.number(performance, "max_ambient_pressure_pa");
    e.throttle_min = doc.number(throttle, "min");
    e.throttle_max = doc.number(throttle, "max");
    e.throttle_rate_per_s = doc.number(throttle, "rate_per_s");
    e.gimbal_max_rad = doc.number(gimbal, "max_angle_deg") * constants::deg_to_rad;
    e.gimbal_rate_rad_s = doc.number(gimbal, "rate_deg_s") * constants::deg_to_rad;
    e.dry_mass_kg = doc.number(mass, "dry_kg");
    e.startup_s = doc.number(timing, "startup_s");
    e.shutdown_s = doc.number(timing, "shutdown_s");
    if (e.thrust_ref_n <= 0.0 || e.isp_ref_s <= 0.0) {
        doc.fail("thrust_ref_n and isp_ref_s must be positive");
    }
    return e;
}

// [[stage.docking_port]] entries of a stage table (also the additions to a reused stage, see load_stage_from).
void append_docking_ports(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto ports = t["docking_port"].as_array()) {
        for (const auto& node : *ports) {
            const auto& p = *node.as_table();
            DockingPortConfig port{doc.text(p, "id"), doc.text(p, "name"), vec3(doc, p, "position_m"),
                                   vec3(doc, p, "axis"), vec3(doc, p, "up")};
            if (std::abs(norm(port.axis) - 1.0) > 1e-9 || std::abs(norm(port.up) - 1.0) > 1e-9 ||
                std::abs(dot(port.axis, port.up)) > 1e-9) {
                doc.fail("stage.docking_port '" + port.id + "': axis and up must be perpendicular unit vectors");
            }
            port.behind_nose_cone = p["behind_nose_cone"].value_or(false);
            const std::string mode = p["soft_capture"].value_or(std::string("passive"));
            if (mode == "active") {
                port.soft_capture = SoftCaptureMode::Active;
            } else if (mode != "passive") {
                doc.fail("stage.docking_port '" + port.id + "': soft_capture must be \"active\" or \"passive\"");
            }
            const std::string mechanism = p["mechanism"].value_or(std::string("idss"));
            if (mechanism == "probe_drogue") {
                port.mechanism = DockingMechanism::ProbeDrogue;
            } else if (mechanism != "idss") {
                doc.fail("stage.docking_port '" + port.id + "': mechanism must be \"idss\" or \"probe_drogue\"");
            }
            if (const auto* elements = p["elements_m"].as_array()) {
                for (const auto& e : *elements) {
                    const auto* xyz = e.as_array();
                    if (!xyz || xyz->size() != 3) {
                        doc.fail("stage.docking_port '" + port.id + "': elements_m needs [x, y, z] entries");
                    }
                    const auto x = (*xyz)[0].value<double>(), y = (*xyz)[1].value<double>(),
                               z = (*xyz)[2].value<double>();
                    if (!x || !y || !z) doc.fail("stage.docking_port '" + port.id + "': elements_m must be numbers");
                    port.elements_m.push_back({*x, *y, *z});
                }
            }
            constexpr size_t min_probe_drogue_elements = 3;  // fewer do not fix the mated pose
            if (port.mechanism == DockingMechanism::ProbeDrogue && port.elements_m.size() < min_probe_drogue_elements) {
                doc.fail("stage.docking_port '" + port.id + "': probe_drogue needs at least 3 elements_m");
            }
            if (port.mechanism == DockingMechanism::Idss && !port.elements_m.empty()) {
                doc.fail("stage.docking_port '" + port.id + "': elements_m belong to a probe_drogue port");
            }
            if (std::any_of(s.docking_ports.begin(), s.docking_ports.end(),
                            [&](const DockingPortConfig& q) { return q.id == port.id; })) {
                doc.fail("stage.docking_port: docking port id '" + port.id + "' twice");
            }
            s.docking_ports.push_back(port);
        }
    }
}

// [[stage.transfer_connector]]: id, tank (a main tank of this stage), position_m. Needs the stage's tanks parsed.
void append_transfer_connectors(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto list = t["transfer_connector"].as_array()) {
        for (const auto& node : *list) {
            const auto& c = *node.as_table();
            TransferConnectorConfig connector{doc.text(c, "id"), doc.text(c, "tank"), TankContents::Oxidizer,
                                              vec3(doc, c, "position_m")};
            const auto tank = std::find_if(s.tanks.begin(), s.tanks.end(),
                                           [&](const TankConfig& k) { return k.id == connector.tank_id; });
            if (tank == s.tanks.end() || tank->role != TankRole::Main || tank->contents == TankContents::Rcs) {
                doc.fail("stage.transfer_connector '" + connector.id +
                         "': tank must be a main propellant tank of the stage");
            }
            connector.contents = tank->contents;
            connector.line_diameter_m = doc.positive(c, "line_diameter_m");
            connector.discharge_coefficient = doc.positive(c, "discharge_coefficient");
            connector.chilldown_kg = doc.number(c, "chilldown_kg");
            if (connector.chilldown_kg < 0.0) {
                doc.fail("stage.transfer_connector '" + connector.id + "': chilldown_kg must not be negative");
            }
            if (std::any_of(s.transfer_connectors.begin(), s.transfer_connectors.end(),
                            [&](const TransferConnectorConfig& q) { return q.id == connector.id; })) {
                doc.fail("stage.transfer_connector: transfer connector id '" + connector.id + "' twice");
            }
            s.transfer_connectors.push_back(connector);
        }
    }
}

// [stage.propellant_transfer]: delivery_pressure_pa, receive_pressure_pa (both positive, receive below delivery).
void read_propellant_transfer(const Document& doc, const toml::table& t, StageConfig& s) {
    if (auto block = t["propellant_transfer"].as_table()) {
        PropellantTransferConfig p{doc.positive(*block, "delivery_pressure_pa"),
                                   doc.positive(*block, "receive_pressure_pa")};
        if (p.receive_pressure_pa >= p.delivery_pressure_pa) {
            doc.fail("stage.propellant_transfer: receive_pressure_pa must be below delivery_pressure_pa");
        }
        s.propellant_transfer = p;
    }
}

StageConfig load_stage(const Document& doc, const toml::table& t) {
    StageConfig s;
    s.id = doc.text(t, "id");
    s.name = doc.text(t, "name");
    s.display_name = doc.text_or(t, "display_name");
    s.length_m = doc.number(t, "length_m");
    s.diameter_m = doc.number(t, "diameter_m");
    s.hull_top_diameter_m = t["hull_top_diameter_m"].value_or(0.0);
    s.dry_mass_kg = doc.number(t, "dry_mass_kg");
    s.dry_cg_station_m = doc.number(t, "dry_cg_station_m");
    if (const auto* offset = t["dry_cg_offset_m"].as_array()) {
        if (offset->size() != 2) doc.fail("dry_cg_offset_m must be [y, z]");
        const auto y = (*offset)[0].value<double>(), z = (*offset)[1].value<double>();
        if (!y || !z) doc.fail("dry_cg_offset_m must be numbers");
        s.dry_cg_offset_m = {0.0, *y, *z};
    }
    s.payload_mass_kg = doc.number_or(t, "payload_mass_kg", 0.0);
    s.fairing_mass_kg = doc.number_or(t, "fairing_mass_kg", 0.0);
    s.jettison_after_separation_kg = doc.number_or(t, "jettison_after_separation_kg", 0.0);
    s.radiation_pressure_coefficient = doc.number_or(t, "radiation_pressure_coefficient", 1.3);
    s.separation = doc.text_or(t, "separation");
    s.nose_cone = t["nose_cone"].value_or(false);

    if (auto tanks = t["tank"].as_array()) {
        for (const auto& node : *tanks) {
            const auto& tank = *node.as_table();
            const auto contents = doc.text(tank, "contents");
            if (contents != "oxidizer" && contents != "fuel" && contents != "rcs") {
                doc.fail("tank contents must be 'oxidizer', 'fuel' or 'rcs'");
            }
            const TankContents kind = contents == "oxidizer" ? TankContents::Oxidizer
                                      : contents == "fuel"   ? TankContents::Fuel
                                                             : TankContents::Rcs;
            s.tanks.push_back({doc.text(tank, "id"), kind, doc.number(tank, "mass_kg"), doc.number(tank, "base_station_m"),
                               kind == TankContents::Rcs ? doc.positive(tank, "density_kg_m3") : 0.0});
            s.tanks.back().capacity_kg = s.tanks.back().mass_kg;  // configured loads are full tanks
            const auto role = doc.text_or(tank, "role");
            if (!role.empty() && role != "main" && role != "header") doc.fail("tank role must be 'main' or 'header'");
            if (role == "header") {
                if (kind == TankContents::Rcs) doc.fail("a header tank holds oxidizer or fuel");
                s.tanks.back().role = TankRole::Header;
            }
            if (tank.contains("diameter_m")) s.tanks.back().diameter_m = doc.positive(tank, "diameter_m");
            if (tank.contains("propellant_management_device")) {
                s.tanks.back().propellant_management_device = doc.boolean(tank, "propellant_management_device");
            }
        }
    }
    append_transfer_connectors(doc, t, s);
    read_propellant_transfer(doc, t, s);
    double header_kg = 0.0;  // the header tanks' contents: the stage's landing propellant
    for (const auto& k : s.tanks) header_kg += k.role == TankRole::Header ? k.capacity_kg : 0.0;
    if (auto groups = t["engine_group"].as_array()) {
        for (const auto& node : *groups) {
            const auto& g = *node.as_table();
            s.engine_groups.push_back({doc.text(g, "id"), doc.text(g, "engine"),
                                       static_cast<int>(doc.number(g, "count")), doc.number(g, "ring_radius_m"),
                                       doc.boolean(g, "gimbal"), doc.boolean(g, "relight"), {}});
            s.engine_groups.back().abort_only = g["abort_only"].value_or(false);
            s.engine_groups.back().station_m = doc.number_or(g, "station_m", 0.0);
            s.engine_groups.back().cant_rad = doc.number_or(g, "cant_deg", 0.0) * constants::deg_to_rad;
            s.engine_groups.back().landing_only = g["landing_only"].value_or(false);
            if (s.engine_groups.back().cant_rad != 0.0 && s.engine_groups.back().gimbal) {
                doc.fail("engine_group: a canted group cannot gimbal");
            }
            if (const auto* angles = g["angles_deg"].as_array()) {
                for (const auto& a : *angles) {
                    const auto value = a.value<double>();
                    if (!value) doc.fail("engine_group.angles_deg must be numbers");
                    s.engine_groups.back().angles_rad.push_back(*value * constants::deg_to_rad);
                }
                if (static_cast<int>(s.engine_groups.back().angles_rad.size()) != s.engine_groups.back().count) {
                    doc.fail("engine_group.angles_deg must list one angle per engine");
                }
            }
        }
    }
    if (auto aero = t["aero"].as_table()) {
        s.aero.reference_diameter_m = doc.number_or(*aero, "reference_diameter_m", s.diameter_m);
        s.aero.ascent_drag_table = doc.text_or(*aero, "ascent_drag_table");
        s.aero.descent_drag_table = doc.text_or(*aero, "descent_drag_table");
        s.aero.crossflow_table = doc.text_or(*aero, "crossflow_table");
        s.aero.nose_length_m = doc.number_or(*aero, "nose_length_m", 0.0);
        s.aero.flap_table = doc.text_or(*aero, "flap_table");
        s.aero.entry_alpha_table = doc.text_or(*aero, "entry_alpha_table");
        s.aero.belly_azimuth_rad = doc.number_or(*aero, "belly_azimuth_deg", 270.0) * constants::deg_to_rad;
        s.aero.capsule_table = doc.text_or(*aero, "capsule_table");
        s.aero.entry_profile = doc.text_or(*aero, "entry_profile");
        if (s.aero.entry_profile.empty()) s.aero.entry_profile = "ship";
        if (s.aero.entry_profile != "ship" && s.aero.entry_profile != "capsule") {
            doc.fail("stage.aero.entry_profile must be \"ship\" or \"capsule\"");
        }
        if (s.aero.entry_profile == "capsule" && s.aero.capsule_table.empty()) {  // review d3 M3
            doc.fail("stage.aero: entry_profile = \"capsule\" needs capsule_table");
        }
        s.aero.fixed_drag_area_m2 = doc.number_or(*aero, "fixed_drag_area_m2", 0.0);
        s.aero.fixed_drag_coefficient = doc.number_or(*aero, "fixed_drag_coefficient", 0.0);
        s.aero.solar_area_m2 = doc.number_or(*aero, "solar_area_m2", 0.0);
        if ((s.aero.fixed_drag_area_m2 > 0.0) != (s.aero.fixed_drag_coefficient > 0.0)) {
            doc.fail("stage.aero: fixed_drag_area_m2 and fixed_drag_coefficient go together");
        }
    } else {
        s.aero.reference_diameter_m = s.diameter_m;
    }
    if (t.contains("inertia_kg_m2")) {
        const math::Vec3 i = vec3(doc, t, "inertia_kg_m2");
        if (!(i.x > 0.0 && i.y > 0.0 && i.z > 0.0)) doc.fail("inertia_kg_m2 must be positive");
        s.inertia_kg_m2 = i;
    }
    if (auto station = t["station"].as_table()) {
        const math::Vec3 deg = vec3(doc, *station, "body_in_lvlh_deg");  // yaw, pitch, roll [E] (torque equilibrium attitude)
        s.body_in_lvlh_rad = deg * constants::deg_to_rad;
        if (auto orbit = (*station)["reference_orbit"].as_table()) {
            s.reference_orbit = config::StationReferenceOrbitConfig{
                doc.number(*orbit, "altitude_km") * 1e3, doc.number(*orbit, "inclination_deg") * constants::deg_to_rad,
                doc.number(*orbit, "raan_deg") * constants::deg_to_rad, doc.text(*orbit, "reference_utc")};
        }
        if (auto prox = (*station)["proximity"].as_table()) {
            auto& sp = s.station_proximity;
            const auto optional_positive = [&](const char* key) -> std::optional<double> {
                if (!prox->contains(key)) return std::nullopt;
                return doc.positive(*prox, key);
            };
            sp.keep_out_radius_m = optional_positive("keep_out_radius_m");
            sp.waypoint0_below_m = optional_positive("waypoint0_below_m");
            sp.waypoint1_range_m = optional_positive("waypoint1_range_m");
            sp.waypoint2_range_m = optional_positive("waypoint2_range_m");
            sp.flyaround_radius_m = optional_positive("flyaround_radius_m");
            if (prox->contains("approach_ellipsoid_m")) {
                const math::Vec3 e = vec3(doc, *prox, "approach_ellipsoid_m");
                if (!(e.x > 0.0 && e.y > 0.0 && e.z > 0.0)) {
                    doc.fail("stage.station.proximity.approach_ellipsoid_m must be positive");
                }
                sp.approach_ellipsoid_m = e;
            }
            if (sp.waypoint1_range_m && sp.waypoint2_range_m && !(*sp.waypoint2_range_m < *sp.waypoint1_range_m)) {
                doc.fail("stage.station.proximity: waypoint2_range_m must be below waypoint1_range_m");
            }
            if (sp.keep_out_radius_m && sp.waypoint1_range_m && !(*sp.keep_out_radius_m < *sp.waypoint1_range_m)) {
                doc.fail("stage.station.proximity: Waypoint 1 must lie outside the keep-out sphere");
            }
        }
    }
    append_docking_ports(doc, t, s);
    if (auto prims = t["collision"].as_array()) {
        for (const auto& node : *prims) {
            const auto& p = *node.as_table();
            CollisionPrimitiveConfig c;
            c.id = doc.text(p, "id");
            c.kind = doc.text(p, "kind");
            if (c.kind == "capsule") {
                c.a_m = vec3(doc, p, "a_m");
                c.b_m = vec3(doc, p, "b_m");
                c.radius_m = doc.positive(p, "radius_m");
            } else if (c.kind == "box") {
                c.a_m = vec3(doc, p, "centre_m");
                c.half_extents_m = vec3(doc, p, "half_extents_m");
            } else {
                doc.fail("stage.collision '" + c.id + "': kind must be \"capsule\" or \"box\"");
            }
            s.collision.push_back(c);
        }
    }
    if (auto fins = t["grid_fins"].as_table()) {
        const auto* azimuths = (*fins)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->empty()) {
            doc.fail("grid_fins.azimuths_deg must list one azimuth per fin");
        }
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("grid_fins.azimuths_deg must be numbers");
            s.grid_fins.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.grid_fins.area_m2 = doc.number(*fins, "area_m2");
        s.grid_fins.station_m = doc.number(*fins, "station_m");
        s.grid_fins.radius_m = doc.number(*fins, "radius_m");
        s.grid_fins.max_deflection_rad = doc.number(*fins, "max_deflection_deg") * constants::deg_to_rad;
        s.grid_fins.rate_rad_s = doc.number(*fins, "rate_deg_s") * constants::deg_to_rad;
        s.grid_fins.table = doc.text(*fins, "table");
    }
    if (auto flaps = t["flap"].as_array()) {
        for (auto& node : *flaps) {
            const auto* ft = node.as_table();
            if (!ft) doc.fail("stage.flap entries must be tables");
            config::FlapConfig f;
            f.id = doc.text(*ft, "id");
            f.hinge_azimuth_rad = doc.number(*ft, "hinge_azimuth_deg") * constants::deg_to_rad;
            f.station_fwd_m = doc.number(*ft, "station_fwd_m");
            f.station_aft_m = doc.number(*ft, "station_aft_m");
            f.span_m = doc.number(*ft, "span_m");
            f.area_m2 = doc.number(*ft, "area_m2");
            f.max_deflection_rad = doc.number(*ft, "max_deflection_deg") * constants::deg_to_rad;
            f.rate_rad_s = doc.number(*ft, "rate_deg_s") * constants::deg_to_rad;
            f.initial_deflection_rad = doc.number_or(*ft, "initial_deflection_deg", 0.0) * constants::deg_to_rad;
            s.flaps.push_back(f);
        }
        if (!s.flaps.empty() && s.aero.flap_table.empty()) doc.fail("stage.flap needs [stage.aero] flap_table");
    }
    if (auto shield = t["heat_shield"].as_table()) {
        s.heat_shield.nose_radius_m = doc.number(*shield, "nose_radius_m");
        s.heat_shield.max_heat_rate_w_m2 = doc.number(*shield, "max_heat_rate_w_m2");
        s.heat_shield.max_heat_load_j_m2 = doc.number(*shield, "max_heat_load_j_m2");
        s.heat_shield.max_load_g = doc.number_or(*shield, "max_load_g", 0.0);
        s.heat_shield.crew_load_limit_g = doc.number_or(*shield, "crew_load_limit_g", 0.0);
    }
    if (auto thermal = t["thermal"].as_table()) {
        auto surface = [&](const toml::table& zone, const std::string& where) {
            const ThermalSurfaceConfig c{doc.number(zone, "absorptivity"), doc.number(zone, "emissivity"),
                                         doc.positive(zone, "wall_resistance_m2k_w")};
            if (!(c.absorptivity > 0.0 && c.absorptivity <= 1.0) || !(c.emissivity > 0.0 && c.emissivity <= 1.0)) {
                doc.fail(where + ": absorptivity and emissivity must lie in (0, 1]");
            }
            return c;
        };
        ThermalConfig c;
        c.hull = surface(*thermal, "stage.thermal");
        c.ullage_resistance_m2k_w = doc.positive(*thermal, "ullage_resistance_m2k_w");
        if (auto shield = (*thermal)["shield"].as_table()) {
            c.shield = surface(*shield, "stage.thermal.shield");
            c.shield_half_width_rad = doc.positive(*shield, "half_width_deg") * constants::deg_to_rad;
            if (c.shield_half_width_rad > constants::pi) doc.fail("stage.thermal.shield: half_width_deg <= 180");
        }
        s.thermal = c;
    }
    if (auto chutes = t["parachute"].as_array()) {
        for (auto& node : *chutes) {
            const auto* pt = node.as_table();
            if (!pt) doc.fail("stage.parachute entries must be tables");
            config::ParachuteConfig p;
            p.id = doc.text(*pt, "id");
            p.role = doc.text(*pt, "role");
            if (p.role != "drogue" && p.role != "main") doc.fail("stage.parachute.role must be \"drogue\" or \"main\"");
            p.count = static_cast<int>(doc.positive(*pt, "count"));
            p.drag_area_m2 = doc.positive(*pt, "drag_area_m2");
            p.attach_station_m = doc.number(*pt, "attach_station_m");
            p.fill_time_s = doc.positive(*pt, "fill_time_s");
            p.max_load_n = doc.positive(*pt, "max_load_n");
            p.deploy_height_m = doc.positive(*pt, "deploy_height_m");
            p.max_deploy_dynamic_pressure_pa = doc.number_or(*pt, "max_deploy_dynamic_pressure_pa", 0.0);
            if (const auto* reefing = (*pt)["reefing"].as_array()) {
                double previous = 0.0;
                for (const auto& r : *reefing) {
                    const auto* pair = r.as_array();
                    if (!pair || pair->size() != 2) doc.fail("stage.parachute.reefing entries are [area_fraction, hold_s]");
                    const auto fraction = (*pair)[0].value<double>();
                    const auto hold = (*pair)[1].value<double>();
                    if (!fraction || !hold || *fraction <= previous || *fraction >= 1.0 || *hold < 0.0) {
                        doc.fail("stage.parachute.reefing: area fractions must ascend in (0, 1), holds >= 0");
                    }
                    p.reefing.push_back({*fraction, *hold});
                    previous = *fraction;
                }
            }
            s.parachutes.push_back(p);
        }
        int drogues = 0, mains = 0;
        for (const auto& p : s.parachutes) (p.role == "drogue" ? drogues : mains)++;
        if (drogues > 1 || mains > 1) doc.fail("stage.parachute: at most one drogue set and one main set");
        for (const auto& d : s.parachutes) {
            for (const auto& m : s.parachutes) {
                if (d.role == "drogue" && m.role == "main" && !(m.deploy_height_m < d.deploy_height_m)) {
                    doc.fail("stage.parachute: the mains must deploy below the drogues");
                }
            }
        }
    }
    if (auto pins = t["catch_pins"].as_table()) {
        const auto* azimuths = (*pins)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->empty()) {
            doc.fail("catch_pins.azimuths_deg must list one azimuth per pin");
        }
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("catch_pins.azimuths_deg must be numbers");
            s.catch_pins.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.catch_pins.station_m = doc.number(*pins, "station_m");
        s.catch_pins.protrusion_m = doc.positive(*pins, "protrusion_m");
        s.catch_pins.max_load_n = doc.positive(*pins, "max_load_n");
    }
    if (auto legs = t["landing_legs"].as_table()) {
        const auto* azimuths = (*legs)["azimuths_deg"].as_array();
        if (!azimuths || azimuths->size() < 3) doc.fail("landing_legs.azimuths_deg must list at least three legs");
        for (const auto& a : *azimuths) {
            const auto value = a.value<double>();
            if (!value) doc.fail("landing_legs.azimuths_deg must be numbers");
            s.landing_legs.azimuths_rad.push_back(*value * constants::deg_to_rad);
        }
        s.landing_legs.footprint_radius_m = doc.positive(*legs, "footprint_radius_m");
        s.landing_legs.foot_station_m = doc.number(*legs, "foot_station_m");
        s.landing_legs.fixed = (*legs)["fixed"].value_or(false);
        s.landing_legs.deploy_time_s = s.landing_legs.fixed ? doc.number_or(*legs, "deploy_time_s", 0.0)
                                                            : doc.positive(*legs, "deploy_time_s");
        s.landing_legs.stroke_m = doc.positive(*legs, "stroke_m");
        s.landing_legs.crush_force_n = doc.positive(*legs, "crush_force_n");
        s.landing_legs.stiffness_n_m = doc.positive(*legs, "stiffness_n_m");
        s.landing_legs.damping_n_s_m = doc.positive(*legs, "damping_n_s_m");
        s.landing_legs.friction = doc.positive(*legs, "friction");
    }
    if (auto rcs = t["rcs"].as_table()) s.rcs = load_rcs(doc, *rcs, s.tanks);
    for (const auto& thruster : s.rcs.thrusters) {
        if (thruster.behind_nose_cone && !s.nose_cone) doc.fail("stage.rcs: behind_nose_cone needs nose_cone = true on the stage");
    }
    if (auto recovery = t["recovery"].as_table()) {
        s.reserve_propellant_kg = doc.number_or(*recovery, "reserve_propellant_kg", 0.0);
        s.landing_propellant_kg = doc.number_or(*recovery, "landing_propellant_kg", 0.0);
        if (header_kg > 0.0 && recovery->contains("landing_propellant_kg") &&
            std::abs(s.landing_propellant_kg - header_kg) > landing_propellant_tolerance_kg) {
            doc.fail("stage.recovery.landing_propellant_kg must equal the header tanks' contents (or be left out)");
        }
        if (header_kg > 0.0) s.landing_propellant_kg = header_kg;  // the headers hold the landing propellant
        s.recovery_method = doc.text_or(*recovery, "method");
        if (s.recovery_method == "splashdown") {
            s.max_splashdown_speed_mps = doc.positive(*recovery, "max_splashdown_speed_mps");
        }
        auto group_list = [&](const toml::array& list, const std::string& key) {
            std::vector<std::string> ids;
            for (const auto& node : list) {
                const auto id = node.value<std::string>().value_or("");
                const bool known = std::any_of(s.engine_groups.begin(), s.engine_groups.end(),
                                               [&](const auto& g) { return g.id == id; });
                if (!known) doc.fail(key + " group '" + id + "' is not an engine group of stage '" + s.id + "'");
                ids.push_back(id);
            }
            return ids;
        };
        if (const auto* groups = (*recovery)["deorbit_engine_groups"].as_array()) {
            s.deorbit_engine_groups = group_list(*groups, "deorbit engine");
        }
        // Return profile: `base` with the keys of `table` that are set.
        auto read_profile = [&](const toml::table& table, config::ReturnProfile p) {
            p.boostback = table["boostback"].value_or(p.boostback);
            if (const auto* groups = table["boostback_engine_groups"].as_array()) {
                p.boostback_engine_groups = group_list(*groups, "boostback engine");
            }
            if (const auto* groups = table["entry_engine_groups"].as_array()) {
                p.entry_engine_groups = group_list(*groups, "entry engine");
            }
            if (const auto* phases = table["landing_engine_phases"].as_array()) {
                p.landing_engine_phases.clear();
                for (const auto& node : *phases) {
                    const auto* list = node.as_array();
                    if (!list) doc.fail("landing_engine_phases must be a list of engine group lists");
                    p.landing_engine_phases.push_back(group_list(*list, "landing engine"));
                }
                if (p.landing_engine_phases.size() != 3) doc.fail("landing_engine_phases must list 3 engine sets");
            }
            if (const auto* throttles = table["landing_phase_throttles"].as_array()) {
                p.landing_phase_throttles.clear();
                for (const auto& node : *throttles) {
                    const auto value = node.value<double>();
                    if (!value || *value <= 0.0 || *value > 1.0) doc.fail("landing_phase_throttles must be in (0, 1]");
                    p.landing_phase_throttles.push_back(*value);
                }
                if (p.landing_phase_throttles.size() != 3) doc.fail("landing_phase_throttles must list 3 throttles");
            }
            if (const auto* tilts = table["landing_phase_max_tilt_deg"].as_array()) {
                p.landing_phase_max_tilt_rad.clear();
                for (const auto& node : *tilts) {
                    const auto value = node.value<double>();
                    if (!value || *value <= 0.0 || *value >= 90.0) doc.fail("landing_phase_max_tilt_deg must be in (0, 90)");
                    p.landing_phase_max_tilt_rad.push_back(*value * constants::deg_to_rad);
                }
                if (p.landing_phase_max_tilt_rad.size() != 3) doc.fail("landing_phase_max_tilt_deg must list 3 limits");
            }
            if (const auto* windows = table["landing_switch_sink_mps"].as_array()) {
                p.landing_switch_sink_mps.clear();
                for (const auto& node : *windows) {
                    const auto* pair = node.as_array();
                    const auto low = pair && pair->size() == 2 ? (*pair)[0].value<double>() : std::nullopt;
                    const auto high = pair && pair->size() == 2 ? (*pair)[1].value<double>() : std::nullopt;
                    if (!low || !high || *low < 0.0 || *high < *low) {
                        doc.fail("landing_switch_sink_mps entries must be [min, max] sink rates");
                    }
                    p.landing_switch_sink_mps.emplace_back(*low, *high);
                }
                if (p.landing_switch_sink_mps.size() != 2) doc.fail("landing_switch_sink_mps must list 2 switch windows");
            }
            p.return_flight_time_s = doc.number_or(table, "return_flight_time_s", p.return_flight_time_s);
            p.boostback_loss_fraction = doc.number_or(table, "boostback_loss_fraction", p.boostback_loss_fraction);
            p.rcs_propellant_kg = doc.number_or(table, "rcs_propellant_kg", p.rcs_propellant_kg);
            p.entry_burn_altitude_m = doc.number_or(table, "entry_burn_altitude_m", p.entry_burn_altitude_m);
            p.entry_burn_delta_v_mps = doc.number_or(table, "entry_burn_delta_v_mps", p.entry_burn_delta_v_mps);
            p.entry_burn_throttle = doc.number_or(table, "entry_burn_throttle", p.entry_burn_throttle);
            if (p.entry_burn_altitude_m > 0.0 && p.entry_engine_groups.empty()) {
                doc.fail("an entry burn (entry_burn_altitude_m) needs entry_engine_groups");
            }
            return p;
        };
        s.return_profile = read_profile(*recovery, {});
        if (const auto* drone_ship = (*recovery)["drone_ship"].as_table()) {
            s.drone_ship_return = read_profile(*drone_ship, s.return_profile);
        }
    }
    return s;
}

// [[stage]] from = "<vehicle>:<stage>": that vehicle's stage exactly as configured there (refuelling r1: the tanker
// and the depot's booster are Block 3's). Only docking ports, transfer connectors and RCS pods (r3) may be added; any
// other key would silently diverge from the source and is an error.
StageConfig load_stage_from(const std::filesystem::path& config_dir, const Document& doc, const toml::table& t,
                            const std::string& propellant_id) {
    constexpr int max_from_depth = 4;  // a longer chain is a cycle
    thread_local int depth = 0;
    if (depth >= max_from_depth) doc.fail("stage.from: chain too long (a cycle?)");
    struct DepthGuard {
        DepthGuard() { ++depth; }
        ~DepthGuard() { --depth; }
    } guard;

    const std::string from = doc.text(t, "from");
    const auto colon = from.find(':');
    if (colon == std::string::npos || colon == 0 || colon + 1 == from.size()) {
        doc.fail("stage.from must be \"<vehicle>:<stage>\", got \"" + from + "\"");
    }
    for (const auto& [key, value] : t) {
        if (key != "from" && key != "docking_port" && key != "transfer_connector" && key != "rcs_pod" &&
            key != "propellant_transfer" && key != "dry_mass_kg" && key != "display_name") {
            doc.fail("stage.from = \"" + from + "\": '" + std::string(key.str()) + "' cannot be set on a reused stage");
        }
    }
    const auto source = load_vehicle(config_dir, from.substr(0, colon));
    if (source.propellant.id != propellant_id) {
        doc.fail("stage.from = \"" + from + "\": propellant '" + source.propellant.id + "' differs from '" +
                 propellant_id + "'");
    }
    const std::string stage_id = from.substr(colon + 1);
    const auto it = std::find_if(source.stages.begin(), source.stages.end(),
                                 [&](const StageConfig& s) { return s.id == stage_id; });
    if (it == source.stages.end()) doc.fail("stage.from = \"" + from + "\": no such stage");
    StageConfig s = *it;
    // a variant's own dry mass (user 2026-10-20, refuelling r5: the tanker without cargo hardware); CG as the source
    if (t.contains("dry_mass_kg")) s.dry_mass_kg = doc.positive(t, "dry_mass_kg");
    s.display_name = doc.text_or(t, "display_name", s.display_name);  // a variant's own name (the tanker's ship)
    append_docking_ports(doc, t, s);
    append_transfer_connectors(doc, t, s);
    read_propellant_transfer(doc, t, s);
    if (auto pods = t["rcs_pod"].as_array()) {
        // refuelling r3: docking thrusters added to a reused ship (the tanker's -x pair)
        if (s.rcs.thrusters.empty()) doc.fail("stage.from = \"" + from + "\": rcs_pod needs [stage.rcs] on the source stage");
        append_rcs_pods(doc, *pods, s.rcs.thrusters);
    }
    return s;
}

}  // namespace

const EngineConfig& VehicleConfig::engine(const std::string& engine_id) const {
    const auto it = engines.find(engine_id);
    if (it == engines.end()) {
        throw ConfigError("engine '" + engine_id + "' not loaded for vehicle " + id);
    }
    return it->second;
}

VehicleConfig load_vehicle(const std::filesystem::path& config_dir, std::string_view vehicle_id) {
    const Document doc(config_dir / "vehicles" / (std::string(vehicle_id) + ".toml"));
    const auto& vehicle = doc.table(doc.root, "vehicle");

    VehicleConfig v;
    v.id = doc.text(vehicle, "id");
    v.name = doc.text(vehicle, "name");
    v.display_name = doc.text_or(vehicle, "display_name", v.name);
    v.propellant = load_propellant(config_dir, doc.text(vehicle, "propellant"));

    std::map<std::string, StageConfig> stages_by_id;
    if (auto stages = doc.root["stage"].as_array()) {
        for (const auto& node : *stages) {
            const auto& table = *node.as_table();
            auto stage = table.contains("from") ? load_stage_from(config_dir, doc, table, v.propellant.id)
                                                : load_stage(doc, table);
            const std::string id = stage.id;
            if (!stages_by_id.emplace(id, std::move(stage)).second) {
                doc.fail("stage '" + id + "' defined twice");
            }
        }
    }
    const auto* order = vehicle["stages_bottom_to_top"].as_array();
    if (!order) {
        doc.fail("missing stages_bottom_to_top");
    }
    for (const auto& node : *order) {
        const auto id = node.value<std::string>().value_or("");
        const auto it = stages_by_id.find(id);
        if (it == stages_by_id.end()) {
            doc.fail("stage '" + id + "' listed in stages_bottom_to_top is not defined");
        }
        v.stages.push_back(it->second);
    }
    v.named_stage_count = v.stages.size();
    // f4 final review: port and connector lookups take the first match across the vehicle - an id on two stages would
    // shadow the upper one
    std::set<std::string> port_ids, connector_ids;
    for (const auto& stage : v.stages) {
        for (const auto& p : stage.docking_ports) {
            if (!port_ids.insert(p.id).second) doc.fail("docking port id '" + p.id + "' on two stages");
        }
        for (const auto& c : stage.transfer_connectors) {
            if (!connector_ids.insert(c.id).second) doc.fail("transfer connector id '" + c.id + "' on two stages");
        }
    }
    for (const auto& stage : v.stages) {
        for (const auto& group : stage.engine_groups) {
            if (!v.engines.contains(group.engine_id)) {
                v.engines[group.engine_id] = load_engine(config_dir, group.engine_id);
            }
        }
    }
    return v;
}

VehicleConfig top_stage_config(VehicleConfig vehicle) {
    if (vehicle.stages.size() > 1) vehicle.stages.erase(vehicle.stages.begin(), vehicle.stages.end() - 1);
    return vehicle;
}

VehicleConfig stack_payload(VehicleConfig launcher, const VehicleConfig& payload) {
    for (const auto& p : payload.stages) {
        for (const auto& l : launcher.stages) {
            if (l.id == p.id) throw ConfigError("stack_payload: stage id '" + p.id + "' occurs in both vehicles");
        }
    }
    launcher.stages.back().payload_mass_kg = 0.0;
    for (auto stage : payload.stages) {
        stage.payload = true;
        launcher.stages.push_back(std::move(stage));
    }
    for (const auto& [id, engine] : payload.engines) launcher.engines.emplace(id, engine);
    launcher.named_stage_count = launcher.stages.size();  // the launcher's display name names the whole stack
    launcher.id += "+" + payload.id;
    launcher.name += " / " + payload.name;
    return launcher;
}

std::vector<double> CsvTable::column(std::string_view name) const {
    for (size_t c = 0; c < columns.size(); ++c) {
        if (columns[c] == name) {
            std::vector<double> values;
            values.reserve(rows.size());
            for (const auto& row : rows) values.push_back(row.at(c));
            return values;
        }
    }
    throw ConfigError("CSV column '" + std::string(name) + "' not found");
}

CsvTable load_csv_table(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw ConfigError("CSV table not found: " + path.string());
    }
    CsvTable table;
    std::string line;
    while (std::getline(file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::vector<std::string> cells;
        std::stringstream stream(line);
        std::string cell;
        while (std::getline(stream, cell, ',')) cells.push_back(cell);
        if (table.columns.empty()) {
            table.columns = std::move(cells);
            continue;
        }
        if (cells.size() != table.columns.size()) {
            throw ConfigError(path.string() + ": row with " + std::to_string(cells.size()) + " cells, expected " +
                              std::to_string(table.columns.size()));
        }
        std::vector<double> row;
        for (const auto& c : cells) row.push_back(std::stod(c));
        table.rows.push_back(std::move(row));
    }
    return table;
}

}  // namespace starsim::config
