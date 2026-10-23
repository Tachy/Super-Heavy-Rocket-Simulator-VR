#include "starsim/gnc/descent_request_io.hpp"

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace starsim::gnc {

namespace {

// Writes "name v1 v2 ..." lines.
struct Writer {
    std::ostringstream out;
    void line(const std::string& name, const std::vector<double>& values) {
        out << name;
        char buffer[32];
        for (const double v : values) {
            std::snprintf(buffer, sizeof buffer, " %.17g", v);
            out << buffer;
        }
        out << '\n';
    }
    void operator()(const std::string& name, double& v) { line(name, {v}); }
    void operator()(const std::string& name, int& v) { line(name, {static_cast<double>(v)}); }
    void operator()(const std::string& name, bool& v) { line(name, {v ? 1.0 : 0.0}); }
    void operator()(const std::string& name, std::vector<double>& v) { line(name, v); }
};

// Assigns the values of the lines it has; fields without a line keep their value.
struct Reader {
    std::map<std::string, std::vector<double>> values;
    const std::vector<double>* find(const std::string& name) const {
        const auto it = values.find(name);
        return it == values.end() ? nullptr : &it->second;
    }
    void operator()(const std::string& name, double& v) const {
        if (const auto* x = find(name); x && !x->empty()) v = x->front();
    }
    void operator()(const std::string& name, int& v) const {
        if (const auto* x = find(name); x && !x->empty()) v = static_cast<int>(x->front());
    }
    void operator()(const std::string& name, bool& v) const {
        if (const auto* x = find(name); x && !x->empty()) v = x->front() != 0.0;
    }
    void operator()(const std::string& name, std::vector<double>& v) const {
        if (const auto* x = find(name)) v = *x;
    }
};

template <class F>
void visit(F& f, const std::string& name, math::Vec3& v) {
    f(name + ".x", v.x);
    f(name + ".y", v.y);
    f(name + ".z", v.z);
}

template <class F, size_t N>
void visit(F& f, const std::string& name, std::array<double, N>& a) {
    for (size_t i = 0; i < N; ++i) f(name + "." + std::to_string(i), a[i]);
}

template <class F>
void visit(F& f, const std::string& name, DescentState& x) {
    for (size_t i = 0; i < DescentState::size(); ++i) f(name + "." + std::to_string(i), x[i]);
}

template <class F>
void visit_model(F& f, DescentModel& m) {
    for (size_t p = 0; p < m.engine_sets.size(); ++p) {
        auto& s = m.engine_sets[p];
        const std::string n = "model.set" + std::to_string(p);
        f(n + ".thrust_max_n", s.thrust_max_n);
        f(n + ".thrust_min_n", s.thrust_min_n);
        f(n + ".mass_flow_per_thrust_s_m", s.mass_flow_per_thrust_s_m);
        f(n + ".gimbal_max_rad", s.gimbal_max_rad);
        f(n + ".pressure_thrust_loss_n", s.pressure_thrust_loss_n);
        f(n + ".throttle_rate_per_s", s.throttle_rate_per_s);
    }
    f("model.gimbal_lever_m", m.gimbal_lever_m);
    visit(f, "model.inertia_kg_m2", m.inertia_kg_m2);
    f("model.reference_area_m2", m.reference_area_m2);
    f("model.cd_by_mach", m.cd_by_mach);
    f("model.mach_step", m.mach_step);
    // Hull normal force: a request dumped before it existed has none of these lines and replays without it.
    f("model.crossflow_cd_by_mach", m.crossflow_cd_by_mach);
    f("model.planform_area_m2", m.planform_area_m2);
    f("model.crossflow_station_above_cg_m", m.crossflow_station_above_cg_m);
    f("model.potential_term", m.potential_term);
    f("model.potential_station_above_cg_m", m.potential_station_above_cg_m);
    f("model.crossflow_enabled", m.crossflow_enabled);
    f("model.normal_force_scale", m.normal_force_scale);
    f("model.density_kg_m3", m.density_kg_m3);
    f("model.speed_of_sound_mps", m.speed_of_sound_mps);
    f("model.height_step_m", m.height_step_m);
    visit(f, "model.gravity_mps2", m.gravity_mps2);
    visit(f, "model.earth_rate_radps", m.earth_rate_radps);
    visit(f, "model.earth_center_m", m.earth_center_m);
}

template <class F>
void visit_target(F& f, DescentTarget& t) {
    int goal = static_cast<int>(t.goal);
    f("target.goal", goal);
    t.goal = static_cast<DescentGoal>(goal);
    visit(f, "target.position_m", t.position_m);
    visit(f, "target.velocity_mps", t.velocity_mps);
    f("target.corridor_top_m", t.corridor_top_m);
    f("target.corridor_radius_m", t.corridor_radius_m);
    f("target.final_set_radius_m", t.final_set_radius_m);
    f("target.tower_keep_out_radius_m", t.tower_keep_out_radius_m);
    f("target.tower_top_cg_height_m", t.tower_top_cg_height_m);
    f("target.ground_cg_height_m", t.ground_cg_height_m);
    visit(f, "target.sea_target_m", t.sea_target_m);
    f("target.sea_min_distance_m", t.sea_min_distance_m);
    visit(f, "target.keep_out_center_m", t.keep_out_center_m);
    f("target.keep_out_radius_m", t.keep_out_radius_m);
    f("target.mount_keep_out_radius_m", t.mount_keep_out_radius_m);
    f("target.mount_top_cg_height_m", t.mount_top_cg_height_m);
}

template <class F>
void visit_settings(F& f, ScvxSettings& s) {
    f("settings.nodes_per_phase", s.nodes_per_phase);
    f("settings.max_iterations", s.max_iterations);
    f("settings.fresh_max_iterations", s.fresh_max_iterations);
    f("settings.substeps", s.substeps);
    f("settings.step_tolerance", s.step_tolerance);
    f("settings.virtual_control_tolerance", s.virtual_control_tolerance);
    f("settings.final_mass_tolerance_kg", s.final_mass_tolerance_kg);
    f("settings.defect_tolerance", s.defect_tolerance);
    f("settings.flyable_defect_tolerance", s.flyable_defect_tolerance);
    f("settings.trust_region_retries", s.trust_region_retries);
    f("settings.usable_primal_residual", s.usable_primal_residual);
    f("settings.trust_region_growth", s.trust_region_growth);
    f("settings.stall_iterations", s.stall_iterations);
    f("settings.fresh_stall_iterations", s.fresh_stall_iterations);
    f("settings.stall_ratio", s.stall_ratio);
    f("settings.stall_virtual_control", s.stall_virtual_control);
    visit(f, "settings.kinematic_guess_s", s.kinematic_guess_s);
    auto& l = s.limits;
    visit(f, "limits.tilt_max_rad", l.tilt_max_rad);
    f("limits.terminal_tilt_rad", l.terminal_tilt_rad);
    f("limits.tilt_taper_nodes", l.tilt_taper_nodes);
    f("limits.corridor_tilt_rad", l.corridor_tilt_rad);
    f("limits.terminal_thrust_margin", l.terminal_thrust_margin);
    for (size_t i = 0; i < l.switches.size(); ++i) {
        auto& sw = l.switches[i];
        const std::string n = "limits.switch" + std::to_string(i);
        f(n + ".max_tilt_rad", sw.max_tilt_rad);
        f(n + ".max_rate_radps", sw.max_rate_radps);
        f(n + ".min_sink_mps", sw.min_sink_mps);
        f(n + ".max_sink_mps", sw.max_sink_mps);
        f(n + ".min_cg_height_m", sw.min_cg_height_m);
    }
    f("limits.final_min_thrust_fraction", l.final_min_thrust_fraction);
    f("limits.final_ease_nodes", l.final_ease_nodes);
    f("limits.throttle_margin", l.throttle_margin);
    f("limits.gimbal_fraction", l.gimbal_fraction);
    f("limits.final_gimbal_fraction", l.final_gimbal_fraction);
    visit(f, "limits.rate_max_radps", l.rate_max_radps);
    f("limits.glide_slope_rad", l.glide_slope_rad);
    f("limits.approach_time_constant_s", l.approach_time_constant_s);
    f("limits.min_mass_kg", l.min_mass_kg);
    f("limits.soft_reserve_kg", l.soft_reserve_kg);
    visit(f, "limits.min_phase_s", l.min_phase_s);
    f("limits.max_phase_s", l.max_phase_s);
    f("limits.phase0_initial_spool", l.phase0_initial_spool);
    f("limits.phase0_startup_s", l.phase0_startup_s);
    f("limits.phase0_min_s", l.phase0_min_s);
    f("limits.initial_thrust_n", l.initial_thrust_n);
    f("limits.initial_thrust_window_s", l.initial_thrust_window_s);
    auto& w = s.weights;
    f("weights.fuel", w.fuel);
    f("weights.virtual_control", w.virtual_control);
    f("weights.trust_region", w.trust_region);
    f("weights.sea_distance", w.sea_distance);
    f("weights.final_lateral", w.final_lateral);
    f("weights.attitude_violation", w.attitude_violation);
    f("weights.corridor_violation", w.corridor_violation);
    f("weights.final_lateral_time_s", w.final_lateral_time_s);
    f("weights.reserve_use", w.reserve_use);
}

template <class F>
void visit_plan(F& f, DescentPlan& p) {
    f("warm.nodes_per_phase", p.nodes_per_phase);
    f("warm.start_phase", p.start_phase);
    int goal = static_cast<int>(p.goal);
    f("warm.goal", goal);
    p.goal = static_cast<DescentGoal>(goal);
    f("warm.converged", p.converged);
    visit(f, "warm.duration_s", p.reference.duration_s);
    std::vector<double> x, u;
    for (const auto& s : p.reference.x) x.insert(x.end(), s.v.begin(), s.v.end());
    for (const auto& t : p.reference.u) u.insert(u.end(), {t.x, t.y, t.z});
    f("warm.x", x);
    f("warm.u", u);
    p.reference.x.assign(x.size() / DescentState::size(), DescentState{});
    for (size_t k = 0; k < p.reference.x.size(); ++k) {
        for (size_t i = 0; i < DescentState::size(); ++i) p.reference.x[k][i] = x[k * DescentState::size() + i];
    }
    p.reference.u.assign(u.size() / 3, math::Vec3{});
    for (size_t k = 0; k < p.reference.u.size(); ++k) p.reference.u[k] = {u[3 * k], u[3 * k + 1], u[3 * k + 2]};
}

template <class F>
void visit_request(F& f, PlanRequest& r) {
    visit_model(f, r.model);
    visit(f, "initial", r.initial);
    f("start_phase", r.start_phase);
    visit_target(f, r.target);
    visit_settings(f, r.settings);
    f("warm_start_elapsed_s", r.warm_start_elapsed_s);
    f("start_offset_s", r.start_offset_s);
}

}  // namespace

void write_plan_request(const std::string& path, const PlanRequest& request) {
    PlanRequest r = request;  // the visitors take references
    Writer w;
    visit_request(w, r);
    bool warm = r.warm_start != nullptr;
    w("has_warm_start", warm);
    if (warm) {
        DescentPlan plan = *r.warm_start;
        visit_plan(w, plan);
    }
    std::ofstream file(path);
    if (!file) throw std::runtime_error("write_plan_request: cannot open " + path);
    file << w.out.str();
}

PlanRequest read_plan_request(const std::string& path) {
    std::ifstream file(path);
    if (!file) throw std::runtime_error("read_plan_request: cannot open " + path);
    Reader reader;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream in(line);
        std::string name;
        if (!(in >> name)) continue;
        std::vector<double> values;
        double v = 0.0;
        while (in >> v) values.push_back(v);
        reader.values[name] = std::move(values);
    }
    PlanRequest r;
    visit_request(reader, r);
    bool warm = false;
    reader("has_warm_start", warm);
    if (warm) {
        DescentPlan plan;
        visit_plan(reader, plan);
        r.warm_start = std::make_shared<const DescentPlan>(std::move(plan));
    }
    return r;
}

std::string dump_plan_request_if_enabled(const PlanRequest& request) {
    const char* directory = std::getenv("STARSIM_DUMP_SCVX");
    if (!directory || !*directory) return {};
    static std::atomic<int> counter{0};
    const std::string path = std::string(directory) + "/scvx_request_" + std::to_string(counter++) + ".txt";
    write_plan_request(path, request);
    return path;
}

}  // namespace starsim::gnc
