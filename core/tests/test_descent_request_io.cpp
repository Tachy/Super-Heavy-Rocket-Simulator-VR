#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "starsim/gnc/descent_discretization.hpp"
#include "starsim/gnc/descent_request_io.hpp"
#include "starsim/gnc/scvx_landing.hpp"
#include "starsim/math/constants.hpp"

using namespace starsim;
using namespace starsim::gnc;
using math::Vec3;

namespace {

constexpr double deg = constants::deg_to_rad;

std::string temp_file(const char* name) {
    return (std::filesystem::temp_directory_path() / (std::string("starsim_") + name + ".txt")).string();
}

PlanRequest sample_request() {
    PlanRequest r;
    for (int p = 0; p < descent_phases; ++p) {
        auto& s = r.model.engine_sets[static_cast<size_t>(p)];
        s.thrust_max_n = 1.0e6 + p;
        s.thrust_min_n = 4.0e5 + p;
        s.mass_flow_per_thrust_s_m = 3.3e-4;
        s.gimbal_max_rad = 5.0 * deg;
        s.pressure_thrust_loss_n = 1.0e4 / 3.0;  // a value with all 17 digits
        s.throttle_rate_per_s = 0.5;
    }
    r.model.gimbal_lever_m = 11.0;
    r.model.inertia_kg_m2 = {1.0e5, 6.0e6, 6.0e6};
    r.model.cd_by_mach = {0.6, 0.7, 0.9};
    r.model.density_kg_m3 = {1.2, 1.1};
    r.model.speed_of_sound_mps = {340.0, 339.0};
    r.model.gravity_mps2 = {-9.79, 0.0, 0.0};
    r.initial = make_state(30000.0, {700.0, -120.0, -140.0}, {-100.0, 15.0, 18.0}, math::Quat{}, {0.0, 0.01, -0.02});
    r.start_phase = 2;
    r.target.goal = DescentGoal::Pad;
    r.target.position_m = {21.0, 0.0, 0.0};
    r.settings.limits.gimbal_fraction = 0.35;
    r.settings.limits.soft_reserve_kg = 2500.0;
    r.settings.weights.reserve_use = 50.0;
    r.settings.limits.switches[1].min_sink_mps = 41.0;
    r.settings.weights.final_lateral = 0.02;
    r.warm_start_elapsed_s = 1.5;
    DescentPlan warm;
    warm.nodes_per_phase = 10;
    warm.start_phase = 1;
    warm.goal = DescentGoal::Pad;
    warm.converged = true;
    warm.reference.duration_s = {0.0, 1.0, 13.0};
    warm.reference.x.assign(28, r.initial);
    warm.reference.u.assign(28, Vec3{5.0e5, 1.0, -2.0});
    r.warm_start = std::make_shared<const DescentPlan>(warm);
    return r;
}

// One line per node of `plan`: where it stands against the limits of the subproblem (diagnostics).
void print_node_table(const PlanRequest& r, const DescentPlan& plan) {
    const auto& l = r.settings.limits;
    const int n = plan.nodes_per_phase;
    const int last = static_cast<int>(plan.reference.x.size()) - 1;
    std::printf("  k     t      h    lat  v_down v_lat | tilt/lim deg | rate/lim | gimbal/lim deg | thrust/max | glide m | approach "
                "| defect (component)\n");
    for (int k = 0; k <= last; ++k) {
        const auto& x = plan.reference.x[static_cast<size_t>(k)];
        const Vec3& u = plan.reference.u[static_cast<size_t>(k)];
        const int p = descent_node_phase(k, n);
        const auto& set = r.model.engine_sets[static_cast<size_t>(p)];
        const double tilt = 2.0 * std::asin(std::min(1.0, std::hypot(x[9], x[10])));
        double tilt_limit = l.tilt_max_rad[static_cast<size_t>(p)];
        if (const int from_end = last - k; p == descent_phases - 1 && from_end < l.tilt_taper_nodes) {
            tilt_limit = l.terminal_tilt_rad + (tilt_limit - l.terminal_tilt_rad) * from_end / l.tilt_taper_nodes;
        }
        const double fraction = p == descent_phases - 1 && l.final_gimbal_fraction > 0.0 ? l.final_gimbal_fraction
                                                                                       : l.gimbal_fraction;
        const double rel_y = x[2] - r.target.position_m.y, rel_z = x[3] - r.target.position_m.z;
        const double lateral = std::hypot(rel_y, rel_z);
        const double glide = x[1] - r.target.position_m.x - std::tan(l.glide_slope_rad) * lateral;
        const double approach = l.approach_time_constant_s > 0.0
                                    ? x[4] - r.target.velocity_mps.x + (x[1] - r.target.position_m.x) / l.approach_time_constant_s
                                    : 0.0;
        double defect = 0.0;
        int component = -1;
        if (k < last) {
            const auto lin = linearize_interval(r.model, k / (n - 1), x, u, plan.reference.u[static_cast<size_t>(k + 1)],
                                                plan.reference.duration_s[static_cast<size_t>(k / (n - 1))], 1.0 / (n - 1),
                                                r.settings.substeps);
            const auto scale = descent_state_scales();
            for (int i = 0; i < 14; ++i) {
                const double d = std::abs(plan.reference.x[static_cast<size_t>(k + 1)][i] - lin.propagated[i]) / scale[static_cast<size_t>(i)];
                if (d > defect) defect = d, component = i;
            }
        }
        std::printf(" %2d %6.2f %6.1f %6.1f %6.1f %5.1f | %5.2f/%5.2f | %.3f/%.3f | %5.2f/%5.2f | %5.3f | %7.1f | %7.2f | %.2e (%d)\n",
                    k, plan.node_time_s(k), x[1], lateral, -x[4], std::hypot(x[5], x[6]), tilt / deg, tilt_limit / deg,
                    std::hypot(x[12], x[13]), l.rate_max_radps[static_cast<size_t>(p)],
                    std::atan2(std::hypot(u.y, u.z), u.x) / deg, std::atan(std::tan(fraction * set.gimbal_max_rad)) / deg,
                    norm(u) / set.thrust_max_n, glide, approach, defect, component);
    }
}

}  // namespace

TEST_CASE("A plan request survives a write and read round trip", "[gnc][descent]") {
    const auto r = sample_request();
    const auto path = temp_file("plan_request_round_trip");
    write_plan_request(path, r);
    const auto back = read_plan_request(path);
    std::filesystem::remove(path);
    CHECK(back.model.engine_sets[2].thrust_max_n == r.model.engine_sets[2].thrust_max_n);
    CHECK(back.model.engine_sets[1].pressure_thrust_loss_n == r.model.engine_sets[1].pressure_thrust_loss_n);
    CHECK(back.model.cd_by_mach == r.model.cd_by_mach);
    CHECK(back.model.inertia_kg_m2 == r.model.inertia_kg_m2);
    CHECK(back.initial == r.initial);
    CHECK(back.start_phase == 2);
    CHECK(back.target.goal == DescentGoal::Pad);
    CHECK(back.target.position_m == r.target.position_m);
    CHECK(back.settings.limits.gimbal_fraction == 0.35);
    CHECK(back.settings.limits.soft_reserve_kg == 2500.0);
    CHECK(back.settings.weights.reserve_use == 50.0);
    CHECK(back.settings.limits.switches[1].min_sink_mps == 41.0);
    CHECK(back.settings.weights.final_lateral == 0.02);
    CHECK(back.warm_start_elapsed_s == 1.5);
    REQUIRE(back.warm_start);
    CHECK(back.warm_start->goal == DescentGoal::Pad);
    CHECK(back.warm_start->nodes_per_phase == 10);
    CHECK(back.warm_start->reference.duration_s == r.warm_start->reference.duration_s);
    CHECK(back.warm_start->reference.x == r.warm_start->reference.x);
    CHECK(back.warm_start->reference.u == r.warm_start->reference.u);
}

TEST_CASE("Reading a plan request keeps the defaults of fields the file does not name", "[gnc][descent]") {
    const auto path = temp_file("plan_request_defaults");
    {
        std::ofstream file(path);
        file << "start_phase 1\nlimits.gimbal_fraction 0.5\n";
    }
    const auto r = read_plan_request(path);
    std::filesystem::remove(path);
    CHECK(r.start_phase == 1);
    CHECK(r.settings.limits.gimbal_fraction == 0.5);
    CHECK(r.settings.limits.throttle_margin == DescentLimits{}.throttle_margin);
    CHECK_FALSE(r.warm_start);
}

TEST_CASE("Replay of a dumped landing plan request", "[.][scvx-diag]") {
    // Diagnostics: STARSIM_SCVX_REQUEST=<file written with STARSIM_DUMP_SCVX>; STARSIM_SCVX_OVERRIDE="name value;..."
    // replaces fields (e.g. "limits.gimbal_fraction 0.6;has_warm_start 0"). Prints the plan node by node.
    const char* file = std::getenv("STARSIM_SCVX_REQUEST");
    if (!file) SKIP("STARSIM_SCVX_REQUEST not set");
    std::string path = file;
    if (const char* overrides = std::getenv("STARSIM_SCVX_OVERRIDE")) {
        std::ifstream in(path);
        std::stringstream text;
        text << in.rdbuf();
        std::string list = overrides;
        std::replace(list.begin(), list.end(), ';', '\n');
        path = temp_file("scvx_replay");
        std::ofstream out(path);
        out << text.str() << list << '\n';  // later lines replace earlier ones
    }
    const auto r = read_plan_request(path);
    DescentPlanner planner;
    const auto plan = default_plan_function()(planner, r);
    std::printf("[scvx-diag] start phase %d goal %d: conv %d it %d vc %.3g step %.3g defect %.3g dur %.2f/%.2f/%.2f final mass %.0f "
                "(min %.0f)\n",
                r.start_phase, static_cast<int>(r.target.goal), plan.converged, plan.iterations, plan.virtual_control,
                plan.last_step, plan.nonlinear_defect, plan.reference.duration_s[0], plan.reference.duration_s[1],
                plan.reference.duration_s[2], plan.reference.x.back()[descent_index::mass], r.settings.limits.min_mass_kg);
    std::printf("  model: inertia %.3g/%.3g/%.3g kg m2, lever %.2f m, set %d: thrust %.0f..%.0f N, gimbal %.2f deg, throttle rate "
                "%.2f/s\n",
                r.model.inertia_kg_m2.x, r.model.inertia_kg_m2.y, r.model.inertia_kg_m2.z, r.model.gimbal_lever_m,
                descent_phases - 1, r.model.engine_sets[2].thrust_min_n, r.model.engine_sets[2].thrust_max_n,
                r.model.engine_sets[2].gimbal_max_rad / deg, r.model.engine_sets[2].throttle_rate_per_s);
    print_node_table(r, plan);
}

namespace {

// Re-plans of the Falcon 9 drone-ship landing, MECO campaign seed 117 (aero x1.07), recorded with STARSIM_DUMP_SCVX
// before the soft reserve: every plan of this burn ends on the propellant floor (dry mass + 5 t reserve; ignition with
// 9.8 t after a long entry burn). At 848 m the re-plan converged on the floor; the next one at 741 m (185 m / 24 m/s
// across, 103.5 m/s down) needs ~0.1 t more than the floor leaves and stalled (virtual control 0.0066, all of it in the
// mass of the last interval).
PlanRequest seed117_replan(const char* name) {
    auto r = read_plan_request(std::string(STARSIM_FIXTURE_DIR) + "/scvx/" + name);
    // as ScvxLandingGuidance sets it for a re-plan
    const ScvxLandingParameters defaults;
    r.settings.limits.soft_reserve_kg = defaults.replan_reserve_fraction * defaults.snapshot.reserve_kg;
    return r;
}

}  // namespace

TEST_CASE("A re-plan short of propellant draws on the soft part of the reserve and lands", "[gnc][descent][scvx][falcon9]") {
    const auto r = seed117_replan("falcon9_asds_seed117_741m.txt");
    DescentPlanner planner;
    const auto plan = default_plan_function()(planner, r);
    const auto& end = plan.reference.x.back();
    const double floor_kg = r.settings.limits.min_mass_kg;
    if (std::getenv("STARSIM_VERBOSE")) {
        std::printf("seed 117 at 741 m: conv %d it %d vc %.2e, %.2f s, final mass %.0f kg (floor %.0f, %.0f kg of the reserve)\n",
                    plan.converged, plan.iterations, plan.virtual_control, plan.duration_s(), end[descent_index::mass], floor_kg,
                    floor_kg - end[descent_index::mass]);
    }
    REQUIRE(plan.converged);
    CHECK(std::abs(end[descent_index::position] - r.target.position_m.x) < 0.3);  // on the deck
    CHECK(std::hypot(end[descent_index::position + 1] - r.target.position_m.y,
                     end[descent_index::position + 2] - r.target.position_m.z) < 0.3);
    CHECK(std::abs(end[descent_index::velocity] - r.target.velocity_mps.x) < 0.1);
    CHECK(end[descent_index::mass] < floor_kg);                                   // it needs the reserve ...
    CHECK(end[descent_index::mass] > floor_kg - 0.2 * r.settings.limits.soft_reserve_kg);  // ... but only a little
}

TEST_CASE("A re-plan that reaches its target above the floor leaves the reserve untouched", "[gnc][descent][scvx][falcon9]") {
    const auto r = seed117_replan("falcon9_asds_seed117_848m.txt");
    DescentPlanner planner;
    const auto plan = default_plan_function()(planner, r);
    REQUIRE(plan.converged);
    CHECK(plan.reference.x.back()[descent_index::mass] >= r.settings.limits.min_mass_kg - 1.0);
}
