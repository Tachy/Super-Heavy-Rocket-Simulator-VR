// Refuelling r6: campaign calculator, its setup from config, the measured-numbers file.
#include <cmath>
#include <cstdio>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/refuelling_campaign.hpp"
#include "starsim/mission/refuelling_costs.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/spice/spice_adapters.hpp"
#include "test_support.hpp"

using namespace starsim;

namespace {
const std::filesystem::path config_dir = STARSIM_CONFIG_DIR;
}

TEST_CASE("lunar_mission_legs: the LLO mission's legs in flight order", "[refuelling][campaign][lunar]") {
    const auto m = config::load_lunar_landing_mission(config_dir, "hls_lunar_llo_depot");
    mission::OutboundPlan plan;
    plan.tli.delta_v_gcrf_mps = {3100.0, 0.0, 0.0};
    plan.insertion.delta_v_gcrf_mps = {0.0, 900.0, 0.0};
    const auto legs = sim::lunar_mission_legs(m, plan, 375.0, 330.0);
    REQUIRE(legs.size() == 5);
    CHECK(legs[0].name == "TLI");
    CHECK(legs[0].delta_v_mps == Catch::Approx(3100.0));
    CHECK(legs[1].name == "LOI");
    CHECK(legs[1].delta_v_mps == Catch::Approx(900.0));
    CHECK(legs[3].isp_s == Catch::Approx(330.0));  // the approach on the landing set
    CHECK(legs[4].name == "Aufstieg");
    CHECK(legs[2].delta_v_mps + legs[3].delta_v_mps == Catch::Approx(m.descent_delta_v_mps));
}

TEST_CASE("The measured-numbers file loads and matches its vehicles", "[refuelling][campaign][config]") {
    const auto r = config::load_refuelling_reference(config_dir);
    CHECK(r.depot.mission == "depot_launch");
    CHECK(r.depot.initial_propellant_kg == Catch::Approx(55.5e3));
    CHECK(r.tanker.mission == "tanker_flight");
    CHECK(r.tanker.delivered_oxidizer_kg + r.tanker.delivered_fuel_kg == Catch::Approx(67.5e3));
    CHECK(r.tanker.dry_mass_sensitivity == Catch::Approx(0.99));
    CHECK(r.hls.mission == "hls_refuel");
    CHECK(r.hls.main_at_docking_kg == Catch::Approx(184.5e3));
    // the reference tanker is the configured one
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    CHECK(tanker.stages.back().dry_mass_kg == Catch::Approx(r.tanker.dry_mass_kg));
}

TEST_CASE("Hohmann, deorbit and drag makeup against hand values", "[refuelling][campaign]") {
    const double mu = constants::earth_gm_m3ps2;
    const double r1 = constants::wgs84_semi_major_axis_m + 200e3;
    const double r2 = constants::wgs84_semi_major_axis_m + 300e3;
    const double a = 0.5 * (r1 + r2);
    const double expected = (std::sqrt(mu * (2 / r1 - 1 / a)) - std::sqrt(mu / r1)) +
                            (std::sqrt(mu / r2) - std::sqrt(mu * (2 / r2 - 1 / a)));
    CHECK(mission::hohmann_delta_v_mps(r1, r2, mu) == Catch::Approx(expected).epsilon(1e-12));
    CHECK(mission::hohmann_delta_v_mps(r2, r1, mu) == Catch::Approx(expected).epsilon(1e-12));
    CHECK(expected == Catch::Approx(58.5).margin(0.1));  // 58.5 m/s from 200 to 300 km
    const double rp = constants::wgs84_semi_major_axis_m + 50e3;
    const double d = std::sqrt(mu / r1) - std::sqrt(mu * (2 / r1 - 2 / (r1 + rp)));
    CHECK(mission::deorbit_delta_v_mps(r1, rp, mu) == Catch::Approx(d).epsilon(1e-12));
    CHECK(mission::deorbit_delta_v_mps(r2, rp, mu) > d);
    CHECK(mission::drag_makeup_kgps(2.5e-10, 7800.0, 2.2, 63.6, 3600.0) ==
          Catch::Approx(0.5 * 2.5e-10 * 7800.0 * 7800.0 * 2.2 * 63.6 / 3600.0));
}

namespace {
constexpr double day = 86400.0;

mission::CampaignModels constant_models(double boil_t_per_day, double tanker_t, double phasing_days) {
    mission::CampaignModels m;
    m.boil_off_kgps = [=](double, double) { return boil_t_per_day * 1e3 / day; };
    m.drag_makeup_kgps = [](double) { return 0.0; };
    m.next_tanker = [=](double e) {
        return std::optional{mission::CampaignFlight{e, e + phasing_days * day, tanker_t * 1e3}};
    };
    m.next_hls = [](double e) { return std::optional{mission::CampaignFlight{e, e + day, 0.0}}; };
    return m;
}

mission::CampaignInputs simple_inputs() {
    mission::CampaignInputs in;
    in.depot_capacity_kg = 1000e3;
    in.depot_initial_kg = 50e3;
    in.demand_kg = 300e3;
    in.attitude_gas_kgps = 0.0;
    in.transfer_vent_fraction = 0.0;
    in.port_spacing_s = 0.0;
    return in;
}
}  // namespace

TEST_CASE("Campaign against a hand computation", "[refuelling][campaign]") {
    // Tankers launch daily, arrive after 2 days with 100 t; the depot loses 1 t/day.
    // T1 arrives day 2: 50 - 2 + 100 = 148 t; T2 day 3: 247 t; T3 day 4: 346 t >= 300 t;
    // the HLS launches day 3 (one interval after T3), arrives day 4 (port spacing 0): 346 t.
    const auto r = mission::refuelling_campaign(simple_inputs(), constant_models(1.0, 100.0, 2.0));
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 3);
    CHECK(r.duration_s == Catch::Approx(4 * day));
    CHECK(r.depot_at_hls_kg == Catch::Approx(346e3));
    CHECK(r.losses.boil_off_kg == Catch::Approx(4e3));
    REQUIRE(r.tankers.size() == 3);
    CHECK(r.tankers[0].depot_after_kg == Catch::Approx(148e3));
    CHECK(r.tankers[1].flight.launch_s == Catch::Approx(day));
    CHECK(r.fill.front().depot_propellant_kg == Catch::Approx(50e3));
}

TEST_CASE("Campaign: vent and attitude gas are booked as losses", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.transfer_vent_fraction = 0.1;
    in.attitude_gas_kgps = 1e3 / day;
    const auto r = mission::refuelling_campaign(in, constant_models(0.0, 100.0, 2.0));
    REQUIRE(r.closes);
    // T1 day 2: 50 - 2 + 90 = 138; T2 day 3: 227; T3 day 4: 316 >= 300; HLS day 4.
    CHECK(r.tanker_flights == 3);
    CHECK(r.depot_at_hls_kg == Catch::Approx(316e3));
    CHECK(r.losses.transfer_vent_kg == Catch::Approx(30e3));
    CHECK(r.losses.attitude_gas_kg == Catch::Approx(4e3));
}

TEST_CASE("Campaign refusals", "[refuelling][campaign]") {
    SECTION("demand above the tank volume") {
        auto in = simple_inputs();
        in.demand_kg = 1200e3;
        const auto r = mission::refuelling_campaign(in, constant_models(1.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("Depotvolumen") != std::string::npos);
        CHECK(r.tankers.empty());
    }
    SECTION("losses eat the delivery") {
        auto in = simple_inputs();
        in.launch_interval_s = 2 * day;
        const auto r = mission::refuelling_campaign(in, constant_models(60.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("Verluste") != std::string::npos);
        CHECK(r.tanker_flights < 5);
        // an emptied depot books only what it held
        CHECK(r.losses.boil_off_kg < 60e3 * 4);
    }
    SECTION("a tanker that delivers nothing ends the campaign") {
        const auto r = mission::refuelling_campaign(simple_inputs(), constant_models(0.0, 0.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("liefert nichts") != std::string::npos);
        CHECK(r.tanker_flights == 1);
    }
    SECTION("a NaN delivery counts as nothing") {
        const auto r = mission::refuelling_campaign(simple_inputs(), constant_models(0.0, std::nan(""), 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("liefert nichts") != std::string::npos);
        CHECK(r.tankers.front().depot_after_kg == Catch::Approx(50e3));
    }
    SECTION("no window") {
        auto m = constant_models(1.0, 100.0, 2.0);
        m.next_tanker = [](double) { return std::optional<mission::CampaignFlight>{}; };
        const auto r = mission::refuelling_campaign(simple_inputs(), m);
        CHECK_FALSE(r.closes);
        CHECK(r.reason == "Kein Startfenster für Tanker 1");
    }
    SECTION("max_tankers") {
        auto in = simple_inputs();
        in.max_tankers = 2;
        const auto r = mission::refuelling_campaign(in, constant_models(1.0, 100.0, 2.0));
        CHECK_FALSE(r.closes);
        CHECK(r.reason.find("2 Tankerflügen") != std::string::npos);
    }
}

TEST_CASE("Campaign: port spacing pushes an arrival to the next window", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.port_spacing_s = 6.0 * 3600.0;
    auto m = constant_models(0.0, 100.0, 2.0);
    // the second tanker's first window would arrive 1 h after the first one: it must take a later window
    m.next_tanker = [](double e) {
        const double arrival = e < 1.0 ? 2 * day : (e < day + 3600.0 ? 2 * day + 3600.0 : e + day);
        return std::optional{mission::CampaignFlight{e, arrival, 100e3}};
    };
    const auto r = mission::refuelling_campaign(in, m);
    REQUIRE(r.tankers.size() >= 2);
    CHECK(r.tankers[1].flight.arrival_s >= r.tankers[0].flight.arrival_s + in.port_spacing_s);
    CHECK(r.tankers[1].flight.launch_s > day);
}

TEST_CASE("Campaign: a full depot clamps the delivery", "[refuelling][campaign]") {
    auto in = simple_inputs();
    in.depot_capacity_kg = 320e3;
    in.demand_kg = 310e3;
    const auto r = mission::refuelling_campaign(in, constant_models(0.0, 100.0, 2.0));
    REQUIRE(r.closes);
    CHECK(r.tankers.back().depot_after_kg == Catch::Approx(320e3));
    CHECK(r.tankers.back().received_kg == Catch::Approx(70e3));  // 250 t + 70 t fills 320 t
}

namespace {
struct Sources {
    bool spice = (test::spice_session(), true);
    const spice::SpiceEphemeris ephemeris;
    const spice::SpiceOrientation earth = spice::SpiceOrientation::earth();
    const spice::SpiceOrientation moon = spice::SpiceOrientation::moon_mean_earth();
    const spice::SpiceTimeConverter clock;
    sim::CampaignSources get() const { return {ephemeris, earth, moon, clock, config_dir}; }
};

void print(const char* tag, const sim::CampaignReport& r) {
    std::printf("[r6] %s: %s, %d tankers, %.1f d, demand %.1f t (HLS needs %.1f t, dry %.1f t, docks with %.1f t), "
                "base %.1f t, altitude %.2f t, phasing ref %.1f h, losses boil %.1f / drag %.2f / gas %.2f / vent %.1f t"
                "%s%s\n",
                tag, r.result.closes ? "closes" : "open", r.result.tanker_flights, r.result.duration_s / 86400.0,
                r.demand_kg / 1e3, r.hls_required_mass_kg / 1e3, r.hls_dry_mass_kg / 1e3,
                r.hls_propellant_at_docking_kg / 1e3, r.tanker_base_delivered_kg / 1e3, r.altitude_cost_kg / 1e3,
                r.reference_phasing_s / 3600.0, r.result.losses.boil_off_kg / 1e3,
                r.result.losses.drag_makeup_kg / 1e3, r.result.losses.attitude_gas_kg / 1e3,
                r.result.losses.transfer_vent_kg / 1e3, r.result.reason.empty() ? "" : " - ",
                r.result.reason.c_str());
    for (const auto& t : r.result.tankers) {
        std::printf("[r6]   launch %.2f d, arrival %.2f d, delivered %.1f t, depot %.1f t\n",
                    t.flight.launch_s / 86400.0, t.flight.arrival_s / 86400.0, t.flight.delivered_kg / 1e3,
                    t.depot_after_kg / 1e3);
    }
}
}  // namespace

TEST_CASE("Reference campaign: the flown windows, a plausible tanker count", "[refuelling][campaign][slow]") {
    const Sources s;
    const auto r = sim::plan_refuelling_campaign(s.get(), {});
    print("reference", r);
    REQUIRE(r.result.closes);
    // the first tanker flies the r5 window (79.8 h after the earliest); its schedule runs from the separation at
    // 120 km to Waypoint 0 within the 48 h limit (the flown 39.7 h count from the Rendezvous mark after the raise)
    REQUIRE_FALSE(r.result.tankers.empty());
    CHECK(r.result.tankers.front().flight.launch_s / 3600.0 == Catch::Approx(79.8).margin(0.2));
    CHECK(r.reference_phasing_s / 3600.0 > 39.7);
    CHECK(r.reference_phasing_s / 3600.0 < 48.0);
    CHECK(r.altitude_cost_kg == Catch::Approx(0.0).margin(1e-6));
    CHECK(r.tanker_base_delivered_kg == Catch::Approx(67.5e3).margin(1.0));
    // the HLS needs less than full tanks, more than it docks with
    CHECK(r.hls_required_mass_kg - r.hls_dry_mass_kg < r.hls_capacity_kg);
    CHECK(r.demand_kg > 0.0);
    // plausibility only (public 4 to high teens; not tuned): a two-digit count is expected from ~1,400 t / ~66 t
    CHECK(r.result.tanker_flights >= 4);
    CHECK(r.result.tanker_flights <= 40);
    for (size_t k = 1; k < r.result.tankers.size(); ++k) {
        CHECK(r.result.tankers[k].flight.arrival_s >= r.result.tankers[k - 1].flight.arrival_s + 6.0 * 3600.0);
    }
}

TEST_CASE("Campaign levers move the count the physical way", "[refuelling][campaign][slow]") {
    const Sources s;
    const auto base = sim::plan_refuelling_campaign(s.get(), {});
    sim::CampaignLevers higher;
    higher.depot_height_m = 300.0e3;
    const auto h = sim::plan_refuelling_campaign(s.get(), higher);
    print("300 km", h);
    CHECK(h.altitude_cost_kg > 0.0);
    CHECK(h.drag_makeup_kgps < base.drag_makeup_kgps);
    sim::CampaignLevers lighter;
    lighter.tanker_dry_mass_kg = 199.0e3;
    const auto l = sim::plan_refuelling_campaign(s.get(), lighter);
    print("dry -10 t", l);
    CHECK(l.tanker_base_delivered_kg == Catch::Approx(base.tanker_base_delivered_kg + 9.9e3).margin(1.0));
    CHECK(l.result.tanker_flights <= base.result.tanker_flights);
    sim::CampaignLevers poor;
    poor.depot_insulation_scale = 0.1;
    const auto p = sim::plan_refuelling_campaign(s.get(), poor);
    print("insulation x0.1", p);
    CHECK(p.result.losses.boil_off_kg > base.result.losses.boil_off_kg);
}

TEST_CASE("Campaign setup refusals", "[refuelling][campaign]") {
    const Sources s;
    sim::CampaignLevers greedy;
    greedy.tanker_reserve_kg = 200.0e3;  // keeps more than it carries
    const auto r = sim::plan_refuelling_campaign(s.get(), greedy);
    INFO(r.result.reason);
    CHECK_FALSE(r.result.closes);
    CHECK(r.result.reason.find("liefert nichts") != std::string::npos);
}
