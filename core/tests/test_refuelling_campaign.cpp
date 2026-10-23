// Refuelling r6: campaign calculator, its setup from config, the measured-numbers file.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>
#include <optional>
#include <limits>
#include <stdexcept>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include "starsim/config/mission_config.hpp"
#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/math/constants.hpp"
#include "starsim/mission/depot_orbit_track.hpp"
#include "starsim/mission/refuelling_campaign.hpp"
#include "starsim/mission/refuelling_costs.hpp"
#include "starsim/sim/lunar_landing_mission.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/sim/refuelling_campaign_text.hpp"
#include "starsim/sim/refuelling_mission.hpp"
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
    // f3 (f4: per leg, as flown): TLI, LOI, DOI + PDI, the ascent's circularization; the approach continues the
    // braking, the ascent itself lights on the surface
    CHECK(sim::lunar_in_orbit_ignitions(legs) == 5u);
    const double free = sim::lunar_mission_budget(m, plan, 1500e3, 120e3, 375.0, 330.0, 0.0).margin_kg;
    CHECK(sim::lunar_mission_budget(m, plan, 1500e3, 120e3, 375.0, 330.0, 400.0).margin_kg ==
          Catch::Approx(free - 5.0 * 400.0).epsilon(1e-12));
    // the NRHO path adds the two-burn transfers down to the LLO and back up
    auto nrho = m;
    nrho.lunar_orbit = "nrho";
    CHECK(sim::lunar_in_orbit_ignitions(sim::lunar_mission_legs(nrho, plan, 375.0, 330.0)) == 9u);
}

TEST_CASE("The measured-numbers file loads and matches its vehicles", "[refuelling][campaign][config]") {
    const auto r = config::load_refuelling_reference(config_dir);
    CHECK(r.depot.mission == "depot_launch");
    CHECK(r.depot.initial_propellant_kg == Catch::Approx(70.7e3));  // planned-kick re-measure (ascent Task 5)
    CHECK(r.tanker.mission == "tanker_flight");
    CHECK(r.tanker.delivered_oxidizer_kg + r.tanker.delivered_fuel_kg == Catch::Approx(74.25e3));  // planned kick
    CHECK(r.tanker.dry_mass_sensitivity == Catch::Approx(0.99));
    CHECK(r.tanker.campaign_offset_below_kg == Catch::Approx(1.1e3));  // flown campaign (ascent Task 5 re-calibration)
    CHECK(r.tanker.campaign_offset_above_kg == Catch::Approx(2.0e3));
    CHECK(r.hls.mission == "hls_refuel");
    CHECK(r.hls.main_at_docking_kg == Catch::Approx(202.8e3));  // planned-kick re-measure
    // the reference tanker is the configured one
    const auto tanker = config::top_stage_config(config::load_vehicle(config_dir, "starship_tanker"));
    CHECK(tanker.stages.back().dry_mass_kg == Catch::Approx(r.tanker.dry_mass_kg));
}

TEST_CASE("A tanker's delivery offset follows its phasing orbit's class against the depot (f2)",
          "[refuelling][campaign]") {
    config::RefuellingReference::Tanker t;
    t.campaign_offset_below_kg = 1.0e3;
    t.campaign_offset_above_kg = -2.0e3;
    const double re = constants::wgs84_semi_major_axis_m;
    const double depot = re + 205e3;
    const mission::PhasingOrbit below{re + 158e3, re + 198e3};
    const mission::PhasingOrbit above{re + 258e3, re + 278e3};
    CHECK(sim::campaign_delivery_offset_kg(t, below, depot) == 1.0e3);
    CHECK(sim::campaign_delivery_offset_kg(t, above, depot) == -2.0e3);
    CHECK(sim::campaign_delivery_offset_kg(t, std::nullopt, depot) == 1.0e3);  // the separation orbit
}

TEST_CASE("Depot orbit track: decay, impulses and the phase they move (f2)", "[refuelling][campaign]") {
    const double mu = constants::earth_gm_m3ps2, j2 = constants::earth_j2, re = constants::wgs84_semi_major_axis_m;
    orbit::StationMeanElements m;
    m.epoch = time::Epoch::from_tdb_seconds_past_j2000(0.0);
    m.semi_major_axis_m = re + 200e3;
    m.inclination_rad = 26.5 * constants::deg_to_rad;
    const double day = 86400.0;
    // without changes: the J2-advanced mean elements
    const mission::DepotOrbitTrack still(m, 0.0, mu, j2, re);
    const auto fixed = orbit::advance_mean_elements(m, m.epoch + 3.0 * day, mu, j2, re);
    CHECK(still.at(m.epoch + 3.0 * day).argument_of_latitude_rad ==
          Catch::Approx(fixed.argument_of_latitude_rad).margin(1e-12));
    CHECK(still.at(m.epoch + 3.0 * day).raan_rad == Catch::Approx(fixed.raan_rad).margin(1e-12));
    // decay: a falls linearly, the phase runs ahead by 3/4 n / a |da/dt| t^2 (hand value: 0.33 km/day over 6 days, the
    // flown free depot of 536 t at 198 km: 940 km ahead in the session probe)
    const double decay = -330.0 / day;
    const mission::DepotOrbitTrack falling(m, decay, mu, j2, re);
    const auto later = falling.at(m.epoch + 6.0 * day);
    CHECK(later.semi_major_axis_m == Catch::Approx(m.semi_major_axis_m - 6.0 * 330.0).margin(1e-6));
    const double n = std::sqrt(mu / std::pow(m.semi_major_axis_m, 3));
    const double ahead_m = std::remainder(later.argument_of_latitude_rad -
                                              orbit::advance_mean_elements(m, later.epoch, mu, j2, re)
                                                  .argument_of_latitude_rad,
                                          constants::two_pi) *
                           m.semi_major_axis_m;
    CHECK(ahead_m == Catch::Approx(0.75 * n * 330.0 / day * std::pow(6.0 * day, 2)).epsilon(1e-9));
    CHECK(ahead_m / 1e3 == Catch::Approx(940.0).epsilon(0.05));  // the session probe's lead (f2 2026-10-22)
    // an impulse: a jumps, the phase stays continuous, the decay rate changes from there
    mission::DepotOrbitTrack pushed(m, decay, mu, j2, re);
    const time::Epoch t1 = m.epoch + 2.0 * day;
    const double da = mission::along_track_delta_a_m(m.semi_major_axis_m, 0.55, mu);
    CHECK(da == Catch::Approx(2.0 * m.semi_major_axis_m * 0.55 / std::sqrt(mu / m.semi_major_axis_m)));
    CHECK(da == Catch::Approx(930.0).epsilon(0.01));  // ~0.9 km per transfer
    const auto before = pushed.at(t1);
    pushed.change(t1, da, 0.0);
    const auto after = pushed.at(t1);
    CHECK(after.semi_major_axis_m == Catch::Approx(before.semi_major_axis_m + da).margin(1e-6));
    CHECK(after.argument_of_latitude_rad == Catch::Approx(before.argument_of_latitude_rad).margin(1e-12));
    CHECK(pushed.at(t1 + day).semi_major_axis_m == Catch::Approx(after.semi_major_axis_m).margin(1e-6));
    CHECK(pushed.at(t1 - day).semi_major_axis_m == Catch::Approx(m.semi_major_axis_m + decay * day).margin(1e-6));
    CHECK_THROWS_AS(pushed.change(t1 - 1.0, 0.0, 0.0), std::invalid_argument);
    // keeping: 200 km decaying at 1 km/day with a 195 km floor and a 200 km target - a reboost every 5 days, the
    // phase continuous over each
    const mission::DepotKeeping keep{re + 195e3, re + 200e3};
    const mission::DepotOrbitTrack kept(m, -1000.0 / day, mu, j2, re, keep);
    const auto boosts = kept.reboosts(m.epoch, m.epoch + 12.0 * day);
    REQUIRE(boosts.size() == 2u);
    CHECK(boosts[0] - m.epoch == Catch::Approx(5.0 * day).margin(1e-6));
    CHECK(boosts[1] - m.epoch == Catch::Approx(10.0 * day).margin(1e-6));
    CHECK(kept.reboosts(m.epoch + 6.0 * day, m.epoch + 12.0 * day).size() == 1u);
    CHECK(kept.at(m.epoch + 7.0 * day).semi_major_axis_m == Catch::Approx(re + 198e3).margin(1e-6));
    const auto just_before = kept.at(boosts[0] - 1.0), just_after = kept.at(boosts[0] + 1.0);
    CHECK(std::remainder(just_after.argument_of_latitude_rad - just_before.argument_of_latitude_rad,
                         constants::two_pi) == Catch::Approx(2.0 * n).epsilon(0.01));  // + the J2 rate's ~0.4 %
    // f2 review I2: a start below the floor is not lifted at once (nothing flies or books that reboost) - it decays
    // on to its next change
    auto low = m;
    low.semi_major_axis_m = re + 194.5e3;
    const mission::DepotOrbitTrack below(low, -1000.0 / day, mu, j2, re, keep);
    CHECK(below.reboosts(m.epoch - 1.0, m.epoch + 3.0 * day).empty());
    CHECK(below.at(m.epoch + 2.0 * day).semi_major_axis_m == Catch::Approx(re + 192.5e3).margin(1e-6));
    // the same 5 days without keeping fall 5 km: the kept phase lags behind the free one
    const mission::DepotOrbitTrack free_fall(m, -1000.0 / day, mu, j2, re);
    CHECK(std::remainder(free_fall.at(m.epoch + 12.0 * day).argument_of_latitude_rad -
                             kept.at(m.epoch + 12.0 * day).argument_of_latitude_rad,
                         constants::two_pi) > 0.0);
    // drag decay: rho Cd A sqrt(mu a) / m
    CHECK(mission::drag_decay_mps(2.5e-10, 140.0, 536e3, m.semi_major_axis_m, mu) ==
          Catch::Approx(-2.5e-10 * 140.0 * std::sqrt(mu * m.semi_major_axis_m) / 536e3));
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

// The configs the reference flights read (vehicles, engines, propellants, missions, the launch site), line ends as LF.
const char* const reference_flight_inputs[] = {
    "vehicles/starship_block3.toml",  "vehicles/starship_tanker.toml",   "vehicles/starship_depot.toml",
    "vehicles/starship_hls.toml",     "vehicles/starship_hls_launch.toml", "engines/raptor3.toml",
    "engines/raptor_vacuum3.toml",    "engines/hls_landing_thruster.toml", "propellants.toml",
    "missions/depot_ascent.toml",     "missions/depot_launch.toml",        "missions/tanker_ascent.toml",
    "missions/tanker_flight.toml",    "missions/hls_ascent.toml",          "missions/hls_refuel.toml",
    "missions/ship_return_starbase.toml", "launch_sites.toml",
};

// FNV-1a 64 over the files, as 16 hex digits.
std::string inputs_digest() {
    std::uint64_t h = 14695981039346656037ull;
    for (const char* name : reference_flight_inputs) {
        std::ifstream in(config_dir / name, std::ios::binary);
        REQUIRE(in);
        for (char c; in.get(c);) {
            if (c == '\r') continue;
            h = (h ^ static_cast<unsigned char>(c)) * 1099511628211ull;
        }
    }
    char text[17];
    std::snprintf(text, sizeof text, "%016llx", static_cast<unsigned long long>(h));
    return text;
}

}  // namespace

TEST_CASE("The measured numbers belong to the current configs", "[refuelling][campaign][config]") {
    // r6 minor: a cheap drift guard - the slow r5 flights re-measure reference_flights.toml; this catches a config
    // change made without that (C++ model changes are not caught)
    std::ifstream in(config_dir / "refuelling" / "reference_flights.toml");
    REQUIRE(in);
    const std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    const std::string key = "inputs_digest = \"";
    const auto at = text.find(key);
    const std::string stored = at == std::string::npos ? std::string() : text.substr(at + key.size(), 16);
    const std::string now = inputs_digest();
    INFO("inputs changed: re-measure via the slow r5 flights, then set inputs_digest = \"" << now << "\"");
    CHECK(stored == now);
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

TEST_CASE("Campaign: the depot's orbit events at the start and after every transfer book their reboosts (f2)",
          "[refuelling][campaign]") {
    // as the hand computation below, with a 2 t reboost after every transfer: 148 - 2 = 146 t, 146 - 1 + 100 - 2 =
    // 243 t, 243 - 1 + 100 - 2 = 340 t >= 300 t
    auto m = constant_models(1.0, 100.0, 2.0);
    std::vector<std::pair<double, bool>> calls;
    m.depot_orbit_event = [&](double t_s, double, bool transfer) {
        calls.emplace_back(t_s, transfer);
        return transfer ? 2e3 : 0.0;
    };
    const auto r = mission::refuelling_campaign(simple_inputs(), m);
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 3);
    CHECK(r.depot_at_hls_kg == Catch::Approx(340e3));
    CHECK(r.losses.drag_makeup_kg == Catch::Approx(6e3));
    REQUIRE(r.tankers.size() == 3);
    CHECK(r.tankers[0].depot_after_kg == Catch::Approx(146e3));
    REQUIRE(calls.size() == 4u);
    CHECK(calls[0] == std::pair{0.0, false});
    CHECK(calls[1].first == Catch::Approx(2 * day));
    CHECK(calls[3].first == Catch::Approx(4 * day));
    CHECK(calls[3].second);
}

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

TEST_CASE("Campaign: the HLS's extra demand must be in the depot at its arrival", "[refuelling][campaign]") {
    // As the hand computation, but the HLS's phasing orbit costs it 50 t more than its reference window (f1): at
    // T3's 346 t the HLS (arriving day 4) would find less than 300 + 50 t, so T4 flies (day 5: 445 t); the HLS
    // launches one interval after T4 and arrives day 5.
    auto models = constant_models(1.0, 100.0, 2.0);
    models.next_hls = [](double e) {
        mission::CampaignFlight f{e, e + day, 0.0};
        f.extra_demand_kg = 50e3;
        return std::optional{f};
    };
    const auto r = mission::refuelling_campaign(simple_inputs(), models);
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 4);
    CHECK(r.duration_s == Catch::Approx(5 * day));
    CHECK(r.depot_at_hls_kg == Catch::Approx(445e3));
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

TEST_CASE("Vent: the gross draw that delivers a net mass", "[refuelling][campaign]") {
    // r6 minor: the vent is a share of the gross (as the tanker side books it), so gross = net / (1 - f)
    CHECK(mission::gross_for_net_kg(10e3, 0.5) == Catch::Approx(20e3));
    CHECK(mission::gross_for_net_kg(10e3, 0.0) == Catch::Approx(10e3));
    // the HLS's extra demand: 30 t at f = 0.5 asks 60 t gross, not 45 t - T3's 346 t (vent 0.5 on 200 t tankers:
    // 100 t net each) no longer covers 300 + 60 t
    auto in = simple_inputs();
    in.transfer_vent_fraction = 0.5;
    auto models = constant_models(1.0, 200.0, 2.0);
    models.next_hls = [](double e) {
        mission::CampaignFlight f{e, e + day, 0.0};
        f.extra_demand_kg = 30e3;
        return std::optional{f};
    };
    const auto r = mission::refuelling_campaign(in, models);
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 4);
    in.transfer_vent_fraction = 1.0;  // nothing would arrive
    CHECK_FALSE(mission::refuelling_campaign(in, models).closes);
}

TEST_CASE("Campaign: the HLS's wait at the depot boils off with the Sun of its arrival", "[refuelling][campaign]") {
    // r6 minor: the wait was booked once with the Sun at the campaign start; now a model of the arrival time.
    // Demand 343 t: T3's 346 t would do without the wait; with 1 t per day of arrival (4 t at day 4) T4 flies.
    auto in = simple_inputs();
    in.demand_kg = 343e3;
    auto models = constant_models(1.0, 100.0, 2.0);
    models.hls_wait_kg = [](double arrival_s) { return 1e3 * arrival_s / day; };
    const auto r = mission::refuelling_campaign(in, models);
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 4);
    CHECK(r.hls_wait_kg == Catch::Approx(5e3));  // the HLS arrives on day 5
    // the wait must fit into the HLS's tanks beside its required propellant
    in.hls_tank_room_kg = 1e3;
    const auto small = mission::refuelling_campaign(in, models);
    CHECK_FALSE(small.closes);
    CHECK_THAT(small.reason, Catch::Matchers::StartsWith("HLS-Tanks zu klein für die Wartezeit"));
}

TEST_CASE("Campaign: a cheaper HLS phasing lowers the fill it needs", "[refuelling][campaign]") {
    // f1 minor M3: the HLS's extra demand was one-sided (max 0). Demand 260 t, the HLS's window 20 t cheaper than its
    // reference: T2's 247 t at day 3 suffice; at -1000 t the requirement stops at zero, not below
    auto in = simple_inputs();
    in.demand_kg = 260e3;
    auto models = constant_models(1.0, 100.0, 2.0);
    models.next_hls = [](double e) {
        mission::CampaignFlight f{e, e + day, 0.0};
        f.extra_demand_kg = -20e3;
        return std::optional{f};
    };
    const auto r = mission::refuelling_campaign(in, models);
    REQUIRE(r.closes);
    CHECK(r.tanker_flights == 2);
    models.next_hls = [](double e) {
        mission::CampaignFlight f{e, e + day, 0.0};
        f.extra_demand_kg = -1000e3;
        return std::optional{f};
    };
    CHECK(mission::refuelling_campaign(in, models).tanker_flights <= 1);
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
        // an emptied depot books only what it held: the initial fill and the deliveries, never the full 60 t/day
        double held_kg = in.depot_initial_kg;
        for (const auto& t : r.tankers) held_kg += t.received_kg;
        CHECK(r.losses.boil_off_kg <= held_kg + 1e-6);
        CHECK(r.losses.boil_off_kg < 60e3 * r.tankers.back().flight.arrival_s / day);  // the unclamped rate
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

TEST_CASE("Campaign: the last flight before the start constrains the first launch and arrival",
          "[refuelling][campaign]") {
    mission::CampaignInputs in;
    in.depot_capacity_kg = 1000.0e3;
    in.depot_initial_kg = 0.0;
    in.demand_kg = 50.0e3;
    in.previous_launch_s = -0.5 * 86400.0;  // a launch 12 h ago: the next one from +12 h
    in.previous_arrival_s = 2.0 * 3600.0;   // arriving in 2 h: no arrival before 8 h (6 h port spacing)
    in.earliest_launch_s = 3600.0;
    mission::CampaignModels m;
    m.boil_off_kgps = [](double, double) { return 0.0; };
    m.drag_makeup_kgps = [](double) { return 0.0; };
    std::vector<double> asked;
    m.next_tanker = [&](double earliest) {  // a window every 3 h, arriving 1 h after its launch
        asked.push_back(earliest);
        const double launch = std::ceil(earliest / 10800.0) * 10800.0;
        return std::optional<mission::CampaignFlight>{{launch, launch + 3600.0, 60.0e3}};
    };
    m.next_hls = [](double earliest) {
        const double launch = std::ceil(earliest / 10800.0) * 10800.0 + 86400.0;
        return std::optional<mission::CampaignFlight>{{launch, launch + 3600.0, 0.0}};
    };
    const auto r = mission::refuelling_campaign(in, m);
    REQUIRE(r.closes);
    REQUIRE_FALSE(r.tankers.empty());
    CHECK(r.tankers.front().flight.launch_s >= 0.5 * 86400.0 - 1.0);
    CHECK(r.tankers.front().flight.arrival_s >= 8.0 * 3600.0 - 1.0);
    REQUIRE_FALSE(asked.empty());
    CHECK(asked.front() >= 0.5 * 86400.0 - 1.0);
    // without a previous flight the earliest launch holds the first one back
    mission::CampaignInputs fresh = in;
    fresh.previous_launch_s = -std::numeric_limits<double>::infinity();
    fresh.previous_arrival_s = -std::numeric_limits<double>::infinity();
    asked.clear();
    mission::refuelling_campaign(fresh, m);
    REQUIRE_FALSE(asked.empty());
    CHECK(asked.front() == Catch::Approx(3600.0));
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
    // f1: the first tanker flies the first window from the earliest (the cheapest feasible phasing orbit at that
    // crossing); the reference flight's schedule runs from the separation to Waypoint 0 within the 48 h limit less
    // the 4 h window margin
    REQUIRE_FALSE(r.result.tankers.empty());
    CHECK(r.result.tankers.front().flight.launch_s >= 0.0);
    CHECK(r.result.tankers.front().flight.launch_s < 4.0 * 86400.0);  // a window within the first days, not 13.8 d
    CHECK(r.reference_phasing_s > 0.0);
    CHECK(r.reference_phasing_s / 3600.0 <= 44.0);
    CHECK(r.altitude_cost_kg == Catch::Approx(0.0).margin(1e-6));
    const auto file = config::load_refuelling_reference(config_dir);  // the measured flight's delivery
    CHECK(r.tanker_base_delivered_kg ==
          Catch::Approx(file.tanker.delivered_oxidizer_kg + file.tanker.delivered_fuel_kg).margin(1.0));
    // the HLS needs less than full tanks, more than it docks with
    CHECK(r.hls_required_mass_kg - r.hls_dry_mass_kg < r.hls_capacity_kg);
    CHECK(r.demand_kg > 0.0);
    // plausibility only (public 4 to high teens; not tuned): a two-digit count is expected from ~1,400 t / ~66 t
    CHECK(r.result.tanker_flights >= 4);
    CHECK(r.result.tanker_flights <= 40);
    for (size_t k = 1; k < r.result.tankers.size(); ++k) {
        CHECK(r.result.tankers[k].flight.arrival_s >= r.result.tankers[k - 1].flight.arrival_s + 6.0 * 3600.0);
    }
    // f2: every flight carries its planned window, and the answer without a live start is unchanged
    for (const auto& t : r.result.tankers) {
        REQUIRE(t.flight.window);
        CHECK(t.flight.window->liftoff - r.start == Catch::Approx(t.flight.launch_s).margin(1e-3));
    }
    REQUIRE(r.result.hls);
    CHECK(r.result.hls->window.has_value());
    // the answer (f2: 14 tankers / 51.7 d after the re-measured delivery, 66.3 t - the raise reaches its mean orbit;
    // 13 / 49.7 d with the flown campaign's per-class calibration, +1.7 t below / +2.25 t above the depot; 14 / 54.7 d
    // with the depot's orbit followed: drag decay and the transfers' pushes move its phase and so the windows; 13 /
    // 52.5 d with the flown decay factor 1.076; 14 / 52.3 d after the ascent Task 2 re-measure: 63.6 t per tanker
    // instead of 67.7 t, the depot's own 69.6 t instead of 55.5 t; 12 / 50.6 d after the ascent Task 5 re-measure: the
    // planned kick (tanker 18.1 deg instead of the webcast-tuned 14) leaves 105.6 t instead of 93.0 t after the ascent,
    // 74.25 t delivered per tanker, the campaign offsets re-calibrated)
    CHECK(r.result.tanker_flights == 12);
    CHECK(r.result.duration_s / 86400.0 == Catch::Approx(50.63).margin(0.05));
}

TEST_CASE("Campaign live start: the flown depot's orbit, fill and last flight", "[refuelling][campaign]") {
    const Sources s;
    const auto first = sim::plan_refuelling_campaign(s.get(), {});
    REQUIRE(first.result.closes);
    // a depot 10 days in, on tanker_flight's [depot] orbit advanced by J2, with 300 t
    const auto tanker = config::load_refuelling_flight_mission(config_dir, "tanker_flight");
    const time::Epoch epoch = first.start + 10.0 * 86400.0;
    sim::CampaignLive live;
    live.epoch = epoch;
    live.depot_gcrf = sim::depot_orbit(tanker.depot, first.start, s.earth)(epoch);
    live.depot_propellant_kg = 300.0e3;
    live.previous_launch = epoch - 6.0 * 3600.0;
    live.demand = first.hls_demand;
    const auto r = sim::plan_refuelling_campaign(s.get(), {}, &live);
    print("live", r);
    INFO(r.result.reason);
    REQUIRE(r.result.closes);
    CHECK(r.start == epoch);
    CHECK(r.demand_kg == Catch::Approx(first.demand_kg));  // the cached demand
    CHECK(r.result.tanker_flights < first.result.tanker_flights);  // 300 t instead of 55.5 t
    REQUIRE_FALSE(r.result.tankers.empty());
    const auto& f = r.result.tankers.front().flight;
    REQUIRE(f.window);
    CHECK(f.window->liftoff - r.start == Catch::Approx(f.launch_s).margin(1e-3));
    CHECK(f.launch_s >= 18.0 * 3600.0 - 1.0);  // the previous launch 6 h ago + 24 h turnaround
    // the energy mean of the synthetic state (mean elements used as osculating ones: up to ~2 km off) [E]
    CHECK(r.depot_height_m == Catch::Approx(200.0e3).margin(3.0e3));
    sim::CampaignLevers height;
    height.depot_height_m = 250.0e3;
    CHECK_FALSE(sim::plan_refuelling_campaign(s.get(), height, &live).result.closes);  // refused with a live start
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

TEST_CASE("Reference phasing reproduces the single-orbit campaign", "[refuelling][campaign][slow]") {
    const Sources s;
    sim::CampaignLevers ref;
    ref.reference_phasing = true;
    const auto r = sim::plan_refuelling_campaign(s.get(), ref);
    print("reference phasing", r);
    REQUIRE(r.result.closes);
    for (const auto& t : r.result.tankers) {
        REQUIRE(t.flight.phasing_orbit);
        CHECK(t.flight.phasing_orbit->periapsis_radius_m ==
              Catch::Approx(constants::wgs84_semi_major_axis_m + 170.0e3).margin(1.0));
    }
}

TEST_CASE("Per-window phasing: more arrivals, fewer days", "[refuelling][campaign][slow]") {
    const Sources s;
    sim::CampaignLevers ref;
    ref.reference_phasing = true;
    const auto single = sim::plan_refuelling_campaign(s.get(), ref);
    const auto chosen = sim::plan_refuelling_campaign(s.get(), {});
    print("per-window phasing", chosen);
    REQUIRE(single.result.closes);
    REQUIRE(chosen.result.closes);
    CHECK(chosen.result.duration_s < single.result.duration_s);
    // shorter phasing means less boil-off per tanker; the count may still differ by one either way
    CHECK(chosen.result.tanker_flights <= single.result.tanker_flights + 1);
    int above = 0;
    for (const auto& t : chosen.result.tankers) {
        REQUIRE(t.flight.phasing_orbit);
        above += t.flight.phasing_orbit->periapsis_radius_m > constants::wgs84_semi_major_axis_m + 200.0e3;
    }
    CHECK(above > 0);
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

TEST_CASE("refuelling_report: levers from the command line", "[refuelling][campaign]") {
    const auto l = sim::campaign_levers_from_args({"--height-km", "300", "--tanker-dry-t", "199", "--interval-h", "12",
                                                   "--insulation", "0.5", "--hls-wait-d", "2"});
    CHECK(*l.depot_height_m == Catch::Approx(300e3));
    CHECK(*l.tanker_dry_mass_kg == Catch::Approx(199e3));
    CHECK(l.launch_interval_s == Catch::Approx(12 * 3600.0));
    CHECK(l.depot_insulation_scale == Catch::Approx(0.5));
    CHECK(l.hls_wait_s == Catch::Approx(2 * 86400.0));
    CHECK_FALSE(l.tanker_reserve_kg.has_value());
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height-km"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height-km", "abc"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--height-km", "300x"}), std::invalid_argument);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--insulation", "-1"}), std::invalid_argument);
    CHECK(*sim::campaign_levers_from_args({"--reserve-t", "0"}).tanker_reserve_kg == 0.0);
    // f1: a flag without a value
    CHECK_FALSE(l.reference_phasing);
    const auto r = sim::campaign_levers_from_args({"--reference-phasing", "--height-km", "250"});
    CHECK(r.reference_phasing);
    CHECK(*r.depot_height_m == Catch::Approx(250e3));
    // f1 minor M6: the flag anywhere, twice; a value missing after it
    const auto middle = sim::campaign_levers_from_args({"--height-km", "250", "--reference-phasing", "--interval-h", "12"});
    CHECK(middle.reference_phasing);
    CHECK(middle.launch_interval_s == Catch::Approx(12 * 3600.0));
    CHECK(sim::campaign_levers_from_args({"--height-km", "250", "--reference-phasing"}).reference_phasing);
    CHECK(sim::campaign_levers_from_args({"--reference-phasing", "--reference-phasing"}).reference_phasing);
    CHECK_THROWS_AS(sim::campaign_levers_from_args({"--reference-phasing", "--height-km"}), std::invalid_argument);
}

TEST_CASE("refuelling_report: the text names the answer and the reason", "[refuelling][campaign]") {
    const Sources s;
    sim::CampaignReport r;
    r.result.closes = true;
    r.result.tanker_flights = 12;
    r.result.tankers.resize(12);
    r.result.tankers.front().flight.phasing_orbit =
        mission::PhasingOrbit{constants::wgs84_semi_major_axis_m + 150.0e3, constants::wgs84_semi_major_axis_m + 190.0e3};
    const auto text = sim::campaign_report_text(r, s.clock);
    CHECK(text.find("Tanker flights: 12") != std::string::npos);
    CHECK(text.find("150 x 190 km") != std::string::npos);  // f1: the chosen phasing orbit
    r.result.closes = false;
    r.result.reason = "Depotvolumen reicht nicht";
    CHECK(sim::campaign_report_text(r, s.clock).find("Depotvolumen reicht nicht") != std::string::npos);
}

TEST_CASE("Campaign: a full depot that cannot bridge the gap to the HLS ends at once", "[refuelling][campaign]") {
    // review I2: 320 t tanks, demand 310 t, 2 t/day boil-off, the HLS 10 days after its launch: 320 - 20 < 310
    auto in = simple_inputs();
    in.depot_capacity_kg = 320e3;
    in.demand_kg = 310e3;
    auto m = constant_models(2.0, 100.0, 2.0);
    m.next_hls = [](double e) { return std::optional{mission::CampaignFlight{e, e + 10 * day, 0.0}}; };
    const auto r = mission::refuelling_campaign(in, m);
    CHECK_FALSE(r.closes);
    CHECK(r.reason.find("bis zur HLS-Ankunft") != std::string::npos);
    CHECK(r.tanker_flights < 6);
}

TEST_CASE("Campaign: an initial fill above the tank volume is clamped", "[refuelling][campaign]") {
    // review I3: received must never turn negative
    auto in = simple_inputs();
    in.depot_capacity_kg = 320e3;
    in.depot_initial_kg = 400e3;
    in.demand_kg = 310e3;
    const auto r = mission::refuelling_campaign(in, constant_models(0.0, 100.0, 2.0));
    REQUIRE(r.closes);
    CHECK(r.fill.front().depot_propellant_kg == Catch::Approx(320e3));
    for (const auto& t : r.tankers) CHECK(t.received_kg >= 0.0);
}

TEST_CASE("Campaign: alternating window gaps - a pair of tankers gains, one long gap loses", "[refuelling][campaign]") {
    // review I4: gaps 5 / 12 days alternate, 10 t/day boil-off, 100 t per tanker: the long gap loses 120 t (more than
    // one delivery), a pair gains 100 + 100 - 170 = 30 t. Fills after the tankers: 200, 250, 230 (< 250: the old
    // consecutive guard stopped here), 280, 260, ... 400 at day 90 (the HLS a day later finds 390), 430 at day 107.
    auto in = simple_inputs();
    in.depot_initial_kg = 100e3;
    in.demand_kg = 400e3;
    in.max_tankers = 40;
    auto m = constant_models(10.0, 100.0, 0.0);
    m.next_tanker = [](double e) {
        // windows at 0, 5, 17, 22, 34, ... days (period 17 d)
        const double period = 17 * day;
        const double base = std::floor(e / period) * period;
        double launch = base;
        if (e > base) launch = e <= base + 5 * day ? base + 5 * day : base + period;
        return std::optional{mission::CampaignFlight{launch, launch, 100e3}};
    };
    m.next_hls = [](double e) { return std::optional{mission::CampaignFlight{e, e, 0.0}}; };
    const auto r = mission::refuelling_campaign(in, m);
    INFO(r.reason);
    CHECK(r.closes);
}

TEST_CASE("Campaign levers are validated", "[refuelling][campaign]") {
    // review I3: one validator for the CLI and the game
    sim::CampaignLevers ok;
    CHECK(sim::campaign_lever_error(ok).empty());
    const auto refused = [](auto set) {
        sim::CampaignLevers l;
        set(l);
        return !sim::campaign_lever_error(l).empty();
    };
    CHECK(refused([](sim::CampaignLevers& l) { l.depot_insulation_scale = 0.0; }));
    CHECK(refused([](sim::CampaignLevers& l) { l.depot_height_m = -100e3; }));
    CHECK(refused([](sim::CampaignLevers& l) { l.launch_interval_s = -5.0; }));
    CHECK(refused([](sim::CampaignLevers& l) { l.tanker_reserve_kg = -50e3; }));
    CHECK(refused([](sim::CampaignLevers& l) { l.hls_wait_s = std::nan(""); }));
    CHECK(refused([](sim::CampaignLevers& l) { l.phasing_limit_s = 0.0; }));
}

TEST_CASE("A looser phasing limit never costs days", "[refuelling][campaign][slow]") {
    // review I1: the window chooser flies the latest launch that reaches the same arrival (the shortest phasing).
    // f1: the calculator takes the first window, so it minimises the days: a looser limit opens earlier windows with
    // longer phasing (more tanker boil-off) - it may cost tankers (72 h: 12 in 27.1 d vs 28 h: 10 in 64.0 d)
    const Sources s;
    sim::CampaignLevers tight, loose;
    tight.phasing_limit_s = 28.0 * 3600.0;
    loose.phasing_limit_s = 72.0 * 3600.0;
    const auto a = sim::plan_refuelling_campaign(s.get(), tight);
    const auto b = sim::plan_refuelling_campaign(s.get(), loose);
    print("28 h", a);
    print("72 h", b);
    REQUIRE(a.result.closes);
    REQUIRE(b.result.closes);
    CHECK(b.result.duration_s <= a.result.duration_s + 3600.0);
}

TEST_CASE("Flown campaign text: rows, losses, verdict", "[refuelling][campaign][campaign_flown]") {
    test::spice_session();
    const spice::SpiceTimeConverter clock;
    sim::CampaignFlightState st;
    st.phase = sim::CampaignFlightPhase::Complete;
    st.start = clock.from_utc({2026, 10, 20, 0, 0, 0.0});
    sim::CampaignFlownRow row;
    row.number = 1;
    row.launch = *st.start + 3.0 * 86400.0;
    row.arrival = *st.start + 4.5 * 86400.0;
    row.delivered_kg = 70.4e3;
    row.depot_after_kg = 122.6e3;
    st.rows.push_back(row);
    st.tankers_flown = 1;
    const auto text = sim::campaign_flight_text(st, {}, clock);
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("70.4"));
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("122.6"));
    CHECK_THAT(text, Catch::Matchers::ContainsSubstring("Comparison: passed"));
    const auto failed = sim::campaign_flight_text(st, {"Tankerzahl: geflogen 1, Rechner 2"}, clock);
    CHECK_THAT(failed, Catch::Matchers::ContainsSubstring("Tankerzahl"));
    CHECK_THAT(failed, Catch::Matchers::ContainsSubstring("Comparison: FAILED"));
}
