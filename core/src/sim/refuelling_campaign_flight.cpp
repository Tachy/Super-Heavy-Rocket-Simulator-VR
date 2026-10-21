#include "starsim/sim/refuelling_campaign_flight.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <utility>

#include "starsim/config/launch_site_config.hpp"
#include "starsim/config/mission_config.hpp"
#include "starsim/config/vehicle_config.hpp"
#include "starsim/sim/propellant_transfer.hpp"
#include "starsim/sim/refuelling_mission.hpp"

namespace starsim::sim {

namespace {

const mission::CampaignInputs booked{};  // the calculator's [E] items (attitude gas, vent): one source
constexpr double gas_batch_kg = 1.0;     // [E] the attitude gas is drawn in batches of this

double main_kg(const FlightSession& s, size_t body) { return s.body(body).vehicle().main_propellant_kg(); }

// Draws `kg` from the body's main tanks in proportion to their contents (attitude gas, vent bookings).
void draw_main(FlightSession& s, size_t body, double kg) {
    const auto& v = s.body(body).vehicle();
    const size_t stage = v.stages().size() - 1;
    const double total = v.main_propellant_kg();
    if (!(total > 0.0) || !(kg > 0.0)) return;
    const double keep = std::max(0.0, 1.0 - kg / total);
    std::vector<std::pair<std::string, double>> masses;
    for (const auto& t : v.stages().back().tanks) {
        if (t.config.role == config::TankRole::Main) masses.emplace_back(t.config.id, t.mass_kg * keep);
    }
    for (const auto& [id, mass] : masses) s.set_tank_mass(body, stage, id, mass);
}

bool transfer_running(const FlightSession& s) {
    const auto p = s.transfer_state().phase;
    return p == TransferPhase::Settling || p == TransferPhase::Chilldown || p == TransferPhase::Flowing ||
           p == TransferPhase::Paused;
}

const FlightSession::MissionMark* mark(const FlightSession& s, IssMissionLeg leg) {
    for (const auto& m : s.iss_mission_marks()) {
        if (m.leg == leg) return &m;
    }
    return nullptr;
}

std::string flight_name(bool hls, int number) { return hls ? std::string("HLS") : "Tanker " + std::to_string(number); }

}  // namespace

const char* to_string(CampaignFlightPhase phase) {
    switch (phase) {
        case CampaignFlightPhase::DepotAscent: return "Depotstart";
        case CampaignFlightPhase::Planning: return "Planung";
        case CampaignFlightPhase::WaitLaunch: return "Warten auf das Startfenster";
        case CampaignFlightPhase::Flying: return "Flug";
        case CampaignFlightPhase::StationKeeping: return "Bahnhaltung";
        case CampaignFlightPhase::Complete: return "Abgeschlossen";
        case CampaignFlightPhase::Ended: return "Beendet";
    }
    return "?";
}

RefuellingCampaignFlight::RefuellingCampaignFlight(CampaignSources sources,
                                                   config::RefuellingCampaignFlownMission mission)
    : sources_(std::move(sources)),
      mission_(std::move(mission)),
      reference_(config::load_refuelling_reference(sources_.config_dir)),
      tanker_(config::load_refuelling_flight_mission(sources_.config_dir, reference_.tanker.mission)),
      hls_(config::load_refuelling_flight_mission(sources_.config_dir, reference_.hls.mission)) {
    state_.status = "Depotstart";
}

void RefuellingCampaignFlight::start_placed(time::Epoch epoch) {
    state_.start = epoch;
    state_.phase = CampaignFlightPhase::Planning;
    state_.status = "Planung";
}

void RefuellingCampaignFlight::update(FlightSession& s) {
    const double dt = last_update_ ? s.epoch() - *last_update_ : 0.0;
    last_update_ = s.epoch();
    if (state_.phase == CampaignFlightPhase::Complete || state_.phase == CampaignFlightPhase::Ended) return;
    if (state_.start) book_attitude_gas(s, dt);
    if (s.station_index()) state_.depot_propellant_kg = main_kg(s, *s.station_index());
    switch (state_.phase) {
        case CampaignFlightPhase::DepotAscent:
            if (s.iss_mission_leg() == IssMissionLeg::Ended) return end(s.iss_mission_status());
            if (s.iss_mission_leg() != IssMissionLeg::Complete || !s.station_index()) return;
            s.keep_only_station();  // the caught booster goes
            return begin(s);
        case CampaignFlightPhase::Planning: return plan_next(s);
        case CampaignFlightPhase::StationKeeping:
            if (s.station_reboost_active()) return;
            if (s.last_reboost()) {
                state_.reboosts.push_back(*s.last_reboost());
                state_.losses.station_keeping_kg += s.last_reboost()->propellant_kg;
            }
            state_.phase = CampaignFlightPhase::Planning;  // the orbit changed: plan again
            return;
        case CampaignFlightPhase::WaitLaunch:
            if (s.epoch() + FlightSession::tick_s < next_->window->liftoff) return;
            return launch(s);
        case CampaignFlightPhase::Flying: return watch_flight(s);
        default: return;
    }
}

std::optional<time::Epoch> RefuellingCampaignFlight::wake_epoch() const {
    if (state_.phase == CampaignFlightPhase::WaitLaunch && next_ && next_->window) return next_->window->liftoff;
    return std::nullopt;
}

void RefuellingCampaignFlight::book_attitude_gas(FlightSession& s, double dt_s) {
    pending_gas_kg_ += booked.attitude_gas_kgps * std::max(0.0, dt_s);
    if (pending_gas_kg_ < gas_batch_kg || !s.station_index() || transfer_running(s)) return;
    draw_main(s, *s.station_index(), pending_gas_kg_);
    state_.losses.attitude_gas_kg += pending_gas_kg_;
    pending_gas_kg_ = 0.0;
}

void RefuellingCampaignFlight::begin(FlightSession& s) {
    state_.start = s.epoch();
    plan_next(s);  // the first plan becomes the calculator's answer
}

void RefuellingCampaignFlight::plan_next(FlightSession& s) {
    const size_t depot = *s.station_index();
    const auto& b = s.body(depot).state();
    CampaignLive live;
    live.epoch = s.epoch();
    live.depot_gcrf = {b.position_gcrf_m, b.velocity_gcrf_mps};
    live.depot_propellant_kg = main_kg(s, depot);
    live.previous_launch = last_launch_;
    live.previous_arrival = last_arrival_;
    live.demand = demand_;
    const auto report = plan_refuelling_campaign(sources_, {}, &live);
    if (!demand_ && report.demand_kg > 0.0) demand_ = report.hls_demand;
    if (mission_.depot_placed && !placed_fill_set_) {  // decision 10: demand - k net deliveries, then the real plan
        if (!demand_) return end(report.result.reason);
        placed_fill_set_ = true;
        const double fill = report.demand_kg - mission_.placed_shortfall_tankers * report.tanker_base_delivered_kg *
                                                   (1.0 - booked.transfer_vent_fraction);
        draw_main(s, depot, live.depot_propellant_kg - std::max(0.0, fill));
        return plan_next(s);
    }
    if (!state_.calculator) {
        state_.calculator = report;
        initial_fill_kg_ = live.depot_propellant_kg;
        state_.hls_required_propellant_kg = report.hls_required_mass_kg - report.hls_dry_mass_kg;
    }
    if (!report.result.closes) return end(report.result.reason);
    const bool tanker = !report.result.tankers.empty();
    next_ = tanker ? report.result.tankers.front().flight : *report.result.hls;
    if (!next_->window) return end("Planung ohne Startfenster");
    next_parameters_ = tanker ? report.tanker_window : report.hls_window;
    state_.tankers_planned = state_.tankers_flown + report.result.tanker_flights;
    state_.next_launch = next_->window->liftoff;
    state_.next_is_hls = !tanker;
    // decision 2: a window later than the previous plan's next one (a flight still under way, a re-plan)
    if (planned_second_launch_ && std::abs(*state_.next_launch - *planned_second_launch_) > 3600.0) {
        moved_note_ = "Fenster verschoben gegenüber der vorigen Planung";
    }
    planned_second_launch_.reset();
    if (tanker && report.result.tankers.size() > 1) {
        planned_second_launch_ = report.start + report.result.tankers[1].flight.launch_s;
    } else if (tanker && report.result.hls) {
        planned_second_launch_ = report.start + report.result.hls->launch_s;
    }
    // decision 6: station keeping when low, nothing in flight and the next arrival far enough away
    const double arrival_in_s = report.start + next_->arrival_s - s.epoch();
    if (s.station_mean_height_m() < mission_.keeping_min_height_m && arrival_in_s > mission_.keeping_min_lead_s) {
        if (s.start_station_reboost(mission_.keeping_target_height_m, mission_.keeping_engine_groups)) {
            state_.phase = CampaignFlightPhase::StationKeeping;
            state_.status = "Bahnhaltung des Depots";
            return;
        }
        state_.status = "Bahnhaltung abgelehnt: " + s.reboost_status();
    }
    state_.phase = CampaignFlightPhase::WaitLaunch;
    state_.status = flight_name(!tanker, static_cast<int>(state_.rows.size()) + 1) + ": warten auf das Startfenster";
}

void RefuellingCampaignFlight::launch(FlightSession& s) {
    const auto& m = state_.next_is_hls ? hls_ : tanker_;
    const auto started = launch_campaign_flight(s, sources_.config_dir, m, *next_->window, next_parameters_);
    if (!started.refusal.empty()) {
        state_.status = "Start abgelehnt: " + started.refusal;
        state_.phase = CampaignFlightPhase::Planning;  // the next window
        return;
    }
    CampaignFlownRow row;
    row.hls = state_.next_is_hls;
    row.number = row.hls ? 0
                         : static_cast<int>(std::count_if(state_.rows.begin(), state_.rows.end(),
                                                          [](const CampaignFlownRow& r) { return !r.hls; })) +
                               1;
    row.launch = s.epoch();
    row.arrival = s.epoch();
    row.phasing_orbit = next_->window->phasing_orbit;
    row.note = std::exchange(moved_note_, {});
    state_.rows.push_back(row);
    last_launch_ = s.epoch();
    state_.phase = CampaignFlightPhase::Flying;
    state_.status = flight_name(row.hls, row.number) + ": im Flug";
}

void RefuellingCampaignFlight::watch_flight(FlightSession& s) {
    const auto leg = s.iss_mission_leg();
    if (leg != IssMissionLeg::Complete && leg != IssMissionLeg::Ended) {
        state_.status = flight_name(state_.rows.back().hls, state_.rows.back().number) + ": " + s.iss_mission_status();
        return;
    }
    auto& row = state_.rows.back();
    row.failed = leg == IssMissionLeg::Ended;
    const size_t depot = *s.station_index();
    if (row.failed) {
        row.note = s.iss_mission_status();
        char range[64];
        std::snprintf(range, sizeof range, " (Abstand %.1f km)",
                      norm(s.body(s.controlled_index()).state().position_gcrf_m - s.body(depot).state().position_gcrf_m) /
                          1000.0);
        row.note += range;
    }
    const auto* arrive = mark(s, IssMissionLeg::Proximity);
    row.arrival = arrive ? arrive->epoch : s.epoch();
    if (arrive) last_arrival_ = arrive->epoch;
    const auto* raise = mark(s, IssMissionLeg::Raise);
    const auto* rendezvous = mark(s, IssMissionLeg::Rendezvous);
    const auto* docked = mark(s, IssMissionLeg::Docked);
    const auto* separation = raise ? raise : rendezvous;
    if (separation && arrive) row.phasing_s = arrive->epoch - separation->epoch;
    if (rendezvous) row.residue_kg = rendezvous->chaser_main_propellant_kg;
    if (rendezvous && docked) {
        row.rendezvous_kg = rendezvous->chaser_main_propellant_kg - docked->chaser_main_propellant_kg;
    }
    const auto* before = mark(s, IssMissionLeg::Transfer);
    const auto* after = mark(s, IssMissionLeg::Departure);
    if (after) row.main_at_undock_kg = after->chaser_main_propellant_kg;
    if (before && after) {
        const double vent = booked.transfer_vent_fraction;
        if (row.hls) {
            row.delivered_kg = after->chaser_main_propellant_kg - before->chaser_main_propellant_kg;
            draw_main(s, s.controlled_index(), vent * row.delivered_kg);
            hls_received_kg_ += row.delivered_kg;
            state_.hls_propellant_kg = s.body(s.controlled_index()).vehicle().propellant_kg();
            state_.hls_arrival = row.arrival;
        } else {
            row.delivered_kg = after->station_main_propellant_kg - before->station_main_propellant_kg;
            draw_main(s, depot, vent * row.delivered_kg);
            depot_vent_kg_ += vent * row.delivered_kg;
            delivered_total_kg_ += row.delivered_kg;
        }
        state_.losses.transfer_vent_kg += vent * row.delivered_kg;
    }
    row.depot_after_kg = main_kg(s, depot);
    if (!row.hls && !row.failed) ++state_.tankers_flown;
    // the k-th delivering tanker against the first plan's k-th (f2: a failed flight does not shift the pairs)
    if (!row.hls && !row.failed && state_.calculator) {
        const auto& calc = state_.calculator->result.tankers;
        if (static_cast<size_t>(state_.tankers_flown) <= calc.size()) row.calculator = calc[state_.tankers_flown - 1];
    }
    if (!row.hls && !row.failed && row.number == 1 && mission_.return_first_tanker) fly_first_return(s);
    const bool hls_done = row.hls && !row.failed;
    s.keep_only_station();
    if (hls_done) return finish(s);
    state_.phase = CampaignFlightPhase::Planning;
}

void RefuellingCampaignFlight::fly_first_return(const FlightSession& s) {
    // decision 3: in its own session from the departure state, to the catch at the tanker's launch site
    const auto& tanker = s.body(s.controlled_index());
    FlightSession side(sources_.ephemeris, sources_.earth, sources_.config_dir, environment::PointMassGravityConfig{},
                       &sources_.moon);
    side.set_boil_off(true);
    side.set_ignition_settling(s.ignition_settling());
    const auto ascent = config::load_ascent_mission(sources_.config_dir, tanker_.ascent_id);
    side.set_launch_site(launch_site_from_config(config::load_launch_site(sources_.config_dir, ascent.site_id)));
    side.place_vehicle(tanker.vehicle().copy_upper(0), tanker.state(), s.epoch());
    const auto deorbit = config::load_ship_return_mission(sources_.config_dir, tanker_.return_id).deorbit;
    if (!side.plan_ship_return(ship_deorbit_parameters(deorbit))) {
        state_.first_return = ReturnResult::Lost;
        state_.first_return_note = side.ship_return_plan() ? side.ship_return_plan()->reason : "Rückkehr nicht möglich";
        return;
    }
    const auto t0 = side.epoch();
    const double limit_s = FlightSession::tanker_reserve_search_s + 86400.0;  // the pass search + a day
    while (side.ship_return_result(side.controlled_index()) == ReturnResult::Flying && side.epoch() - t0 < limit_s &&
           side.active()) {
        side.advance_to(side.epoch() + 3600.0);
    }
    state_.first_return = side.ship_return_result(side.controlled_index());
    state_.first_return_main_kg = side.body(side.controlled_index()).vehicle().main_propellant_kg();
    state_.first_return_note = *state_.first_return == ReturnResult::Flying ? std::string("Rückkehr nicht beendet")
                                                                            : std::string(to_string(*state_.first_return));
}

void RefuellingCampaignFlight::finish(FlightSession& s) {
    state_.phase = CampaignFlightPhase::Complete;
    state_.status = "Kampagne abgeschlossen";
    const double depot_now_kg = main_kg(s, *s.station_index());
    auto& l = state_.losses;
    l.boil_off_kg = initial_fill_kg_ + delivered_total_kg_ - hls_received_kg_ - l.station_keeping_kg -
                    l.attitude_gas_kg - depot_vent_kg_ - depot_now_kg;
    state_.next_launch.reset();
}

void RefuellingCampaignFlight::end(const std::string& reason) {
    state_.phase = CampaignFlightPhase::Ended;
    state_.status = "Kampagne beendet: " + reason;
    state_.next_launch.reset();
}

std::vector<std::string> RefuellingCampaignFlight::comparison_misses() const {
    if (!state_.calculator) return {"keine Rechnerplanung"};
    std::vector<std::string> misses;
    const auto& calc = state_.calculator->result;
    char line[200];
    if (state_.tankers_flown != calc.tanker_flights) {
        std::snprintf(line, sizeof line, "Tankerzahl: geflogen %d, Rechner %d", state_.tankers_flown,
                      calc.tanker_flights);
        misses.emplace_back(line);
    }
    if (!state_.hls_arrival || !state_.start) {
        misses.emplace_back("HLS nicht angekommen");
    } else if (const double flown_s = *state_.hls_arrival - *state_.start;
               std::abs(flown_s - calc.duration_s) > mission_.duration_tolerance_s) {
        std::snprintf(line, sizeof line, "Dauer: geflogen %.2f d, Rechner %.2f d", flown_s / 86400.0,
                      calc.duration_s / 86400.0);
        misses.emplace_back(line);
    }
    for (const auto& r : state_.rows) {
        if (r.hls || r.failed || !r.calculator) continue;
        const double c = r.calculator->depot_after_kg;
        if (std::abs(r.depot_after_kg - c) > mission_.fill_tolerance * c) {
            std::snprintf(line, sizeof line, "Tanker %d: Depot %.1f t geflogen, %.1f t Rechner", r.number,
                          r.depot_after_kg / 1000.0, c / 1000.0);
            misses.emplace_back(line);
        }
    }
    if (state_.hls_arrival && state_.hls_propellant_kg < state_.hls_required_propellant_kg) {
        std::snprintf(line, sizeof line, "HLS: %.1f t Treibstoff, %.1f t gefordert", state_.hls_propellant_kg / 1000.0,
                      state_.hls_required_propellant_kg / 1000.0);
        misses.emplace_back(line);
    }
    return misses;
}

MissionStart start_refuelling_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                              const config::RefuellingCampaignFlownMission& mission,
                                              const time::TimeScaleConverter& time) {
    MissionStart s;
    s.kind = config::MissionKind::RefuellingCampaignFlown;
    if (!session.moon_orientation()) {
        s.refusal = "Kampagne: die Sitzung hat keine Mondorientierung";
        return s;
    }
    session.set_supervisor(nullptr);
    auto driver = std::make_shared<RefuellingCampaignFlight>(
        CampaignSources{session.ephemeris(), session.earth_orientation(), *session.moon_orientation(), time,
                        config_dir},
        mission);
    const auto reference = config::load_refuelling_reference(config_dir);
    if (mission.depot_placed) {
        const auto tanker = config::load_refuelling_flight_mission(config_dir, reference.tanker.mission);
        const time::Epoch earliest = time.from_utc(tanker.earliest_utc);
        auto depot = config::top_stage_config(config::load_vehicle(config_dir, tanker.depot.vehicle_id));
        double capacity_kg = 0.0;
        for (const auto& t : depot.stages.back().tanks) {
            if (t.role == config::TankRole::Main) capacity_kg += t.capacity_kg;
        }
        fill_main_tanks(depot.stages.back(), capacity_kg);  // full: the driver draws it down to the placed fill
        session.clear_bodies();
        session.add_station(depot, depot_orbit(tanker.depot, earliest, session.earth_orientation())(earliest), earliest);
        driver->start_placed(earliest);
        s.epoch = earliest;
    } else {
        const auto launch = start_depot_launch_mission(
            session, config_dir, config::load_depot_launch_mission(config_dir, reference.depot.mission), time);
        s.epoch = launch.epoch;
        s.site_id = launch.site_id;
    }
    session.set_supervisor(driver);
    s.return_planned = true;
    return s;
}

}  // namespace starsim::sim
