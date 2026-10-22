#pragma once

// Refuelling f2: the campaign flown in one session (spec f2 "Fully flown campaign") - installed as the session's
// supervisor. After the depot became the station it plans every flight live (plan_refuelling_campaign with
// CampaignLive), warps to the launch, flies it (launch_campaign_flight), removes it after its departure, reboosts the
// depot (FlightSession::start_station_reboost) and books the calculator's [E] attitude gas and transfer vent. The
// first plan is the calculator's answer the flight is compared with.

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "starsim/config/refuelling_mission_config.hpp"
#include "starsim/config/refuelling_reference.hpp"
#include "starsim/mission/refuelling_campaign.hpp"
#include "starsim/sim/flight_session.hpp"
#include "starsim/sim/landing_outcome.hpp"
#include "starsim/sim/mission_start.hpp"
#include "starsim/sim/refuelling_campaign_setup.hpp"
#include "starsim/time/time_scale_converter.hpp"

namespace starsim::sim {

enum class CampaignFlightPhase { DepotAscent, Planning, WaitLaunch, Flying, StationKeeping, Complete, Ended };
const char* to_string(CampaignFlightPhase phase);  // German

struct CampaignFlownRow {
    int number = 0;  // tankers 1..n in launch order (failed ones included); the HLS: 0
    bool hls = false;
    bool failed = false;
    std::string note;             // German: why it failed, a window moved past the plan
    time::Epoch launch, arrival;  // liftoff; Waypoint 0 (the Proximity mark; failed before it: the end)
    std::optional<mission::PhasingOrbit> phasing_orbit;
    double delivered_kg = 0.0;    // tanker: into the depot (before the vent); HLS: received (before the vent)
    double depot_after_kg = 0.0;  // the depot's main tanks after the transfer and the vent booking
    // the flight's own numbers from its leg marks (as reference_flights.toml measures them)
    double phasing_s = 0.0;           // separation (Raise mark, else Rendezvous) -> Waypoint 0
    double residue_kg = 0.0;          // chaser main tanks at the Rendezvous mark
    double rendezvous_kg = 0.0;       // Rendezvous -> Docked
    double main_at_undock_kg = 0.0;   // chaser main tanks at the Departure mark
    // diagnostic: the flown depot ahead (+) of the first plan's depot along its orbit when this flight was planned
    double depot_lead_m = 0.0;
    std::optional<mission::CampaignTanker> calculator;  // the first plan's k-th tanker for the k-th delivering one
};

struct CampaignFlightLosses {  // flown and booked, from the start to the end
    double boil_off_kg = 0.0;  // [D] the remainder of the depot's balance (boil-off and the docked pairs' effects)
    double station_keeping_kg = 0.0, attitude_gas_kg = 0.0, transfer_vent_kg = 0.0;
};

struct CampaignFlightState {
    CampaignFlightPhase phase = CampaignFlightPhase::DepotAscent;
    std::string status;                // German
    std::optional<time::Epoch> start;  // the depot became the station
    int tankers_flown = 0;             // delivered (not failed)
    int tankers_planned = 0;           // the latest live plan: flown + still planned
    double depot_propellant_kg = 0.0;
    std::optional<time::Epoch> next_launch;
    bool next_is_hls = false;
    std::vector<CampaignFlownRow> rows;
    CampaignFlightLosses losses;
    std::vector<ReboostRecord> reboosts;
    std::optional<CampaignReport> calculator;  // the first plan
    std::optional<time::Epoch> hls_arrival;
    double hls_propellant_kg = 0.0, hls_required_propellant_kg = 0.0;
    std::optional<ReturnResult> first_return;
    double first_return_main_kg = 0.0;
    std::string first_return_note;  // German: why it did not fly / its status
};

class RefuellingCampaignFlight : public SessionSupervisor {
public:
    RefuellingCampaignFlight(CampaignSources sources, config::RefuellingCampaignFlownMission mission);
    void update(FlightSession& session) override;
    std::optional<time::Epoch> wake_epoch() const override;
    const CampaignFlightState& state() const { return state_; }
    // A placed depot (the session's only body, the station) at `epoch`: planning starts at once.
    void start_placed(time::Epoch epoch);
    // The spec's pass check against the first plan: German lines of every miss (empty: passed) - tanker count,
    // duration, depot fill per arrival, the HLS's propellant.
    std::vector<std::string> comparison_misses() const;

private:
    void begin(FlightSession& s);
    void plan_next(FlightSession& s);
    void launch(FlightSession& s);
    void watch_flight(FlightSession& s);
    void fly_first_return(const FlightSession& s);
    void finish(FlightSession& s);
    void end(const std::string& reason);
    void book_attitude_gas(FlightSession& s, double dt_s);

    CampaignSources sources_;
    config::RefuellingCampaignFlownMission mission_;
    config::RefuellingReference reference_;  // reference_flights.toml (the missions)
    config::RefuellingFlightMission tanker_, hls_;
    CampaignFlightState state_;
    std::optional<CampaignHlsDemand> demand_;
    std::optional<mission::CampaignFlight> next_;
    mission::LaunchWindowParameters next_parameters_;
    std::optional<time::Epoch> last_launch_, last_arrival_, last_update_;
    std::optional<time::Epoch> planned_second_launch_;  // the previous plan's next-but-one launch (moved-window note)
    double pending_gas_kg_ = 0.0;
    double delivered_total_kg_ = 0.0, hls_received_kg_ = 0.0, initial_fill_kg_ = 0.0;
    double depot_vent_kg_ = 0.0;  // the vent booked on the depot (the HLS's is drawn from the HLS)
    std::string moved_note_;      // decision 2: the next row's note
    mission::StationStateAt first_depot_at_;  // the first plan's depot orbit (the lead diagnostic)
    double depot_lead_m_ = 0.0;               // at the latest plan
    bool placed_fill_set_ = false;
};

// start_mission's refuelling_campaign_flown: the depot launched (depot_launch) or placed, the driver installed. Needs
// a session with the Moon orientation (the HLS demand); MissionStart.refusal (German) otherwise.
MissionStart start_refuelling_campaign_flight(FlightSession& session, const std::filesystem::path& config_dir,
                                              const config::RefuellingCampaignFlownMission& mission,
                                              const time::TimeScaleConverter& time);

}  // namespace starsim::sim
