#pragma once
// Refuelling r5: the measured numbers of a refuelling flight, read from the mission's leg marks.
#include <algorithm>
#include <cstdio>

#include "starsim/sim/flight_session.hpp"

namespace starsim::refuelling_test {

struct TankerFlightNumbers {
    double residue_after_ascent_kg = 0.0;  // main tanks at the Rendezvous mark (pad start only)
    double rendezvous_kg = 0.0;            // main tanks Rendezvous mark -> Docked mark (burns, RCS gas, boil-off)
    double delivered_oxidizer_kg = 0.0, delivered_fuel_kg = 0.0;
    double transfer_s = 0.0, settling_gas_kg = 0.0, chilldown_kg = 0.0;
    double main_at_undock_kg = 0.0;  // Departure mark
    double main_at_catch_kg = 0.0;   // Complete mark: what the reserve left over
    double flight_s = 0.0;           // first mark -> Complete
};

inline const sim::FlightSession::MissionMark* mark(const sim::FlightSession& s, sim::IssMissionLeg leg) {
    const auto& marks = s.iss_mission_marks();
    const auto it = std::find_if(marks.begin(), marks.end(), [leg](const auto& m) { return m.leg == leg; });
    return it == marks.end() ? nullptr : &*it;
}

inline TankerFlightNumbers tanker_numbers(const sim::FlightSession& s) {
    TankerFlightNumbers n;
    const auto* rv = mark(s, sim::IssMissionLeg::Rendezvous);
    const auto* docked = mark(s, sim::IssMissionLeg::Docked);
    const auto* transfer = mark(s, sim::IssMissionLeg::Transfer);
    const auto* departure = mark(s, sim::IssMissionLeg::Departure);
    const auto* complete = mark(s, sim::IssMissionLeg::Complete);
    if (rv) n.residue_after_ascent_kg = rv->chaser_main_propellant_kg;
    if (rv && docked) n.rendezvous_kg = rv->chaser_main_propellant_kg - docked->chaser_main_propellant_kg;
    const auto& t = s.transfer_state();
    n.delivered_oxidizer_kg = t.delivered_kg[0];
    n.delivered_fuel_kg = t.delivered_kg[1];
    n.settling_gas_kg = t.settling_gas_kg;
    n.chilldown_kg = t.chilldown_kg[0] + t.chilldown_kg[1];
    if (transfer && departure) n.transfer_s = departure->epoch - transfer->epoch;
    if (departure) n.main_at_undock_kg = departure->chaser_main_propellant_kg;
    if (complete) n.main_at_catch_kg = complete->chaser_main_propellant_kg;
    if (complete && !s.iss_mission_marks().empty()) n.flight_s = complete->epoch - s.iss_mission_marks().front().epoch;
    return n;
}

inline void print(const char* tag, const TankerFlightNumbers& n) {
    std::printf("[r5] %s: residue %.1f t, rendezvous %.2f t, delivered LOX %.1f t / CH4 %.1f t in %.0f s, gas %.2f t, "
                "chill %.2f t, main at undock %.2f t, at catch %.2f t, flight %.1f h\n",
                tag, n.residue_after_ascent_kg / 1e3, n.rendezvous_kg / 1e3, n.delivered_oxidizer_kg / 1e3,
                n.delivered_fuel_kg / 1e3, n.transfer_s, n.settling_gas_kg / 1e3, n.chilldown_kg / 1e3,
                n.main_at_undock_kg / 1e3, n.main_at_catch_kg / 1e3, n.flight_s / 3600.0);
}

}  // namespace starsim::refuelling_test
