#include <algorithm>

#include "flight_snapshot.hpp"
#include "starsim/sim/mission_timeline.hpp"

namespace starsim::bindings {

namespace {

godot::Dictionary event_dictionary(const sim::MissionEvent& e) {
    godot::Dictionary d;
    d["kind"] = godot::String::utf8(sim::id_of(e.kind));
    d["label"] = godot::String::utf8(sim::to_string(e.kind));
    d["tdb_s"] = e.epoch.tdb_seconds_past_j2000();
    d["body_serial"] = static_cast<int64_t>(e.body_serial);
    d["body_name"] = godot::String::utf8(e.body_name.c_str());
    d["predicted"] = e.predicted;
    return d;
}

}  // namespace

godot::Dictionary build_mission_timeline(const sim::FlightSession& session, time::Epoch epoch) {
    const auto& timeline = session.timeline();
    std::vector<sim::MissionEvent> merged = timeline.recorded();
    merged.insert(merged.end(), timeline.predicted().begin(), timeline.predicted().end());
    std::stable_sort(merged.begin(), merged.end(),
                     [](const sim::MissionEvent& a, const sim::MissionEvent& b) { return a.epoch < b.epoch; });
    godot::Dictionary d;
    const auto t0 = timeline.t0();
    const auto clock = timeline.mission_clock_s(epoch);
    d["t0_tdb_s"] = t0 ? godot::Variant(t0->tdb_seconds_past_j2000()) : godot::Variant();
    d["clock_s"] = clock ? godot::Variant(*clock) : godot::Variant();
    godot::Array events;
    for (const auto& e : merged) events.push_back(event_dictionary(e));
    d["events"] = events;
    return d;
}

}  // namespace starsim::bindings
