#pragma once

// Refuelling f2 (user 2026-10-22): the depot's mean orbit through a campaign. The flown depot is no fixed orbit: drag
// lowers its mean semi-major axis (0.3 km/day at 198 km and 572 t) and every transfer's settling push along the
// velocity raises it (~0.55 m/s: ~0.9 km), and a mean motion off by 1 km moves the phase ~150 km per day - the first
// plan of the flown campaign missed the HLS window by 4 days with a fixed orbit. The track keeps the mean elements in
// segments from one change to the next: a decays linearly in each, the argument of latitude follows the changing
// mean motion; the station keeping reboosts it from a floor to a target (also in the future the window search sees).

#include <optional>
#include <vector>

#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Station keeping: when the decaying mean semi-major axis reaches the floor from above, a reboost [E: instantaneous]
// lifts it to the target (a segment starting below the floor decays on to its next change).
struct DepotKeeping {
    double floor_semi_major_axis_m = 0.0;
    double target_semi_major_axis_m = 0.0;
};

class DepotOrbitTrack {
public:
    // `decay_mps`: da/dt of the first segment (<= 0 under drag). `keeping`: none - the orbit decays without a floor.
    DepotOrbitTrack(const orbit::StationMeanElements& start, double decay_mps, double gm_m3ps2, double j2,
                    double radius_m, std::optional<DepotKeeping> keeping = {});

    // The mean elements at `epoch` (before the start: the first segment extrapolated): a = a0 + da/dt t, back to the
    // keeping target at each floor crossing; the argument of latitude at each piece's J2 rate plus the integral of
    // the mean motion's change, -3/2 n / a da/dt t^2 / 2 [E: linear in a, (da / a)^2 dropped]; RAAN and perigee at
    // the segment start's J2 rates.
    orbit::StationMeanElements at(time::Epoch epoch) const;

    // The keeping reboosts in (from, to].
    std::vector<time::Epoch> reboosts(time::Epoch from, time::Epoch to) const;

    // From `epoch` on (not before the last change): a jumps by `delta_a_m` (impulses: the phase continuous) and decays
    // at `decay_mps`. Throws std::invalid_argument for an epoch before the last change.
    void change(time::Epoch epoch, double delta_a_m, double decay_mps);

    time::Epoch last_change() const { return segments_.back().start.epoch; }

private:
    struct Segment {
        orbit::StationMeanElements start;
        double decay_mps = 0.0;
    };
    const Segment& segment_at(time::Epoch epoch) const;
    // The segment advanced to `epoch` through its floor crossings; `reboost_epochs` collects them.
    orbit::StationMeanElements advance(const Segment& s, time::Epoch epoch,
                                       std::vector<time::Epoch>* reboost_epochs) const;

    std::vector<Segment> segments_;
    double gm_m3ps2_, j2_, radius_m_;
    std::optional<DepotKeeping> keeping_;
};

// The change of the semi-major axis by a small along-track impulse on a near-circular orbit: 2 a dv / v (vis-viva,
// linearised).
double along_track_delta_a_m(double semi_major_axis_m, double delta_v_mps, double gm_m3ps2);

// da/dt of a body under drag on a near-circular orbit: -rho Cd A sqrt(gm a) / m (the energy lost at the orbit speed).
double drag_decay_mps(double density_kg_m3, double drag_area_m2, double mass_kg, double semi_major_axis_m,
                      double gm_m3ps2);

}  // namespace starsim::mission
