#include "starsim/sim/station_start.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "starsim/math/constants.hpp"
#include "starsim/report/report.hpp"
#include "starsim/orbit/synthetic_station_orbit.hpp"
#include "starsim/time/utc_date_time.hpp"

namespace starsim::sim {

namespace {

constexpr double gm = constants::earth_gm_m3ps2;
constexpr double equatorial_radius_m = constants::wgs84_semi_major_axis_m;
constexpr int radius_samples_per_revolution = 32;

// Mean geocentric radius over [from, to], sampled from the OEM (inside its usable window).
double mean_radius_m(const orbit::Oem& oem, time::Epoch from, time::Epoch to) {
    double sum = 0.0;
    for (int k = 0; k < radius_samples_per_revolution; ++k) {
        const double f = (k + 0.5) / radius_samples_per_revolution;
        sum += norm(oem.state_gcrf(from + f * (to - from)).position_m);
    }
    return sum / radius_samples_per_revolution;
}

// +1 for a raising burn, -1 for a lowering one: the OEM's own mean radius one revolution after the TIG against one
// revolution before (the event table lists the magnitude only). Where the window does not reach a full revolution
// on both sides the sign cannot be read reliably (one sampled radius swings by several km, a 1 m/s burn moves a by
// ~1.8 km): nullopt, and the caller assumes a posigrade burn (reboosts are).
std::optional<double> impulse_sign(const orbit::Oem& oem, const orbit::OemEvent& event, double period_s) {
    const time::Epoch start = oem.header().usable_start, stop = oem.header().usable_stop;
    if (event.tig + period_s > stop || event.tig - period_s < start) return std::nullopt;
    return mean_radius_m(oem, event.tig, event.tig + period_s) >= mean_radius_m(oem, event.tig - period_s, event.tig)
               ? 1.0
               : -1.0;
}

// The OEM's CREATION_DATE (UTC text) as Unix seconds; NaN when it does not parse.
double oem_unix_seconds(const std::string& creation_date) {
    const auto utc = time::UtcDateTime::parse_iso(creation_date);
    return utc ? utc->unix_seconds() : std::numeric_limits<double>::quiet_NaN();
}

StationStart from_oem(const config::VehicleConfig& iss, const orbit::Oem& oem, time::Epoch epoch) {
    StationStart out;
    out.vehicle = iss;
    auto& stage = out.vehicle.stages.front();
    const auto& header = oem.header();
    if (header.mass_kg) stage.dry_mass_kg = *header.mass_kg;
    if (header.drag_area_m2) stage.aero.fixed_drag_area_m2 = *header.drag_area_m2;
    if (header.drag_coefficient) stage.aero.fixed_drag_coefficient = *header.drag_coefficient;
    out.state_gcrf = oem.state_gcrf(epoch);
    out.epoch = epoch;
    const double r = norm(out.state_gcrf.position_m);
    const double period_s = constants::two_pi * std::sqrt(r * r * r / gm);
    std::string assumed;
    for (const auto& event : oem.events()) {
        if (event.delta_v_mps > 0.0 && event.tig > epoch) {
            const auto sign = impulse_sign(oem, event, period_s);
            if (!sign) assumed += (assumed.empty() ? "" : ", ") + event.name;
            out.impulses.push_back({event.tig, sign.value_or(1.0) * event.delta_v_mps, event.name});
        }
    }
    std::sort(out.impulses.begin(), out.impulses.end(),
              [](const StationImpulse& a, const StationImpulse& b) { return a.epoch < b.epoch; });
    out.from_oem = true;
    // the event names are data from the OEM file, shown as written
    const report::Param created = report::utc("date", oem_unix_seconds(header.creation_date));
    out.note = assumed.empty() ? report::make(report::Code::station_oem_source, {created})
                               : report::make(report::Code::station_oem_source_assumed,
                                              {created, report::data_name("maneuvers", assumed)});
    return out;
}

StationStart synthetic(const config::VehicleConfig& iss, const orbit::OemArchive& archive,
                       const time::TimeScaleConverter& time, const frames::BodyOrientation& earth, time::Epoch epoch,
                       std::optional<double> argument_of_latitude_rad) {
    orbit::StationMeanElements mean;
    std::optional<std::string> oem_date;  // the OEM the orbital plane comes from; none: the configuration
    if (const orbit::Oem* latest = archive.latest()) {
        mean = orbit::mean_elements_from_oem(*latest, gm, earth);
        oem_date = latest->header().creation_date;
    } else {
        const auto& reference = iss.stages.front().reference_orbit;
        if (!reference) throw std::runtime_error("The ISS configuration has no reference orbit ([stage.station.reference_orbit]).");
        const auto utc = time::UtcDateTime::parse_iso(reference->reference_utc);
        if (!utc) throw std::runtime_error("Invalid reference_utc of the ISS reference orbit: " + reference->reference_utc);
        mean.epoch = time.from_utc(*utc);
        mean.semi_major_axis_m = equatorial_radius_m + reference->altitude_m;
        mean.inclination_rad = reference->inclination_rad;
        mean.raan_rad = reference->raan_rad;
    }
    const auto advanced = orbit::advance_mean_elements(mean, epoch, gm, constants::earth_j2, equatorial_radius_m,
                                                       argument_of_latitude_rad);
    StationStart out;
    out.vehicle = iss;
    out.state_gcrf = orbit::synthetic_station_state(advanced, gm, earth);
    out.epoch = epoch;
    out.note = oem_date ? report::make(report::Code::station_synthetic_from_oem, {report::utc("date", oem_unix_seconds(*oem_date))})
                        : report::make(report::Code::station_synthetic_from_config);
    return out;
}

}  // namespace

IssOrbitSource parse_iss_orbit_source(std::string_view text) {
    if (text == "oem") return IssOrbitSource::Oem;
    if (text == "synthetic") return IssOrbitSource::Synthetic;
    if (text == "auto") return IssOrbitSource::Auto;
    throw std::invalid_argument("iss_orbit must be \"oem\", \"synthetic\" or \"auto\", not \"" + std::string(text) + "\"");
}

StationStart iss_start(const config::VehicleConfig& iss, const orbit::OemArchive& archive,
                       const time::TimeScaleConverter& time, const frames::BodyOrientation& earth, time::Epoch epoch,
                       IssOrbitSource source, std::optional<double> argument_of_latitude_rad) {
    const orbit::Oem* covering = archive.for_epoch(epoch);
    switch (source) {
        case IssOrbitSource::Oem:
            if (!covering) {
                // The player is told through the code; the file hint lives in the table text.
                throw report::Refusal(report::make(report::Code::data_iss_orbit_missing));
            }
            return from_oem(iss, *covering, epoch);
        case IssOrbitSource::Synthetic:
            return synthetic(iss, archive, time, earth, epoch, argument_of_latitude_rad);
        case IssOrbitSource::Auto:
            break;
    }
    if (covering) return from_oem(iss, *covering, epoch);
    auto out = synthetic(iss, archive, time, earth, epoch, argument_of_latitude_rad);
    out.note = report::with_cause(report::make(report::Code::station_no_oem), std::move(out.note));
    return out;
}

}  // namespace starsim::sim
