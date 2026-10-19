#include "starsim/orbit/oem.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <regex>
#include <sstream>

#include "starsim/time/utc_date_time.hpp"

namespace starsim::orbit {

using math::StateVector;
using math::Vec3;

namespace {

constexpr double metres_per_km = 1000.0;
constexpr double step_tolerance_s = 1e-3;  // two steps are "equal" within this

std::string trim(const std::string& s) {
    const auto first = s.find_first_not_of(" \t");
    if (first == std::string::npos) return {};
    const auto last = s.find_last_not_of(" \t");
    return s.substr(first, last - first + 1);
}

double parse_number(const std::string& text, const std::string& what) {
    try {
        size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size()) throw std::invalid_argument(text);
        return value;
    } catch (const std::exception&) {
        throw OemError("OEM: bad number for " + what + ": '" + text + "'");
    }
}

time::UtcDateTime parse_utc(const std::string& text, const std::string& what) {
    const auto utc = time::UtcDateTime::parse_iso(text);
    if (!utc) throw OemError("OEM: bad time for " + what + ": '" + text + "'");
    return *utc;
}

bool is_leap_year(int year) { return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0; }

int day_of_year(const time::UtcDateTime& utc) {
    static constexpr int cumulative[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    return cumulative[utc.month - 1] + utc.day + (utc.month > 2 && is_leap_year(utc.year) ? 1 : 0);
}

time::UtcDateTime from_day_of_year(int year, int doy, int hour, int minute, double second) {
    static constexpr int month_days[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    int month = 0;
    for (; month < 12; ++month) {
        const int days = month_days[month] + (month == 1 && is_leap_year(year) ? 1 : 0);
        if (doy <= days) break;
        doy -= days;
    }
    if (month == 12 || doy < 1) throw OemError("OEM: bad day of year in the event table");
    return {year, month + 1, doy, hour, minute, second};
}

// "KEY = VALUE" -> (KEY, VALUE); empty key if the line has no '='.
std::pair<std::string, std::string> split_keyword(const std::string& line) {
    const auto eq = line.find('=');
    if (eq == std::string::npos) return {};
    return {trim(line.substr(0, eq)), trim(line.substr(eq + 1))};
}

// Lagrange interpolation of the rows [first, first + count) at t (seconds relative to the first row).
StateVector lagrange(const std::vector<time::Epoch>& epochs, const std::vector<StateVector>& states, size_t first,
                     size_t count, time::Epoch epoch) {
    const time::Epoch origin = epochs[first];
    const double t = epoch - origin;
    StateVector out{};
    for (size_t j = 0; j < count; ++j) {
        const double tj = epochs[first + j] - origin;
        double weight = 1.0;
        for (size_t m = 0; m < count; ++m) {
            if (m == j) continue;
            const double tm = epochs[first + m] - origin;
            weight *= (t - tm) / (tj - tm);
        }
        out.position_m = out.position_m + states[first + j].position_m * weight;
        out.velocity_mps = out.velocity_mps + states[first + j].velocity_mps * weight;
    }
    return out;
}

}  // namespace

Oem Oem::parse(std::istream& in, const time::TimeScaleConverter& time, const std::string& name) {
    static const std::regex event_row(
        R"(^COMMENT\s+(.*?)\s+(\d{3}):(\d{2}):(\d{2}):(\d{2}(?:\.\d+)?)(?:\s+\d+)?\s+(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*$)");
    struct RawEvent {
        std::string name;
        int doy, hour, minute;
        double second, dv_mps, ha_km, hp_km;
    };

    Oem oem;
    std::vector<RawEvent> raw_events;
    std::optional<time::UtcDateTime> start_utc, stop_utc;
    bool in_meta = false, in_events = false, saw_meta = false;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        line = trim(line);
        if (line.empty()) continue;
        if (line == "META_START") {
            in_meta = saw_meta = true;
            continue;
        }
        if (line == "META_STOP") {
            in_meta = false;
            continue;
        }
        if (line.rfind("COMMENT", 0) == 0) {
            if (line.find("TRAJECTORY EVENT SUMMARY:") != std::string::npos) {
                in_events = true;
            } else if (line.find("End sequence of events") != std::string::npos) {
                in_events = false;
            } else if (in_events) {
                if (line.find_first_of("(|=") != std::string::npos) continue;
                std::smatch m;
                if (std::regex_match(line, m, event_row)) {
                    raw_events.push_back({m[1].str(), std::stoi(m[2].str()), std::stoi(m[3].str()),
                                          std::stoi(m[4].str()), parse_number(m[5].str(), "event TIG"),
                                          parse_number(m[6].str(), "event DV"), parse_number(m[7].str(), "event HA"),
                                          parse_number(m[8].str(), "event HP")});
                }
            } else {
                // "COMMENT MASS=460320.00": vehicle properties without spaces around '='
                const auto [key, value] = split_keyword(trim(line.substr(7)));
                std::optional<double>* target = key == "MASS"              ? &oem.header_.mass_kg
                                                : key == "DRAG_AREA"       ? &oem.header_.drag_area_m2
                                                : key == "DRAG_COEFF"      ? &oem.header_.drag_coefficient
                                                : key == "SOLAR_RAD_AREA"  ? &oem.header_.solar_radiation_area_m2
                                                : key == "SOLAR_RAD_COEFF" ? &oem.header_.solar_radiation_coefficient
                                                                           : nullptr;
                if (target) *target = parse_number(value, key);
            }
            continue;
        }
        const auto [key, value] = split_keyword(line);
        if (!key.empty()) {
            if (key == "CREATION_DATE") {
                oem.header_.creation_date = value;
            } else if (key == "OBJECT_NAME") {
                oem.header_.object_name = value;
            } else if (key == "USEABLE_START_TIME") {
                start_utc = parse_utc(value, key);
            } else if (key == "USEABLE_STOP_TIME") {
                stop_utc = parse_utc(value, key);
            } else if (key == "REF_FRAME" && value != "EME2000") {
                throw OemError(name + ": unsupported REF_FRAME " + value + " (need EME2000)");
            } else if (key == "TIME_SYSTEM" && value != "UTC") {
                throw OemError(name + ": unsupported TIME_SYSTEM " + value + " (need UTC)");
            } else if (key == "CENTER_NAME" && value != "Earth") {
                throw OemError(name + ": unsupported CENTER_NAME " + value + " (need Earth)");
            }
            continue;
        }
        if (in_meta) continue;
        // data row: epoch x y z vx vy vz (km, km/s)
        std::istringstream fields(line);
        std::string stamp;
        double v[6];
        fields >> stamp;
        for (double& x : v) {
            if (!(fields >> x)) throw OemError(name + ": malformed data row '" + line + "'");
        }
        std::string extra;
        if (fields >> extra) throw OemError(name + ": malformed data row '" + line + "'");
        const time::Epoch epoch = time.from_utc(parse_utc(stamp, "data row"));
        if (!oem.epochs_.empty() && !(epoch > oem.epochs_.back())) {
            throw OemError(name + ": data rows not strictly increasing in time at " + stamp);
        }
        oem.epochs_.push_back(epoch);
        oem.states_.push_back({Vec3{v[0], v[1], v[2]} * metres_per_km, Vec3{v[3], v[4], v[5]} * metres_per_km});
    }
    if (!saw_meta || !start_utc || !stop_utc) throw OemError(name + ": missing META block or usable window");
    oem.header_.usable_start = time.from_utc(*start_utc);
    oem.header_.usable_stop = time.from_utc(*stop_utc);

    // Event TIGs are "day-of-year:hh:mm:ss" in the year of the usable start; a smaller day is in the next year.
    const int start_doy = day_of_year(*start_utc);
    for (const auto& e : raw_events) {
        const int year = start_utc->year + (e.doy < start_doy ? 1 : 0);
        oem.events_.push_back({trim(e.name), time.from_utc(from_day_of_year(year, e.doy, e.hour, e.minute, e.second)),
                               e.dv_mps, e.ha_km * metres_per_km, e.hp_km * metres_per_km});
    }
    oem.find_segments();
    return oem;
}

Oem Oem::load(const std::filesystem::path& file, const time::TimeScaleConverter& time) {
    std::ifstream in(file, std::ios::binary);
    if (!in) throw OemError("OEM file not found: " + file.string());
    return parse(in, time, file.filename().string());
}

Oem Oem::filtered(const std::function<bool(size_t)>& keep_row) const {
    Oem out;
    out.header_ = header_;
    out.events_ = events_;
    for (size_t i = 0; i < epochs_.size(); ++i) {
        if (!keep_row(i)) continue;
        out.epochs_.push_back(epochs_[i]);
        out.states_.push_back(states_[i]);
    }
    if (!out.epochs_.empty()) {
        out.header_.usable_start = std::max(out.header_.usable_start, out.epochs_.front());
        out.header_.usable_stop = std::min(out.header_.usable_stop, out.epochs_.back());
    }
    out.find_segments();
    return out;
}

// A segment starts at row k when the grid restarts there: the step after k differs from the step before it and
// the new step holds for the next step too. A single odd step (240, 90, 240 s) thus ends the old segment at the
// seam row k, which also starts the new one; the step change at the row before k is not a seam.
void Oem::find_segments() {
    segment_start_.assign(1, 0);
    const size_t n = epochs_.size();
    for (size_t k = 1; k + 2 < n; ++k) {
        const double before = epochs_[k] - epochs_[k - 1];
        const double after = epochs_[k + 1] - epochs_[k];
        const double next = epochs_[k + 2] - epochs_[k + 1];
        if (std::abs(after - before) > step_tolerance_s && std::abs(next - after) <= step_tolerance_s) {
            segment_start_.push_back(k);
        }
    }
}

bool Oem::covers(time::Epoch epoch) const {
    return !epochs_.empty() && epoch >= header_.usable_start && epoch <= header_.usable_stop &&
           epoch >= epochs_.front() && epoch <= epochs_.back();
}

StateVector Oem::state_gcrf(time::Epoch epoch) const {
    if (!covers(epoch)) throw OemError("OEM: epoch outside the usable window");
    // segment: the last start at or before the epoch; it ends at (and includes) the next start
    const auto seg = std::upper_bound(segment_start_.begin(), segment_start_.end(), size_t{0},
                                      [&](size_t, size_t start) { return epoch < epochs_[start]; }) -
                     1;
    const size_t first_row = *seg;
    const size_t last_row = (seg + 1 == segment_start_.end()) ? epochs_.size() - 1 : *(seg + 1);
    const size_t rows = last_row - first_row + 1;
    const size_t count = std::min(rows, static_cast<size_t>(interpolation_degree + 1));
    // the row at or before the epoch, then centre the window on it and shift it into the segment
    const size_t at = static_cast<size_t>(
        std::upper_bound(epochs_.begin() + first_row, epochs_.begin() + last_row + 1, epoch) - epochs_.begin() - 1);
    const size_t half = (count - 1) / 2;
    size_t start = at >= first_row + half ? at - half : first_row;
    start = std::min(start, last_row + 1 - count);
    return lagrange(epochs_, states_, start, count, epoch);
}

// ISO 8601 creation dates sort chronologically as strings.
OemArchive OemArchive::scan(const std::filesystem::path& dir, const time::TimeScaleConverter& time) {
    OemArchive archive;
    std::error_code error;
    if (!std::filesystem::is_directory(dir, error)) return archive;
    for (const auto& entry : std::filesystem::directory_iterator(dir, error)) {
        if (!entry.is_regular_file() || entry.path().extension() != ".txt") continue;
        try {
            archive.files_.push_back(Oem::load(entry.path(), time));
        } catch (const std::exception& e) {
            archive.skipped_.push_back(entry.path().filename().string() + ": " + e.what());
        }
    }
    std::sort(archive.files_.begin(), archive.files_.end(), [](const Oem& a, const Oem& b) {
        return a.header().creation_date < b.header().creation_date;
    });
    return archive;
}

const Oem* OemArchive::for_epoch(time::Epoch epoch) const {
    for (auto it = files_.rbegin(); it != files_.rend(); ++it) {
        if (it->covers(epoch)) return &*it;
    }
    return nullptr;
}

const Oem* OemArchive::latest() const { return files_.empty() ? nullptr : &files_.back(); }

}  // namespace starsim::orbit
