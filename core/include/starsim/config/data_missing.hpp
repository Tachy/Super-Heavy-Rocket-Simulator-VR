#pragma once

#include <string>

#include "starsim/config/vehicle_config.hpp"
#include "starsim/report/report.hpp"

namespace starsim::config {

// A required downloaded data file is absent. Still a ConfigError (existing catchers keep working); report() is what
// the player is told. The file path stays in what() for the developer log.
class DataMissing : public ConfigError {
public:
    DataMissing(report::Report report, const std::string& developer_message)
        : ConfigError(developer_message), report_(std::move(report)) {}
    const report::Report& report() const noexcept { return report_; }

private:
    report::Report report_;
};

}  // namespace starsim::config
