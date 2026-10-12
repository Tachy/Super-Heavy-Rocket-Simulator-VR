#pragma once

// Internal helper: converts CSPICE error state into a C++ exception.
// Only included by core/src/spice/*.cpp.

#include <string>

#include "SpiceUsr.h"
#include "starsim/spice/spice_session.hpp"

namespace starsim::spice::detail {

inline void throw_if_failed(const char* context) {
    if (!failed_c()) {
        return;
    }
    SpiceChar message[1841];
    getmsg_c("LONG", sizeof(message), message);
    reset_c();
    throw SpiceError(std::string(context) + ": " + message);
}

}  // namespace starsim::spice::detail
