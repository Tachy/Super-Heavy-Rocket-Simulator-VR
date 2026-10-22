#pragma once

#include "starsim/gnc/entry_predictor.hpp"
#include "starsim/gnc/landing_site.hpp"

namespace starsim::gnc {

// Where a given-up ship aims (crew escape): from the pad along the sea direction, `margin_m` beyond the shoreline, on
// the site's ground height, no nose direction. Throws std::invalid_argument for a site without a sea side.
EntryTarget sea_give_up_target(const LandingSite& site, double margin_m);

}  // namespace starsim::gnc
