#pragma once

#include "starsim/config/vehicle_config.hpp"
#include "starsim/contact/docking_mechanism.hpp"

namespace starsim::contact {

// Mechanism parameters of the refuelling ships' probe/drogue docking (refuelling r3), all [E]: sized for a ~500 t
// tanker against a depot up to ~1,600 t. A 0.10 m/s contact peaks at the joint damper's m_c * omega * v (~49 kN for
// 490 t), below the 100 kN dynamic limit; 0.25 m/s overloads.
DockingMechanismParameters probe_drogue_mechanism(const config::DockingPortConfig& chaser,
                                                  const config::DockingPortConfig& target);

}  // namespace starsim::contact
