#include "starsim/contact/probe_drogue_mechanism.hpp"

#include "starsim/contact/probe_drogue_interface.hpp"

namespace starsim::contact {

DockingMechanismParameters probe_drogue_mechanism(const config::DockingPortConfig& chaser,
                                                  const config::DockingPortConfig& target) {
    DockingMechanismParameters m;
    m.probe_drogue = probe_drogue_parameters(chaser, target);
    m.joint_rotation_frequency_radps = 0.5;  // [E] as the translation: time constant 2 s
    m.end_stop_stiffness_n_per_m = 2.0e7;    // [E] the probes' hard stop for a ~500 t chaser
    m.hook_count = 4;                        // [E] one structural latch per probe
    m.separation_energy_j = 150.0;           // [E] ~0.03 m/s for 450 t / 1,600 t
    m.dynamic_compression_limit_n = 1.0e5;   // [E] 25 kN per probe
    m.static_compression_limit_n = 6.0e4;    // [E]
    m.tension_limit_n = 6.0e4;               // [E]
    m.shear_limit_n = 5.0e4;                 // [E]
    m.bending_limit_nm = 2.0e6;              // [E] probes +-8 m apart
    m.torsion_limit_nm = 2.0e6;              // [E]
    m.limit_name = "Auslegungsgrenze";
    return m;
}

}  // namespace starsim::contact
