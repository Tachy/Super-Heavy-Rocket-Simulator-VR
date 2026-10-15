#pragma once

#include <vector>

#include "starsim/environment/gravity.hpp"
#include "starsim/ephemeris/ephemeris.hpp"
#include "starsim/math/state_vector.hpp"
#include "starsim/orbit/cr3bp.hpp"
#include "starsim/time/epoch.hpp"

namespace starsim::mission {

// Seed of a resonant near-rectilinear halo orbit from the Earth-Moon CR3BP.
struct NrhoSeed {
    orbit::Cr3bpState apolune;  // CR3BP state at apolune (nondimensional) [S]
    int revolutions = 9;        // resonance: revolutions ...
    int synodic_months = 2;     // ... per synodic months [S]
};

struct NrhoReferenceOptions {
    int revolutions = 6;                     // span of the reference
    double continuity_tolerance_m = 1.0;     // position jump allowed at a patch point
    double continuity_tolerance_mps = 1e-4;  // velocity jump allowed at a patch point
    int max_iterations = 20;                 // Newton iterations per revolution added
};

// NRHO reference trajectory in the full force model (the gravity model's bodies, DE440 positions).
// The reference's phase is chosen by `first_perilune` alone: there is no Gateway phasing. A real Gateway ephemeris
// would replace the seed when rendezvous comes with M9. The gravity model must outlive the reference.
class NrhoReference {
public:
    // Builds the reference whose first perilune falls at `first_perilune`: CR3BP periodic orbit (fixed resonant
    // period) whose perilune state, converted to GCRF, anchors every revolution; patch points (four per half
    // revolution, crowded towards the perilunes) seeded by full-model coasts from the perilunes; minimum-norm Newton
    // on the patch states (epochs fixed), continued revolution by revolution. Later perilune epochs are the closest
    // approaches found on the way (the free orbit's passages drift by hours from the resonant grid). Throws
    // std::runtime_error (German) when the CR3BP correction or the shooting does not converge.
    NrhoReference(const environment::GravityModel& earth_centred_gravity, const ephemeris::Ephemeris& ephemeris,
                  const NrhoSeed& seed, time::Epoch first_perilune, NrhoReferenceOptions options = {});

    bool covers(time::Epoch epoch) const;                   // within [first patch, last patch]
    math::StateVector state_gcrf(time::Epoch epoch) const;  // Earth-centred; throws std::out_of_range outside
    const std::vector<time::Epoch>& perilune_epochs() const { return perilune_epochs_; }
    // Apolunes after the first perilune: index i is the apolune following perilune i.
    const std::vector<time::Epoch>& apolune_epochs() const { return apolune_epochs_; }
    double max_continuity_error_m() const { return max_continuity_error_m_; }
    int iterations() const { return iterations_; }

private:
    const environment::GravityModel& gravity_;
    std::vector<time::Epoch> patch_epochs_;
    std::vector<math::StateVector> patch_states_;
    std::vector<time::Epoch> perilune_epochs_;
    std::vector<time::Epoch> apolune_epochs_;
    double max_continuity_error_m_ = 0.0;
    int iterations_ = 0;
};

}  // namespace starsim::mission
