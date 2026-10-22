#pragma once

// Refuelling r6: the r5 reference flights' measured numbers, config/refuelling/reference_flights.toml - the campaign
// calculator's inputs.

#include <filesystem>
#include <string>

namespace starsim::config {

struct RefuellingReference {
    struct Depot {
        std::string mission;
        double initial_propellant_kg = 0.0;  // main tanks after the ascent
        double drag_decay_factor = 1.0;      // f2: the flown mean-orbit decay against the plain drag formula
    } depot;
    struct Tanker {
        std::string mission;
        double dry_mass_kg = 0.0;              // the tanker the numbers belong to
        double residue_after_ascent_kg = 0.0;  // main tanks at the Rendezvous mark
        double rendezvous_kg = 0.0;            // Rendezvous -> Docked: burns, RCS gas, boil-off
        double delivered_oxidizer_kg = 0.0, delivered_fuel_kg = 0.0;
        double chilldown_kg = 0.0, settling_gas_kg = 0.0;
        double main_at_undock_kg = 0.0;  // the reserve kept for the return
        double main_at_catch_kg = 0.0;
        double dry_mass_sensitivity = 0.0;  // residue gained per kg of dry mass removed
        // f2: its main tanks' boil-off in the free coast of the phasing flight (the flown heat balance; the campaign
        // books the phasing time's difference to the reference with it)
        double phasing_boil_off_kgps = 0.0;
        // f2: what the flown campaign's tankers delivered beyond the calculator's model of their own window, per
        // phasing orbit class (signed; the depot's mean radius splits the classes)
        double campaign_offset_below_kg = 0.0, campaign_offset_above_kg = 0.0;
        // f2: the depot's velocity gain from a transfer's settling push along the velocity (raises its orbit)
        double transfer_push_dv_mps = 0.0;
    } tanker;
    struct Hls {
        std::string mission;
        double main_at_docking_kg = 0.0;
        double phasing_boil_off_kgps = 0.0;  // f2: as the tanker's
    } hls;
};

// ConfigError (via detail::Document) for a missing file, block or number, or a negative mass.
RefuellingReference load_refuelling_reference(const std::filesystem::path& config_dir);

}  // namespace starsim::config
