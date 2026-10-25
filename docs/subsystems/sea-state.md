# Sea state and deck motion

History and the reasons behind superseded decisions: `docs/history/sea-state.md`.

Spec `docs/superpowers/archive/specs/2026-10-24-sea-state-deck-motion-design.md`. The drone ship's deck heaves, rolls and
pitches in a mission's sea, the onboard computer refuses a landing in a sea too rough, and the first stage lands on
and rides the moving deck. Without `[sea_state]` (calm) or with the fidelity flag `deck_motion` off everything is
exactly as before (the fixed level deck; test `"A fixed deck gives exactly today's contact"`).

## Model

- **Sea** (`environment::SeaState`): long-crested JONSWAP spectrum (Hasselmann et al. 1973; DNV-RP-C205 sec. 3.5.5
  normalisation, gamma 3.3) discretised into 48 linear (Airy) components, deep-water dispersion omega^2 = g k,
  equal-width bins over 0.5..3 omega_p (truncation < 1 %), amplitudes rescaled so the discrete variance is exactly
  Hs^2 / 16, phases from `std::mt19937_64(seed)` mapped by hand (portable). All waves travel towards the heading. The
  sea's time origin is the origin epoch given to `FlightSession::set_sea_state` rounded down to a whole TDB hour;
  `start_mission` passes the mission's requested epoch, so the same mission at the same epoch gets the same sea
  whatever the session ran before (the one-argument form takes the session epoch at the call).
- **Mission table**: `[sea_state]` `significant_wave_height_m`, `peak_period_s`, `heading_deg` (towards, from north),
  `seed` (`config::mission_sea_state`, `AscentMission::sea_state`); every `sim::start_mission` sets it on the session
  before the launch plans the recovery (none: calm, a previous mission's sea removed). Bad values are load errors
  naming the key.
- **Hull** (`config/drone_ships.toml`, derivations in its header): Marmac 300 class deck barge as a box, 91.4 x 30.5 m
  waterplane (the 52 m deck's wing extensions stand clear of the water), depth 6.0 m, freeboard 3.0 m -> draft 3.0 m
  [D]; radii of gyration roll 0.35 B, pitch 0.25 L [E]; added mass / inertia factors heave 2.5, roll 0.5, pitch 3.3 [E]
  (2D strip theory of a shallow rectangle, Lewis / Vugts, 3D end correction); damping ratios heave 0.15, roll 0.10
  (deliberately the low end: conservative for the refusal), pitch 0.15 [E]. Natural periods with KG at the waterline:
  **heave 6.50 s, roll 5.32 s, pitch 6.26 s** (`"Natural periods follow the hydrostatics"`).
- **Deck response** (`contact::DeckMotion`): per wave component the quasi-static least-squares plane fit of the wave
  over the waterplane rectangle (Froude-Krylov of a wall-sided box; analytic sinc factors of k L / 2, k B / 2) gives
  the heave, roll and pitch forcing; one damped mode each, driven by the strip-theory relative-motion excitation
  (Korvin-Kroukovsky): hydrostatic, added-mass and damping forces act on the motion relative to the wave, reduced by
  the Smith factor exp(-k d): H = exp(-k d) (1 - r^2 ca / (1 + ca) - 2 i zeta r) / (1 - r^2 - 2 i zeta r),
  r = omega / omega_n. Resonant amplification heave 1.04, roll 2.27, pitch 0.93. Long waves: the deck rides the
  surface; short waves average out. Non-goals: surge, sway, yaw, mode coupling. Validity: linear to ~6 deg of roll
  (the wings touch the water at ~6.6 deg, the hull's deck edge at ~11 deg); beyond, the prediction is an upper bound
  (safe for the refusal). Significant values (2 sigma; tilt = 2 sqrt(var roll + var pitch)) from the spectrum
  (`statistics`, checked against the time series):

  | Hs 2.5 m / Tp 8 s | heave | heave rate | tilt |
  |---|---|---|---|
  | head sea (waves along the deck) | 0.27 m | 0.20 m/s | 1.46 deg |
  | beam sea (waves across the deck) | 1.05 m | 0.84 m/s | 6.9 deg |

- **Landing limits and refusal**: `max_deck_tilt_deg` 3 deg and `max_heave_rate_mps` 1.0 m/s [E] (significant values,
  against the legs' gentle contact <= 3 m/s, <= 10 deg with the touchdown speed on top). `sim::sea_state_refusal`
  compares the deck's statistics with them; above -> "Seegang im Landegebiet zu hoch: Decksneigung 4.2° über der
  Grenze 3.0° (signifikante Werte bei Hs 6.0 m, Tp 10 s)." `plan_launch` checks the provisional ship before the
  drone-ship trials (a sea too rough for every heading costs no drone-ship trial) and the chosen ship again after the
  impact prediction with its deck heading. RTLS and tower catches ignore the sea; calm, none or `deck_motion` off: no
  check. Limits in Hs (linear: the statistics scale with Hs; the tilt binds first everywhere), printed by the hidden
  `"Sea-state limits of the drone ship in Hs"` (`[sea_state_limits]`):

  | Tp | waves along the deck (least tilt, the ship's heading) | waves across the deck |
  |---|---|---|
  | 8 s | **5.15 m** (heave rate: 12.7 m) | 1.09 m (heave rate: 3.0 m) |
  | 10 s | **4.24 m** (heave rate: 6.9 m) | 1.45 m (heave rate: 3.2 m) |

  A deck held beam-on would reach the limit at ~1.1-1.5 m (the barge's roll resonance at 5.3 s).
- **Ship heading rule**: in a moving sea with `deck_motion` on the ship holds station heading where its deck tilts
  least, `sim::least_deck_motion_heading_rad` (5 deg grid over 0..180 deg, a tie within 1e-9 rad keeps the ground
  track) - waves along the deck; calm, none or the flag off: along the stage's ground track. Example, Starlink-class
  station with Hs 2 m / Tp 8 s across the track (44.6 deg): beam-on 5.51 deg (refused), turned to 135 deg 1.17 deg.
  Landing guidance aims at the deck centre and does not use the heading; the sea beside the deck (`drone_ship_site`:
  `SeaConfig` direction heading + 90 deg) turns with it.
- **Moving-deck contact and hold** (`sim::LegLandingSim::set_deck_motion`): the leg contact is evaluated in the
  deck-attached frame (the site frame carried by the deck's heave, roll and pitch), so penalty forces and the
  touchdown judgement (<= 3 m/s, <= 10 deg) use the motion relative to the deck. A landed stage rides the deck
  kinematically (`FlightBody::move_landed_hold`: pose and motion fixed in the deck frame, set after every tick and
  coast), also through time warp. Changing the sea or the flag under a landed stage changes over without a jump
  (`change_deck_motion`: held where it is, then carried from its pose relative to the deck now).
- **Mean sea beside the deck** (`contact::DeckSeaField`): beside the deck rectangle the water stays at the mean sea
  level (site-frame plane z = -freeboard) and does not move with the deck; a stage that misses the deck splashes down
  there.
- **Game**: `FlightSim.landing_zone` adds the hull (`hull_length_m`, `hull_beam_m`, `hull_draft_m`), the deck pose
  (`deck_moving`, `deck_heave_m`, `deck_roll_deg`, `deck_pitch_deg` and rates; zero / level without a moving sea) with
  the tilted deck axes `deck_bow` / `deck_port` / `deck_normal` from `contact::deck_frame` (the leg contact's
  transform), and `sea_state` {`hs_m`, `tp_s`, `heading_deg`} for a moving sea. `game/world/drone_ship_view.gd` draws
  the ship (hull 91.4 x 30.5 m from 6 m below the deck top to the 1 m deck slab [E], deck 91 x 52 m, 2 m walls along
  the long sides [E: drawn only, not in the physics], matte grey) as a child of the Earth at the zone origin, posed
  every frame from the snapshot (drawn at the snapshot instant like the vehicles). The sea shading's wave set
  (`WaterWaves.set_sea_state`) follows a sea state: first octave at the peak wavelength g Tp^2 / (2 pi) (Tp 8 s:
  99.9 m), the others at the calm ratios (23 / 60, 9 / 60, 3.5 / 60), all towards the sea's heading, every octave's
  slope k_p Hs / 4 (resolved slope variance (k_p Hs / 2)^2 / 2 of the significant wave; Hs 2.5 m / Tp 8 s: 0.039, the
  calm set's 0.04), capped at 0.08 [E] (resolved variance below the glint's Cox-Munk); applied globally while the
  flight's landing zone has a sea state. Readout in the time panel ("Seegang Hs ... - Deck: Hub ..., Rollen ...,
  Stampfen ...", `docs/ansichten.md`).

## Standing decisions

- Linear (Airy) long-crested sea, no surge / sway / yaw / mode coupling; low roll damping 0.10 kept as the
  conservative end (Task 4 design).
- Landing limits 3 deg tilt / 1.0 m/s heave rate, significant values [E]; RTLS and tower catches ignore the sea.
- Ship heading: least deck tilt in a moving sea (controller ruling, 2026-10-24); no-questions run, rulings confirmed.
- Landing guidance aims at the mean deck height and deck centre; no terminal descent referenced to the measured deck
  height (the flown touchdown was gentle, not needed).

## Validation

- `"The drone-ship mission lands on a moving deck"` (`[slow][falcon9][asds][sea_state]`, started as
  `sim::start_mission` does; `config/missions/falcon9_asds_sea.toml`: `falcon9_asds` in Hs 2.5 m / Tp 8 s, waves
  towards 60 deg [E], seed 1): deck turned to **60.0 deg** (waves along the deck, stern to bow, no roll); first leg
  contact T+526.48 s, **gelandet**, 0.02 m from the deck centre, 5.0 t left. Just before contact: relative sink
  1.32 m/s (relative speed 1.32 m/s), tilt to the deck 0.36 deg; deck pose at contact heave -0.00 m rising at
  0.22 m/s, roll 0.00 deg, pitch -0.36 deg (rates 0.00 / 0.00 deg/s: at a pitch extreme). Riding the deck for 60 s:
  velocity error and CG height change below 1e-4 m/s / m.
- `"Natural periods follow the hydrostatics"` (6.50 / 5.32 / 6.26 s), `"A fixed deck gives exactly today's contact"`,
  `"Sea-state limits of the drone ship in Hs"` (hidden, `[sea_state_limits]`); `DeckMotion::statistics` checked
  against the time series.
- Render check `falcon9_asds_sea` 20 s after landing: the stage's CG 13.359 m above the drawn deck against 13.371 m
  above its feet (12 mm leg compression), 0.08 m from the deck centre (the legs themselves are not drawn by
  `vehicle_view.gd`).

## Open points and known limits

- VR check of the drone-ship view and sea open (per project notes).
- Prediction beyond ~6 deg roll is an upper bound only (linear model).
