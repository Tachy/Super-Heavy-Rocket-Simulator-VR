# Lunar missions: HLS landing and ship return from lunar orbit - design

Status: draft 2026-10-14, for the user's review. Milestone (l), after the booster MECO milestone (main `82b31eb`).

## Intent

The user's request (2026-10-14, German): a Starship in a 200 x 200 km Earth orbit flies to the Moon, enters lunar orbit,
lands at a chosen site, launches back to lunar orbit, returns to Earth, slows down in the atmosphere ("wie Artemis 1, von
40.000 auf 26.000 km/h") and finally returns to Starbase; orbits are nodes from which the next leg is planned.

Decisions taken with the user in the brainstorm (2026-10-14; memory `project-lunar-mission`):
- **Realistic simulation only.** No propellant is ever set during a mission. A mission may start from a state that
  real operations would have produced before it (a ship refuelled by tankers); real tanker flights, rendezvous and
  propellant transfer stay in M9. Refuelling in lunar orbit (depot) or in a high elliptical Earth orbit is noted for
  later.
- **A real HLS.** SpaceX does not plan a Starship return from the Moon to Earth: HLS (no heat shield, no flaps, legs)
  stays in lunar space and Orion carries crews home. The lunar lander is therefore an HLS; the Earth return is flown by
  a normal Block 3 ship.
- **Two missions first** (the mission planner chains them later, M6):
  1. **Lunar landing:** an HLS, fully fuelled in a 200 x 200 km Earth orbit (after the LEO depot, as in SpaceX's
     plan: depot + >10 tankers, then the HLS docks, fills up and flies on - NASA OIG report IG-26-004, March 2026 [S]),
     flies to lunar orbit, lands at a chosen site, launches and returns to lunar orbit. The mission ends there.
  2. **Lunar return:** a Block 3 ship in lunar orbit, tanked with just the propellant for its return, flies to
     Starbase and is caught - either by a **direct** (skip) entry or by an **aerocapture ("Aircatch")** into an Earth
     orbit first, then the existing ship return from orbit (b4).
- **Both lunar orbits:** a 100 km polar low lunar orbit (LLO) and the Artemis near-rectilinear halo orbit (NRHO), as a
  mission parameter.
- **HLS landing propulsion:** the Raptors brake; landing engines high on the hull [E] fly the last part and the liftoff
  (SpaceX's concept, against plume-surface interaction).
- **Lunar ground:** a local LOLA elevation tile at the landing site (physics only; rendering stays M8).
- **HLS dry mass 120 t [E]:** SpaceX publishes no HLS dry mass. ~100-120 t is Musk's older target for the ship, and
  SpaceX's LEO-departure plan only closes at <= ~140 t (9.3 km/s via the NRHO from 1,600 t). An independent bottom-up
  estimate gives ~204 t, and our Block 3 ship is 255 t [D]. The figure is an estimate with this provenance; a test shows
  where the missions stop closing.
- **Architecture:** vehicle states stay Earth-centred GCRF. A body-neutral `ReferenceBody` interface (Earth, Moon;
  a planet later would be configuration) supplies everything that is relative to a body. Integration is not switched
  at a sphere of influence (option "B" of the brainstorm). In double, the GCRF resolution at the Moon is ~1e-7 m (at
  Mars ~1e-4 m), and the NRHO's apolune (~70,000 km) lies about the Moon's sphere of influence (~66,000 km), which a
  frame switch would cross every revolution.
- Drawing and UI wait for the user's UI specification; the game interface (bindings) is built ahead.

Success:
- Mission 1: from 200 x 200 km, the HLS reaches a 100 km polar LLO (< 1 km off after the corrections - the master
  plan's M6 criterion) or the NRHO, lands on LOLA terrain at the chosen site (< 1 m off, <= 1 m/s), launches and
  reaches the LLO / NRHO again. With 204 t dry the mission planning refuses the mission (not enough delta-v).
- Mission 2: from LLO or NRHO, the ship is caught at Starbase after a direct entry, and after an aerocapture it ends in
  an Earth orbit, raises its perigee and is caught by b4. Heat-shield and load limits are judged honestly with the
  existing limits (no limits raised to make it pass).

## Decomposition

| Part | Content | Validation |
|---|---|---|
| l1 | `ReferenceBody` (Earth, Moon): GM, shape, body-fixed frame, atmosphere or none, ground; `FlightBody` / `FlightSession`: ground contact, resting, altitude, orbit elements, LVLH and telemetry relative to the nearest reference body | A body rests on the Moon and follows it; LLO and NRHO propagated Earth-centred for a day agree with a Moon-centred reference |
| l2 | Vehicle `starship_hls`; LOLA tile at the landing site (data pipeline, licence); height-field landing surface under each foot | Drop tests on 0/5/10/15 deg terrain; height and normal against the raw data |
| l3 | Outbound autopilot: TLI, mid-course corrections, LOI to a 100 km polar LLO; NRHO reference by multiple shooting in the full model; NRHO insertion; NRHO <-> LLO transfers | LLO < 1 km; NRHO period and perilune / apolune radii vs the literature; three revolutions near the reference; delta-v vs the literature |
| l4 | Descent (descent orbit, Apollo braking guidance to a high gate, SCvx approach on Raptors then landing engines, touchdown on legs), surface stay, ascent (landing engines then Raptors, PEG), abort to orbit; mission 1 complete | Delta-v vs the literature; dispersed landing campaign 21/21; ascent to LLO < 1 km; aborts from 3 / 1 / 0.3 km reach orbit |
| l5 | Mission 2: return propellant planning, TEI targeting to the entry interface, skip entry to Starbase, aerocapture guidance + perigee raise + b4; radiative heating | TEI delta-v; corridor width; campaigns of both variants |
| l6 | Game interface: `start_mission` kinds `lunar_landing` / `lunar_return`, reference body and terrain in the telemetry; docs | Smoke test |

Order l1 -> l2 -> l3 -> l4 -> l5 -> l6 (l5 needs only l1 and may move ahead). Each part has its own plan; part
decisions go into that plan's ledger.

## What exists and what changes

- `mission/lunar_transfer` (`LunarTransferTargeter`): TLI targeting in the full force model (periselene altitude and
  lunar inclination, arrival time free), Lambert first guess, best departure in the parking orbit, parking-orbit RAAN
  for a plane containing the Moon at arrival, impulsive LOI to a circular orbit. Tests fly finite TLI, a mid-course
  correction and LOI - outside the autopilot.
- `environment/gravity`: point masses with a selectable central body (Earth or Moon) - usable for the Moon-centred
  reference propagations of l1.
- `spice::SpiceOrientation::moon_mean_earth()` (`MOON_ME`, the frame of lunar maps) exists; `moon_mean_radius_m` is in
  `math/constants.hpp`.
- Earth is built into: `FlightBody` (environment holds `earth_orientation`; `ground_height_m`, `hold_fixed_to_earth`,
  `hold_landed`, `follow_pad`, `resolve_ground_contact`, analytic coasting of resting bodies "follow the Earth"),
  `FlightSession` (telemetry altitude, orbit elements, coast segments, landing zones), `contact/landing_surface`
  (ground plane or deck), `sim/leg_landing_sim`, `gnc/descent_snapshot` (plan frame from ITRF + WGS84),
  `gnc/descent_model` (US76 air), `gnc/ascent_guidance` (Earth GM, air), `gnc/orbital_attitude` (LVLH about the Earth).
  l1 routes these through `ReferenceBody`; the Earth case keeps its numbers bit for bit where possible (all existing
  campaigns re-run unchanged in outcome).

## l1 design: reference body

```cpp
namespace starsim::environment {
// Everything that is relative to a celestial body. Earth and Moon are instances; nothing in it is Moon-specific.
class ReferenceBody {
public:
    ephemeris::Body id() const;
    double gm_m3ps2() const;
    math::StateVector state_gcrf(time::Epoch) const;                 // body centre (Earth: zero)
    const frames::BodyOrientation& orientation() const;              // ITRF93 / MOON_ME
    const Atmosphere* atmosphere() const;                            // nullptr: none
    double surface_height_m(const math::Vec3& fixed_m) const;        // ellipsoid / sphere + terrain (l2)
    // geodetic (Earth: WGS84) or spherical (Moon) latitude / longitude / height of a body-fixed point
    frames::Geodetic to_surface_coordinates(const math::Vec3& fixed_m) const;
};
}
```
- `FlightSession` owns the reference bodies (Earth, Moon). A body's **reference body** is the nearest by gravitational
  dominance with hysteresis: Moon when within 60,000 km of the Moon, Earth again beyond 100,000 km [E] (the NRHO stays
  with the Moon). This choice only decides what is relative to which body (telemetry, ground, LVLH, orbit elements,
  guidance frames); the integrated state is always GCRF.
- Ground contact, resting, analytic coasting of resting bodies, landing surfaces and the leg simulation take the
  reference body's frame and surface instead of ITRF / WGS84. Air (drag, heating, flaps, grid fins) exists only where
  the reference body has an atmosphere.
- Telemetry gains `reference_body`, altitude over the reference surface, speed relative to the body-fixed frame and
  orbit elements about the reference body.
- Coast segments: unchanged (full force model, GCRF). A body below 5 km over a surface is ticked, as now.
- Validation: a 100 km LLO and the NRHO propagated one day Earth-centred agree with the Moon-centred propagation of the
  same force model within 1 m [E]; a body resting on the Moon stays within 1 mm of its body-fixed point over a day.

## l2 design: HLS vehicle and lunar terrain

- `config/vehicles/starship_hls.toml`: Block 3 hull and tanks (1,600 t [S]), 3 sea-level + 3 vacuum Raptor 3 (as the
  ship), dry mass 120 t [E] (provenance above), no flaps, no heat shield, no catch pins, ship RCS layout. Landing engines
  as their own engine group [E]: methalox thrusters high on the hull (count, position, cant, thrust and minimum throttle
  estimated so that the group lifts ~1.5 lunar weights of the landing mass and can hover), never used in Earth's
  atmosphere. Legs [E] fixed (always deployed), spring-damper + crush core as Falcon 9's (`contact/landing_legs`),
  scaled to the HLS footprint.
- Landing site: a mission parameter (latitude / longitude in `MOON_ME`); default one of NASA's nine Artemis III
  candidate regions of 2024 (e.g. Mons Mouton Plateau; coordinates [S] in the plan).
- LOLA tile: the LOLA polar gridded DEM (NASA PDS, public domain; 5-20 m/px; the exact product is chosen in the plan),
  cropped by `tools/data_pipeline` to ~20 x 20 km around the site (`data/terrain/`, git-ignored; `sources.toml` entry).
  `contact::LandingSurface` gains a height field: bilinear height and its normal under each foot. Outside the tile the
  Moon is the sphere R = 1737.4 km.
- Tests: drop tests on 0/5/10/15 deg terrain ("gelandet" / "umgekippt"); height and normal against the raw grid.

## l3 design: outbound flight

- Start: the HLS fully fuelled in 200 x 200 km at the mission's epoch, its plane from `parking_orbit_raan` (TLI needs no
  plane change).
- Autopilot steps (like `CapsuleReturnStep`): wait for the best departure -> TLI (finite burn on the vacuum Raptors,
  burn executor) -> coast in Sun attitude -> two mid-course corrections (~1 and ~2.5 days [E]) -> LOI at periselene
  into a 100 km polar LLO. For a south-polar site a polar orbit passes near the site every revolution; for other sites
  the HLS waits in LLO until the ground track passes within the descent's reach (no plane changes in lunar orbit).
- NRHO: the Artemis 9:2 synodic-resonant southern L2 NRHO (period ~6.6 d, perilune over the north pole, apolune
  ~70,000 km over the south pole [S]), generated by multiple shooting in the full model (DE440 Earth, Moon, Sun) from a
  CR3BP initial state of the literature [S] over the mission's span. ~~Direct transfer, insertion at perilune
  (~0.4-0.45 km/s [S]).~~ Amended 2026-10-15 (user): a direct transfer meets the NRHO perilune head-on (160 deg,
  ~3.5 km/s insertion), so the outbound flight uses the in-direct transfer [S: Kikuchi, Matsumoto & Ueda, ISSFD 2024]:
  powered lunar swing-by at 100 km, NRHO insertion ~1.2 d later at true anomaly 160 deg (swing-by 173 + insertion
  217 = 390 m/s in the source). NRHO -> polar LLO and back by a two-burn transfer (~0.75-0.9 km/s each way [S]); then the LLO
  descent. Station keeping only when the HLS waits several revolutions.
  Amended 2026-10-15 (user): LLO -> NRHO arrives anywhere on the reference, not only at a perilune - the NRHO plane
  turns with the Moon's orbit (~86 deg per revolution) while the LLO plane stays inertial, so a perilune arrival costs
  2.1 km/s after one revolution (1.1 km/s at the best alignment, ~26 d); arcs of 0.25-4 d to any reference point cost
  ~0.7 km/s within ~2.5 d of the LLO arrival or after ~21-30 d, 0.9-1.65 km/s in between (stay-dependent).
- Validation: LLO 100 km circular within 1 km after the corrections; TLI ~3.1-3.2 km/s and LOI ~0.82-0.9 km/s; NRHO
  period and perilune / apolune radii vs the literature; the HLS stays near the reference for three revolutions;
  NRHO <-> LLO delta-v in the literature's range.

## l4 design: landing and ascent

> **Correction 2026-10-15 (l4 Task 9):** ~1,185 t was l3's propellant USED, not the LLO mass; real ~573 t in the LLO, 500-566 t at PDI, 260-340 t landed - see `docs/superpowers/plans/2026-10-15-lunar-l4-mass-correction.md` (landing thrusters 12 x 90 kN, landing thrusters as the last braking set, braking range from the mass).

- Descent orbit: half a revolution before the site a ~20 m/s burn lowers periselene to 15 km (Apollo profile).
- Braking phase: from periselene, ~400-500 km before the site, ~6-7 min on the vacuum Raptors (attitude by the RCS; user 2026-10-15: Isp 380 s saves ~25-30 t), then one gimballed sea-level Raptor, with the Apollo lunar
  descent guidance (quadratic acceleration, Klumpp 1974 [S]) to a high gate ~2 km above the terrain a few km before the
  site. (The curvature and horizon of this phase do not fit the SCvx plan frame.)
- Approach: SCvx (`gnc/descent_*`, frame from `MOON_ME`, no air, lunar gravity) from the high gate - phase on the Raptors,
  then the landing engines (a Raptor at 30 % minimum throttle exceeds the light HLS's lunar weight: it cannot hover) -
  to the touchdown point on the LOLA height at ~1 m/s sink, the glide cone over the terrain. Engines off on leg contact;
  judgement as Falcon 9's ("gelandet", "umgekippt", "Aufprall").
- Touchdown point: the mission's point; the planning checks the slope under the footprint and takes the nearest
  admissible point within reach if it exceeds the limit (~8-10 deg [E]).
- Surface stay: the HLS rests on the Moon and follows it (l1); duration a mission parameter. Cryogenic boil-off is not
  modelled (no HLS data; documented as a known gap).
- Ascent: the landing engines lift off, the three vacuum Raptors take over after a few tens of metres (user 2026-10-15:
  fixed, RCS attitude, Isp 380 s - ~10 t saved; aborts on them too; engine-out of an RVac in ascent or braking: cut
  the other two at once, RCS corrects, the three sea-level Raptors start and the guidance continues); vertical rise, then PEG
  (`gnc/ascent_guidance` without air) to 15 x 100 km, circularisation at aposelene, for the NRHO variant the transfer.
  Launch window: the LLO plane passing over the site.
- Abort: as `abort_landing` - from the running descent to orbit with the ascent guidance from the current state;
  refused without the propellant for orbit.
- Validation: descent ~2.0 km/s and ascent ~1.9 km/s; campaign from periselene (state +-1 km / +-5 m/s, thrust +-2 %,
  mass +-5 %, touchdown points on real terrain): 21/21 landed within 1 m at <= 1 m/s; ascent reaches the LLO within
  1 km; aborts from 3 / 1 / 0.3 km reach orbit. Mission 1 complete with 120 t; refused with 204 t.

## l5 design: return to Earth

- Start: a Block 3 ship (heat shield, flaps, catch pins) in the LLO or the NRHO. The mission planning computes its
  propellant by planning the whole return with a nominal ship (fixed-point iteration on the mass): TEI, corrections,
  for the aerocapture the perigee raise and the deorbit burn, flip and catch (35 t), a reserve [E].
- TEI targeting (new, after the pattern of `LunarTransferTargeter`): the delta-v vector at a chosen burn epoch, in the
  full model, to the entry interface (120 km) with the planned flight-path angle and an along / cross position relative
  to Starbase that the entry guidance can fly out; Earth's rotation is matched by the burn epoch and the flight time
  (~3-4 days). Two mid-course corrections. TEI delta-v ~0.85-0.9 km/s from LLO, ~0.4-0.5 km/s from the NRHO [S].
- Direct variant: entry at ~11 km/s; the b2 entry guidance extended so that its predictor carries the arc above the
  atmosphere (numerical predictor-corrector skip entry, the principle of Orion's guidance [S: Brunner & Lu 2008]):
  a skip lengthens the range, then the second entry, the terminal phase to the gate and b3's flip and catch. The
  entry corridor is computed by the planning (full lift up must not skip out uncontrolled, full lift down must stay
  within load and heating); the planned flight-path angle sits in its middle.
- Aerocapture variant: one pass from ~40,000 to ~26,000 km/h, the bank steered on the apogee after exit (predictor-
  corrector aerocapture guidance [S: Lu et al., FNPAG]), apogee target 400 km [E]; at apogee a ~50-60 m/s burn raises
  the perigee to 200 km; then b4 (deorbit, entry, catch).
- Heat shield: radiative heating (Tauber-Sutton [S]) is added to the convective Sutton-Graves rate; the existing heat
  shield limits [E] stay - a failure is shown as "Hitzeschild versagt".
- Validation: TEI delta-v; corridor width plausible against Apollo / Orion (~1-2 deg at ~11 km/s depending on L/D);
  campaigns of both variants (flight-path angle +-0.1 deg, aero +-10 %, air density +-15 %): direct at the gate as
  b2's campaign then caught; aerocapture apogee within a band [E] and the raised perigee clear of the atmosphere. A
  comparison with Artemis I in magnitude only (Orion is a capsule with another L/D).

## l6 design: game interface

- `FlightSim.start_mission`: kinds `lunar_landing` (mission 1: orbit LLO / NRHO, site, stay) and `lunar_return`
  (mission 2: orbit, variant direct / aerocapture); mission configs in `config/missions/`.
- Telemetry: reference body, altitude over terrain, orbit elements about the reference body, mission step.
- Terrain query (height and normal at a point of the tile), landing site and touchdown point.
- Covered by the smoke test; `docs/ansichten.md` untouched until the user's UI specification.

## Error handling

- Mission start refuses with a German message: delta-v short (120 t / 204 t test), no pass over the site within the
  planning span, LOLA tile missing (hint to `fetch.py`), no touchdown point within the slope limit.
- In flight: landing out of reach -> abort to orbit; propellant exhausted -> "Aufprall"; heat-shield or load limit ->
  "zerbrochen"; aerocapture skips out uncontrolled or stays in the atmosphere -> outcome with its cause.

## Testing

Every new physical model gets unit tests and at least one reference case with a documented tolerance: reference body
and terrain, radiative heating against published Tauber-Sutton / Apollo / Orion values, NRHO against the literature,
Apollo braking guidance against Klumpp's reference trajectory. Whole missions and campaigns run as before with tags
(`[lunar]...[validation]`) and hidden single-case tests with seed variables. All existing campaigns re-run after l1
unchanged in outcome.

## Out of scope

Tanker flights, rendezvous, docking and propellant transfer (M9); Orion; HEEO or lunar-orbit refuelling; plume-surface
interaction (cratering, ejecta); boil-off; LOLA terrain rendering and terrain beyond the tile (M8); lunar gravity field
beyond the point mass (GRGM1200A, optional later; matters for long LLO stays); the mission planner UI (M6).
