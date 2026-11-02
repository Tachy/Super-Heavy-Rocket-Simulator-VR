# ISS rendezvous and docking

History and the reasons behind superseded decisions: `docs/history/iss.md`.
Spec `docs/superpowers/archive/specs/2026-10-19-iss-docking-design.md` (parts i1-i6; only Crew Dragon docks). Milestone i
COMPLETE 2026-10-20.

## Model

### ISS target body
- **Data.** NASA "ISS Trajectory Data" (CCSDS OEM, public domain, `tools/data_pipeline/sources.toml`). `fetch.py`
  archives each download as `data/iss_oem/ISS.OEM_J2K_EPH_<CREATION_DATE>.txt` (`tools/data_pipeline/iss_oem.py`; an
  archived creation date is skipped). Committed fixture: the full 2026-10-14 file in
  `tests/validation/fixtures/iss_oem/`.
- **Format** (2026-10-14 file): OEM 2.0 KVN, `REF_FRAME = EME2000` (taken as GCRF axes; bias ~23 mas, < 1 m, not
  modelled), `TIME_SYSTEM = UTC` (converted to TDB at load), km and km/s, 15 days on a 4 min grid that restarts at
  three rows (90 s step onto 2026-10-17T17:57:30 and 2026-10-21T04:35:00, 210 s step onto 2026-10-28T08:42:30; a last
  90 s step reaches the stop 2026-10-29T12:00, no new segment). Continuous across the seams (7-250 m). CRLF line ends.
  Header comments `MASS=460320.00`, `DRAG_AREA=1177.76`, `DRAG_COEFF=1.50`, `SOLAR_RAD_AREA=0.00` and the
  "TRAJECTORY EVENT SUMMARY" table (event, TIG day-of-year GMT, optional ORB, DV m/s, HA/HP km; imperial "(...)" lines
  skipped).
- **`orbit/oem`**: `Oem` parses one file (rejects other frames, unordered rows), Lagrange degree 8 (CCSDS) inside runs
  of equal steps ("segments": a segment starts at row k when the step after k differs from the step before and the
  next step equals the step after; the seam row is shared, no window crosses a seam). `OemArchive` (one directory)
  picks the newest file (by CREATION_DATE) whose usable window contains the epoch; unparsable files in `skipped()`.
- **`orbit/synthetic_station_orbit`** (dates without OEM coverage): mean elements of the newest OEM's first
  revolution, advanced by secular J2 rates (RAAN, argument of perigee, mean argument of latitude); the scenario sets the
  argument of latitude. Elements refer to the **true equator of date** (z = ITRF pole; x = GCRF x projected onto that
  equator, `true_equator_to_gcrf`): i = 51.633 deg, RAAN error after 14 days +0.018 deg (in GCRF it was 0.156 deg).
  The mean elements are used as osculating [E: J2 short-period terms ~+-10 km in radius not restored]. Without any
  OEM: `[stage.station.reference_orbit]` of `iss.toml` (415 km [E], 51.64 deg [S], RAAN 69.3148 deg [D] at
  2026-10-14T12:00 UTC; a test keeps it equal to the fixture's mean RAAN). The 2026-10-14 OEM has mean a - R 420.7 km
  (HA/HP 418.6/421.1 km).
- **ISS vehicle** (`config/vehicles/iss.toml`): one stage, no engines. 460,320 kg, drag area 1,177.76 m^2, C_D 1.5 [S:
  OEM header] (replaced by the header of the OEM in use); fixed drag C_D A along the oncoming flow
  (`aero/fixed_area_aero`); SRP `solar_area_m2` 2,500 m^2 [E] (the OEM uses 0); principal inertia [E]; ports IDA-2
  (Harmony forward, +XVV: V-bar) and IDA-3 (Harmony zenith: R-bar) with position, outward axis, clocking `up` [D] (Port clocking below);
  collision capsules (modules) and boxes (truss, arrays) [E]. Drag acts at every height US76 covers (to 1,000 km).
- **Ideal attitude** (documented simplification): a body with `[stage.station]` snaps to LVLH (+XVV: body x along the
  velocity, z nadir) rotated by the TEA `body_in_lvlh_deg` [E: yaw -3, pitch -6, roll 0.5 deg] after each integrator
  step (`FlightBody::set_ideal_lvlh_attitude`, `frames/lvlh`); contact torques do not change it (CMGs strong enough).
  Translation is free.
- **Session start.** `sim::iss_start(iss, archive, time, earth, epoch, source, argument_of_latitude)`; fidelity flag
  `iss_orbit` (`IssOrbitSource`): `oem` outside every window is a German error (no fallback), `synthetic`, or `auto`
  (OEM where covered, else synthetic; `note` says which). `FlightSession::add_station(config, state, epoch, impulses)`
  adds the station as an uncontrolled body - **call it after `place_body` / `launch_from_pad` / `place_in_orbit`**
  (they clear all bodies, the station included). Without bodies the session starts at the station's epoch; with
  bodies an epoch more than one tick off, a second station and a vehicle without `[stage.station]` are `logic_error`.
  A station alone coasts (orbital rate 1.1e-3 rad/s exceeds the 1e-3 rad/s settled threshold; the check skips ideal
  attitude).
- **Impulses.** OEM events with DV > 0 after the start epoch become impulsive Delta-v along the velocity at their TIG
  [D] (a coast stops there). Sign from the OEM's mean radius one revolution after vs before (32 samples each); within
  one revolution of the window edges it cannot be read: assumed posigrade (reboosts are), the `note` names it. Event
  names may contain "(": only lines starting with "(", "|" or "=" are skipped.

### Layout measured from NASA model D (Task 7, plan 2026-11-02-falcon-dragon-iss-models)
- **Source and method.** NASA 3D Resources ISS (D) IGOAL (`screenshots/falcon9_ref/models/iss/d_igoal/`, git-ignored,
  public domain). `tools/structures/measure/iss_measure.py` reads the glTF JSON and decodes the Draco vertex buffers
  (DracoPy, `-I`, nothing executed, no mesh data in the repo): per-node vertex extents, barrel radii, joint origins,
  IDA ring fits. Result table with [D] tags: `config/vehicles/detail/iss.toml` (config station frame). Frames: the root
  node has a negative uniform scale; undone, the file's local axes are x forward, y nadir, z port (checked: Columbus
  starboard / Kibo port of Harmony, PMA-3 above it); station frame (x fwd, y starboard, z nadir) = (x, -z, y).
- **Scale: 0.02542 m per vertex unit (the file's unit is the inch), not "truss = 109 m".** Taking the end-to-end truss
  (97.9 m measured) as 109 m gives 0.02830 and makes every module 11-19 % too big (Zarya 4.65 m diameter against 4.11,
  Zvezda 4.96 against 4.35, iROSA 21.1 x 6.7 m against about 18.9 x 6.1). The 109 m is the span across the P6 / S6
  arrays turned broadside (centre to centre 97.5 m plus one blanket width 11.4 m = 108.9 m; scale from that: 0.025420,
  = 1.0008 inch). Independent checks at that scale: S0 13.43 m (published 13.4), S1 / P1 13.83 (13.7), SAW 35.2 x 11.4
  (35 x 11.6), iROSA 18.96 x 5.99, Zarya 12.76 long x 4.14 (12.56 x 4.11), Zvezda 13.17 (13.1), Harmony 7.16 (7.2).
  Published module values as recalled, error of the scale about +-0.5 %. NASA "Space Station Facts and Figures": truss
  94 m, solar array wingspan 109 m, pressurised length 67 m, solar array length 73 m (the old `length_m` 73 was the array
  length). The measured truss (97.9 m) is +4 % against NASA's 94 m, unexplained (probably where the P6 / S6 mesh ends).
- **Layout in the config frame** (x from the aft end of the Russian segment, Zvezda hull at model x -35.76 m): Zvezda
  0-13.2, Zarya 13.2-25.9 (the config's Russian capsule 0-26 holds), PMA-1 25.5-28.3, Unity 27.8-33.7, Destiny
  33.3-42.4, Harmony 42.0-49.1 (7.16 m, barrel radius 2.2, CBM petals to 2.5), PMA-2 48.7-51.4, IDA-2 face 51.87; truss
  S0 centre at x 35.76 (mesh 33.0-42.8), truss axis 4.85 m zenith of the module axis, truss y +-48.8 m (arrays: P6 /
  S6 centres y +-48.8, P4 / S4 +-33.8, blankets 35.2 x 11.4 m, iROSA 18.96 x 5.99 m, alpha joints y +-17.8, beta
  joints y +-33.5 / +-48.5, radiator joints (TRRJ) y +-14.7); radiators P1 / S1 23.2 x 11.6 m (y +-14.7, trailing).
- **Ports.** Both IDAs sit **0.72 m off the Harmony axis**: IDA-2 ring axis at z +0.73 (nadir of the module axis), IDA-3
  ring axis 0.73 m forward of the Harmony centre (x 47.45 against 46.72). Ring (mesh) outer face = mating plane: IDA-2
  x 51.87, IDA-3 z -5.03; petals 0.295 m long beyond the base, tip radius 0.65 m, ring base radius 0.75 m. Petal
  clocking (port frame x outward, z up, y = z x x; angle from y towards up, +-3 deg): IDA-2 petal bodies at 58 / 178 / 302
  deg, feet at 359 / 117 / 241; IDA-3 (up = forward) bodies at 358 / 122 / 238, feet at 61 / 179 / 297 (identical
  parts, the 60 deg step follows from the two port frames). Looked at in close views
  (`screenshots/iss_measure/ida2_*.png`, `ida3_*.png`).
- **Config change** (outside the error, old -> new; tag table below): IDA-2 `position_m` [73.0, 0, 0] -> [51.87, 0, 0.72], IDA-3 [66.0,
  0, -5.0] -> [47.45, 0, -5.0] (z -5.03 measured, kept). The old x came from an inflated forward half (US stack to 69,
  Harmony at 66): measured 21 m too far forward. The hull capsules that hang on the ports follow (US modules end 49.1,
  PMA-2 49.1-51.37, PMA-3 on the Harmony axis, Columbus / Kibo at x 46.72), CG 38 m and truss / array boxes unchanged
  (CG 38 m stays an UNVERIFIED [E]: Destiny's centre at 37.9 says nothing about the mass; the review's rough tally gives
  about 33 m, which would make the IDA-2 lever about 19 m; physics unchanged, left open until a mass budget exists).
  `length_m` is now 51.87 m [D] (stack length; the game draws the station hull from it). Error of the new x: plane definition +-0.1 m, scale 0.25 m, aft
  end +-0.3 m: **+-0.5 m**. Lever of IDA-2 from the CG: 35 m -> 13.9 m.
- **Tags after the port change** (`[iss] [docking] [proximity] [rendezvous] [iss_mission]`, 266 cases, 265 passed; the
  one red is the known `Docked tanker: the ship return flies to the catch`, unchanged). Verbose numbers, Task 6 build
  (old) against Task 7 (new), limit as tested:

  | quantity | old | new | limit |
  |---|---|---|---|
  | chain IDA-2: contact / docked after / propellant | 61.8 min / 10.2 min / 31.8 kg | 62.5 / 10.0 / 31.6 | docked 8-13 min |
  | chain IDA-2: peak compression / shear / bending | 956 / 92 / 250 N m | 928 / 93 / 434 | 6500 N / 3200 N / 2800 N m |
  | chain IDA-3: contact / docked after / propellant | 98.0 / 10.0 / 63.5 | 98.8 / 10.2 / 65.7 (now 99.2 / 10.2 / 71.6, table below) | docks within 4 h |
  | chain IDA-3: peak compression / shear / bending | 970 / 108 / 230 | 970 / 88 / 270 | as above |
  | IDSS grid, 2 x 16 cases: docked; max comp / shear / bending / torsion | 32/32; 1254 / 631 / 2054 / 761 | 32/32; 1254 / 631 / 2055 / 761 | 6500 N / 3200 N / 2800 N m / 1500 N m |
  | undock IDA-2: separation / departure complete / propellant | 0.0844 m/s / 41.0 min / 5.0 kg | 0.0844 / 40.3 / 5.0 | 60 min / 60 kg |
  | departure CW guidance IDA-2: sphere / ellipsoid exit; 0.30 m/s, kg | 474 s / 2273 s; 6.5 | 625 s / 2300 s; 6.1 (re-measured Task 9: levers and clocking of Tasks 7-8) | ellipsoid within 3600 s |
  | departure CW guidance IDA-3 | 477 s / 1619 s; 6.7 | 454 s / 1783 s; 6.2 (Task 9) | same |
  | undock -> splashdown: departure kg / RCS left | 5.2 / 1890 kg | 5.2 / 1878 | splashdown reached |

  The shifts (contact +0.7 min, IDA-3 +2.2 kg, bending +184 N m at IDA-2) came with the shorter lever from the station CG (the
  port moves 21 m closer to the CG, the station's rotation drags the port 0.015 instead of 0.039 m/s); the later step in propellant
  comes from `up`, not from a lever (below), and the CG is unverified. No limit is touched. Test fixtures that said "station x = N m" now convert
  through the configured IDA-2 position (`docking_test_support.hpp`), `beside_modules` stands at station x 30 m (45 m is
  inside Columbus now), the hull-contact check at x 30.
- **Gaps of the collision shapes against the real station** (the drawing has them since Task 8, `vehicle-models.md`; the collision capsules and boxes are unchanged, the view does not touch the simulation): no Unity / Destiny / Node 3 / Cupola /
  Quest / Bishop / BEAM / PMM / Z1 shapes (the US stack is one capsule), no Nauka (MLM, nadir of Zvezda, x 10-20.4,
  13 m long, with the ERA arm), Prichal (nadir of Nauka, x 10.5-13.6), Rassvet (MRM1, nadir of Zarya, x 22-27), Poisk
  (MRM2, zenith of Zvezda, x 10.8-13.3), no iROSA (6 in the model on 8 array positions), no radiators; the truss box
  (half y 54.5) is 5.7 m wider than the truss (48.8, without array width); the array boxes stand at y +-38 against
  blankets at +-33.8 / +-48.8; truss centre x 38 against S0 at 35.76 (mesh centre 37.9). The name search of Task 1 found no
  Nauka / Prichal because the nodes are `MLM` and `Russian_RSNode_DockingModule`.

### Port clocking (Task 8, plan 2026-11-02-falcon-dragon-iss-models)
- **Change.** IDA-2 `up` [0, 0, -1] (zenith) -> [0, -0.866, -0.5]; IDA-3 `up` [1, 0, 0] (forward) -> [-0.5, -0.866, 0]. The rings could not meet model D with the old `up`: the IDD puts petals at theta 90 / 210 / 330 about `up`, and model D's IDA-2 petals stand 60 deg off that about the zenith.
- **Evidence.** (1) Measured: model D's IDA-3 petals fit `up` = forward; IDA-3 is the same part as IDA-2 turned by 180 deg between the two old port frames (azimuth pattern correlation peak at exactly 180), so IDA-2's `up` is nadir or +-120 deg from it: three candidates per port; the model has no guide pins or hooks that could pick one. (2) Dragon's hinge carries a petal (`D_Iss065e002827`, low resolution, my read; as Task 6 drew it). (3) Photos: `D_Crew_Dragon_Endeavour_and_H_II...` (IDA-2): hinge about zenith; `D_Iss072e096331`, `D_Iss072e144451` (IDA-3): nose to the right with Kibo's logistics module to the left, read as starboard. The candidate that fits: hinge 30 deg from zenith towards starboard at IDA-2 (the camera's side), 60 deg from starboard towards aft at IDA-3. **The photos read to about +-30 deg, the IDD's `up` of a real IDA is in no document we hold: a measured clock class (hard, model D, tag [D]) with a photo-picked member (soft, tag [E]).** Two links of the member pick are unverified: (a) `D_Iss065e002827` shows the hinge but does not resolve the petals, so "a petal on the hinge side" is not confirmed, and the hinge could stand 60 deg off, i.e. 30 deg from zenith towards PORT, which the +-30 deg photo read cannot separate from the chosen value; (b) IDA-3: "nose to starboard" leaves aft and forward (both 60 deg from starboard) open, aft comes only from model D's same-part relation (IDA-3 = IDA-2 + 180 deg), not from a photo. The member is a ruling for the user, reversible (one tag run). Dragon's `up` body -z unchanged.
- **Tags** (`[iss] [docking] [proximity] [rendezvous] [iss_mission]`, 266 cases, 265 passed; the one red is the known `Docked tanker: the ship return flies to the catch`). Task 7 -> this change, limit as tested:

  | quantity | old up | new up | limit |
  |---|---|---|---|
  | chain IDA-2: contact / docked after / propellant | 62.5 min / 10.0 min / 31.6 kg | 62.3 / 10.2 / 37.4 | docked 8-13 min |
  | chain IDA-2: comp / shear / bending | 928 / 93 / 434 | 955 / 84 / 95 | 6500 N / 3200 N / 2800 N m |
  | chain IDA-3: contact / docked after / propellant | 98.8 / 10.2 / 65.7 | 99.2 / 10.2 / 71.6 | docks within 4 h |
  | chain IDA-3: comp / shear / bending | 970 / 88 / 270 | 925 / 79 / 209 | as above |
  | IDSS grid 32 cases: docked; max comp / shear / bending / torsion | 32/32; 1254 / 631 / 2055 / 761 | 32/32; 1252 / 627 / 2061 / 759 | 6500 / 3200 / 2800 / 1500 |
  | undock IDA-2: separation / departure complete / propellant | 0.0844 m/s / 40.3 min / 5.0 kg | 0.0844 / 32.3 / 4.7 | 60 min / 60 kg |
  | departure CW IDA-2: sphere / ellipsoid exit; kg | 474 s / 2273 s; 6.5 | 625 s / 2300 s; 6.1 | 3600 s |
  | departure CW IDA-3 | 477 s / 1619 s; 6.7 | 454 s / 1783 s; 6.2 | 3600 s |
  | undock -> splashdown: departure kg / RCS left | 5.2 / 1878 | 4.9 / 1890 | splashdown reached |

  **What the `up` change did, shown by runs (2026-11-02, current tree, `STARSIM_VERBOSE=1`, the five cases chain IDA-2 / IDA-3, undock IDA-2, departure guidance IDA-2 / IDA-3, one `up` changed at a time).** Old `up` on both ports gives exactly the old numbers (IDA-2 chain 62.5 min / 31.6 kg / bending 434 N m, IDA-3 chain 65.7 kg / bending 270 N m, undock departure complete 40.3 min / 5.0 kg); IDA-2 `up` alone moves the IDA-2 chain and the undock, IDA-3 `up` alone moves the IDA-3 chain, nothing crosses between ports. So the +5.8 / +5.9 kg, the bending drop 434 -> 95 N m and the departure step 40.3 -> 32.3 min come from `up`, not from the fixtures or another change in the tree. The departure CW rows (sphere / ellipsoid exit 625 s / 2300 s IDA-2, 454 / 1783 s IDA-3) are identical for old and new `up`: their change against the Task 7 column (474 s ...) is not from `up` (earlier tree change, not attributed here).
  **Not shown.** The earlier explanation "the port's lateral axes no longer lie on Dragon's body axes" was wrong: mated attitude holds Dragon's axes on the port's (`flight_session_proximity.cpp`), and `apply_translation` commands in port axes, so port-axis authority does not depend on `up`. What `up` turns is LVLH against the body (V-bar / R-bar corrections, the departure burn through `port_to_lvlh`, `departure_guidance.cpp`). That LVLH-against-body mapping is the working hypothesis (pure lateral use scales with |cos| + |sin| = 1.366 at the 30 / 60 deg clockings, about 16 kg of lateral use for the +5.8 kg); the runs do not separate it from any other path through `up`, and no per-axis propellant breakdown was made (`STARSIM_DEBUG_PROXIMITY=1` would give it). The departure step is the discrete direction pick (Validation) moving with `up`, as before: unproven. The IDA-2 chain bending (434 -> 95 N m) and the IDA-3 numbers are shown to depend on `up`, their mechanism is open. No limit touched, no tuning, no physics changed. Three fixtures assumed "IDA-2 port frame = station (x, -y, -z)" (`beside_modules`, two truss-tip cases) and failed first: they stood somewhere else after the change; `docking_test_support.hpp` now converts station axes through the configured `up`. The Proximity and Docking tables in Validation hold the older `up` where not listed here.

### Crew Dragon translation and relative navigation (i2)
- **Draco layout** [E] kept (user 2026-10-19). Test threshold lateral >= 0.04 m/s^2 [E] (1 m/s in 25 s).
- **Pure translation capability:** `RcsAllocator::pure_force_capability_n` (largest force whose allocation leaves torque
  and cross force < 1 % of force / gross thruster torque), `Vehicle::rcs_pure_force_capability`,
  `VehicleCommand::rcs_force_pure` (force fraction refers to it). Burns keep the coupled `force_capability_n`.
  Measured (cone open, 12.5 t): +x 1556.3 N = 0.1245 m/s^2, 4.25 kg per m/s; -x 1556.3 N, 4.55 kg/(m/s); +-y +-z
  566.7 N = 0.0453 m/s^2, 16.20 kg/(m/s) (3.8x ideal; bound 4.5x [E]); residual torque <= 0.01 N m. Closed nose cone:
  pure lateral and -x authority < 1 N (lateral force needs the nose Dracos under the cone).
- **Dragon port** (`crew_dragon.toml`): `idss`, station 4.22 m [E: the drawn HCS mating plane, `vehicle-models.md`; 4.0 until 2026-11-02], axis body +x, clocking `up` body -z [E],
  `behind_nose_cone = true` [S] (covered while the cone is closed).
- **Port frame** (`vehicle/docking_port`, `port_kinematics(body, id)`): x outward docking axis, z clocking `up`,
  y = z x x. Lever from the current CG; port velocity = CG velocity + omega x lever. Mated ports: axes opposed, ups equal.
- **Relative state** (`gnc/relative_navigation`, `relative_port_state(chaser, target)`): chaser port relative to the
  target port in the target port frame (turns with the target; a co-moving chaser reads zero velocity). Closing speed
  = -d(range)/dt; misalignment = angle chaser axis vs reversed target axis; clocking = chaser up vs target up about the
  target axis (positive about +x_target, (-pi, pi]). Fidelity flag `relative_nav`: only `"ideal"`.
- **Session API:** `set_rendezvous_target(target_index, chaser_port, target_port)` (out_of_range for a bad index or the
  controlled body, German invalid_argument for an unknown port), `clear_rendezvous_target()`, `relative_port_state()`
  (nullopt without target), `set_translation_command(f)` (target port frame, fraction of the pure capability, |f| > 1
  scaled; logic_error without target). Target and command clear with the bodies and whenever the controlled body
  changes (`stage`, payload separation, trunk jettison). The command is flown only in an attitude hold or rate mode
  outside the autopilot's own steering (Off / Orbit / Lunar; `translation_flown()`), and only then the session does not
  coast; a stale command elsewhere neither acts nor stops warp. Applied next to the hold's torque, **torque first**
  (`rcs_keep_force = false`: hold stays < 0.72 deg at 100 %; with force kept it drifted 2.8-13.8 deg in 20 s); a
  saturated translation is cut; hold turns may push (cross <= 0.064 m/s over 20 s). A pilot's body-axis translation
  (`input_.translation`) takes precedence in rate mode.

### Phasing (i3)
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i3-phasing.md`.
- **Profile** (Crew-2 timeline, Spaceflight Now 2021-04-22): separation, phase burn +37 min, boost +15.4 h, coelliptic
  burn, approach initiation **7.5 km** out, mid-course 25 min later, **Waypoint 0 400 m below the ISS** 45 min after
  approach initiation. Demo-2 burn sizes (everydayastronaut.com): phase 16.1, boost 44.2, circularisation 57.9 m/s.
  Derived [D]: coelliptic closing 4.9 m/s between 30 and 10 km = 1.5 n dh -> **dh 2.9 km**; coelliptic coast 2.8 h. All
  are `mission::PhasingParameters` fields with tags.
- **Spec corrections (user 2026-10-19):** (1) i3 ends at Waypoint 0, braked to rest in the station's LVLH frame; i4
  starts there. (2) i3 is port-independent. (3) The phase burn is used only with `earliest_arrival_s` (phase burn +
  boost at the same perigee add to the same Hohmann transfer: the cheapest correct window is the earliest one, waiting on
  the separation orbit). (4) Orbit plane: J2 node regression is ~0.58 deg/day faster at 205 than at 415 km; tests aim the
  start plane to drift onto the ISS plane by the boost (`rendezvous_test_support.hpp`; the launch-window planner does the
  same); the rest is removed by the 3D-targeted coelliptic and approach burns.
- **Clohessy-Wiltshire** (`orbit/clohessy_wiltshire`, LVLH x V-bar, y H-bar, z R-bar): propagation, two-impulse
  transfer, `lvlh_relative_state`; first guess and timing aid only (42.7 m against two-body after the test arc, bound
  60 m: the rectilinear V-bar's curvature offset x^2/2r).
- **Geometry** (`mission/rendezvous_geometry`): aim points behind/below the target, approach ellipsoid metric
  (semi-axes 2000/1000/1000 m [S: NASA]), keep-out sphere 200 m [S]. `PositionTargeter` has an optional drag term
  (`set_drag_area_per_mass`; 0 = old path byte-identical).
- **Schedule** (`mission/phasing_schedule`): burn epochs from mean motions with secular J2 rates. Default: wait on the
  start orbit until the phase is right, then boost; with `earliest_arrival_s` a prograde phase burn at
  `first_burn_lead_s` (30 min [E]) raises the apoapsis (at most to the coelliptic height: catch-up rate 11.2 -> 5.7
  deg/h) and the boost follows after `phasing_revolutions`. A phase needing more than one lap adds `extra_laps`; beyond
  `mission_limit_s` (28 h [S]) refused in German; a start whose apoapsis reaches the coelliptic height or an open orbit
  is refused.
- **Planner** (`mission/phasing_planner`): each burn targeted in the full force model with drag on both bodies
  (`gnc::coast_with_drag`, US76), flown as a finite Draco burn (`apply_finite_burn`). Phase and boost are prograde
  magnitudes solved by secant (boost on the radius at the coelliptic epoch; a waiting plan first solves the boost epoch
  for the phase); coelliptic -> approach initiation (7.5 km, dh below) and approach initiation -> Waypoint 0 by the
  position targeter (CW first guess); the mid-course correction is planned in flight only (skipped below 0.02 m/s [D];
  non-converged: "Kurskorrektur nicht berechenbar", the braking still follows; a skipped one carries ~24 m); braking
  leaves the chaser at rest in the target's LVLH frame. Budget by the rocket equation against the Dracos' propellant with
  `reserve_kg` 900 kg [E] (deorbit ~600 kg + proximity and abort); `min_ellipsoid_metric_before_ai` (arc sampled every
  minute) must stay > 1. `boost_phase_error_m` predicts the along-track miss from live states.
- **Drag.** Dragon C_D 2.2 [E] on its cross-section; ISS from its OEM header. In flight the session tracks both
  (`gnc::estimate_drag_area_per_mass`, Dragon since its last burn, station since plan or last impulse) once an arc is
  >= 3 h (`rendezvous_track_min_arc_s` [E]); estimates replace the plan's fields before every burn. A waiting plan
  checks the boost geometry `rendezvous_check_lead_s` = 3 h [E] before the boost and re-plans from live states beyond
  `rendezvous_replan_tolerance_m` = 2 km [E]; a burn that cannot be targeted before the boost re-plans once, later it
  aborts (memory feedback-replan-from-flight-data).
- **Session** (`flight_session_rendezvous.cpp`): `plan_rendezvous(PhasingParameters)` -> `AutopilotState::Rendezvous`,
  `RendezvousStep` None / Coast ("Phasing") / Burn ("Manöver") / Waypoint0 ("Wegpunkt 0"), `rendezvous_plan()`, German
  `rendezvous_status()`, `rendezvous_burns()` (`RendezvousBurnRecord`), `rendezvous_replans()`, `rendezvous_warnings()`
  (station reboost inside the plan's span). Refused (German) without a station, with the station controlled, a busy
  autopilot, a rendezvous or lunar flight under way, a vehicle with main-engine burns (only Crew Dragon docks), or the
  planner's reason. Every burn re-targeted from live states at its command epoch, flown through `execute_burn` (Dracos
  along +x); big burns commanded `burn_drift_lead_s` before preparation, mid-course and braking 600 s
  (`rendezvous_small_turn_lead_s` [E]); between burns the vehicle drifts.
- **Nose cone:** `plan_rendezvous` opens Crew Dragon's nose cone (restored on a refusal); closed, Waypoint 0 was missed
  by 83-391 m, open by 2-12 m. A `BurnExecutor` change (sensing attitude pulses before ignition) was reverted (broke the
  ship return and lunar return campaigns).

### Proximity operations (i4)
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i4-proximity.md`.
- **Profile** (Crew-2 / Demo-1, Spaceflight Now): Waypoint 0 04:25 (400 m below), go to enter the keep-out sphere,
  **Waypoint 1 04:49 220 m in front**, **Waypoint 2 05:00 at 20 m**, contact at IDA-2 05:08-05:10; Crew-5: hold and
  docking go/no-go at Waypoint 2. Demo-2 (NASA, 2020-05-31): final approach **0.30 m/s**, slowed to **0.10 m/s** before
  soft capture, nose cone open. Zenith port (Crew-4, Crew-2 relocation): Waypoints 0/1/2 then a fly-around (geometry
  unpublished: [E]). Derived [D]: Waypoint 0 -> 1 is 1140 s plus a 5 min hold. All values are
  `gnc::ProximityParameters` fields with tags.
- **Spec corrections (user 2026-10-19):** (1) transfers outside the keep-out sphere are CW two-impulse transfers re-solved
  every tick from the live state, flown with a deadband (CW vs full model Waypoint 0 -> 1 miss 0.95 m, bound 2 m).
  (2) IDA-3 (R-bar) gets a fly-around: Waypoint 0 -> 250 m in front on V-bar -> 30 deg legs on a 250 m arc about the
  station CG -> Waypoint 1 220 m above IDA-3; IDA-2 goes Waypoint 0 -> 1 directly. (3) Inside the sphere a range-scheduled
  axial approach (0.30 m/s far, range / 600 s between the limits, 0.10 m/s at Waypoint 2, 0.10 -> 0.075 m/s in the final
  approach) with lateral convergence onto the axis. (4) i4 ends at "Kontakt", the crossing of the contact plane
  (`contact_plane_m` 0 [E]; set to the petal mesh plane 0.70 m by the docking session). (5) Contact speed target
  0.075 m/s [E].
- **Capture envelope** [S since i5, IDSS IDD Rev. E]: axial 0.05-0.10 m/s, lateral <= 0.04 m/s, offset <= 0.10 m,
  angle <= 4 deg, rate <= 0.2 deg/s.
- **Geometry** (`gnc/proximity_geometry`): `proximity_path(port, parameters)` (LVLH points outside, `on_axis` points on
  the port axis inside), `corridor_status` (cone 10 deg [E] half angle about the port axis, never narrower than 0.3 m;
  beyond 2x the bound: retreat; x <= 0 counts as retreat), docking attitude, `capture_envelope_check`.
- **Transfer** (`gnc/proximity_transfer`): CW two-impulse transfer with keep-out clearance (sphere + 10 m margin,
  sampled; a path through the sphere is refused: abort "kein sicherer Transferweg"). Re-solve stops
  `transfer_final_s` 60 s before arrival; the hold law brakes onto the waypoint. **Breakout:** the smallest of 0.3 / 0.5
  / 1.0 / 1.5 m/s [E] along candidate directions that leaves the sphere within 1800 s and stays outside for 24 h (CW, 30
  s samples).
- **Translation loop** (`gnc/translation_loop`): desired relative velocity -> translation fraction per axis with a
  hysteresis deadband (start above 0.02 m/s, stop below 0.005 m/s [E]; fraction = error / (authority x 1 s)); final
  approach a finer loop (start 0.005 m/s). Laws: axial speed schedule, lateral velocity = -offset x 0.1/s (max 0.03
  m/s), position hold velocity = -error / 60 s (max 0.05 m/s), retreat 0.10 m/s outward along the axis.
- **State machine** (`gnc::ProximityGuidance`, German phases): "Wegpunkt" (hold, go/no-go 300 s each [E]), "Annäherung"
  (CW transfer), "Umflug", "Anflug" (axial approach to an on-axis waypoint), "Endanflug" (Waypoint 2 -> contact), "Halt",
  "Rückzug", "Abbruch", "Kontakt". Monitors: leaving the corridor -> "Halt: Korridor verlassen" (beyond 2x: retreat);
  closing faster than 1.5 x schedule + 0.02 m/s -> "Halt: zu schnell"; inside the sphere off the corridor -> "Abbruch:
  Sicherheitszone verletzt"; RCS propellant below the deorbit reserve (600 kg [E]) -> "Abbruch: Treibstoffreserve".
  Commands `hold`, `retreat` (from a waypoint or an interrupting hold: to the previous waypoint; from a monitor: to the
  last reached), `abort`, `resume` latch a request resolved in the next update; a second command before then is refused;
  each refusal sets a German `refusal()` (shown as `proximity_status()`), e.g. retreat at Waypoint 0, resume without a
  hold, resume after an abort while the breakout is flown, beyond 5 km, inside the sphere or below the deorbit reserve.
  The session refreshes the live state (`observe`) before every command. A retreat to an on-axis waypoint flies the axial
  law also between sphere and waypoint; a monitor retreat at an on-axis waypoint already reached goes one waypoint
  further back. A transfer starting inside the 10 m margin only has to stay outside its start range. A start within 10 m
  [E] of the first waypoint holds there. After the breakout the phase stays "Abbruch" (passive, status
  "Ausweichmanöver ausgeführt"[", nicht passiv sicher"]); `resume` flies back to Waypoint 0.
- **Session** (`flight_session_proximity.cpp`): `start_proximity(station_port, ProximityParameters)` ->
  `AutopilotState::Proximity` ("Anflug"); getters `proximity_phase()`, `proximity_status()`, `proximity_events()`,
  `proximity_contact()`, `proximity_warnings()` (station impulses within 3 h of the start); commands
  `proximity_hold/retreat/abort/resume()`. Refused (German) without a station, with the station controlled, a busy
  autopilot (not Off / Orbit), a rendezvous other than at Waypoint 0, a lunar flight, no docking port on the vehicle, an
  unknown or unsuitable station port, a start beyond 5 km, or inside the sphere off the start axis. Opens the nose cone
  and sets the rendezvous target (chaser port: the vehicle's first docking port). The docking attitude is an LVLH hold
  re-derived every tick from the port kinematics; translation authority along the target port axes is measured once at the
  start. The guidance runs in the autopilot dispatch before the tick's vehicle command; `advance_to` keeps ticking during
  the flight and may coast only after a completed breakout. The flight ends with the rendezvous target
  (`clear_rendezvous_target` and all paths calling it), `disengage_autopilot`, `set_rendezvous_target`, and at contact
  (autopilot off, flight stays readable).

### Docking mechanism, undocking and departure (i5)
Plan `docs/superpowers/archive/plans/2026-10-19-iss-i5-docking.md`.
- **Sources.** IDSS IDD Rev. E (Oct 2016): Table 3.3.1.1-2 initial contact (axial 0.05-0.10 m/s, lateral 0.04 m/s,
  pitch/yaw/roll rate 0.2 deg/s, lateral misalignment 0.10 m, pitch/yaw 4 deg, roll 4 deg), §3.2 SCS mating plane at the
  1,200 mm petal outline, §3.2.2.1 three inward petals, §3.1.1.1 sequence (petals -> latches -> attenuation ->
  retraction -> guide pins -> 12 hooks), Table 3.3.1.4-1 SCS loads (compression 3,500 N static / 6,500 N up to 0.1 s,
  tension 3,900 N, shear 3,200 N, bending 2,800 N m, torsion 1,500 N m), §3.2.3.6 separation energy 39.2-47.5 N m,
  Table 3.3.1.2-1 IDSS-350T inertia (ISS inertia = that scaled by mass, [D]). Demo-2: soft capture 10:16, hard capture
  10:27 EDT. Crew-4 / Demo-1: undock burn 1 to 0.15 m/s, keep-out sphere left 6 min 48 s, approach ellipsoid 17 min.
- **Interface** (`contact/docking_interface`, passive port frame, reduced order): active ring face 0.30 m [E] out of the
  Dragon port; ring-face penalty contact (2e5 N/m, 2.9e4 N s/m [E]: 0.10 m/s impact peaks ~4.0 kN, 0.25 m/s ~10 kN);
  three petal pairs as one cone (outline radius 0.60 m [S], height 0.20 m, lean 30 deg, meshed clearance 0.01 m [E]):
  capture radius 0.241 m (beyond it tips strike the opposing petals, axial contact, no centring). The light active ring
  gives way on its platform (2000 N/m up to 0.20 m [E]); conforms in tilt up to 6 deg and roll up to 6 deg [E]. Latches:
  ring faces within 5 mm, deflection, tilt and roll within ranges. Acts only within 1.2 m laterally and 0.15 m ring
  penetration (beyond: hull contact).
- **Mechanism** (`contact/docking_mechanism`): Ready -> Kontakt -> Weichfang (damped 6-DOF joint: translation critically
  damped at 0.5 rad/s on the chaser mass; rotation from IDSS loads - half the bending / torsion limit at 4 deg, damping
  ratio 1 with the chaser inertia; axial stroke 0.10 m, then a 2e6 N/m end stop) -> Einzug (after >= 120 s and settled,
  0.30 m at 1 mm/s) -> guide pins (5 mm, 0.5 deg) -> Haken schließen (180 s) -> angedockt; first contact -> hard capture
  10.0-10.2 min. Failures: no contact for 2 s with the ring 0.05 m beyond the petal tips: "abgeprallt" / "Versatz zu
  groß" (tips only) / "Winkel zu groß" (ring touched, latch blocked); compression above 6,500 N or above 3,500 N for >
  0.1 s: "Überlast" (latches release). Tension / shear / bending / torsion above the limits only warn. Below 0.05 m/s it
  still captures (documented). Undock: Haken öffnen (180 s) -> getrennt; separation speed sqrt(2 E / mu), E = 43.35 N m
  [E]: 0.0844 m/s for the Dragon on the ISS.
- **Hull collision** (`contact/station_collision`): hull rings every 0.5 m (16 points) over the trunk (3.25 m) and the
  capsule tapering 4.0 -> 1.8 m [E: `hull_top_diameter_m`], the top metre below the docking port left out; penalty 1e6
  N/m against ISS capsules and boxes within 120 m of the station CG. Any contact: "Kollision" with impulse (sum |F| dt),
  the proximity flight ends (no structural model).
- **Session** (`sim/docking_sim`, `flight_session_docking.cpp`): `set_rendezvous_target` arms the mechanism for an
  active chaser port and a passive station port (`[[stage.docking_port]] soft_capture`); `start_proximity` refuses other
  pairs and sets the contact plane to 0.70 m. Every tick the mechanism and hull wrenches act on both bodies (equal and
  opposite; station torque dropped - ideal attitude). Free drift (no RCS) from first contact to the hooks. Coasting near
  the station is limited by the clearance over the relative speed in the rotating station frame. **Compound:** at hard
  capture the chaser becomes a point mass of the station vehicle at its CG (`Vehicle::set_attached_mass`), the station
  state moves to the compound CG with momentum-weighted velocity, the chaser is held (`FlightBody::hold_docked`) at the
  mated pose; `compound_mass()` reports summed mass and inertia (unused while attitude is ideal); the Dragon drag area is
  not added (~0.6 % of the station C_D A). A failed capture sends the proximity flight back to Waypoint 2
  (`ProximityGuidance::capture_failed`, "Rückzug: Fangen fehlgeschlagen (<reason>)"); the mechanism re-arms 1 m beyond
  the petal tips. While docked or capturing, `start_proximity`, `plan_rendezvous`, `plan_capsule_return`, `ignite`,
  `stage` are refused (German). Getters `docking_phase/status/events/loads`, `docked`, `docking_busy`, `compound_mass`,
  `collision_impulse_ns`, `docking_merge_record`.
- **Undocking and departure** (`gnc/departure_guidance`): `undock()` (docked, autopilot Off / Orbit) -> hooks open ->
  springs split the relative speed by mass along the port axis (momentum exact) -> `AutopilotState::Departure` "Abflug":
  "Abkoppelimpuls" to 0.15 m/s, "Rückzug vom Port" on the axis to 10 m, "Abflugmanöver" = the i4 breakout with ellipsoid
  exit required within 3600 s (passively safe 24 h), "Abflug" drift (coasting, guidance tick every 60 s), "Abflug
  abgeschlossen" at the ellipsoid exit (autopilot off, target cleared). Later departure burns are not modelled; the
  deorbit planned from the drift orbit replaces them.

### ISS missions, launch window and game interface (i6)
Plan `docs/superpowers/archive/plans/2026-10-20-iss-i6-missions.md`.
- **Missions** (`config/missions/`, `sim/iss_mission.hpp`, dispatched by `sim::start_mission`): `crew_dragon_iss`
  (`kind = "iss_rendezvous"`: Dragon at the predicted separation of the first launch window of
  `falcon9_crew_dragon_iss` after `epoch_utc`, or `phase_deg` behind the ISS in its J2-biased plane),
  `falcon9_crew_dragon_iss` (`kind = "iss_launch"`: launch window, then the `falcon9_crew_dragon` ascent) and
  `crew_dragon_iss_departure` (`kind = "iss_departure"`: docked start, undocking timed to a return pass, the
  `crew_dragon_return` capsule return). ISS from `data/iss_oem/` (`sim::iss_archive`) per `iss_orbit`; a scenario
  argument of latitude on the synthetic orbit is kept as an in-plane offset from the J2-advanced one.
- **Sequence** (`FlightSession::set_iss_mission`, `flight_session_iss_mission.cpp`): legs Ascent -> Rendezvous
  (`plan_rendezvous` once the Dragon flies free) -> Proximity (`start_proximity` at Waypoint 0) -> Docking -> Docked, or
  WaitUndock -> Departure -> Return (`plan_capsule_return` with `lead_s` aimed at the previewed pass) -> Complete. Runs
  every tick (also autopilot Off / Orbit); the coast stops at every due leg change and the undock epoch. A proximity
  abort keeps the leg; a collision or lost vehicle ends the mission ("Mission beendet: ..."). Cleared with the bodies
  (`clear_station`).
- **Launch window** (`mission/iss_launch_window`): `AscentGuidance` fixes its inertial target plane at liftoff through
  the pad (`gnc::ascent_plane_normal`), so the window is the upward zero of the node difference between that plane and the
  ISS plane turned east by the J2 bias (`mission::plane_bias_rad`, ~0.58 deg/day at 205 vs 415 km), sampled every 600 s
  and bisected, the bias iterated twice onto the schedule's boost epoch. Separation state from two calibrated numbers
  [D] (`falcon9_crew_dragon_iss.toml`): **`insertion_s` 542 s** liftoff -> separation and **`insertion_arc_deg` 15.01**
  inertial in-plane arc from the pad (recalibrated after ascent changes; flown separation 542.0 s after liftoff, phase
  error 0.004 deg; check them after any change to the Falcon 9 ascent, staging or Merlin data - see
  `docs/subsystems/ascent-staging.md`). A crossing whose phasing does not close in the mission limit is rejected and the
  search moves a day on (up to `search_days`, default 3 [E]).
- **Departure timing** [S: Crew-9 2025-03-18, NASA: undock 05:05 UTC, deorbit burn ~21:11 UTC, splashdown 21:57 UTC]:
  undocking 16.1 h before the deorbit burn. The start previews the first pass at least that long after `epoch_utc`
  (`FlightSession::capsule_return_preview`, `search_window_h` 168), places the docked pair (`FlightSession::dock_in_place`:
  mated pose, mechanism `start_docked`, compound merge) 600 s [E] before undocking and flies on to splashdown.
- **Game interface** (`FlightSim`, `bindings/src/flight_snapshot_iss.cpp`): getters `iss_state`, `rendezvous_plan`,
  `docking_state`, `station_geometry`, `iss_mission` (keys in `flight_sim.hpp`), commands `proximity_hold`,
  `proximity_retreat`, `proximity_abort`, `proximity_resume`, `undock`; `start_mission`'s message carries the warnings
  (ISS source note, reboosts in the mission span). Covered by the smoke test.

## Standing decisions
- Only Crew Dragon docks with the ISS (spec 2026-10-19); the Draco layout [E] is kept (user 2026-10-19).
- Spec corrections of i3 / i4 / i5 / i6 approved by the user on 2026-10-19 / 2026-10-20 (listed above).
- Cheapest correct window = the earliest one (call, i3 correction 3); the earliest window is flown
  (`earliest_arrival_h = 0`), real crews arrive after ~24-28 h (crew day), set by `earliest_arrival_h`.
- Failed corrections re-plan from live state (user, memory feedback-replan-from-flight-data).
- Ideal station attitude (LVLH), `relative_nav` ideal, compound as a point mass: documented simplifications.
- Torque first when translation and attitude hold compete (call, i2).

## Validation
- **OEM** (`[iss]`, SRP off, start 2026-10-16T00:00 UTC, full force model with US76): after 6 h R 21.8 m, T -894.5 m, N
  -13.3 m; after 24 h R -11.7 m, T -3,228.7 m, N -43.0 m (tolerances 1,400 m / 4,900 m). Along track: US76 density at
  ~420 km differs from NASA's; NRLMSISE should shrink it. Leave-one-out on the thinned 8 min grid: median 20 m, worst
  529 m (one-sided window after a seam), 1.3 m/s; error ~h^9, published grid ~4 cm. Station tests < 0.5 s each. The
  fixture has only a 0 m/s docking event; the sign paths use generated two-body OEMs.
- **Translation campaign** (`[rendezvous][session][validation]`, Dragon 50 m in front of IDA-2, OEM ISS 2026-10-16, LVLH
  hold; 20 s full command per axis, 5 s settle; dv / expected, cross, propellant / expected, worst attitude error): +x
  2.296/2.301 m/s, 0.064, 11.14/11.13 kg, 0.63 deg; -x 2.335/2.334, 0.039, 13.24/13.31, 0.71; +y 0.899/0.908, 0.007,
  14.52/14.51, 0.58; -y 0.896/0.909, 0.006, 14.95/14.96, 0.68; +z 0.903/0.917, 0.024, 15.13/15.13, 0.68; -z
  0.912/0.917, 0.027, 14.29/14.27, 0.59 (tolerances 15 % dv, cross < 10 % + 0.01 m/s, 25 % propellant, 1 deg). ~2 s.
- **Phasing planner** (`[validation]`): plan from 120 deg: boost 64.65 m/s @+9.80 h, coelliptic 62.47 @+10.55 h, approach
  initiation 1.86 @+13.35 h, braking 0.13 @+14.10 h; total 129.1 m/s, 537 kg; ellipsoid metric before approach
  initiation 20.4; Waypoint 0 miss 0.0 m. Crew day (120 deg, earliest arrival 20 h): phase 55.10 m/s, 11 revolutions,
  total 137.1 m/s. Campaign 0-330 deg (48 h limit, 12 cases): 122.5-135.4 m/s, Waypoint 0 after 6.5-35.9 h, metric >=
  20.4, miss 0.0 m. Refusals: phase beyond the mission limit, without the reserve, chaser above the coelliptic height.
- **Phasing flights** (OEM ISS, `[session][validation]`, bounds Waypoint 0 <= 100 m and <= 0.5 m/s [E], arc outside the
  ellipsoid before approach initiation, never inside the keep-out sphere): 30 deg - boost 63.94 @+2.23 h, coelliptic
  57.57 @+2.98 h, approach initiation 1.69 @+5.78 h, mid-course skipped, braking 0.15 @+6.53 h; Waypoint 0 LVLH (-5.0,
  0.0, 396.9) m, 0.017 m/s, 539 kg. 150 deg with 1.5x truth air density: boost check error -75.9 km -> 1 re-plan; boost
  64.25 @+12.69 h, coelliptic 61.00, approach initiation 1.27, braking 0.08 @+17.00 h; Waypoint 0 (1.8, 0.1, 400.6) m,
  0.027 m/s, 576 kg. Crew day (120 deg, earliest 20 h) with 1.5x air: phase 55.06 @+0.50 h, boost 8.33 @+17.04 h,
  coelliptic 68.66, approach initiation 5.93, braking 0.13 @+21.35 h; Waypoint 0 (-7.9, 0.6, 399.9) m, 0.021 m/s, 603 kg.
  Real Demo-2 in-plane total ~118 m/s.
- **Proximity**, guidance alone (CW point mass, dt 0.05 s): IDA-2 62.8 min / 49.8 kg, IDA-3 98.8 min / 112.7 kg.
  Session campaigns (`[proximity]`, full model, 200 Hz, OEM ISS, 10-24 s each, not `[slow]`):

  | case | time | propellant | axial | lateral | offset | angle | rate |
  |---|---|---|---|---|---|---|---|
  | IDA-2 from Waypoint 0 (chain) | 62.3 min | 37.4 kg | 0.079 m/s | 0.0010 m/s | 0.052 m | 0.61 deg | 0.022 deg/s |
  | IDA-3 (3 fly-around legs, chain) | 99.2 min | 71.6 kg | 0.078 m/s | 0.0018 m/s | 0.035 m | 0.48 deg | - |
  | 8 dispersed Waypoint 2 starts | - | 1.7-2.0 kg | 0.078-0.080 | <= 0.0024 | 0.030-0.050 | <= 0.59 | <= 0.039 |

  From the i3 Waypoint 0 state in the braking attitude (180 deg off, drifting 0.02 m/s): aligned < 2 deg after 170 s,
  worst Waypoint 0 offset 1.16 m, contact inside, 24.4 kg. Station reboost 0.3 m/s at 600 s into the transfer: one
  warning, Waypoint 1 at 30.2 min (28.8 without), no abort. Waypoint 1 hold across `advance_to(+290 s)`: offset 1.20 m.
  Abort at x 9.6 m: breakout in 10 s, 0.291 m/s, passively safe, 24 h drift closest 285 m. Hold at x 11.4 m for 120 s:
  drift 0.20 m, 0.0019 m/s; resume -> contact inside. Breakout (45,0,0) -> 0.30 m/s +z, closest 222 m; full model 24 h
  closest after exit 236 m.
- **Docking** (`[docking]`; chains, grid and splashdown `[slow]`; time and kg of the session table above re-measured 2026-11-02 on the final tree, its axial to rate columns measured before Tasks 7 and 8, not re-measured):

  | case | result |
  |---|---|
  | IDA-2 from Waypoint 0 | contact 62.3 min, docked 10.2 min later, 37.4 kg, bending 95 N m (comp, shear: before Tasks 7 and 8, 928 N, 93 N, not re-measured) |
  | IDA-3 from Waypoint 0 | contact 98.8 min, docked 10.2 min later, 65.7 kg, comp 970 N, shear 88 N, bending 270 N m |
  | IDSS grid 2^(6-2) x 2 ports (axial, offset, angle, roll, lateral, rate) | 32/32 docked; max comp 1254 N, shear 631 N, bending 2055 N m, torsion 761 N m |
  | 0.30 m offset / 10 deg / 0.28 m/s | "Versatz zu groß" / "Winkel zu groß" / "Überlast" -> retreat to Waypoint 2 -> resume -> docked |
  | docked 24 h with a 0.3 m/s reboost | the pair moves together, port error < 1 mm |
  | undock -> departure -> capsule return | 0.0844 m/s, departure 4.9 kg, deorbit 58 h later (7-day pass search), splashed down, 1890 kg RCS left |

  Departure CW guidance: IDA-2 sphere left after 625 s, ellipsoid after 2300 s, 0.30 m/s, 6.1 kg; IDA-3 454 s / 1783 s, 6.2 kg (re-measured 2026-11-02, Task 9, final tree; was 473 s / 2290 s / 6.5 kg and 475 s / 1610 s with the old levers)
  (Demo-1 6.8 / 17 min with larger burns). Session IDA-2: departure complete 32.3 min after the springs, 4.7 kg (current)
  (limits 60 min, 60 kg; 33.3 min / 3.4 kg before the Dragon port moved from 4.0 to 4.22 m, 41.0 min / 5.0 kg
  after it, 40.3 min with the model-D station ports of Task 7, 32.3 min / 4.7 kg with the Task 8 clocking, Task 9 re-run). The step is not a lever effect:
  `breakout_delta_v` takes the smallest magnitude (0.3 m/s) and then, among 27 directions, the one with the largest
  minimum range after the exit; the leading candidates tie within < 1 m of ~220 m, so any small change of the start state
  can flip the pick (review emulation: 0.22 m flips 14 of 75 cases). Propellant per m/s differs ~4x by axis (+x 4.25,
  lateral 16.2 kg/(m/s)), the exit time follows the arc. This cause is the review's probable one, **not proven**: the
  direction chosen by the old run was not recorded. Departure time and propellant therefore move in steps with small
  changes of Dragon or the state; both limits hold.
- **Missions** (OEM fixture): `crew_dragon_iss` start (window + plan) < 0.1 s, hard capture at IDA-2 7.0 h after the
  window's liftoff, 583 kg RCS; IDA-3 at 40 deg on the synthetic orbit docked 9.2 h after the start; pad -> hard capture
  **7.5 h** after liftoff, 4 rendezvous burns, 0 re-plans, 612 kg. The flown ascent lands 0.0055 deg off the planned plane
  and 0.004 deg off the predicted phase (200.0 x 210.0 km). From 2026-10-16 00:00 UTC the first northerly crossing
  (+9.45 h) is out of reach in 28 h; the window is +33.05 h (bias 0.033 deg, Waypoint 0 5.5 h after liftoff). Departure:
  undocking 42.7 h after 2026-10-16 00:00, deorbit 16.10 h later as previewed, 629 kg RCS for departure + return.
  Tests `[launch_window]`, `[iss_mission]` (`test_iss_launch_window.cpp`, `test_iss_mission_config.cpp`,
  `test_iss_mission_sequence.cpp`, `test_iss_mission_campaign.cpp` with the `[slow]` end-to-end cases). Tags for the
  rest: `[iss]`, `[rendezvous]`, `[proximity]`, `[docking]`.

## Diagnostics
- `STARSIM_DEBUG_RENDEZVOUS=1`: planning exceptions, each burn's execution error (delta-v along/radial, position), the
  boost phase error, a status line per step (time, step, range, mass).
- `STARSIM_DEBUG_PROXIMITY=1`: guidance events, every 60 s port state, command, attitude error, propellant.
- `STARSIM_DEBUG_DOCKING=1`: mechanism and departure events, every second while ticked (or within 2 m of the port) the
  pose, force and peak compression.

## Open points and known limits
- Port clocking `up`: the class is [D] (model D), the member of the three positions model D allows is [E], read from photos (+-30 deg), two links unverified: see Port clocking. A direct source (IDA clocking drawing, a photo of the guide pins) would settle it. Why `up` moves propellant (+5.8 / +5.9 kg), IDA-2 chain bending and the departure time is open (LVLH-against-body hypothesis, not isolated).
- Collision shapes still lag the drawing (truss box half y 54.5, array boxes): the drawn Nauka, Prichal, Rassvet, Poisk, Node 3 etc. have no collision. Physics decision.
- A phase plan (with `earliest_arrival_s`) has no boost check: the coelliptic and approach burns absorb the phase
  orbit's drag error (~-34 km, ~+13 m/s at 1.5x air); a re-plan from the eccentric phase orbit cost more (635 kg: the
  boost leaves the perigee); a phase trim burn after drag tracking would be the fix.
- Manual proximity control is M7.
- US76 density vs NASA's model dominates the OEM along-track error (above).
- Petal lean: physics 30 deg [E, no source] against 45 deg in the drawn IDD (height 200 mm, radial 600 -> 400 mm). Physics
  unchanged. At 45 deg the capture radius would be 0.41 m (now 0.01 + 0.4 tan 30 = 0.24 m) and the axial force per
  lateral force 1.0 (now 0.58), so the axial load from off-axis petal contact is understated. Changing it needs a re-tune
  of `ring_lateral_range_m` (its comment rests on 0.24) and a docking campaign; the decision is the user's.
- Departure direction pick is a discrete selector (see Validation, departure): step changes of time and propellant.
- Station attitude is ideal; J2 short-period terms are not restored on the synthetic orbit; no structural model for
  collisions; later departure burns are not modelled.
- End state of the detail-model plan (2026-11-02): the port `up` is a photo-picked member [E, +-30 deg] of a measured class [D] (see "Port clocking"); IDA-3 aft vs forward rests on model D only; the Dragon port is 4.22 m [E]; the CG 38 m is unverified [E] (tally about 33 m); collision shapes lag the drawing; the Task 8 measuring scripts are not in the repo. Departure CW rows re-measured in Task 9: 625 s / 2300 s / 6.1 kg (IDA-2), 454 s / 1783 s / 6.2 kg (IDA-3), undock departure 32.3 min / 4.7 kg. Cost of the drawing (rings worst case): `vehicle-models.md` "Cost of the Falcon 9, Dragon and ISS models".
