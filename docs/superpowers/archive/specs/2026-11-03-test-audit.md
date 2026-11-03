# Test suite audit (2026-11-03) - list only, nothing changed

Saved by the controller from the audit agent's hand-back (the agent could not write .md). Data files beside it: inventory.csv, classified.csv (category per case), tagsets.md, flags.txt, planner_constants*.txt, planner_fitted.txt, godot_scripts.txt, untested_units.txt, w_checks.txt, pins.txt, e_list.txt.

## Totals
1529 ctest cases, 1518 green, 20 382 s CPU, 1954 s wall (ctest -j 24, times inflated by load). Wall is almost all #466 "Flown campaign: the depot launched" (1941 s); without it ~1050 s. CPU by tag: slow 15 280 (121), validation 8735, lunar 6892, return 5796, refuelling 4731, sim 3077, mission 3011. Slowest file: test_lunar_return_campaign.cpp 4963 s (aerocapture 3 x ~1040 s). 130 of 222 files cost < 5 s together. 38 hidden `[.]` cases not in ctest.

## Categories
| Cat | cases | CPU now | CPU after |
|---|---|---|---|
| A physics / limit / validation | 1114 | 6309 | 5215 |
| A-func refusals / config / bookkeeping | 338 | 1190 | 1190 |
| B own-value pins | 7 | 33 | 33 |
| C overlap | 18 | 1917 | 233 |
| D dead / vacuous | 0 (5 hidden `[.]` candidates) | - | - |
| E dispersed campaigns | 41 | 10 690 | 3927 |
| W webcast agreement | 11 | 242 | 112 |
A and A-func sampled by file, not one by one.

## W (user 2026-11-03: report, not gate; physical gate kept)
- ship_entry_telemetry:225 (32 s, red #1147): speed/low rms, saturation, subsonic vs flights 12-14. Params: entry mass (255 vs ~175 t), CG, crossflow Cd, flap CdA. Keep: no impact, trim not saturated.
- starship_ascent:82 (93 s): bands vs flight 14. Params: liftoff mass, Raptor thrust, throttle, drag. Keep: orbit 165x276 km 30.5 deg.
- falcon9_ascent:218 (4 s) vs CRS-35. Keep: feasible, LZ recovery.
- descent_telemetry:78, falcon9_descent_telemetry:78: rms vs flights. Keep: none.
- booster_return:239 (50 s): boostback 152+-5 s, apogee, ignition vs flight 14. Keep: FAA residual, plan vs flown propellant < 3 t.
- falcon9_rtls:270 (19 s): entry/landing times, sink, 12.1 t, flip 13.7 s, KNOWN MISS pins. Keep: orbit, landed upright on LZ-40.
- falcon9_asds:480 (13 s), :268 (1.4 s, 13.2+-1 t gauge). Keep: orbit, on deck; engine set, dv band, impact < 300 m.
- staging_sequence:151 (12 s): boostback start 13.7+-3 s. Keep: flips on RCS, engines off.
- capsule_splashdown:393 (17 s): chute heights/speeds, 54 min vs Crew-10 +-15 %. Keep: splashed, < 2 km, peak < 3.6 g.

## B
refuelling_campaign:65 pins 69.3/73.4/202.9 t (trim to dry-mass match); :214 digest guard (keep while file exists); refuelling_campaign_flight:497/:539 boil-off factors (keep); entry_skip:145 bank 77.5+-0.01 deg (drop pin); docking_mechanism:142 KNOWN STATE (report); plan_speed reference plans (keep, red).

## C (saving estimates)
mission_start:69 (369 s) = mission_timeline:760/:800 (-369); mission_timeline:482 = :451, placed orbit (-90); :564 from hand-over state (-20); staging_sequence:205 from placed MECO (-110); ascent_profile_planner:227 memo with F9 request (-177); lunar_return_session:171 = direct seeds (-78); proximity_session:184/:189 = docking_campaign:84/:100 = iss_mission_campaign:48/:75, one per port (-62); crew_dragon:435 = iss_mission_campaign:178 (-43); refuelling_missions:586/:694/:1078 = :710 + #466, nightly (-593); refuelling_missions:95/:127 placed caught booster (-88); ship_return_mission:250 uncertain; atmosphere_tables:79 = atmosphere_scattering:441 small grid (-50).

## D hidden candidates
ascent_guidance:415 [.tuning] pitch kick; falcon9_boostback_dump:27 (webcast fit); refuelling_missions:784; ignition_settling:580; 3 timeline probes.

## E
Lunar return aerocapture 21 seeds 3112 s; direct 21 seeds 1851 s (seeds 1-3 identical TEI 881.3 and braking 1110 m/s: dispersion matters from the interface); flown refuelling #466/#464 2571 s; ship return 862; lunar landing 423; HLS outbound 348 (two families); F9 ASDS 365, entry guidance 276, F9 flip 201, docking grid 175, SCvx catch 145, ship catch 83, Dragon return 138, capsule entry 40, F9 RTLS 72 (untagged), proximity 24, phasing 5.

## Flags
- 32 cases > 40 s without [slow] (list in flags.txt; mission_start:69 369 s, ascent_profile_planner x6, atmosphere x4, cloud_optics x4, ...). 18 [slow] cases < 5 s.
- Ascent planner runs inside cases tagged [mission][sim], [timeline], [staging], [dragon]: a planner change misses them by tag.
- No tag covers a vehicle-config change.
- build.ps1 treats `-TestFilter "[gnc][validation][ascent]"` as a UNION: 260 cases ~1044 s instead of the 5-case intersection (~93 s).
- Possible shared temp file: entry_guidance_campaign.cpp:50 crossflow_<index>.csv (:173, :209), unverified.

## Godot / pytest
55 scripts in game/tests; only smoke_test.gd is run by a tool (package.ps1:139). Never referenced or run: clock_hold_test, free_camera_run, local_time_test, trajectory_view_test, ui_rate_test, water_waves_test, weather_fetcher_test. Proposal: runner for every *_test.gd headless when game/ changes. pytest: 31 files, ~287 functions, untimed.

## Gaps
Low-propellant ship abort (no test); gnc/booster_return_prediction.cpp and booster_return_burns.cpp (no unit test); sim/predicted_events.cpp only via [timeline]; weather_decode real-sample reference unclear; F9 landing outcome has no pad rule.

## Recommendation
1. #466 and #464 nightly: full wall 1954 -> ~450 s.
2. Lunar return: 4 direct seeds (2, 12, 15, 18) + 3 aerocapture per change, all 42 nightly; better: fly TEI + coast once, disperse from the interface (~45 s/seed), once shown that TEI dispersion is absorbed.
3. Ascent plans in tests (est. 1500-2000 s CPU): plan cache kept across relinks keyed on planner sources + configs, or ctest FIXTURES_SETUP.
4. Atmosphere convergence (342) and full resolution (198) nightly; small grid otherwise.
5. Refuelling :586, :694, :1078 nightly (-593 s).
6. F9 flip campaign split in 3 cases: [falcon9] 201 -> ~125 s.
7. tanker_flight :710 (335 s) then sets the wall of [docking], [ship], [mission], [refuelling]; split at the docking ~180 s (uncertain).
Tiers: 1 per change = tags without [slow] (10-100 s); 2 [slow] reduced seeds before push/review (<= ~335 s per tag); 3 new [nightly] label, excluded unless asked (~2000 s background: full campaigns, W reports, atmosphere convergence, kick sweep).
Per tag after: [lunar]/[return] 1044 -> 160 s, [refuelling] 1941 -> 335, [atmosphere] 341 -> 98, [sim] 369 -> 148, [ascent_planner] 365 -> 148, [timeline] 96 -> 42, full CPU 20 382 -> ~10 700.

## Planner audit (vehicle values in code or fitted, should follow config/vehicles)
Booster preset config-driven (booster_return.cpp:66-120); the ship's is not.
- ship_landing.cpp:49-52 engine groups and 3->2->2 phases in code (follow engine layout config); :54 reserve 1000 kg (header tank / unusable); :66 lateral effort 0.03 tuned on lunar seed 18 (actuator delay, inertia, gimbal); :70-71 tilt 20/20/15 deg, rate 0.3/0.2/0.2 (gimbal, thrust, inertia); :72-74 3->2 switch at 20-40 m/s from webcast [D] (thrust at min throttle vs weight).
- ship_landing.hpp:19-37 flip throttle 0.8, tolerance 15 deg, settled rate 0.2, flaps until 30 deg, gate lead 220 m, two-engine sink 20 m/s [D], margin 1.3.
- entry_guidance.hpp:33-55 bank rates 1/3 deg/s, max q 25 kPa, terminal Mach 5/3, min alpha 50, alpha authority 20, slew 3 deg/s, tuned on b1-b4 (q limit, flap authority, trim range, roll inertia).
- booster_return.hpp:24-25 flight time 230 s, flip 10 s default used by Super Heavy (inertia, RCS / gimbal torque).
- falcon9_block5.toml flip_duration_s, boostback_loss_fraction fitted [D].
- scvx_landing.hpp:46-50 throttle 0.35 "13 engines", divert 200 m/s / 0.4, switch 22 m/s, final 0.4: defaults for every vehicle without a recovery profile.
- booster_return_detail.hpp:27 alpha <= 8 deg (grid fin config); :39-42 leg deploy 5 s, touchdown tilt 4 deg (leg config).
- lunar_landing_guidance.cpp:141-143 guess {50,10,30} s, lateral effort 0.05; lunar_ascent_guidance.hpp:35-45 Raptor start 30 m, rise 150 m, abort margin 0.15 (HLS data).
- ascent_profile_planner.cpp:53 provisional downrange 600 km (ballistic estimate).
- flight_session_capsule.cpp:49-50 Dragon bank 30 deg, crossrange 40 km (capsule L/D); deorbit_planner.hpp:21 crossrange 60 km from b2 (ship L/D).
- proximity_geometry.hpp:94 / phasing_schedule.hpp:24 reserves 600/900 kg (deorbit plan).
- tanker_reserve.hpp:9-16 O/F 3.6, margin 1.10, boil-off 2000 kg, fuel share 0.56 [D] from r5, unusable 1000 kg.
- refuelling_campaign.hpp:48 vent 1.5 %; config/refuelling/reference_flights.toml measured on own flights (compute from configs).
- iss_launch_window.hpp:60-61 and mission insertion_s calibrated from a flown ascent [D].
- lunar_return_mission.cpp:35-36 aerocapture raise 120 m/s, deorbit 100 m/s in code.
Struct defaults may be overwritten by callers not found.
