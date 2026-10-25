# Guidance, vehicles and missions (index)

History and the reasons behind superseded decisions: `docs/history/guidance-missions.md`.

Split by topic; read only the file a task needs:
- `docs/subsystems/gnc-core.md` - shared GNC: ascent guidance basics, catch tower, booster return, SCvx landing
  planner and aborts, grid fins, RCS (allocator, modulator, propellant), HUD / tower view keys, booster diagnostics.
- `docs/subsystems/ship-entry-return.md` - ship entry (flaps, corridor, settling, guidance), flip and catch, upright
  abort, gate lead, ship return from orbit (deorbit planner), with validation campaigns and ship diagnostics.
- `docs/subsystems/falcon9.md` - Falcon 9 vehicle data, landing legs, RTLS, drone ship, game interface.
- `docs/subsystems/crew-dragon.md` - Crew Dragon vehicle (d2), entry (d3), parachutes and splashdown (d4), game
  interface (d5).
- `docs/subsystems/lunar.md` - lunar missions l1-l6: reference bodies, HLS and terrain, outbound (NRHO / LLO),
  landing and ascent, return to Earth (TEI, braking, aerocapture, entry heating), game interface and simulation thread.

Related: `ascent-staging.md`, `vehicle-data.md`, `earth-terrain.md`, `iss.md`, `refuelling.md`, `crew-escape.md`,
`mission-timeline.md`.
