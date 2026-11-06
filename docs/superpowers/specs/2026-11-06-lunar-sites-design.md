# Lunar landing sites anywhere - design

Status: draft 2026-11-06 (brainstorm with the user; builder check done, answers folded in; awaiting user review). Follows the Moon procedural surface plan
(archive `2026-11-04-moon-procedural-surface*`) and the i18n plan. Precedes M7 (cockpit).

## Intent

The user wants to land the HLS **anywhere on the Moon**, not only at the one polar site that exists today
(`mons_mouton_im2`). Later, in the M7 cockpit, the landing site is picked by clicking on the Moon in a graphical
planner; this plan delivers the **function underneath** so that the graphical planner only adds the picking UI.

The user gives the place; the planner answers **when** a landing there is possible and **how the Sun stands** then.
Apollo 11, 15 and 17 are reference / validation cases (look against real NASA photos, mission geometry against the
historical flights), not a fixed list.

## What exists today (for the builder)

- Terrain: global LOLA `LDEM_16` (16 px/deg, ~1.9 km, 32 MB) everywhere; one site tile from `LDEM_80S_20M`
  (20 m, 20 x 20 km, 2 MB) for `mons_mouton_im2`, cropped by one HTTP range request in `tools/data_pipeline/lunar_tile.py`
  via `fetch.py`. `environment::LunarTerrain` = tile inside, global grid outside, 1 km smoothstep blend.
  `config/lunar_sites.toml` is already a list; `set_lunar_landing_site` / `sim::load_lunar_terrain` / `sim::lunar_site`
  switch at run time. Details: `docs/subsystems/lunar.md`.
- Look: `data/textures/moon_color.png` = CGI Moon Kit WAC colour mosaic (4k, ~2.7 km/px), its large-scale brightness
  tied to the WAC Hapke w map (`tools/data_pipeline/moon_albedo.py`); lighting fully computed (Hapke, terrain shadows,
  earthshine); everything below ~2 km procedural (craters, Diviner-driven rocks, regolith). Details:
  `docs/subsystems/moon-surface.md`, `terrain-rendering.md`.
- Mission: LLO and the southern 9:2 NRHO are built around the polar site: the LLO waits for a pass of the site within
  `max_crossrange_km` (20 km) and trims the plane ~90 deg before it; budget `mission::lunar_budget`; return by TEI ->
  MCCs -> direct entry / aircatch -> catch at Starbase. Code: `core/src/mission/lunar_*.cpp`.
- Run-time downloads already exist in the game layer: `game/world/weather/weather_fetcher.gd` (HTTPRequest, Range,
  `.part` + rename, injectable for tests), orchestration in `weather_service.gd`, never blocking the frame. The core
  stays network-free.
- Mission selection UI: `game/ui/mission_catalogue.gd` (the existing mission wizard / catalogue).

## Goals

1. **Free site:** any latitude / longitude on the Moon (MOON_ME, the LOLA frame) becomes a landing site. Named sites in
   `config/lunar_sites.toml` remain as presets (Apollo 11, 15, 17 added; Mons Mouton kept).
2. **Terrain, three layers** (user: "erstmal mit 60 Metern, kann man spaeter verfeinern"):
   - Global, shipped: replace `LDEM_16` by **`LDEM_64`** (~470 m) - orbit view, approach, fallback everywhere.
   - Per site, downloaded on selection and cached: **60 m height tile, ~150 x 150 km** [builder may adjust size with
     reason: it must cover the view from ~15 km altitude down to touchdown]. Source 60 S - 60 N: **SLDEM2015**
     (LOLA + Kaguya; the pure LOLA 512 ppd grid shows track striping near the equator). Poleward: LOLA polar products
     (20 m at 80-90 deg as today); which product covers 60-80 deg at <= 60 m is for the builder to find (fallback: the
     best available, documented).
   - Below 60 m: procedural as today.
3. **Look:** per site an **albedo tile at ~100 m** from an **illumination-normalised** product (e.g. the LROC WAC
   Hapke-normalised 643 nm mosaic) - brightness only, no baked shading - blended into the global colour mosaic with the
   same large-scale calibration as today. Shows rays, dark halos, mare / highland boundaries at the site.
   High-resolution photos (LROC NAC, Apollo surface / orbital photos) are **comparison references only**, never rendered.
4. **Download:** the game may go online when a site is chosen (user: "er kann ruhig online gehen"). Height + albedo
   tiles (~20 MB) are fetched by range requests in the game layer (pattern of `weather_fetcher.gd`), with progress,
   cached on disk (`data/terrain/sites/` or similar), reused on the next selection. Apollo 11/15/17 and Mons Mouton
   tiles are prepared by the data pipeline and shipped so tests and the reference cases run offline.
5. **Landing windows:** given a site, the planner returns a **small list (3-5) of possible landing dates** in roughly
   the next 3 months from the chosen start epoch, each with: landing epoch (UTC at the UI), Sun elevation and azimuth at
   touchdown, daylight remaining after touchdown, propellant reserve (outbound + surface + return closed). Dates that
   do not work are not listed; if none works, the answer gives the reason (budget, Sun, orbit geometry).
   Sun rule [my call]: hard condition = Sun above the local horizon at touchdown and daylight covering the surface stay;
   the 5-20 deg band (Apollo-like low Sun shows relief, config) is a **preference** that ranks the dates, not a filter -
   near the poles the Sun never climbs above ~7 deg, so a hard band would exclude them.
   Fidelity [my call]: candidates may be screened analytically, but every listed date is backed by the real mission
   planners (the same ones the flight then uses), so a listed date does not fail in the flight. List order: by
   preference, ties chronological; the first 3-5 that pass.
   A site whose budget does not close at any date is a valid result: "green" for hard sites (far side, high latitude)
   means the planner returns windows that fly, or a refusal with the correct reason - never tuning (see 6).
   **Surface stay is chosen by the user** per mission (user 2026-11-06: "mit Aufenthaltsdauer"), default from the
   mission config (`stay_hours`, today 24 h); the window search checks daylight for the whole stay and the ascent /
   return after it, so the list depends on (site, stay, start epoch).
6. **Mission to any site:** the LLO is no standard orbit: its **inclination and plane are free per site and window**
   (user 2026-11-06: "die Neigung des Orbits kann frei gewaehlt werden, je nach Landeplatz"); chosen (from LOI on) so that the ground
   track passes the site at the chosen window, accounting for lunar rotation; descent and landing work at any
   latitude; ascent back to that LLO, TEI and the existing return to Starbase. The NRHO variant stays allowed only where
   its budget closes (in practice near the south pole); elsewhere the planner says it is not reachable from the NRHO and
   why. Standing rule (user): the planner reads the vehicle config, no tuning to make a site work.
7. **Interim UI (until M7):** in the existing mission wizard: preset list + free latitude / longitude entry + stay ->
   download progress -> window list -> pick one -> start. All texts via `config/i18n/{de,en}.toml`; core results as
   report codes. The M7 graphical planner later calls the same core / service functions.
8. **Validation:** Apollo 11 (Mare Tranquillitatis), 15 (Hadley-Apennine), 17 (Taurus-Littrow): landing campaigns at
   each site green; renders at the historical Sun elevation / azimuth compared side by side with Apollo photos
   (orbital / LM window) and LROC NAC images (look references, recorded in `sources.toml`, not shipped); km-scale
   relief judged against the photos. Plus at least one far-side and one high-northern-latitude site in the window
   search and campaign tests.

## Non-goals

- The graphical (click-on-the-Moon) planner - M7.
- Real terrain finer than 60 m away from the polar 20 m tiles (LROC NAC DTMs etc.) - later refinement.
- Communications / far-side relay constraints.
- Hazard detection or site-safety assessment beyond what the landing guidance already does.

## Anti-goals (would make it a failure even if it "works")

- Baked shading from satellite photos in the rendered surface (double / wrong shadows when the Sun moves).
- Networking in `core`; a blocking frame during download or window search.
- A hard-coded site list as the only way to choose.
- A planner that silently refuses or silently hops epochs: every refusal has a stated reason.
- Mission parameters tuned per site to make a test green.

## Constraints

- Architecture and conventions of `CLAUDE.md` (core float64, frame / unit in names, report codes, no magic numbers,
  configs not code, licence of every new source in `tools/data_pipeline/sources.toml`, CREDITS regenerated).
- Free data only (NASA PDS / LROC products are public domain - verify per product).
- Site switch at run time rebuilds terrain + detailed surface without a restart (~1-2 s acceptable).
- Window search runs off the main thread with progress; the result list is cached per (site, start epoch, vehicle).
- `LDEM_64` (~530 MB raw int16) is shipped as a data part, not in the base installer [my call].
- Offline and site not cached: the selection is refused with a clear message and retry [my call: the landing terrain
  must be the real 60 m tile, not the 470 m fallback].

- Per window the whole outbound geometry may change (TLI epoch, LOI plane, LLO plane) [my call]; the ascent must
  meet an LLO plane that passes the site at lift-off (Moon rotation during the stay included) - plane change in LLO
  allowed when the budget closes.
- Packaging [my call]: Apollo 11/15/17 + Mons Mouton tiles (~20 MB each) ship with the base data; `LDEM_64` is a data
  part and may be stored in a compressed / downsampled-for-far-LOD runtime format; tests needing it skip with a clear
  message when it is missing (as the Diviner map does). 60-80 deg band: best available product, documented, is accepted.

## Left to the builder

- Exact data products / URLs for the 60-80 deg band and the normalised albedo mosaic (criteria above), tile file
  format, cache layout, blending widths between tile and global data.
- How the LLO plane / LOI targeting is generalised (Apollo-style free-return / LOI plane targeting is the reference
  idea), the window search algorithm, and its time budget.
- Splitting into tasks; test tags per touched subsystem; which existing lunar tests need re-baselining (with reasons).
