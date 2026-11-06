# Bilingual simulator: status codes from the core, all texts in config (design)

Date 2026-11-06. User request: "der Simulator-Kern sollte nur Fehlercodes zurückgeben. Der Simulator sollte zweisprachig
werden. Alle Texte in eine config. Deutsch/Englisch zentral umschaltbar. Diesen Umbau bitte unbedingt ohne die
aufwendigen Simulatortests." Design agreed in chat 2026-11-06 ("passt").

## Intent
Clean separation: the C++ core computes and reports *what* happened (a code plus numbers); the game decides *how* to
say it, in German or English. Every user-visible text lives in one place in `config/`, so translations and wording
changes never touch code. The language is switched centrally and takes effect immediately.

## Goals
- The core returns no user-visible text: refusals, reasons, statuses, phase names, warnings and outcome texts become
  codes with numeric parameters. (Today: ~450-500 German literals across ~26 core files, free `std::string` fields
  `reason` / `status` / `name` / `message`, phase-to-string switches, `snprintf` texts with numbers.)
- All user-visible texts (game UI, HUD, loading screen, events, notices, wizard, webcast gauge labels, vehicle /
  mission / site / place names) in text tables in `config/`, one per language: German and English.
- One saved language setting, switchable live; German default.
- Bilingual from day one: every key has a German and an English text.

## Non-goals
- The future start menu (it will later bind to the same setting; the temporary switch sits in the time panel).
- Docs stay as they are (English repo docs; `docs/ansichten.md` stays German, no English copy).
- Developer-facing texts are not translated: config-file validation errors (`ConfigError`, `doc.fail`), internal
  `runtime_error`s for programming errors, debug prints and logs. They are written in English. Exception: errors the
  player can meet in normal use (e.g. missing terrain data -> "fetch.py") become codes.
- The installer (already German/English via Inno Setup `[CustomMessages]`), README files, CREDITS, dev tools in
  `tools/`, test files.
- No new languages beyond German and English (the design must not prevent a third).

## Anti-goals
- Physics or behaviour changes. This is a pure refactor of how outcomes are reported; numbers, decisions and timing
  stay bit-identical.
- Running the expensive simulator tests (user: "unbedingt ohne"). See Tests.
- A text appearing empty or as a raw key in normal use.

## Decided (user, 2026-11-06)
- The language is a saved setting, not a panel feature: `[ui] language` in `user://settings.cfg` (the existing
  `ConfigFile` used by `earth_near_shadows` / `moon_near_shadows` / weather). Values `"de"`, `"en"`; default `"de"`.
- Temporary control: a switch "Sprache: Deutsch / English" in the time panel (`game/ui/time_panel.gd`), next to the
  other saved settings. The later start menu replaces it by binding to the same setting.
- One central language service in the game loads the tables from `config/` and returns the text for a key in the
  active language; on a switch it emits a signal and every display redraws - no restart.
- Core reports code + numbers, e.g. "landing site too steep" with slope 8.3 deg and limit 8.0 deg; the game builds
  the sentence from the table.
- Missing English text -> German text; missing key in both -> the key itself (visible, never empty) and a log
  warning. A check asserts every code / key has both texts.
- Webcast gauge labels, vehicle / mission / site names are texts too and move into the tables.

## Shape (builder's latitude inside these lines)
### Core
- A small bottom-level core module (no dependencies, e.g. `core/include/starsim/report/`) defines the report type:
  a code (stable, ASCII, dotted id such as `lunar.site_too_steep`, enumerated in C++ so the compiler checks use) plus
  named numeric parameters, each with its physical kind (angle, mass, length, speed, time, count, plain number) and
  its value in SI. No text, no formatting, no units conversion in the core.
- Every former text field becomes this type (`reason`, `status`, `refusal`, `burn_abort_reason`, `breakup_reason`,
  `engage_pad_ascent`'s returned string, waypoint and leg names, ...). Empty-string-means-ok becomes an explicit ok.
- Phase names: the core exposes the phase enums (they exist: `DockingPhase`, `AscentPhase`, `ReturnPhase`, ...);
  the game maps them to keys. The phase-to-German switch functions go away.
- Config `name` fields read by the core (`lunar_site_config`, `splashdown_zone_config`, ...) are no longer display
  text; the core carries ids. CLAUDE.md's "UI strings in the core are German" becomes "the core returns codes, no UI
  text".
- The code list is the contract: one table in the core lists every code with its parameter names, exported so a check
  can compare it with the text tables.

### Bindings
- Pass codes and parameters to GDScript as plain data (e.g. `{code: "lunar.site_too_steep", params: {slope:
  0.1449, limit: 0.1396}, kinds: {...}}`); no text composed in the bindings (`flight_sim.cpp` composes a few today).
  Dictionary key names stay as they are where possible; command results `{done, ok, message}` and the
  `command_finished` signal carry the report instead of a German message.

### Text tables (`config/i18n/`)
- One TOML file per language (`de.toml`, `en.toml`), flat dotted keys, values with named placeholders
  (`"Landeplatz zu steil: {slope} (Grenze {limit})"`). Grouped by area (ui, hud, loading, event, report codes,
  phase, gauge, vehicle, mission, site, place).
- Config display names move out of vehicle / mission / site / place / gauge files into the tables, keyed by the
  existing ids (`mission.<id>.name`, `mission.<id>.display_name`, `vehicle.<id>.name`, ...). The data files keep
  ids and physics only. Proper nouns (e.g. "Mons Mouton (IM-2)") may be identical in both languages.
- Trademarks rule (CLAUDE.md) unchanged: display names come from config, no "Starship"/"SpaceX" in shown text.

### Game
- The language service is an autoload: text-for-key with parameters, current language, change signal, load/save of
  the setting. Parameter formatting is the service's job: unit and precision per physical kind (deg, t, km, m/s, s
  as today's displays use them) and the locale's decimal separator (German comma, English point).
- Every displayed text goes through the service; texts built in code (HUD rows, events, loading steps, wizard,
  notices, panel buttons and tooltips) use keys. HUD rows separate label and value so the column layout survives
  both languages.
- Live switch: every display that shows text listens to the change signal and redraws (labels set once at build time
  must be re-set).

## Tests (user rule for this refactor: no expensive simulator tests)
- Allowed: builds (core, bindings), the fast unit tests of the touched core modules (`-Fast` with the touched modules'
  tags, never `[slow]`, never `-Nightly`, no campaigns), the smoke test, small new checks.
- New checks: (1) every code in the core's code table has a key in both language tables with matching placeholder
  names; (2) every key in `de.toml` exists in `en.toml` and vice versa, placeholders match; (3) a headless game check
  that switching the setting changes a shown text and saves the setting; (4) a scan that fails on non-ASCII string
  literals in `core/src` / `core/include` (outside comments) so German UI text cannot come back.
- Because physics must not change, the refactor is reviewed for unchanged numbers and decisions (code paths that
  decided a refusal still decide it identically), not re-measured by campaigns.

## Controller's calls after the builder check (2026-11-06; open for the user's review)
1. Parameters: numeric (kind + SI value) and reference parameters (an id plus its table namespace, e.g. site
   `mons_mouton` -> `site.mons_mouton.name`, phase enum -> `phase.<enum>`); the service resolves references inside a
   sentence. No free text parameters.
2. Text-dependent logic: the plan starts with an audit of every place that compares, concatenates, parses or
   forwards these strings (core, bindings, game; e.g. `reason.empty()`, prefix concatenation in
   `flight_controller.gd:148` / `time_panel.gd:315`). Each becomes an explicit ok/code check; the audit list is part
   of the review evidence for "decisions unchanged".
3. Variable structure: a report holds one code; where several reasons/warnings exist today it holds a list of
   reports. No grammar engine: plurals, optional clauses and "A or B" become separate keys/codes.
4. Migration: the report type first; then per core area one task converts the core fields, the bindings and the game
   consumers together, and adds the German text (today's wording) and the English text to the tables in the same
   task - every commit builds and runs. Revised after planning (2026-11-06): statuses are glued as text across areas
   (e.g. the ISS status from rendezvous/proximity/docking/transfer texts), so a temporary `legacy` code with one text
   parameter bridges unconverted producers; consumers convert before producers; the bridge is deleted in the last task,
   where a build failure exposes any leftover.
5. Formatting: one default unit/precision per kind, overridable per placeholder in the table (`{slope:deg1}`,
   `{alt:km2}`) so today's displays keep their units and decimals; the placeholder check compares names, not formats.
6. Scope in game/: every user-visible literal in game/ (GDScript, .tscn, shaders' on-screen text if any) moves; a
   GDScript-side scan (non-ASCII and a German-word allowlist-based check of string literals in non-test .gd/.tscn)
   joins check 4.
7. Config: `name` / `display_name` / `label` fields leave the data files; the core loaders stop reading them (an old
   file with them still loads - unknown display fields ignored with no error). Provenance tags `[S]/[E]/[D]` stay
   where they are. If a stored format (telemetry log, plan cache) holds texts, it switches to codes with a version
   bump; old caches are rebuilt, not migrated.
8. Existing unit tests that assert German text are rewritten to assert codes (and parameters) - part of each area's
   task.
9. Reports are data; text is resolved only when displayed and re-resolved on a language switch. The language service
   autoload loads before the loading screen, so no buffering is needed.
10. TOML in GDScript: the existing `TomlLite` reader.

## Left to the builder
- Exact module / type names, the code naming scheme within the dotted-id rule, how parameters are stored.
- Whether the text tables are one file per language or split per area (one place per language is required).
- Task order and batching (e.g. core report type first, then module by module, then bindings, then game, then config
  names), as long as every intermediate commit builds and the game runs.
- English wording: natural, short, same tone as the German; space terms as in NASA/ESA usage.

## Inventory
`.superpowers/sdd/i18n-inventory.md` (2026-11-06 survey; counts are grep estimates).
