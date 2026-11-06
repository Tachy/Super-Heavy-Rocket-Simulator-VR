# Status Codes from the Core, Bilingual Texts in Config Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The C++ core reports outcomes as codes with SI parameters instead of German text, and every user-visible text
comes from `config/i18n/de.toml` / `en.toml` through one game language service that can be switched live (German default).

**Architecture:** A new bottom-level core module `report` (code enum generated from one X-macro list, parameters with a
physical kind and an SI value, nested causes, a `Refusal` exception). The bindings turn reports into plain Dictionaries
and phase enums into qualified ASCII ids. A static GDScript class `I18n` loads the tables with `TomlLite`, formats
parameters per kind and locale, saves `[ui] language` and emits a change signal. The conversion runs top-down (consumers
before the producers they embed) per feature chain; a transitional `legacy` code bridges not-yet-converted strings and
is deleted in the last task.

**Tech Stack:** C++20 core (MSVC, Catch2, toml++ in tests), godot-cpp bindings, Godot 4 GDScript, Python 3.13 (pytest,
tomllib) for the table and literal checks.

**Spec:** `docs/superpowers/specs/2026-11-06-i18n-error-codes-design.md` (inventory: `.superpowers/sdd/i18n-inventory.md`,
grep estimates - each task verifies its own area).

## Global Constraints

- **Tests (user rule for this refactor, "unbedingt ohne die aufwendigen Simulatortests"):** per task ONLY builds
  (core, bindings), `tools/build.ps1 -Fast -TestFilter "<tags>"` with the tags of the touched modules / edited test
  files, the smoke test, the new small checks (`python tools/i18n_check.py`, `python -m pytest tools/test_i18n_check.py`,
  `game/tests/i18n_test.gd`) and the game unit test scripts the task edits. NEVER `[slow]` (always `-Fast`), NEVER
  `-Nightly`, no campaigns (`[campaign_flown]`, flown refuelling / ISS / lunar campaigns), no `[validation]` filters, no
  render checks. This overrides CLAUDE.md's "same tags with `[slow]` before a push". Run `-ListOnly` first; if a filter
  matches more than ~150 cases or a campaign, narrow it by intersection (`[iss][docking]`).
- **Physics and decisions bit-identical:** no number, threshold, branch, timing or order of evaluation changes. A
  condition that tested a string (`reason.empty()`, `rfind("Gasansaugung")`, `== "aus"`) becomes the equivalent code
  test (`r.ok()`, `r.code == Code::...`, `== "autopilot_state.off"`) deciding exactly the same cases.
- **Text-logic audit per task:** before editing, run the audit greps on the task's files and list every hit in the task
  report as `file:line old condition -> new condition` (review evidence for "decisions unchanged"):
  `rg -n "(reason|status|refusal|message|warning|note)\w*(\(\))?\s*(\.empty\(\)|==|!=|\.find|\.rfind|\.starts_with|\.compare|\+\s*\")" <files>`,
  `rg -n "to_string\(|german_name\(" <files>`, and in GDScript `rg -n '(==|!=| in )\s*\[?"' <files>`.
- **Core returns codes, no UI text** (new CLAUDE.md rule, Task 1). Core / bindings string literals are ASCII; developer
  texts (config validation `ConfigError` / `doc.fail`, internal `runtime_error`s, logs, printf diagnostics) are English.
  Player-facing errors in normal use (missing terrain data -> fetch.py) become codes.
- **Code ids:** dotted lowercase ASCII `<area>.<what>` (`lunar.site_too_steep`), regex `^[a-z][a-z0-9_]*(\.[a-z0-9_]+)*$`;
  the C++ enumerator is the id with `.` -> `_` (`Code::lunar_site_too_steep`); one line per code in
  `core/include/starsim/report/codes.def`, grouped by area comments, parameter names lowercase snake, comma-separated.
  Plurals, optional clauses and "A or B" are separate codes. A code that renders a nested report lists `cause`.
- **Parameter kinds and SI:** `angle` (rad), `mass` (kg), `length` (m), `speed` (m/s), `time` (s), `count`, `number`,
  `ref` (a table key, resolved by the game), transitional `legacy_text`. No free-text parameters otherwise.
- **Phase / state ids:** every enum shown to the user gets `const char* id_of(Enum)` returning the snake_case of the
  enumerator name (`AutopilotState::UpperStage` -> `"upper_stage"`), written as a `switch` with one
  `case Enum::X: return "x";` per line, the definition line carrying `// i18n-ns: <snake_case enum type name>`
  (duplicated type names get the namespace as prefix, `gnc_return_phase`). Bindings send the qualified id
  `"<ns>.<id>"`; the table key is `phase.<ns>.<id>`. The German `to_string` switches go away.
- **Text tables:** `config/i18n/de.toml`, `config/i18n/en.toml`. Sections are the groups `report`, `phase`, `ui`, `hud`,
  `loading`, `event`, `notice`, `wizard`, `gauge`, `vehicle`, `mission`, `site`, `place`, `lunar_site`, `zone`,
  `drone_ship`; keys inside are dotted (`[report]` `lunar.site_too_steep = "..."` is the key `report.lunar.site_too_steep`).
  `TomlLite` subset only: one-line basic strings, no escape sequences, no `"` inside a value (use „…“ / '…'), no key that
  is both a leaf and a prefix of another key. Every key exists in both files. German text = today's wording; English
  natural, short, NASA/ESA usage; no "Starship" / "SpaceX" in shown text. Proper nouns may be identical.
- **Placeholders:** `{name}` or `{name:spec}`; spec = unit + decimals: `deg<n>` (rad -> °, no space), `t<n>`, `kg<n>`,
  `km<n>`, `m<n>`, `mps<n>` (" m/s"), `kmh<n>`, `s<n>`, `min<n>`, `h<n>`, `pct<n>` (fraction -> %), `n<n>`, `int`, `clock`
  (mm:ss, h:mm:ss from 1 h). Defaults per kind: angle `deg1`, mass `t1`, length `km1`, speed `mps1`, time `s0`, count
  `int`, number `n2`. Decimal separator: `,` in German, `.` in English. NaN / inf render as `–`. The checks compare
  placeholder names, not specs.
- **Fallback:** missing English text -> German text; missing in both -> the key itself plus one `push_warning` per key;
  never an empty string (an `ok` report renders as "").
- **Setting:** `[ui] language` in `user://settings.cfg` (the existing `ConfigFile`), values `"de"` / `"en"`, default
  `"de"`; an unknown value reads as `"de"`.
- **Bindings report shape:** `{"code": String, "params": {name: float | String}, "kinds": {name: String}, "causes": Array}`;
  ok = `{"code": "ok", ...empty}`. No text composed in the bindings. Dictionary key names stay; only `message` becomes
  `report`: command results `{done, ok, report}`, signal `command_finished(id: int, name: String, ok: bool, report: Dictionary)`.
- **Transitional `legacy` (deleted in Task 16):** code `legacy` with one `legacy_text` parameter `text`, table text
  `"{text}"`. Allowed only (a) in the bindings for a core string not yet converted, (b) in the core where a converted
  report embeds a not-yet-converted producer string as `cause`. The task that converts a producer removes every
  `legacy` use of it. A producer may only be converted when all its string-concatenating consumers are converted
  (hence the task order).
- **Workflow (CLAUDE.md):** Conventional Commits, commit on `main` and `git push origin main` after each task once its
  allowed tests are green; never git worktrees, junctions or symlinks; read files in excerpts; long output to files.
  Subagents on Sonnet; the plan's final review on Opus.

## Review Focus

1. Live switch while a display already shows an older report (time panel start refusal, HUD event, loading step): it
   must re-render in the new language, not stay German - reports are stored as data and resolved at draw time (Task 2
   `i18n_test` bound-label + stored-report case; Task 14 HUD event case).
2. A code / phase id without table text (new enumerator, new code): visible key + warning, never empty, and the checks
   fail in the same task (Task 1 pytest `test_code_without_text_fails`, Task 3 `test_phase_id_without_key_fails`,
   Task 2 missing-key case).
3. Odd numbers in parameters (NaN, inf, negative zero, huge times): `–`, no "nan"/"inf", clock beyond 24 h
   (Task 2 `i18n_test` formatting cases).
4. Old files on the user's disk: a vehicle / mission file still carrying `name` / `display_name` / `label`, a plan cache
   holding a German reason, a settings file with `language = "fr"` or none: all load, the cache is rebuilt, German is
   used (Task 13 loader test, Task 10 cache-version test, Task 2 settings cases).
5. Former text-based decisions (`burn_abort_reason_.rfind("Gasansaugung", 0)`, `reason.empty()` as feasibility,
   game `ap["state"] in ["Booster", "Oberstufe"]`, `phase == "Kontakt"`): same branches (audit lists per task; Task 11
   asserts the gas-ingestion code ends the ISS mission; Task 3 HUD / probe-drogue id comparisons covered by
   `i18n_test` phase cases and the smoke test).

---

### Task 1: Core report module, code list, i18n check tool, CLAUDE.md rule

**Files:**
- Create: `core/include/starsim/report/report.hpp`, `core/include/starsim/report/codes.def`, `core/src/report/report.cpp`
- Modify: `core/CMakeLists.txt` (add source), `core/tests/CMakeLists.txt` (add test), `CLAUDE.md` (rules)
- Create: `core/tests/test_report.cpp`, `tools/i18n_check.py`, `tools/test_i18n_check.py`

**Interfaces:**
- Produces (namespace `starsim::report`, no dependencies on other core modules):
  - `enum class Code : std::uint16_t` from `codes.def` via `STARSIM_CODE(name, "id", "p1,p2")`; initial entries
    `ok` (`"ok"`, no params), `legacy` (`"legacy"`, `"text"`), `command_internal_error` (`"command.internal_error"`, no params).
  - `enum class Kind : std::uint8_t { angle, mass, length, speed, time, count, number, ref, legacy_text };`
  - `struct Param { const char* name; Kind kind; double value_si = 0.0; std::string text; };` (`text`: ref key or legacy text)
  - `struct Report { Code code = Code::ok; std::vector<Param> params; std::vector<Report> causes; bool ok() const; };`
  - `using Reports = std::vector<Report>;`
  - `Report make(Code code, std::initializer_list<Param> params = {});`
  - `Report with_cause(Report report, Report cause);`
  - `Param angle(const char*, double rad); mass(const char*, double kg); length(const char*, double m); speed(const char*, double mps); time(const char*, double s); count(const char*, long long); number(const char*, double); ref(const char*, std::string key); legacy_text(const char*, std::string text);`
  - `struct CodeInfo { Code code; std::string_view name; std::string_view id; std::string_view params; };`
    `std::span<const CodeInfo> code_table(); std::string_view id_of(Code); std::optional<Code> code_from_id(std::string_view); const char* id_of(Kind);`
  - `class Refusal : public std::runtime_error { public: explicit Refusal(Report report); const Report& report() const noexcept; };` (`what()` = code id)
- Produces (tool): `python tools/i18n_check.py [codes] [tables] [phases] [literals [PATH ...]]` - no argument = all
  checks over default scopes; exit 1 with one line per finding. Functions `parse_codes_def(text) -> list[tuple[str, list[str]]]`,
  `placeholders(text) -> set[str]`, `flatten(table: dict) -> dict[str, str]`, `check_tables(de, en) -> list[str]`,
  `check_codes(codes, de, en) -> list[str]`, `parse_phase_ids(cpp_text) -> dict[str, list[str]]`,
  `scan_cpp(text, path) -> list[str]`, `scan_gd(text, path) -> list[str]`. Literal scopes: `core/src`, `core/include`,
  `bindings/src`, non-test `game/**/*.gd` and `.tscn`. A literal fails when it holds non-ASCII or a whole German word
  from `GERMAN_WORDS` (case-insensitive; start with: der die das und nicht kein keine wird werden ist auf aus bei nach
  zum zur vom beim fehlt laden geladen Grenze bereit Abstieg Landung Rueckkehr Wetter Schatten Ansicht Bewege Erkannt
  abgebrochen abgelehnt moeglich geplant Treibstoff Triebwerk); comments are skipped; a line containing `i18n-ok` is exempt.
  Until Task 16 the tool is run with explicit paths for the task's files.

- [ ] **Step 1: Write the failing tests**

`core/tests/test_report.cpp`, tag `[report]`:
```cpp
TEST_CASE("A default report is ok", "[report]") { report::Report r; CHECK(r.ok()); CHECK(report::id_of(r.code) == "ok"); }
TEST_CASE("Parameters carry kind and SI value", "[report]") {
    auto r = report::make(report::Code::legacy, {report::legacy_text("text", "x")});
    CHECK_FALSE(r.ok()); CHECK(r.params[0].kind == report::Kind::legacy_text); CHECK(r.params[0].text == "x");
    CHECK(report::angle("slope", 0.1449).value_si == 0.1449); CHECK(report::angle("slope", 0.1449).kind == report::Kind::angle);
    CHECK(report::ref("site", "lunar_site.mons_mouton.name").text == "lunar_site.mons_mouton.name");
}
TEST_CASE("Code table: unique dotted ASCII ids, enumerator = id with _", "[report]") { /* regex, set, replace('.','_') == name, params match ^[a-z_][a-z0-9_]*(,..)*$ or empty */ }
TEST_CASE("code_from_id inverts id_of", "[report]") { /* all codes round-trip; code_from_id("nope") == nullopt */ }
TEST_CASE("Refusal carries its report", "[report]") { /* throw/catch; report().code == Code::legacy; std::string(what()) == "legacy" */ }
TEST_CASE("with_cause nests a report", "[report]") { /* causes.size() == 1, causes[0].code */ }
```
`tools/test_i18n_check.py`: `test_parse_codes_def` (`STARSIM_CODE(lunar_site_too_steep, "lunar.site_too_steep", "slope,limit")`
-> `[("lunar.site_too_steep", ["slope", "limit"])]`), `test_placeholders` (`"zu steil: {slope:deg1} (Grenze {limit})"` ->
`{"slope", "limit"}`), `test_flatten_nested_and_sections`, `test_key_missing_in_en_fails`,
`test_placeholder_mismatch_fails`, `test_code_without_text_fails`, `test_leaf_prefix_conflict_fails`,
`test_scan_cpp` (`"Höhe"` and `"Grenze erreicht"` flagged; `// Grenze` comment, `"engine"`, a line with `// i18n-ok`
not flagged), `test_scan_gd` (`"Höhe"` flagged, `# Höhe` not), `test_parse_phase_ids` (a `// i18n-ns: docking_phase`
switch -> `{"docking_phase": ["approach", "soft_capture"]}`), `test_phase_id_without_key_fails`.

- [ ] **Step 2: Run the tests to verify they fail**

Run: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -NoBindings -Fast -TestFilter "[report]"` -> build error
(no `starsim/report/report.hpp`). `python -m pytest tools/test_i18n_check.py -q` -> import error.

- [ ] **Step 3: Implement the module and the tool**

`report.cpp` holds the table built from `codes.def` (`{Code::name, #name, id, params}`) and the helpers. The tool uses
`tomllib`; `codes` check reads `codes.def` and requires `report.<id>` in both tables with placeholder names equal to the
listed params; `tables` checks key sets, placeholder names and leaf/prefix conflicts; `phases` reads every
`core/src/**/*.cpp` `// i18n-ns:` switch and requires `phase.<ns>.<id>` in both tables. The tables do not exist yet:
`codes` / `tables` / `phases` report "config/i18n/de.toml missing" and exit 1 until Task 2.

- [ ] **Step 4: Edit CLAUDE.md**

Replace "C++ sources are UTF-8 (/utf-8); UI strings in the core are German." with "C++ sources are UTF-8 (/utf-8). The
core returns report codes with SI parameters (`core/include/starsim/report/`), no UI text; developer messages (config
validation, internal errors, logs) are English." Replace "UI text is German." with "UI text comes from
`config/i18n/<de|en>.toml` (German default, English; `python tools/i18n_check.py` keeps them complete)." Add `report`
to the module list.

- [ ] **Step 5: Run the tests to verify they pass**

Same commands -> `[report]` green; pytest green.

- [ ] **Step 6: Commit and push**

`git add core/include/starsim/report core/src/report core/CMakeLists.txt core/tests/CMakeLists.txt core/tests/test_report.cpp tools/i18n_check.py tools/test_i18n_check.py CLAUDE.md`
`git commit -m "feat(report): core report codes with SI parameters and the i18n check tool"`; `git push origin main`.

---

### Task 2: Language service, text tables, language setting, reports through the bindings

**Files:**
- Create: `game/util/i18n.gd` (class `I18n`), `game/util/i18n_events.gd` (class `I18nEvents`), `config/i18n/de.toml`,
  `config/i18n/en.toml`, `bindings/src/report_dict.hpp`, `bindings/src/report_dict.cpp`, `game/tests/i18n_test.gd`
- Modify: `bindings/src/flight_sim.hpp` (`Outcome`), `bindings/src/flight_sim.cpp` (command results 160-200, signal 891,
  every `Outcome{...}`, `mission_start_status`, `mission_warnings`), `bindings/src/flight_snapshot*.cpp` (every key whose
  value is core text: `breakup_reason`, `burn_abort_reason`, return `reason` / `dispersion_note`, crew escape `reason`,
  ISS / lunar / refuelling `reason` / `status` / `note`, warnings arrays, `comparison_misses`), `bindings/src/starsim_core.cpp`
  (`report_codes()`), `game/ui/time_panel.gd` (language switch; start status 290/315), `game/vehicle/flight_controller.gd`
  (146-153, 169), `game/ui/flight_hud.gd` (breakup, dispersion note), `game/tests/smoke_test.gd` (744-764 message checks),
  bindings build files if sources are listed
- Test: `game/tests/i18n_test.gd`, `game/tests/smoke_test.gd`

**Interfaces:**
- Consumes: Task 1 `report::Report`, `report::id_of`, `report::code_table()`, `report::Refusal`.
- Produces (bindings): `godot::Dictionary report_dict(const starsim::report::Report&)`,
  `godot::Array report_array(const starsim::report::Reports&)`, transitional
  `godot::Dictionary legacy_report(const std::string& text)` (empty -> ok); `struct Outcome { bool ok; godot::Dictionary report; }`;
  `StarsimCore.report_codes() -> Array` of `{code: String, params: PackedStringArray}`. Commands: a caught
  `report::Refusal` -> its report; any other exception -> `push_error` + `legacy_report(e.what())` until Task 16.
- Produces (game, static class `I18n`, no autoload needed: works in `--script` tests; lazily loads on first use, so it is
  ready before the loading screen):
  - `static var events: I18nEvents` with `signal language_changed(language: String)`
  - `const LANGUAGES := ["de", "en"]`, `const DEFAULT_LANGUAGE := "de"`, `static var settings_path := "user://settings.cfg"`
  - `static func language() -> String`, `static func set_language(code: String) -> void` (unknown -> warning, no change; saves, emits)
  - `static func text(key: String, params: Dictionary = {}) -> String` (String values verbatim, numbers per spec / `n2`)
  - `static func report_text(report: Dictionary) -> String`, `static func phase_text(qualified_id: String) -> String`
  - `static func format_value(value: float, kind: String, spec: String = "") -> String`, `static func number(value: float, decimals: int) -> String`
  - `static func bind_text(node: Node, property: StringName, key: String, params: Dictionary = {}) -> void` (sets now and on every change while the node lives)
  - `static func has_key(key: String) -> bool`, `static func saved_language() -> String`, `static func reload() -> void`,
    test hook `static func use_tables(de: Dictionary, en: Dictionary) -> void` (flat key -> text)
- Produces (tables): sections per Global Constraints; first keys `report.legacy = "{text}"`,
  `report.command.internal_error`, `ui.language.label` ("Sprache" / "Language"), `ui.language.de` ("Deutsch" both),
  `ui.language.en` ("English" both), plus the keys of the consumers converted here (e.g. `ui.time_panel.start_planning`,
  `ui.time_panel.start_refused` "Start abgelehnt: {reason}", `ui.time_panel.start_refused_unknown`, `event.no_return`
  "Keine Rückkehr: {reason}", `event.no_return_no_ship`, `event.no_return_no_capsule`).

- [ ] **Step 1: Write the failing test** `game/tests/i18n_test.gd` (SceneTree script, `_check` pattern as in
  `loading_steps_test.gd`; `I18n.settings_path = "user://i18n_test_%d.cfg" % OS.get_process_id()`, file removed at the end):
  - real tables: `I18n.text("ui.language.label")` is "Sprache" in de and "Language" in en
  - fixture `use_tables({"report.t.steep": "zu steil: {slope:deg1} (Grenze {limit})", "phase.t.coast": "Freiflug", "only.de": "nur"}, {"report.t.steep": "too steep: {slope:deg1} (limit {limit})", "phase.t.coast": "coast"})`:
    report `{code: "t.steep", params: {slope: 0.1449, limit: 0.1396}, kinds: {slope: "angle", limit: "angle"}, causes: []}`
    -> de `"zu steil: 8,3° (Grenze 8,0°)"`, en `"too steep: 8.3° (limit 8.0°)"`
  - en falls back to de for `only.de`; `text("no.such.key") == "no.such.key"`; ok report -> `""`
  - `ref` param `"phase.t.coast"` resolves; a `cause` renders the nested report; legacy report -> its text
  - `format_value`: 12345 kg default -> de `"12,3 t"`; 123456 m `km1` -> `"123,5 km"`; 3725 s `clock` -> `"1:02:05"`;
    90061 s `clock` -> `"25:01:01"`; NAN and INF -> `"–"`; -0.0 `n1` -> `"0,0"`; count 3 -> `"3"`
  - live switch: a Label with `bind_text(label, "text", "ui.language.label")` reads "Language" after `set_language("en")`;
    the signal fired once; a stored report re-rendered after the switch is English; the settings file has
    `[ui] language="en"`; `set_language("fr")` keeps "en"; `saved_language()` with no file -> "de", with `"fr"` stored -> "de"
- [ ] **Step 2: Run it to verify it fails**: `external/godot/bin/godot.windows.editor.double.x86_64.console.exe --headless --path game --script res://tests/i18n_test.gd` -> parse error (no `I18n`).
- [ ] **Step 3: Implement** the service (flatten `TomlLite.load_file` result: `section + "." + key`), the tables, the
  bindings helpers and the `Outcome` / command / snapshot conversions (every former String text value becomes
  `legacy_report(...)` / `report_array(...)`; enum-valued keys stay for Task 3), the game consumers (`I18n.report_text`;
  `time_panel.gd:315` `message != ""` -> `report["code"] != "ok"`), the "Sprache" OptionButton in the time panel next to
  the other saved settings (items via `ui.language.de/en`, label via `bind_text`), smoke test message checks rewritten to
  `report` (until Task 8 the lunar check reads `report["params"]["text"]`).
- [ ] **Step 4: Run**: build `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Fast -TestFilter "[report]"`;
  `i18n_test.gd` PASS; smoke test PASS; `python tools/i18n_check.py tables codes` PASS;
  `python tools/i18n_check.py literals game/util/i18n.gd bindings/src/report_dict.cpp` PASS.
- [ ] **Step 5: Commit and push**: `feat(i18n): language service, de/en text tables, live language switch, reports through the bindings`.

---

### Task 3: Phase and state enums as ids

**Files:**
- Modify (core): every German enum switch - `contact/docking_mechanism.{hpp,cpp}` (DockingPhase), `gnc/ascent_guidance`
  (AscentPhase), `gnc/booster_return` (ReturnPhase), `gnc/departure_guidance` (DeparturePhase), `gnc/ejection_seat`
  (SeatPhase, SeatMode), `gnc/entry_guidance` (EntryPhase), `gnc/lunar_ascent_guidance`, `gnc/lunar_landing_guidance`
  (LunarDescentPhase), `gnc/parachute_sequencer`, `gnc/proximity_guidance` (ProximityPhase), `gnc/ship_landing`
  (ship phase), `mission/phasing_planner` (PhasingBurnKind `german_name`), `sim/catch_tower_sim` (CatchPhase),
  `sim/crew_escape` (CrewOutcome), `sim/flight_session.hpp` + sources (AutopilotState, CapsuleReturnStep, LunarStep,
  RendezvousStep, IssMissionLeg), `sim/landing_outcome` (LandingOutcome, ReturnResult), `sim/leg_landing_sim`
  (LegTouchdown), `sim/mission_timeline` (MissionEventKind: keep `id_of`, drop `to_string`),
  `sim/refuelling_campaign_flight` (CampaignFlightPhase), `vehicle/vehicle.hpp` (ParachuteState), lunar touchdown
  outcome, `optim/socp_problem` (SocpStatus: log only, no ns marker); printf diagnostics switch to `id_of`
- Modify (bindings): `flight_snapshot.cpp`, `flight_snapshot_crew_escape.cpp`, `flight_snapshot_iss.cpp`,
  `flight_snapshot_lunar.cpp`, `flight_snapshot_refuelling.cpp` (TransferPhase `phase_name` -> core-side or bindings
  `id_of` with ns `transfer_phase`), `flight_snapshot_timeline.cpp` (drop `label`; `kind` is the id); literal guidance
  values `"SCvx"` / `"explizit"` / `"aufrecht"` -> `guidance_mode.scvx|explicit|upright`, `"aufgegeben"` -> a phase id
- Modify (game): `ui/flight_hud.gd` (tower / return / autopilot / entry rows via `I18n.phase_text`; comparisons
  `"aus"`, `"fliegt"`, `"Booster"`, `"Oberstufe"` -> qualified ids), `vehicle/probe_drogue_view.gd:141-143`
  (`"Kontakt"` / `"Weichfang"` / `"Einzug"` -> `docking_phase.*`), `ui/webcast/timeline_arc.gd` (label from
  `phase.mission_event_kind.<kind>`), other consumers found by `rg -n '"(phase|state|step|leg|outcome|mode|guidance|result|touchdown|label)"' game --glob '!tests'`
- Modify: `config/i18n/*.toml` (`phase.*` for every id), core tests and game tests that assert German phase names
- Test: `python tools/i18n_check.py phases`, `game/tests/i18n_test.gd` (add a real-table case), edited core tests

**Interfaces:**
- Consumes: Task 2 `I18n.phase_text`.
- Produces: `const char* id_of(<Enum>)` for each enum above (ASCII snake ids, `// i18n-ns:` marker); snapshot enum fields
  carry `"<ns>.<id>"`, e.g. `autopilot_status()["state"] == "autopilot_state.off"`, return `outcome == "landing_outcome.flying"`.
  Transitional exceptions (kept with `// transitional: removed by Task N`): `mission::german_name(PhasingBurnKind)` (Task 5),
  `sim::to_string(LunarStep)` and the touchdown-outcome text (Task 8) - only where a not-yet-converted status embeds them.

- [ ] **Step 1: Failing check**: add `phases` real-table case to `i18n_test.gd` (`I18n.phase_text("autopilot_state.off")`
  is not the key in de and en) and run `python tools/i18n_check.py phases` -> fails (no `phase.*` keys / no markers).
- [ ] **Step 2: Audit** the enum consumers (greps of Global Constraints over core, bindings, game); list every
  comparison against a phase string.
- [ ] **Step 3: Implement** `id_of` per enum, delete the German switches (except the listed transitional ones), bindings
  qualified ids, game consumers, German texts (today's words) + English texts in `[phase]`.
- [ ] **Step 4: Run**: `powershell -ExecutionPolicy Bypass -File tools/build.ps1 -Fast -TestFilter "<tags of the edited test files>"`
  (e.g. `[docking][contact],[guidance],[crew_escape][gnc],[timeline],[catch][sim]`, checked with `-ListOnly`);
  `i18n_check.py phases tables` PASS; `i18n_test.gd`, smoke test PASS.
- [ ] **Step 5: Commit and push**: `refactor(core): phase and state enums as ids, names from the text tables`.

---

### Task 4: ISS mission, reboost and station start reports

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (`iss_mission_status()`, `reboost_status()`, `iss_end`,
  `iss_enter`, `end_reboost`, `IssMission...` reason fields), `core/src/sim/flight_session_iss_mission.cpp`,
  `flight_session_reboost.cpp`, `iss_mission.cpp`, `station_start.cpp`, `station_limits.cpp`,
  `core/include/starsim/sim/iss_mission.hpp` / `station_start.hpp`, `bindings/src/flight_snapshot_iss.cpp` (ISS status),
  `codes.def` (section `iss`), tables, tests asserting these texts (`test_iss_mission_sequence.cpp`, `test_iss_station.cpp`, ...)

**Interfaces:**
- Consumes: Task 1/2/3 (`IssMissionLeg` id for the leg ref `phase.iss_mission_leg.<id>`).
- Produces: `const report::Report& FlightSession::iss_mission_status() const`, `const report::Report& reboost_status() const`,
  `void iss_end(report::Report reason)`, `void iss_enter(IssMissionLeg, report::Report status)`,
  `void end_reboost(bool completed, report::Report reason)`; status = code `iss.leg_status` (params `leg` ref, `cause`)
  with the leg's sub-status as cause. Not yet converted producers embedded as `legacy` causes: `rendezvous_status_`
  (Task 5), `proximity_status_` / `docking_status_` (Task 6), `transfer_state().status` (Task 7),
  `burn_abort_reason_` (Task 11). `burn_abort_reason_.rfind("Gasansaugung", 0) == 0` (line ~233) stays textual until Task 11.

- [ ] **Step 1: Rewrite the affected tests** to assert codes / parameters (e.g. `CHECK(session.iss_mission_status().code == report::Code::iss_complete)`) -> fail to compile.
- [ ] **Step 2: Audit** `flight_session_iss_mission.cpp`, `flight_session_reboost.cpp`, `iss_mission.cpp`, `station_start.cpp`, `station_limits.cpp` and list hits.
- [ ] **Step 3: Implement** the reports, `codes.def` `iss.*` / `reboost.*` / `station.*`, de + en texts, bindings
  `report_dict(session.iss_mission_status())`.
- [ ] **Step 4: Run**: `tools/build.ps1 -Fast -TestFilter "[iss_mission],[iss][session],[reboost]"` (narrow per `-ListOnly`);
  `i18n_check.py codes tables` + `literals <touched files>` PASS (literals of the remaining legacy concatenation sites
  excepted - list them); smoke test PASS.
- [ ] **Step 5: Commit and push**: `refactor(iss): ISS mission, reboost and station start report codes`.

---

### Task 5: Rendezvous, phasing and launch window reports

**Files:**
- Modify: `core/src/sim/flight_session_rendezvous.cpp` (`rendezvous_status_`, `rendezvous_abort`, `rendezvous_warnings_`),
  `flight_session.hpp`, `core/src/mission/phasing_planner.cpp` + `.hpp` (`reason`, transitional `german_name` removed),
  `mission/phasing_schedule.cpp`, `mission/iss_launch_window.{hpp,cpp}` (`reason` :77), `bindings/src/flight_snapshot_iss.cpp`
  (rendezvous status, warnings, plan reason), `flight_session_iss_mission.cpp` (replace the rendezvous `legacy` causes),
  `codes.def` (`rendezvous.*`, `phasing.*`, `launch_window.*`), tables, tests (`test_iss_launch_window.cpp`, rendezvous / phasing tests)

**Interfaces:**
- Produces: `const report::Report& rendezvous_status() const`, `const report::Reports& rendezvous_warnings() const`,
  `void rendezvous_abort(report::Report)`, `report::Report reason` in the phasing / launch-window result structs (ok
  when feasible); burn kind as ref `phase.phasing_burn_kind.<id>`; Δv as `speed`.

- [ ] Step 1: tests to codes (fail) - Step 2: audit (incl. `iss_launch_window.cpp` reason check) - Step 3: implement -
  Step 4: `tools/build.ps1 -Fast -TestFilter "[rendezvous],[phasing],[iss][launch_window]"` (per `-ListOnly`), checks,
  smoke - Step 5: `refactor(rendezvous): rendezvous, phasing and launch window report codes`.

---

### Task 6: Proximity, docking and departure reports

**Files:**
- Modify: `core/include/starsim/gnc/proximity_guidance.hpp` (`status` :45, warnings), `gnc/proximity_guidance.cpp`,
  `gnc/proximity_geometry.{hpp,cpp}` (`reason` :120, waypoint `name` :29 -> index parameter `waypoint` count),
  `gnc/departure_guidance.{hpp,cpp}` (`status` :34), `contact/docking_mechanism.{hpp,cpp}` (`status` :71, `refusal_`),
  `vehicle/port_mating.cpp`, `core/src/sim/flight_session_proximity.cpp`, `flight_session_docking.cpp`, `docking_sim.cpp`,
  the `docking_status_` assignments in `flight_session.cpp`, `flight_session_capsule.cpp`, `flight_session_staging.cpp`,
  `flight_session_transfer.cpp`, `flight_session_iss_mission.cpp` (replace legacy causes), `bindings/src/flight_sim.cpp`
  (Outcomes 720-763 incl. "Settling nicht möglich", "Ungültiger Körperindex" -> codes), `flight_snapshot_iss.cpp`,
  `codes.def` (`proximity.*`, `docking.*`, `departure.*`, `command.*`), tables, tests (`test_docking_mechanism.cpp`,
  `test_departure_guidance.cpp`, proximity / docking session tests)

**Interfaces:**
- Produces: `report::Report status` in `ProximityGuidance` / `DepartureGuidance` outputs, `report::Report status()` and
  `report::Report refusal()` on the docking mechanism, `const report::Report& proximity_status() const`,
  `const report::Reports& proximity_warnings() const`, `const report::Report& docking_status() const`; `CorridorStatus`
  reason as code. Transfer statuses embedded into `docking_status_` stay `legacy` until Task 7.

- [ ] Step 1 tests -> codes (fail) - Step 2 audit - Step 3 implement - Step 4
  `tools/build.ps1 -Fast -TestFilter "[proximity],[docking][contact],[departure]"` (narrow per `-ListOnly`), checks,
  smoke - Step 5 `refactor(docking): proximity, docking and departure report codes`.

---

### Task 7: Propellant transfer and refuelling reports

**Files:**
- Modify: `core/src/sim/propellant_transfer.cpp` + header (`status`), `flight_session_transfer.cpp`,
  `refuelling_campaign_flight.cpp`, `refuelling_campaign_setup.cpp`, `refuelling_mission.cpp`,
  `core/src/mission/refuelling_campaign.cpp` (+ `reason`), `flight_session.hpp` (`tanker_reserve(std::string*)`,
  `tanker_transfer_request`, `rendezvous_floors`, `TankerReserveInputs::reason` -> `report::Report*`),
  `refuelling_campaign_text.cpp` (developer report: English, ASCII), the transfer `legacy` causes in
  `flight_session_iss_mission.cpp` / `flight_session_transfer.cpp`, `bindings/src/flight_snapshot_refuelling.cpp`
  (status, notes, reason; `comparison_misses` are developer diagnostics: English ASCII strings, documented),
  `codes.def` (`transfer.*`, `refuelling.*`), tables, refuelling tests that assert text

**Interfaces:**
- Produces: `report::Report status` in the transfer state, `std::optional<mission::TankerReserve> tanker_reserve(report::Report* reason = nullptr) const`
  (same for `tanker_transfer_request`, `rendezvous_floors`), campaign `report::Report reason`.

- [ ] Step 1 tests -> codes (fail) - Step 2 audit - Step 3 implement - Step 4
  `tools/build.ps1 -Fast -TestFilter "[prop_transfer],[refuelling][mission],[refuelling][session]"` (no
  `[campaign_flown]`; narrow per `-ListOnly`), checks, smoke - Step 5 `refactor(refuelling): transfer and refuelling report codes`.

---

### Task 8: Lunar mission chain reports

Largest task: the lunar status is one field written from five session files and it embeds the mission modules' reasons,
so they convert together.

**Files:**
- Modify: `core/src/mission/lunar_budget.{hpp,cpp}` (`reason` :20, leg `name` :10 -> leg id ref `phase.lunar_leg.<id>`),
  `mission/lunar_descent` (`reason` :141), `mission/lunar_orbit_transfer` (:21), `mission/lunar_outbound.cpp`,
  `mission/lunar_transfer.cpp`, `mission/nrho_reference.cpp`, `mission/touchdown_search.cpp`, `mission/lunar_touchdown.cpp`,
  `core/src/sim/flight_session_lunar.cpp`, `flight_session_lunar_coast.cpp`, `flight_session_lunar_descent.cpp`,
  `flight_session_lunar_ascent.cpp`, `flight_session_lunar_return.cpp`, `flight_session_lunar_detail.hpp`,
  `lunar_landing_mission.{hpp,cpp}`, `lunar_return_mission.cpp`, `lunar_site.cpp`, `gnc/lunar_ascent_guidance.cpp:105`, `config/lunar_site_config.cpp:24` (player-facing
  missing terrain -> `report::Refusal` code `data.lunar_terrain_missing`), transitional `to_string(LunarStep)` /
  touchdown text removed, `bindings/src/flight_sim.cpp` (Outcomes 638/645), `flight_snapshot_lunar.cpp`,
  `game/tests/smoke_test.gd` (nrho refusal -> code), `codes.def` (`lunar.*`, `nrho.*`, `data.*`),
  tables (incl. `lunar_site.<id>.name` for the sites used as refs), lunar tests asserting text

**Interfaces:**
- Produces: `const report::Report& lunar_status() const`, `void lunar_abort(report::Report)`, `report::Report reason`
  in every lunar planner result (ok when feasible), site as ref `lunar_site.<id>.name`, slope / limit as `angle`.
  Lunar reasons that `mission_start.cpp` copies into `MissionStart::status` go in as `legacy` causes until Task 10;
  burn-abort texts embedded here stay `legacy` until Task 11.

- [ ] Step 1 tests -> codes (fail) - Step 2 audit (all five session files + mission modules) - Step 3 implement -
  Step 4 `tools/build.ps1 -Fast -TestFilter "[lunar][mission],[lunar][session],[descent][lunar],[hls][legs]"` (no
  `[validation]`, narrow per `-ListOnly`), checks, smoke - Step 5 `refactor(lunar): lunar mission chain report codes`.

---

### Task 9: Earth return chain reports

**Files:**
- Modify: `core/include/starsim/gnc/deorbit_planner.hpp` (`reason` :71, `dispersion_note`), `gnc/deorbit_planner.cpp`
  (snprintf :400), `gnc/catch_feasibility.{hpp,cpp}` (`reason` :13, snprintf :50), `gnc/entry_reach_check.{hpp,cpp}`
  (`name` :15 -> zone id ref), `gnc/entry_corridor.{hpp,cpp}`, `gnc/booster_return.hpp` texts, `mission/earth_return.{hpp,cpp}`
  (`reason` :98), `config/splashdown_zone_config` (name use as ref `zone.<id>.name`), `core/src/sim/flight_session_return.cpp`,
  `flight_session_capsule.cpp`, `capsule_return_mission.cpp`, `landing_outcome.cpp`, `flight_session.hpp`
  (`ship_return_settings(std::string*)`, `vehicle_return_settings`, `boil_off_shortfall` -> `report::Report`),
  lunar-return uses of the deorbit reason (replace Task 8 `legacy`), `bindings/src/flight_snapshot.cpp` (return
  `reason`, `dispersion_note`), `game/vehicle/flight_controller.gd` (no-return event), `codes.def` (`deorbit.*`,
  `catch.*`, `entry.*`, `return.*`), tables, tests (`test_deorbit_planner.cpp`, `test_catch_feasibility.cpp`,
  `test_earth_return.cpp`, `test_entry_reach.cpp`, ...)

**Interfaces:**
- Produces: `report::Report reason` and `report::Report dispersion_note` (ok = none) in `DeorbitPlan`; catch / reach /
  corridor results with `report::Report reason`; `std::optional<gnc::DeorbitParameters> ship_return_settings(report::Report* reason) const`.

- [ ] Step 1 tests -> codes (fail) - Step 2 audit (feasibility tests on `reason.empty()`) - Step 3 implement - Step 4
  `tools/build.ps1 -Fast -TestFilter "[deorbit],[catch][gnc],[entry][gnc],[return][mission]"` (narrow per `-ListOnly`),
  checks, smoke - Step 5 `refactor(return): earth return chain report codes`.

---

### Task 10: Ascent, pad start, mission start, staging and tower reports

**Files:**
- Modify: `core/include/starsim/sim/ascent_profile_planner.hpp` (`engage_pad_ascent` :161 returns `report::Report`),
  `ascent_profile_planner.cpp`, `ascent_plan_cache.cpp` (stores `reason` -> code id + params, format version bump; an
  old-version file is a miss), `mission_start.{hpp,cpp}` (`status` incl. "Landeplatz wird geprüft", `warnings`; replace
  the Task 8 lunar `legacy` causes), `game/tests/smoke_test.gd:744-748` (start status -> code), `flight_session_staging.cpp`,
  `catch_tower_sim.cpp`, `staging_sequence.cpp`, remaining texts in `flight_session.cpp` / `flight_session_command.cpp`
  except burn abort / gas ingestion (Task 11), `bindings/src/flight_sim.cpp` (engage_pad_ascent Outcome :221,
  `mission_start_status`, `mission_warnings`), `codes.def` (`ascent.*`, `start.*`, `staging.*`, `tower.*`), tables,
  tests (`test_ascent_profile_planner.cpp`, `test_ascent_plan_cache.cpp`, mission start tests)

**Interfaces:**
- Produces: `report::Report sim::engage_pad_ascent(FlightSession&, ...)` (ok = engaged), `report::Report MissionStart::status`,
  `report::Reports MissionStart::warnings`; plan cache helpers `std::string encode_report(const report::Report&)` /
  `std::optional<report::Report> decode_report(std::string_view)` local to `ascent_plan_cache.cpp` (uses `report::code_from_id`).

- [ ] **Step 1: Failing tests**: existing tests to codes, plus `TEST_CASE("A plan cache file of the previous version is a miss", "[ascent_planner][cache]")`
  (write a file with the old version header -> `load` returns nullopt) and a round trip of a refusal reason through the cache.
- [ ] Step 2 audit (`reason.empty()` at `flight_sim.cpp:221` -> `.ok()`) - Step 3 implement - Step 4
  `tools/build.ps1 -Fast -TestFilter "[ascent_planner],[mission_start],[staging],[catch][sim]"` (narrow per `-ListOnly`),
  checks, smoke - Step 5 `refactor(ascent): ascent, start, staging and tower report codes`.

---

### Task 11: Burn abort, breakup, gas ingestion, crew escape and parachute reports

**Files:**
- Modify: `core/include/starsim/sim/flight_session.hpp` (`gas_ingestion_reason` :429 -> code, `burn_abort_reason()`,
  `crew_arm`), `flight_session.cpp`, `flight_session_burns.cpp`, `flight_session_crew_escape.cpp`, `crew_escape.cpp`
  (+ `reason`, `give_up_reason`), `vehicle/flight_body.{hpp,cpp}` (`breakup_reason()`), `vehicle/vehicle.cpp`
  (parachute texts), `gnc/ejection_seat.cpp`, `gnc/parachute_sequencer.cpp`, every remaining `legacy` cause of these in
  `flight_session_iss_mission.cpp` (incl. `rfind("Gasansaugung", 0)` -> `burn_abort_reason_.code == report::Code::burn_gas_ingestion`),
  `flight_session_reboost.cpp`, `flight_session_return.cpp`, lunar files, `bindings/src/flight_snapshot.cpp`
  (`breakup_reason`, `burn_abort_reason`), `flight_snapshot_crew_escape.cpp`, `codes.def` (`burn.*`, `breakup.*`,
  `crew_escape.*`, `parachute.*`), tables, tests (`test_flight_body.cpp`, crew escape tests, the gas-ingestion test)

**Interfaces:**
- Produces: `const report::Report& FlightSession::burn_abort_reason() const`, `const report::Report& FlightBody::breakup_reason() const`,
  `void crew_arm(report::Report reason, bool early)`, crew escape `report::Report reason` / `give_up_reason`.

- [ ] **Step 1: Failing tests**: assertions to codes; in the existing gas-ingestion ISS case add
  `CHECK(session.burn_abort_reason().code == report::Code::burn_gas_ingestion)` and that the ISS mission ended with it as cause.
- [ ] Step 2 audit - Step 3 implement - Step 4 `tools/build.ps1 -Fast -TestFilter "[crew_escape],[vehicle][breakup],[iss_mission],[parachute]"`
  (narrow per `-ListOnly`), checks, smoke - Step 5 `refactor(core): burn abort, breakup and crew escape report codes`.

---

### Task 12: Player-facing data errors as codes, developer texts in English

**Files:**
- Modify: `core/src/config/moon_surface_config.cpp` (30-53), `config/vehicle_config.cpp` (562-1141), `config/mission_config.cpp`,
  `config/rock_abundance_file.cpp:12`, `config/refuelling_reference.cpp`, `config/*` remaining German messages,
  `environment/surface_samples.cpp`, `environment/rock_abundance_map.cpp`, `environment/unresolved_shading.cpp`,
  `sim/earth_terrain_loader.cpp`, `sim/lunar_site.cpp` leftovers, `bindings/src/starsim_core.cpp` (logs 288-325,
  `weather_failure` 154, errors 350-1451) and `bindings/src/flight_sim.cpp` logs (71-78), any core / bindings file
  still flagged by `python tools/i18n_check.py literals core bindings/src`; tests asserting those messages
- Decision per site (listed in the report): player-facing in normal use (missing terrain / geoid / rock map / atmosphere
  data, weather source failure the game shows) -> `report::Refusal` / report with code `data.*` (ref to the file kind,
  no paths in text; the path stays in the English log) and a game display through `I18n`; developer-facing ->
  English ASCII message, same exception type, same throw point.

- [ ] **Step 1**: update tests that match these messages (e.g. `CHECK_THROWS_WITH(..., Catch::Matchers::ContainsSubstring("fetch.py"))` keeps working; German substrings -> English / code checks) -> fail.
- [ ] **Step 2**: audit (`rg -n "e.what\(\)|\[\"error\"\]" bindings game --glob '!tests'` for consumers of these texts).
- [ ] **Step 3**: implement; `codes.def` `data.*`.
- [ ] **Step 4**: `tools/build.ps1 -Fast -TestFilter "[config],[moon_surface],[environment][terrain],[earth_terrain]"`
  (narrow per `-ListOnly`); `python tools/i18n_check.py literals core bindings/src` PASS except listed `legacy`
  sites; smoke PASS.
- [ ] **Step 5**: `refactor(core): data errors as codes, developer messages in English`.

---

### Task 13: Config display names move into the text tables

**Files:**
- Modify (core loaders stop reading display fields; an old file that still has them loads, fields ignored):
  `config/vehicle_config.{hpp,cpp}` (vehicle / stage `name`, `display_name`, :156 engine `name`, :183 port `name` - audit
  first which are ids used in logic), `config/mission_config.{hpp,cpp}` (`name`, `display_name`, `mission_display_name`),
  `config/launch_site_config.cpp`, `config/lunar_site_config.{hpp,cpp}`, `config/splashdown_zone_config.cpp`,
  `config/drone_ship_config.cpp`, `vehicle/vehicle.{hpp,cpp}` (`shown_name()` -> `std::string shown_name_key() const`, ASCII key),
  `sim/mission_timeline` (`body_name` -> body key)
- Modify (bindings): `flight_snapshot.cpp` (stage `name` -> `name_key`, zone `name`, `display_name`),
  `flight_snapshot_iss.cpp` (port name -> key), `flight_snapshot_lunar.cpp` (site name -> key),
  `flight_snapshot_timeline.cpp`, `flight_sim.cpp` `mission_display_name` / `body_display_name` return keys
- Modify (game): `ui/mission_catalogue.gd` (names via `I18n.text("mission.<id>.name")` / `vehicle.<id>.display_name`),
  `ui/time_panel.gd` (site / place names), `vehicle/flight_controller.gd:78`, `ui/flight_hud.gd:42`,
  `ui/webcast/gauge_config.gd` + `webcast_overlay.gd:54` (`gauge.<id>.label`; `unit` stays), webcast name displays,
  `world/map_label.gd` if it shows config names
- Modify (config): remove `name` / `display_name` / `label` lines from `config/vehicles/*.toml`, `config/missions/*.toml`,
  `launch_sites.toml`, `start_places.toml`, `lunar_sites.toml`, `splashdown_zones.toml`, `drone_ships.toml`,
  `ui/webcast_gauges.toml` (keep provenance tags); `config/structures/starbase_olp2_lights.toml` names only if shown;
  `config/README.md` (names live in `config/i18n/`); `tools/vehicle_report.py` falls back to the id
- Modify: tables (`vehicle.<id>.name`, `vehicle.<id>.display_name`, `vehicle.<id>.stage.<stage>.display_name`,
  `vehicle.<id>.port.<port>.name`, `mission.<id>.name`, `mission.<id>.display_name`, `site.<id>.name`, `place.<id>.name`,
  `lunar_site.<id>.name`, `zone.<id>.name`, `drone_ship.<id>.name`, `gauge.<id>.label`), `core/tests/test_display_names.cpp`

**Interfaces:**
- Produces: `std::string Vehicle::shown_name_key() const`; bindings `body_display_name(i)` / `mission_display_name()`
  return keys; snapshot `name_key` fields.

- [ ] **Step 1: Failing tests** in `test_display_names.cpp` (`[config][display]`): every vehicle, shown stage, mission,
  launch site, place, lunar site, zone, drone ship and gauge id in `config/` has its name key in both
  `config/i18n/de.toml` and `en.toml` (toml++, flattened); a temp vehicle file (unique name) with `name` /
  `display_name` still loads; `shown_name_key()` of the tanker's ship is `vehicle.starship_tanker.display_name` (or the
  variant rule as today); no "Starship" / "SpaceX" in any shown text of the tables.
- [ ] Step 2 audit (engine / port / stage `name` uses in logic: if any is an id, keep it as id and document) - Step 3
  implement - Step 4 `tools/build.ps1 -Fast -TestFilter "[config][display],[config][vehicle],[config][mission]"` (narrow
  per `-ListOnly`); `python tools/vehicle_report.py starship_block3` runs; `mission_catalogue_test.gd`,
  `webcast_overlay_test.gd`, `i18n_test.gd`, smoke PASS - Step 5 `refactor(config): display names in the text tables`.

---

### Task 14: Game texts - HUD, events, loading screen, notices

**Files:**
- Modify: `game/ui/flight_hud.gd` (KEY_HELP -> keys per line; telemetry rows: label key + value via
  `I18n.format_value` / `I18n.number`, label column padded to the widest label of the active language; optional
  clauses as separate keys; `_dv_bar` glyph line marked `i18n-ok`), `game/vehicle/flight_controller.gd` (`last_event`
  becomes `last_event_report: Dictionary` resolved at draw: "Zündung" 275, "Klammern gelöst" 278, "Rückkehr geplant"
  146-151, ...), `game/main.gd` (19 `LoadingScreen.step(...)` -> keys `loading.*`), `game/ui/loading_screen.gd`,
  `loading_steps.gd` (stores keys), `loading_view.gd` (resolves at render), `game/ui/model_notice.gd` (STALE_TEXT,
  MISSING_TEXT -> `notice.*`), `game/vehicle/probe_drogue_view.gd` texts, tables (`hud.*`, `event.*`, `loading.*`,
  `notice.*`), `game/tests/loading_steps_test.gd`, `game/tests/loading_view_shot.gd` (keys)

**Interfaces:**
- Consumes: `I18n.text`, `I18n.report_text`, `I18n.format_value`, `I18n.events.language_changed`.
- Produces: `FlightController.last_event_report: Dictionary` (`{key: String, params: Dictionary}` for game events or a
  core report dict); `LoadingSteps.add(key: String, now_ms: int)` unchanged signature, values are keys.

- [ ] **Step 1: Failing test** in `i18n_test.gd`: a stored HUD event `{key: "event.ignition"}` renders "Zündung" in de and
  "Ignition" in en after the switch (Review Focus 1); `loading_steps_test.gd` uses keys and checks `LoadingView` lines
  change language.
- [ ] Step 2 audit (game comparisons in these files) - Step 3 implement - Step 4: `i18n_test.gd`,
  `loading_steps_test.gd`, smoke PASS; `python tools/i18n_check.py tables literals <these files>` PASS - Step 5
  `feat(i18n): HUD, events, loading screen and notices from the text tables`.

---

### Task 15: Game texts - panels, wizard, world, webcast and the rest

**Files:**
- Modify: `game/ui/time_panel.gd` (rest: "Wetter" 89, "Mondschatten nah" 97, "Schatten Erde" 106, "Start vorbereiten"
  116, tooltips 98/107, UTC placeholder 56, "SPICE-Daten fehlen" 144, `_button` 318, site list suffix 74, sun coverage
  169, `camera_height_line`), `game/ui/free_camera_wizard.gd`, `game/ui/mission_catalogue.gd` (FREE_FLIGHT_NAME ->
  `ui.catalogue.free_flight`), `game/world/world.gd`, `game/world/atmosphere.gd` (`status_text`),
  `game/world/weather/cloud_layer.gd`, `cloud_effect.gd`, `game/world/free_camera_control.gd`, `game/util/local_time.gd`,
  `game/world/map_label.gd`, `game/ui/webcast/*` (top bar, mission clock, timeline texts), `game/ui/flight_overlay.gd`,
  `game/ui/nav_ball.gd`, every other non-test `.gd` / `.tscn` the game literal scan flags; tables (`ui.*`, `wizard.*`);
  tests asserting these texts (`smoke_test.gd:48-49` camera height line, `local_time_test.gd`, `webcast_overlay_test.gd`,
  `mission_catalogue_test.gd`)
- Labels / buttons / tooltips set once at build time use `I18n.bind_text`; texts rebuilt per frame resolve each time.

- [ ] Step 1: tests to keys / both languages (fail) - Step 2: audit - Step 3: implement - Step 4: the edited game test
  scripts, `i18n_test.gd`, smoke PASS; `python tools/i18n_check.py tables literals game` PASS - Step 5
  `feat(i18n): panels, wizard, world and webcast texts from the text tables`.

---

### Task 16: Remove the transitional bridge, all checks green, docs

**Files:**
- Modify: `codes.def` (delete `legacy`), `report.hpp` (delete `Kind::legacy_text`, `legacy_text()`), `report_dict.{hpp,cpp}`
  (delete `legacy_report`), `flight_sim.cpp` (non-`Refusal` exceptions -> `push_error(e.what())` + report
  `command.internal_error`), `game/util/i18n.gd` (drop legacy handling), `config/i18n/*.toml` (drop `report.legacy`),
  whatever the build then flags
- Modify docs: `docs/architecture.md` (report module, bindings report shape, `I18n`), new `docs/subsystems/i18n.md`
  (model, code / key schemes, formatting, fallback, checks, how to add a code or a text) + index `docs/subsystems.md`,
  `docs/setup.md` (`python tools/i18n_check.py`, `pytest tools/test_i18n_check.py`, `i18n_test.gd` in the test workflow),
  `config/README.md` check, `docs/ansichten.md` (the language switch in the time panel, German), CLAUDE.md "Where to find
  what" row for `docs/subsystems/i18n.md`

- [ ] **Step 1**: delete the bridge; build -> every leftover use fails to compile; fix each by converting it (if any
  remains, it belongs to an earlier task's area: convert it here and note it).
- [ ] **Step 2**: `python tools/i18n_check.py` (all checks, default scopes) PASS; `python -m pytest tools/test_i18n_check.py -q` PASS.
- [ ] **Step 3**: `tools/build.ps1 -Fast -TestFilter "<union of the tags run in Tasks 1-15>"` (still no `[slow]`, no
  `-Nightly`; if `-ListOnly` exceeds ~10 min of measured waits, run in the background with output to a file);
  `tools/run_godot_tests.ps1` (headless scripts) and the smoke test PASS; in-game spot check of the live switch
  documented with the commands (no render checks).
- [ ] **Step 4**: docs as listed; the plan's ledger state for the final review.
- [ ] **Step 5**: commit and push `refactor(i18n): remove the transitional text bridge; docs`; final whole-plan review on Opus.
