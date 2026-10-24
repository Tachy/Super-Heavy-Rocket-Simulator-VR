# Release Packaging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A Windows release of the game - exported exe, portable ZIPs, an Inno Setup installer with the validated runtime
data - built locally by `tools/package.ps1` and published by the `/build` command; `/build-local` builds a test
installer without publishing.

**Architecture:** Copy of the sibling project's pattern (`../Stunt-Track-Racer-VR`: `tools/build.ps1`,
`tools/installer.iss`, `tools/release/*.cmd`, `.claude/commands/build.md` + `build-local.md`, release-please with
`skip-github-release`). Differences: our engine is a self-built double-precision Godot, so the release export template is
built from `external/godot` (not downloaded); the game needs `config/` and `data/` next to the exe; the data ships as
three parts so no release asset exceeds GitHub's 2 GB limit.

**Tech Stack:** PowerShell 5.1, Godot 4.7.2 double (custom `template_release`), Inno Setup 6, `tar.exe` (Windows
bsdtar) for ZIPs, `gh`, release-please.

**Spec:** none - the user chose "take the sibling's pattern, no questions" (2026-10-24). Requirements from memory
`project-release-packaging`: Inno Setup .exe, `/build` + `/build-local` like the sibling, a separate data package split
into terrain / imagery / the rest, CREDITS.md included, the release ships the FINISHED imagery pyramid
(`data/imagery/earth`), never `data/raw`; `fetch.py` stays the alternative.

## Decisions (made by Claude, for the user to check)

- Product name **"Super Heavy Rocket Simulator VR"** (the public repo/README name, with its non-affiliation note);
  exe `SuperHeavyRocketSimulatorVR.exe`; file prefix `SuperHeavyRocketSimulatorVR-<version>`. `project.godot`
  `config/name` changes from "Starship Simulator" to it (trademarks internal only).
- The repo is private, so the installer cannot download data from the release: **the installer contains all data**;
  the terrain and imagery are optional components (checked by default). Without them the game runs with the flat
  ground and no aerial imagery (existing fallbacks).
- Release assets: `...-setup.exe` (everything), `...-windows.zip` (game + base data, playable), `...-data-terrain.zip`
  (`data/terrain/earth`), `...-data-imagery.zip` (`data/imagery`). Each unpacks into the game folder. Each must be
  < 2 GB (the script fails otherwise).
- Base data = `data/` minus `raw/`, `webcasts/`, `validation/`, `terrain/earth/`, `imagery/`, `README.md`
  (i.e. spice, geoid, stars, textures, fonts, iss_oem, lunar terrain).
- New script `tools/package.ps1` (our `tools/build.ps1` already builds the core). Tests in it: core + bindings build
  (`tools/build.ps1 -NoTests`), the Godot smoke test, and a self-check of the EXPORTED exe in the package folder. No
  ctest suite (user rule: run only the affected tags during development). `-SkipTests` skips smoke + self-check.
- The GDExtension DLL stays `template_debug` (the `.gdextension` already maps release to it); the export copies it.
- Installer per user, no admin: `%LOCALAPPDATA%\Programs\Super Heavy Rocket Simulator VR`; start-menu entries run the
  release `.cmd` launchers (they set `SDL_GAMECONTROLLERCONFIG_FILE` for the HOTAS axes) minimised.
- No LICENSE file exists: the installer has no licence page; CREDITS.md ships. (Open point for the user: a licence
  for the own code.)

## Global Constraints

- Code, comments, docs, commit messages English; UI strings German; the commands reply in German.
- Conventional Commits; work on `main`, push after each task.
- The Godot executable for build steps: `external/godot/bin/godot.windows.editor.double.x86_64.console.exe` (or
  `GODOT=` from `start.cfg`, as `Start-Desktop.cmd` does).
- Release template: `external/godot/bin/godot.windows.template_release.double.x86_64.exe`, built by
  `scons platform=windows target=template_release precision=double production=yes` in `external/godot` (OpenXR patch
  already applied there).
- Long builds (template ~10-15 min, installer compression) run in the background with output in a file.
- `build/` is git-ignored; nothing built is committed.

## Review Focus

- Game started from a folder path with spaces (`%LOCALAPPDATA%\Programs\Super Heavy ...`): `.cmd` launchers and
  `Paths` must quote/handle it - the Task 3 self-check runs from `build\package` AND Task 4 installs to the default dir.
- Package without the terrain / imagery parts: the game must start (flat ground, no imagery) - Task 3 self-check runs
  once with `data/terrain/earth` and `data/imagery` absent.
- Editor runs (`Start-Desktop.cmd`, all `--script` tests) must still find `data/` and `config/` at the repo root after
  the `Paths` change - Task 1 runs the smoke test.
- An installer over an older installation (same AppId) replaces the files; a component deselected later must not leave
  a stale half: `[InstallDelete]` for `data\terrain\earth` and `data\imagery` before copying - Task 4.
- A data part above 2 GB or a missing `data/` folder at build time: the script stops with a clear message - Task 3.

---

### Task 1: Exported-game layout, version and self-check

**Files:**
- Modify: `game/util/paths.gd`
- Create: `game/util/game_version.gd`
- Modify: `game/main.gd`, `game/project.godot` (`config/name`)
- Modify: `release-please-config.json` (`extra-files`), `.github/workflows/release-please.yml`
  (`skip-github-release: true`, header comment as the sibling's)

**Interfaces:**
- Produces: `Paths.repo_dir()` = `OS.get_executable_path().get_base_dir()` when `OS.has_feature("template")`, else the
  current `res://..`; `GameVersion.VERSION` (`const VERSION := "0.0.0" # x-release-please-version`, class_name
  `GameVersion`); user arg `--self-check` (after `--`): main.gd waits until the world is set up, prints exactly one line
  `SELF-CHECK OK version=<v> spice=ok sites=<n> earth_terrain=<yes|no> imagery=<yes|no>` or
  `SELF-CHECK FAIL <reason>` and quits with exit code 0 / 1.

- [ ] **Step 1:** Implement `Paths`, `GameVersion`, the window title `"<config/name> <VERSION>"`, and `--self-check`
  (spice opened via `core.open`, `sites.size() > 0`, earth terrain = the flight session has its terrain loaded,
  imagery = `data/imagery/earth/index.toml` exists). Set `config/name="Super Heavy Rocket Simulator VR"`.
- [ ] **Step 2:** Run in the editor binary:
  `godot...console.exe --headless --xr-mode off --path game -- --no-xr --self-check` -> `SELF-CHECK OK ... earth_terrain=yes imagery=yes`, exit 0.
- [ ] **Step 3:** Smoke test still passes (`--headless --path game --script res://tests/smoke_test.gd`).
- [ ] **Step 4:** release-please: `"extra-files": ["game/util/game_version.gd"]`; workflow `skip-github-release: true`.
- [ ] **Step 5:** Commit `feat(game): version, self-check and executable-relative paths for exported builds`; push.

### Task 2: Export preset and release template

**Files:**
- Create: `game/export_presets.cfg` (preset `"Windows Desktop"`, `embed_pck=true`, `exclude_filter="tests/*"`,
  `custom_template/release` = absolute-free path resolved by package.ps1 at export time, `application/*` names =
  product name, `export_d3d12` as needed for the D3D12 driver)
- Create: `tools/package.ps1` (first steps only: Godot + template + export)

**Interfaces:**
- Produces: `tools/package.ps1 [-Version x.y.z[-suffix]] [-SkipTests] [-NoInstaller]`; steps 1-3 here: find Godot,
  build the template if missing (scons, background-friendly log `build/template_build.log`), export
  `build/SuperHeavyRocketSimulatorVR.exe` with the version stamped (as the sibling, restoring the preset afterwards).
  The preset's `custom_template/release` is written for the export only, like the version stamp.

- [ ] **Step 1:** Build the template once (background, log to file); confirm the exe exists.
- [ ] **Step 2:** Write the preset and package.ps1 steps 1-3 (core + bindings via `tools/build.ps1 -NoTests`, smoke
  test unless `-SkipTests`, export).
- [ ] **Step 3:** Run `package.ps1 -NoInstaller -Version 0.0.0-dev` up to the export; the exe and
  `starsim.windows.template_debug.x86_64.dll` (and any D3D12 DLLs) lie in `build/`. Start it by hand from `build/`
  with `-- --no-xr --self-check --headless`: it must FAIL with a clear missing-data reason (no data next to it), not
  crash.
- [ ] **Step 4:** Commit `build: Godot export preset and release template build`; push.

### Task 3: Package folder, data parts, ZIPs

**Files:**
- Create: `tools/release/Start-VR.cmd`, `tools/release/Start-Desktop.cmd` (exe next to them; set
  `SDL_GAMECONTROLLERCONFIG_FILE=%~dp0config\sdl_gamecontrollerdb.txt`; `start "" "%~dp0SuperHeavyRocketSimulatorVR.exe" --xr-mode on -- --vr`
  / `--xr-mode off -- --no-xr`)
- Modify: `tools/package.ps1`

**Interfaces:**
- Produces: `build/package/` = exe, DLLs, the two `.cmd`, `config/`, base `data/`, `CREDITS.md`, `README.md`,
  `README.de.md`; `build/package_terrain/data/terrain/earth`, `build/package_imagery/data/imagery` (staging for the
  parts); ZIPs `build/SuperHeavyRocketSimulatorVR-<v>-windows.zip`, `-data-terrain.zip`, `-data-imagery.zip` via
  `tar.exe -a -c -f <zip> -C <dir> *`.

- [ ] **Step 1:** Package steps: stop if `data/spice` is missing ("run tools/data_pipeline/fetch.py"); copy per the
  Decisions; size check: any ZIP >= 2 GB -> throw.
- [ ] **Step 2:** Self-check of the package (unless `-SkipTests`): run the packaged exe headless with `--self-check`
  twice - base only (expect `earth_terrain=no imagery=no`) and with the parts copied in (expect `yes yes`); a FAIL
  line or exit != 0 throws. Use a package path with a space for the second run (`build/package test`) - Review Focus.
- [ ] **Step 3:** Run `package.ps1 -NoInstaller -Version 0.0.0-dev`; report the ZIP sizes; launch
  `build\package\Start-Desktop.cmd` by hand once (window opens, world renders) - screenshot via the user or a
  `render_check`-style capture is not needed.
- [ ] **Step 4:** Commit `build: release package with data parts and portable ZIPs`; push.

### Task 4: Inno Setup installer

**Files:**
- Create: `tools/installer.iss` (from the sibling's: new AppId GUID, product name/URLs of
  `Tachy/Super-Heavy-Rocket-Simulator-VR`, no LicenseFile, German + English, `[Components]` `main` (fixed),
  `terrain` "Erd-Gelände (hochauflösend)", `imagery` "Luftbilder der Startplätze", both default; `[InstallDelete]`
  `{app}\data\terrain\earth` and `{app}\data\imagery`; `[Files]` from `/DSourceDir`, `/DTerrainDir`, `/DImageryDir`;
  `[Icons]` to the `.cmd` launchers with `Flags: runminimized`; desktop icon task; post-install run desktop mode,
  unchecked)
- Modify: `tools/package.ps1` (installer step: find or `winget`-install Inno Setup as the sibling)

- [ ] **Step 1:** Write the .iss and the step; output `build/SuperHeavyRocketSimulatorVR-<v>-setup.exe`; fail if >= 2 GB
  (then switch `DiskSpanning`, report to the user).
- [ ] **Step 2:** Build `package.ps1 -Version 0.0.0-dev` (background). Install silently into a temp dir
  (`/VERYSILENT /DIR=<scratch>\inst test /CURRENTUSER`), run the installed exe's `--self-check` -> OK yes/yes;
  uninstall silently; the dir is gone.
- [ ] **Step 3:** Commit `build: Inno Setup installer`; push.

### Task 5: /build, /build-local, docs

**Files:**
- Create: `.claude/commands/build.md`, `.claude/commands/build-local.md` (the sibling's text adapted: no server;
  version file `game/util/game_version.gd`; script `tools/package.ps1`; assets = setup.exe + three ZIPs; build-local
  `fast` -> `-SkipTests`; install dir and start-menu names of this product; note the private repo: assets are visible
  only to collaborators)
- Modify: `CLAUDE.md` (Build & test: release build paragraph; one-time template build line), `README.md` +
  `README.de.md` (Installation: installer / ZIP + data parts / from source), `config/README.md` untouched.

- [ ] **Step 1:** Write commands and docs.
- [ ] **Step 2:** Run `/build-local` flow end to end (`package.ps1 -Version <GameVersion>-local`); report files and
  sizes; ask the user to try the installer (VR + desktop).
- [ ] **Step 3:** Commit `docs: release build commands and installation`; push. Record the plan as complete in memory;
  ask for `/clear`.
