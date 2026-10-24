---
description: Build a local test installer of the current state (no release, nothing published)
---

Build a Windows test installer of the current working state so the user can try installing it. Reply in German.
Nothing is merged, tagged, uploaded or committed - this is not a release (that is /build). Stop on any error and show
it, do not work around it.

1. Version: read it from `game/util/game_version.gd` (`const VERSION := "x.y.z"`) and append `-local`, e.g.
   `0.1.0-local` (the exe's file version takes the numbers only).
2. Build: `powershell -ExecutionPolicy Bypass -File tools/package.ps1 -Version <x.y.z>-local` (core + extension, smoke
   test, export, self-check of the exported build, ZIPs, installer). If the user passed `fast` as an argument, add
   `-SkipTests`. Run it in the background with its output in `build/package_run.log` and report progress.
3. Report:
   - the installer `build\SuperHeavyRocketSimulatorVR-<x.y.z>-local-setup.exe` with its size (and the three ZIPs next
     to it),
   - whether the working tree had uncommitted changes (`git status --short`): they are in this build,
   - how to test: run the installer (per user, no admin rights; it installs to
     `%LOCALAPPDATA%\Programs\Super Heavy Rocket Simulator VR`; the Earth terrain and the aerial imagery are
     components that can be deselected), start it from the start menu ("Super Heavy Rocket Simulator VR" = VR,
     "... (Desktop-Modus (ohne VR))" = desktop), uninstall via Windows "Apps" (Installierte Apps). Without
     installing: `build\package\Start-Desktop.cmd` (base data only).

`build/` is in `.gitignore`: the files are never committed.
