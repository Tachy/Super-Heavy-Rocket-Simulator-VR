---
description: Release the version release-please has prepared - build the game, its data parts and the installer, then publish them as one GitHub release
---

Release the simulator: build the Windows game (exe, ZIPs, installer) for the version release-please has prepared,
then publish everything as one GitHub release. Reply in German. Do the steps in order and stop on any error (show the
error, do not work around it).

How the pieces fit: the GitHub Action (`.github/workflows/release-please.yml`) only keeps a release PR up to date
(version in `game/util/game_version.gd`, `CHANGELOG.md`, manifest) from the Conventional Commits on main. It never
publishes (`skip-github-release`). A release exists only through this command, and it appears with all its files at
once. The repository is private: the release and its assets are visible to collaborators only.

## 1. Starting point

- `git status` must be clean (untracked scratch like `tmp/` aside) and the branch `main`, up to date with
  `origin/main` (`git fetch` then compare). If not: say so and stop.
- The runtime data must be complete: `data/spice`, `data/terrain/earth`, `data/imagery/earth/index.toml` (the script
  checks it too; else `python tools/data_pipeline/fetch.py`).
- Find the open release PR of release-please:
  `gh pr list --state open --label "autorelease: pending" --json number,title,headRefName`
  Its title is `chore(main): release X.Y.Z`; X.Y.Z is the version.
- No release PR: nothing to release (no new feat:/fix: commits since the last release). Say so and stop - no build.

## 2. Build on the release PR's state

- `gh pr checkout <number>` (the PR branch carries X.Y.Z in `game/util/game_version.gd` and `CHANGELOG.md`). Check
  that game_version.gd has X.Y.Z; if not, stop (release-please config is off).
- `powershell -ExecutionPolicy Bypass -File tools/package.ps1 -Version X.Y.Z` (core + extension, smoke test, export,
  self-check of the exported build, ZIPs, installer; ~15-25 min, the first time plus the template build). Run it in
  the background with its output in `build/package_run.log` and report progress.
- Write the release notes: the section `## [X.Y.Z]...` of `CHANGELOG.md` up to the next `## [` heading, into
  `build/release-notes-X.Y.Z.md`. Append a short German/English install note: the installer contains everything; the
  ZIP `...-windows.zip` is playable on its own, `...-data-terrain.zip` and `...-data-imagery.zip` unpack into the same
  folder (Earth terrain and aerial imagery of the launch sites).
- `git checkout main`.
- Show the user the files with sizes - `build/SuperHeavyRocketSimulatorVR-X.Y.Z-setup.exe`, `...-windows.zip`,
  `...-data-terrain.zip`, `...-data-imagery.zip` - and the release notes, and ask (AskUserQuestion) whether to
  publish: they may first want to try `build\package\Start-Desktop.cmd` / `Start-VR.cmd` or the installer. Without a
  yes: stop here, the build stays in `build/`.

## 3. Publish

- Merge the release PR: `gh pr merge <number> --squash`. Its merge commit:
  `gh pr view <number> --json mergeCommit -q .mergeCommit.oid`.
- Create tag and release with all files in one step (gh uploads the files to a draft and publishes it only then, so
  the release never shows up without them; the four files are ~1-2 GB, the upload takes a while - run it in the
  background):
  `gh release create vX.Y.Z --target <merge commit> --title vX.Y.Z --notes-file build/release-notes-X.Y.Z.md --latest build/SuperHeavyRocketSimulatorVR-X.Y.Z-setup.exe build/SuperHeavyRocketSimulatorVR-X.Y.Z-windows.zip build/SuperHeavyRocketSimulatorVR-X.Y.Z-data-terrain.zip build/SuperHeavyRocketSimulatorVR-X.Y.Z-data-imagery.zip`
- Mark the PR as released, as release-please itself would (otherwise it stops preparing the next release PR:
  "untagged, merged release PRs outstanding"):
  `gh pr edit <number> --remove-label "autorelease: pending" --add-label "autorelease: tagged"`
- `git pull` on main (the release commit).
- Report the release URL (`gh release view vX.Y.Z --json url`) and its assets.
