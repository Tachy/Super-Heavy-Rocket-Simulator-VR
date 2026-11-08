# Look fixes after the eye-adaptation comparison (2026-11-07)

User decision 2026-11-07: "Alles wie vorgeschlagen umsetzen" - the proposals of the look investigation
(`.superpowers/look-investigation-2026-11-07.md`, images `screenshots/look_investigation/`) in the recommended order.
Field rule D (highlight protection) is the game default since b2a7fe16.

Tasks run one after another (they share the GPU and partly the same shaders). Each: implementer, A/B renders against
the previous state into `screenshots/look_fixes/<task>/`, affected tests only, commit + push, Sonnet review.

| # | Task | Kind | Model |
|---|---|---|---|
| T1 | F6: start view and the comparison render setups with weather (`analog`); the Earth from space shows clouds | usage | Sonnet |
| T2 | Clouds seen from far away (start view 2e7 m, orbit) blocky and salmon-coloured: find the cause, fix | bug | Opus |
| T3 | F5: ocean reflectance from water optics (open-ocean water-leaving reflectance ~0.002/0.006/0.025 R/G/B plus Fresnel sky reflection and sun glint) instead of the imagery's ocean colour; land keeps the imagery | physics/data | Opus |
| T4 | F4: the metering field wider than the frame (~60 deg around the view direction, e.g. from the sky radiance cube + frame), so looking up at the sky does not brighten it by a stop; rule D otherwise unchanged | perception | Opus |
| T5 | F2 + F3 colour package: daylight Hunt effect (CIECAM02 colourfulness above 1e4 cd/m^2, no cap at 1) and a chroma-preserving tone curve in place of plain AgX (hue-stable: no blue -> violet shift) | perception/display | Opus |
| T7 | Cloud cover from space too closed / smeared vs DSCOVR EPIC (user 2026-11-07): typical-weather monthly means on a 1 deg grid give a near-overcast veil; find what makes real broken cloud fields (snapshot data at 0.25 deg / satellite cloud imagery / sub-grid cloud structure from cover + cloud type) and make the Earth from space read like EPIC | data/model | Opus |
| T8 | Bug (user 2026-11-07): a jump in cloud cover at ~55 km altitude - the two cloud representations on either side do not match (tier / level-of-detail hand-over) | bug | Opus |
| T9 | Atmosphere spectrum check (user 2026-11-08): direct beam 4936 K at AM 1.35 from the 3-wavelength model + AOD 0.1 vs measured noon daylight 5500-6500 K; measure a finer spectrum / aerosol assumption and PROPOSE - no atmosphere config change without user OK | investigation | Opus |
| T10 | Bug (user 2026-11-08): Earth from the lunar night fully overexposed with a blur rim; find the rim source (Godot glow vs physical glare veil) and add local (retinal) adaptation so small bright objects keep detail; switchable | bug/perception | Opus |
| T11 | ON HOLD (likely folded into T13): Moon day keeps the brighter key (DAY_EXPOSURE 1.5 for the Moon only, documented exception; user 2026-11-08) - after T10 | perception | Sonnet |
| T12 | ON HOLD (likely folded into T13): Earth from space too white: gentler highlight shoulder above the TOA-Sun white plate, switchable, measured vs EPIC (user 2026-11-08) - after T7c | display | Opus |
| T13 | Unified appearance model (user 2026-11-08): one observer model (iCAM06 / Pattanaik multiscale style: global temporal + local adaptation, chromatic adaptation and colour appearance, fixed display mapping) replacing the special-case rules; reference scene suite with measured / photo targets that every later change must pass - after T10 | perception | Opus |
| T6 | F1: DAY_EXPOSURE out of the field rule (the field lands at 0.18, not 0.27); compare against T5, user verdict on brightness (lunar day must stay bright enough - user prefers the bright Moon) | bug fix | Sonnet |

After T6: Opus final review over the whole range, then the M7 cockpit plan continues (look follow-up: helmet shell
from `screenshots/m7-cockpit-refs/user/anzug/spacex-anzug.jpg`, console modules with red strips / labelled buttons /
T-handle; then Task 6). Local (retinal) adaptation for the Earth from the lunar night: offered, not yet decided.

Ledger: `.superpowers/sdd/2026-11-07-look-fixes/progress.md`.
