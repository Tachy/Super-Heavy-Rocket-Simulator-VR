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
| T6 | F1: DAY_EXPOSURE out of the field rule (the field lands at 0.18, not 0.27); compare against T5, user verdict on brightness (lunar day must stay bright enough - user prefers the bright Moon) | bug fix | Sonnet |

After T6: Opus final review over the whole range, then the M7 cockpit plan continues (look follow-up: helmet shell
from `screenshots/m7-cockpit-refs/user/anzug/spacex-anzug.jpg`, console modules with red strips / labelled buttons /
T-handle; then Task 6). Local (retinal) adaptation for the Earth from the lunar night: offered, not yet decided.

Ledger: `.superpowers/sdd/2026-11-07-look-fixes/progress.md`.
