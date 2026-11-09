# Bug: Sun flickers in VR between plain disc and disc with halo (2026-11-09)

Status: DONE (verified in the stereo mock with OpenXR's pose timing; not yet on the headset).

## Root cause
`AdaptationMeter` masked the Sun disc with eye poses read on the MAIN thread (World `_adaptation_view`:
`interface.get_transform_for_view` during `_process`). In OpenXR the view poses are located right before rendering
(`OpenXRAPI::pre_render` -> `render_state.views`), so a main-thread read returns the poses the PREVIOUS frame was
rendered with. The frame itself is drawn with this frame's poses. When the head turns, the mask (radius 1.2 x 0.27 deg)
sits one frame of head rotation away from the drawn disc. The "seen" counter then falls (0.4 deg/frame: 78 061 -> 20 455)
and reaches 0 at about 0.5 deg/frame or more (a normal 45+ deg/s head turn at 90 Hz). That sets
`Atmosphere.last_veil_visibility` to 0, which turns off the Sun's analytic veil in the glare spread (`sun_scene_lux` ~ visibility)
and the meter's veil. The result is a plain disc while the head moves and the halo back when it slows down.
The desktop is unaffected: there the camera pose is the same in both places. The stereo mock had a static head.
Ruled out: per-eye state, frame parity, the local factor clamp, the glare compositor views. The glare spread already
takes its eye frame from the render data (a93d4938). The async readback is not the cause: the 4-frame result cadence only
holds a wrong value for 4 frames.

## Fix
`AdaptationMeter.sun_mask_parameters(view, projection, head, size)` now builds the mask on the render thread from the
render data: `scene_data.get_view_projection(i)` and `get_cam_transform()` via `GlareSpreadEffect.view_frame`, which is
the same eye frame the glare spread uses and also covers canted displays and Vulkan's y flip. From the view dictionary
it takes only the Sun's world direction and the mask cosine. No smoothing was added.

## Reproduction / gate
- `StereoMockXr`: `head_yaw_amplitude_rad` / `head_yaw_period_frames`. A sinusoidal head yaw whose poses are updated in
  `_pre_render`, as OpenXR does, so main-thread reads see the previous frame's poses.
- `render_check --stereo=<json> --halo_sequence=40 --head_turn=<peak deg/frame>,<period>`: every frame reads back both eye
  images and measures the rings 0.4-1 / 1-2.5 / 3-5 / 8-12 deg around the Sun in each eye, using the pose that frame was
  rendered with (inner two rings on every pixel). It also logs the meter's terms (visibility, mask / seen, veil). It fails
  when a ring steps by more than 2 % of max(mean, 0.002), and prints the visibility range.
- Numbers (Sun in space from 400 km, Pimax JSON, `--head_turn=0.8,40`):
  A (before): visibility 0 on 22 of 40 frames, seen 0 .. 78 061, 0.4-1 deg ring 0.0029 <-> 0.0000 (halo gone on 23 frames),
  worst step 144 % FAIL. At 0.4 deg/frame seen 20 455 .. 78 061, with visibility still saturated at 1.
  B (after): space 400 km 0.70 % ok, 200 km 0.70 % ok, ground noon 0.10 % ok, visibility 1.0 throughout, seen 77 844 .. 79 289.
  At 0.4 deg/frame: space 1.66 % (sparse ring sampling then) / ground 0.04 %.
- Desktop check (400 km, 60 deg, drift 0.013): mask 15 300 / seen 13 260, visibility 1 (unchanged behaviour). The
  desktop sequence's 31 % "step" is the known 8-bit quantisation of the 1e-5 rings (the desktop gate has no floor).

## Tests
`adaptation_meter_test` PASS (new case: the mask follows the render pose, stale eyes ignored; 0.4 deg turn),
`glare_veil_test` PASS, `stereo_mock_xr_test` PASS, `smoke_test` PASSED.

## Images (screenshots/look_fixes/bugs-2026-11-09/sun_vr/)
`sheet_space_t08_A_B.png` (eye 0 crops x8, frames 10 / 20 / 30; A frame 20 plain disc, B halo throughout),
`A_/B_*_t08.log` (time series), `*_halo0N0_eye0/1.png`.

## Docs
eye-adaptation.md (Sun mask from the render data, VR gate), history/eye-adaptation.md (superseded mask + A numbers),
setup.md (render_check `--head_turn`).

## Open
- Still stale by one frame (smaller effects, not changed): `Atmosphere.glare_veil`'s in-frame test and
  `AdaptationSurround`'s frame weights read the same main-thread eyes. Near the frame edge a fast turn can move the
  in-frame decision by one frame. Out of frame the visibility is the surround's V (1 in the open), so the Sun's veil does
  not switch off there.
- Not verified on the headset.
