#!/usr/bin/env bash
# Pad B evidence renders (GPU, one at a time): bash tools/structures/measure/render_views.sh <prefix>
# Writes screenshots/pad-b/<prefix>_<view>.png through render_check.gd; logs next to the PNGs.
set -u
ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
G="$ROOT/external/godot/bin/godot.windows.editor.double.x86_64.console.exe"
OUT="$ROOT/screenshots/pad-b"
P="${1:-t5}"
UTC="--utc=2026-10-12T14:10:00"
V="python $ROOT/tools/structures/measure/view_args.py"
mkdir -p "$OUT"

shot() {  # name, extra args...
  local name="$1"; shift
  timeout 300 "$G" --path "$ROOT/game" --script res://tests/render_check.gd -- $UTC "$@" --out="$OUT/${P}_${name}.png" \
    > "$OUT/${P}_${name}.log" 2>&1
  tail -1 "$OUT/${P}_${name}.log"
}

# catch camera: at the arms' height, 160 m from the tower, arms open / standby / closed
shot catch_open    $($V 160 95 95 135) --fov=45 --tower_arms_deg=40
shot catch_standby $($V 160 95 95 135) --fov=45 --tower_arms_deg=6.5
shot catch_closed  $($V 160 95 95 135) --fov=45 --tower_arms_deg=0
# pad at 2 m eye height, 60 m from the tower, looking up
shot pad2m         $($V 60 2 80 135) --fov=75 --tower_arms_deg=6.5
# 1 km
shot km1           $($V 1000 2 70 135) --fov=25 --tower_arms_deg=6.5
# stereo mock (Pimax Dream Air projections), catch camera
shot stereo        $($V 160 95 95 135) --tower_arms_deg=6.5 --stereo=$ROOT/game/tests/data/pimax_dream_air_projections.json
