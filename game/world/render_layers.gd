class_name RenderLayers
## Visual layers (VisualInstance3D.layers / Camera3D.cull_mask). Layer 1 (bit 0) is the default every node is on.
## OVERLAY: guidance and HUD geometry in the world (planned orbit, flown trail, Moon orbit line, map labels). These are aids for
## the pilot, not part of the scene a physical camera sees: the cabin's screen camera (ScreenCamera) excludes them.
## The main camera keeps the default cull mask (all layers).

const OVERLAY := 1 << 12  ## bit 12; bit 11 is CockpitView.CONSOLE_LAYER


## Moves a node onto the OVERLAY layer only (a camera sees a node when its cull mask shares any bit with the node's layers: the
## main camera's default mask has it, the feed camera's does not).
static func mark_overlay(node: VisualInstance3D) -> void:
	node.layers = OVERLAY
