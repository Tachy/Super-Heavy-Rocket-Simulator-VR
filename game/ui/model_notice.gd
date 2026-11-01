class_name ModelNotice
## The on-screen notice for a view that shows placeholder shapes because its generated models are stale or missing
## (StructureModels / VehicleModels keep the reason). Models that do not exist by design ("none") give no notice.

const STALE_TEXT := "Platzhaltermodell: Modelle veraltet (build_models ausführen)"
const MISSING_TEXT := "Platzhaltermodell: Modelle fehlen (build_models ausführen)"
const FONT_SIZE := 48
const PIXEL_SIZE_M := 0.0007
const OUTLINE_SIZE := 12
const COLOR := Color(1.0, 0.75, 0.2)


## German notice text for a loader's fallback reason ("stale" | "missing" | "none" | ""); "" when nothing is to be said.
static func text_for(reason: String) -> String:
	match reason:
		"stale":
			return STALE_TEXT
		"missing":
			return MISSING_TEXT
	return ""


## A billboard label with `text` (fixed apparent size, drawn over the geometry), or null for an empty text.
static func label(text: String) -> Label3D:
	if text == "":
		return null
	var node := Label3D.new()
	node.name = "ModelNotice"
	node.text = text
	node.font_size = FONT_SIZE
	node.pixel_size = PIXEL_SIZE_M
	node.outline_size = OUTLINE_SIZE
	node.modulate = COLOR
	node.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	node.fixed_size = true
	node.no_depth_test = true
	return node
