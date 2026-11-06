class_name ModelNotice
## The on-screen notice for a view that shows placeholder shapes because its generated models are stale or missing
## (StructureModels / VehicleModels keep the reason). Models that do not exist by design ("none") give no notice.

const STALE_KEY := "notice.stale"
const MISSING_KEY := "notice.missing"
const FONT_SIZE := 48
const PIXEL_SIZE_M := 0.0007
const OUTLINE_SIZE := 12
const COLOR := Color(1.0, 0.75, 0.2)


## Text-table key of the notice for a loader's fallback reason ("stale" | "missing" | "none" | ""); "" when nothing is to be said.
static func text_for(reason: String) -> String:
	match reason:
		"stale":
			return STALE_KEY
		"missing":
			return MISSING_KEY
	return ""


## A billboard label showing the notice `key` (fixed apparent size, drawn over the geometry; it follows a language
## switch), or null for an empty key.
static func label(key: String) -> Label3D:
	if key == "":
		return null
	var node := Label3D.new()
	node.name = "ModelNotice"
	I18n.bind_text(node, &"text", key)
	node.font_size = FONT_SIZE
	node.pixel_size = PIXEL_SIZE_M
	node.outline_size = OUTLINE_SIZE
	node.modulate = COLOR
	node.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	node.fixed_size = true
	node.no_depth_test = true
	return node
