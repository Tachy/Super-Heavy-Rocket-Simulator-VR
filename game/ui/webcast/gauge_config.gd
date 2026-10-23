class_name GaugeConfig
extends RefCounted
## The webcast overlay's gauge selection and looks, read from config/ui/webcast_gauges.toml. Load problems are
## collected in `errors` (each also push_error'ed): an unusable gauge or list entry is dropped, the rest stays usable.

const CONFIG_FILE := "ui/webcast_gauges.toml"
const SITUATIONS: Array[String] = ["docking", "transfer", "orbit", "default"]

static var _shared: GaugeConfig

var hide_delay_s := 0.0
var max_per_side := 0
## name -> float; a threshold the file does not define reads 0.0.
var thresholds := {}
## id -> {label, unit, full_scale, decimals, log_min?, signed?, condition}
var gauges := {}
var left: Array[String] = []
## situation name -> Array[String] (right side by priority)
var situations := {}
var errors: Array[String] = []


## The configuration of the shipped file, loaded on first use.
static func shared() -> GaugeConfig:
	if _shared == null:
		_shared = load_file(Paths.config_dir().path_join(CONFIG_FILE))
	return _shared


static func load_file(path: String) -> GaugeConfig:
	if not FileAccess.file_exists(path):
		var missing := GaugeConfig.new()
		missing._error("cannot open %s" % path)
		return missing
	return parse(FileAccess.get_file_as_string(path))


static func parse(text: String) -> GaugeConfig:
	var config := GaugeConfig.new()
	var data := TomlLite.parse(text)
	var rules: Dictionary = data.get("rules", {})
	config.hide_delay_s = float(rules.get("hide_delay_s", 0.0))
	config.max_per_side = int(rules.get("max_per_side", 0))
	for key: String in data.get("thresholds", {}):
		config.thresholds[key] = float(data["thresholds"][key])
	for key: String in data:
		if key.begins_with("gauge."):
			config._add_gauge(key.trim_prefix("gauge."), data[key])
	config.left = config._known_ids(data.get("left", {}).get("gauges", []), "[left]")
	for key: String in data:
		if key.begins_with("situation."):
			var name := key.trim_prefix("situation.")
			if name in SITUATIONS:
				config.situations[name] = config._known_ids(data[key].get("gauges", []), "[%s]" % key)
			else:
				config._error("unknown situation '%s'" % name)
	return config


func threshold(name: String) -> float:
	return float(thresholds.get(name, 0.0))


func _add_gauge(id: String, spec: Dictionary) -> void:
	var condition := String(spec.get("condition", ""))
	if not condition in GaugeRules.CONDITIONS:
		_error("gauge '%s': unknown condition '%s'" % [id, condition])
		return
	for key in ["label", "unit", "full_scale", "decimals"]:
		if not spec.has(key):
			_error("gauge '%s': missing '%s'" % [id, key])
			return
	gauges[id] = spec


func _known_ids(ids: Array, where: String) -> Array[String]:
	var known: Array[String] = []
	for id: String in ids:
		if gauges.has(id):
			known.append(id)
		else:
			_error("%s: unknown gauge '%s'" % [where, id])
	return known


func _error(message: String) -> void:
	var text := "GaugeConfig: " + message
	errors.append(text)
	push_error(text)
