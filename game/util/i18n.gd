class_name I18n
## The game's language service: texts from config/i18n/<language>.toml, reports from the core resolved to text,
## numbers formatted per physical kind and locale, the language setting (user://settings.cfg [ui] language) and a
## change signal. Static (no autoload): works in --script tests and is ready before the loading screen.
## Displays keep reports as data and call report_text / text at draw time, so a live switch re-renders them.

const LANGUAGES: Array[String] = ["de", "en"]
const DEFAULT_LANGUAGE := "de"
const DASH := "–"  # i18n-ok: the symbol of a missing value, same in every language

static var events: I18nEvents = I18nEvents.new()
static var settings_path := "user://settings.cfg"

static var _language := ""
static var _de: Dictionary = {}
static var _en: Dictionary = {}
static var _loaded := false
static var _warned: Dictionary = {}
static var _bound: Array = []
static var _bound_connected := false


## The active language code ("de" / "en"); read from the settings file on first use.
static func language() -> String:
	_ensure()
	return _language


## Switches the language: saves it and emits events.language_changed. An unknown code is a warning and changes nothing.
static func set_language(code: String) -> void:
	_ensure()
	if not LANGUAGES.has(code):
		push_warning("I18n: unknown language '%s'" % code)
		return
	var changed := code != _language
	_language = code
	var config := ConfigFile.new()
	config.load(settings_path)  # keeps the other settings; a missing file starts empty
	config.set_value("ui", "language", code)
	config.save(settings_path)
	if changed:
		events.language_changed.emit(code)


## The language stored in the settings file; "de" when there is none or the value is unknown.
static func saved_language() -> String:
	var config := ConfigFile.new()
	if config.load(settings_path) != OK:
		return DEFAULT_LANGUAGE
	var code := str(config.get_value("ui", "language", DEFAULT_LANGUAGE))
	return code if LANGUAGES.has(code) else DEFAULT_LANGUAGE


## Reloads the tables (not named reload: that is Script.reload) from config/i18n and the language from the settings file.
static func reload_tables() -> void:
	_loaded = false
	_language = ""
	_ensure()


## Test hook: replaces both tables (flat key -> text).
static func use_tables(de: Dictionary, en: Dictionary) -> void:
	_de = de
	_en = en
	_loaded = true
	_warned.clear()
	if _language == "":
		_language = saved_language()


static func has_key(key: String) -> bool:
	_ensure()
	return _de.has(key) or _en.has(key)


## The text of a key in the active language (English falls back to German). A key in neither table is returned as it is,
## with one warning per key. Placeholders {name} / {name:spec}: String values verbatim, numbers formatted per spec.
static func text(key: String, params: Dictionary = {}) -> String:
	var template := _template(key)
	return key if template == "" else _fill(template, params, {}, [])


## The raw table text of a key (active language, German fallback); "" and one warning per key when there is none.
static func _template(key: String) -> String:
	_ensure()
	var template := ""
	if _language == "en" and _en.has(key):
		template = str(_en[key])
	elif _de.has(key):
		template = str(_de[key])
	else:
		if not _warned.has(key):
			_warned[key] = true
			push_warning("I18n: no text for key '%s'" % key)
	return template


## A report dictionary from the bindings ({code, params, kinds, causes}) as text; an ok report is "".
static func report_text(report: Dictionary) -> String:
	var code := str(report.get("code", "ok"))
	if code == "ok" or code == "":
		return ""
	var template := _template("report." + code)
	if template == "":
		return "report." + code
	return _fill(template, report.get("params", {}), report.get("kinds", {}), report.get("causes", []))


## The text of a phase / state id as the bindings send it ("<namespace>.<id>"); "" for no id.
static func phase_text(qualified_id: String) -> String:
	if qualified_id == "":
		return ""
	return text("phase." + qualified_id)


## A value in SI as text. spec = unit + decimals (deg1, t1, kg0, km2, m0, mps1, kmh0, s0, min1, h1, pct0, n2, int,
## clock); an empty spec takes the kind's default (angle deg1, mass t1, length km1, speed mps1, time s0, count int,
## number n2). NaN and infinity are a dash.
static func format_value(value: float, kind: String, spec: String = "") -> String:
	if is_nan(value) or is_inf(value):
		return DASH
	var effective := spec if spec != "" else _default_spec(kind)
	var digits := effective.length()
	while digits > 0 and effective[digits - 1] >= "0" and effective[digits - 1] <= "9":
		digits -= 1
	var unit := effective.substr(0, digits)
	var decimals := int(effective.substr(digits)) if digits < effective.length() else 0
	match unit:
		"deg":
			return number(rad_to_deg(value), decimals) + "°"  # i18n-ok: unit symbol
		"t":
			return number(value / 1000.0, decimals) + " t"
		"kg":
			return number(value, decimals) + " kg"
		"km":
			return number(value / 1000.0, decimals) + " km"
		"m":
			return number(value, decimals) + " m"
		"mps":
			return number(value, decimals) + " m/s"
		"kmh":
			return number(value * 3.6, decimals) + " km/h"
		"s":
			return number(value, decimals) + " s"
		"min":
			return number(value / 60.0, decimals) + " min"
		"h":
			return number(value / 3600.0, decimals) + " h"
		"pct":
			return number(value * 100.0, decimals) + " %"
		"int":
			return number(value, 0)
		"clock":
			return _clock(value)
		"utc":
			return _utc(value, true)
		"date":
			return _utc(value, false)
	return number(value, decimals)


## A plain number with a fixed count of decimals and the active language's decimal separator; no negative zero.
static func number(value: float, decimals: int) -> String:
	if is_nan(value) or is_inf(value):
		return DASH
	var result := ("%." + str(maxi(decimals, 0)) + "f") % value
	if result.begins_with("-") and result.trim_prefix("-").replace("0", "").replace(".", "") == "":
		result = result.substr(1)
	if language() == "de":
		result = result.replace(".", ",")
	return result


## Sets node.<property> to the text of the key now and again on every language change while the node lives.
static func bind_text(node: Node, property: StringName, key: String, params: Dictionary = {}) -> void:
	node.set(property, text(key, params))
	_bound.append({"node": weakref(node), "property": property, "key": key, "params": params})
	if not _bound_connected:
		_bound_connected = true
		events.language_changed.connect(_refresh_bound)


static func _refresh_bound(_code: String) -> void:
	var alive: Array = []
	for entry: Dictionary in _bound:
		var node: Object = (entry["node"] as WeakRef).get_ref()
		if node != null:
			node.set(entry["property"], text(entry["key"], entry["params"]))
			alive.append(entry)
	_bound = alive


static func _ensure() -> void:
	if _loaded:
		return
	_loaded = true
	_de = _load_table("de")
	_en = _load_table("en")
	_warned.clear()
	if _language == "":
		_language = saved_language()


static func _load_table(code: String) -> Dictionary:
	var flat := {}
	var parsed := TomlLite.load_file(Paths.config_dir().path_join("i18n").path_join(code + ".toml"))
	for section: String in parsed:
		var group: Variant = parsed[section]
		if group is Dictionary:
			for key: String in group:
				flat[section + "." + key] = group[key]
	return flat


static func _default_spec(kind: String) -> String:
	match kind:
		"angle":
			return "deg1"
		"mass":
			return "t1"
		"length":
			return "km1"
		"speed":
			return "mps1"
		"time":
			return "s0"
		"count":
			return "int"
		"utc":
			return "utc"
	return "n2"


## UTC seconds since the Unix epoch as "YYYY-MM-DD HH:MM UTC" (with_time) or "YYYY-MM-DD"; same in every language.
static func _utc(unix_s: float, with_time: bool) -> String:
	var dt := Time.get_datetime_dict_from_unix_time(int(floorf(unix_s)))
	var date := "%04d-%02d-%02d" % [dt["year"], dt["month"], dt["day"]]
	if not with_time:
		return date
	return "%s %02d:%02d UTC" % [date, dt["hour"], dt["minute"]]


static func _clock(seconds: float) -> String:
	var total := int(floorf(absf(seconds)))
	var prefix := "-" if seconds < 0.0 and total > 0 else ""
	var hours := floori(total / 3600.0)
	var minutes := floori(total / 60.0) % 60
	var secs := total % 60
	if hours > 0:
		return "%s%d:%02d:%02d" % [prefix, hours, minutes, secs]
	return "%s%02d:%02d" % [prefix, minutes, secs]


## Fills the placeholders of a template. Parameters are the report's (value + kind) or plain values; {cause} is the
## rendering of the nested reports.
static func _fill(template: String, params: Dictionary, kinds: Dictionary, causes: Array) -> String:
	var out := ""
	var i := 0
	while i < template.length():
		var open := template.find("{", i)
		var close := template.find("}", open + 1) if open >= 0 else -1
		if open < 0 or close < 0:
			out += template.substr(i)
			break
		out += template.substr(i, open - i)
		var token := template.substr(open + 1, close - open - 1)
		var colon := token.find(":")
		var name := token if colon < 0 else token.substr(0, colon)
		var spec := "" if colon < 0 else token.substr(colon + 1)
		out += _placeholder(name, spec, params, kinds, causes, template.substr(open, close - open + 1))
		i = close + 1
	return out


static func _placeholder(name: String, spec: String, params: Dictionary, kinds: Dictionary, causes: Array,
		original: String) -> String:
	if not params.has(name):
		if name == "cause":
			var parts: Array[String] = []
			for cause: Dictionary in causes:
				parts.append(report_text(cause))
			return "; ".join(parts)
		return original
	var value: Variant = params[name]
	if value is String or value is StringName:
		return text(str(value)) if str(kinds.get(name, "")) == "ref" else str(value)
	return format_value(float(value), str(kinds.get(name, "number")), spec)
