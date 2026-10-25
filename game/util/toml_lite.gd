class_name TomlLite
## Minimal reader for the TOML subset used in config/: comments, [table], [[array-of-tables]] with [array.sub] tables,
## key = string | number | boolean | single-line array of those. Enough for flat data files; the C++ core uses a full parser.


static func load_file(path: String) -> Dictionary:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		push_error("TomlLite: cannot open %s" % path)
		return {}
	return parse(file.get_as_text())


static func parse(text: String) -> Dictionary:
	var root := {}
	var current := root
	for raw_line in text.split("\n"):
		var line := _strip_comment(raw_line).strip_edges()
		if line.is_empty():
			continue
		if line.begins_with("[["):
			var key := line.trim_prefix("[[").trim_suffix("]]").strip_edges()
			if not root.has(key):
				root[key] = []
			current = {}
			root[key].append(current)
		elif line.begins_with("["):
			var key := line.trim_prefix("[").trim_suffix("]").strip_edges()
			current = {}
			# [site.viewpoint] after [[site]]: a sub-table of the array's last entry (entry["viewpoint"]); any other
			# dotted name stays a flat root key ("gauge.engines", read that way by GaugeConfig).
			var dot := key.find(".")
			var parent: Variant = root.get(key.substr(0, dot)) if dot > 0 else null
			if parent is Array and not (parent as Array).is_empty():
				(parent as Array).back()[key.substr(dot + 1)] = current
			else:
				root[key] = current
		else:
			var separator := line.find("=")
			if separator < 0:
				continue
			current[line.substr(0, separator).strip_edges()] = _value(line.substr(separator + 1).strip_edges())
	return root


static func _strip_comment(line: String) -> String:
	var in_string := false
	for i in line.length():
		if line[i] == '"':
			in_string = not in_string
		elif line[i] == "#" and not in_string:
			return line.substr(0, i)
	return line


static func _value(text: String) -> Variant:
	if text.begins_with("["):  # single-line array of scalars, no nesting
		var items: Array = []
		var inner := text.trim_prefix("[").trim_suffix("]").strip_edges()
		if inner.is_empty():
			return items
		var start := 0
		var in_string := false
		for i in inner.length():
			if inner[i] == '"':
				in_string = not in_string
			elif inner[i] == "," and not in_string:
				items.append(_value(inner.substr(start, i - start).strip_edges()))
				start = i + 1
		var last := inner.substr(start).strip_edges()
		if not last.is_empty():  # a trailing comma leaves nothing after it
			items.append(_value(last))
		return items
	if text.begins_with('"'):
		return text.substr(1, text.length() - 2)
	if text == "true" or text == "false":
		return text == "true"
	return text.replace("_", "").to_float()
