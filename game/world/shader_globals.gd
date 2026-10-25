class_name ShaderGlobals
extends RefCounted
## The global shader parameters (project.godot [shader_globals]) with a script-side copy: every script sets them
## through set_value(), which forwards to RenderingServer and keeps the value, so the compute passes (CloudEffect via
## CloudShaderPort) read exactly what the Godot shaders read - RenderingServer.global_shader_parameter_get is an
## editor-only function (it errors and stalls outside the editor). A global never set holds its project.godot default.

static var _values := {}


static func set_value(name: StringName, value: Variant) -> void:
	_values[name] = value
	RenderingServer.global_shader_parameter_set(name, value)


static func value(name: StringName) -> Variant:
	if _values.has(name):
		return _values[name]
	return default_value(name)


## Whether the global was set (else it holds its project.godot default).
static func has_value(name: StringName) -> bool:
	return _values.has(name)


## The project.godot default of a global (null for a sampler or an unknown name).
static func default_value(name: StringName) -> Variant:
	var setting: Variant = ProjectSettings.get_setting("shader_globals/" + String(name))
	if setting is Dictionary and (setting as Dictionary).has("value"):
		var v: Variant = setting["value"]
		return null if v is String else v  # a sampler's default is "" (no texture)
	return null
