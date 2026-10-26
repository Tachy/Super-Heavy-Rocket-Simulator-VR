class_name CloudShaderPort
extends RefCounted
## Turns Godot shader includes written in the common subset of the Godot shading language and GLSL 450 (atmosphere,
## clouds) into a GLSL compute shader for the cloud pass (CloudEffect, M8 weather and clouds, Task 8): the
## `#include "res://..."` lines are inlined (include guards are left to the GLSL preprocessor), every
## `global uniform <type> <name> [: hints];` becomes a member of one std140 uniform block (set 0, binding 0) or, for a
## sampler, its own binding in set 0 with a sampler state from the hints, and the Godot built-in constant PI is
## defined. pack() fills the block from the current global shader parameters (RenderingServer), sampler_textures()
## lists the RD textures of the sampler globals - so the compute pass reads exactly what the Godot shaders read.

const SAMPLER_TYPES := ["sampler2D", "sampler3D", "sampler2DArray", "samplerCube"]

var source := ""  ## the GLSL compute source
var members: Array[Dictionary] = []  ## {name, id (StringName), type, offset} of the uniform block, std140
var samplers: Array[Dictionary] = []  ## {name, id (StringName), type, binding, linear, repeat, mipmap}
var block_size := 0  ## bytes of the uniform block (a multiple of 16)
var _defaults := {}  ## project.godot defaults of the members never set, looked up once (pack: the render thread)


## defines: preprocessor names (e.g. "CLOUD_MARCH"); files: res:// include paths in order. with_globals: false for
## kernels without the includes' globals (their `global uniform` lines are dropped).
static func build(defines: PackedStringArray, files: PackedStringArray, with_globals := true) -> CloudShaderPort:
	var port := CloudShaderPort.new()
	var seen := {}
	var body := ""
	for file in files:
		body += port._inline(file, seen) + "\n"
	var regex := RegEx.new()
	regex.compile("(?m)^[ \\t]*global[ \\t]+uniform[ \\t]+(\\w+)[ \\t]+(\\w+)[ \\t]*(?::([^;]*))?;[^\\n]*$")
	var declared := {}
	for m in regex.search_all(body):
		var type := m.get_string(1)
		var name := m.get_string(2)
		if declared.has(name):
			continue
		declared[name] = true
		if type in SAMPLER_TYPES:
			var hints := m.get_string(3)
			port.samplers.append({"name": name, "id": StringName(name), "type": type, "binding": port.samplers.size() + 1,
					"linear": not hints.contains("filter_nearest"), "repeat": hints.contains("repeat_enable"),
					"mipmap": hints.contains("_mipmap")})
		else:
			port.members.append({"name": name, "id": StringName(name), "type": type})
	body = regex.sub(body, "", true)
	var header := "#version 450\n"
	for d in defines:
		header += "#define %s\n" % d
	header += "#define PI 3.14159265358979\n"
	if with_globals:
		header += port._layout_block()
		for s in port.samplers:
			header += "layout(set = 0, binding = %d) uniform %s %s;\n" % [s["binding"], s["type"], s["name"]]
	port.source = header + body
	return port


## The sampler state of the hint filter_linear_mipmap (with repeat_enable; the shape prefilter's mips): linear in
## texels and between mip levels. The effects key their samplers "linear|repeat|mipmap".
static func mipmap_sampler_state() -> RDSamplerState:
	var state := RDSamplerState.new()
	state.min_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	state.mag_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	state.mip_filter = RenderingDevice.SAMPLER_FILTER_LINEAR
	state.repeat_u = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	state.repeat_v = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	state.repeat_w = RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT
	return state


func _inline(file: String, seen: Dictionary) -> String:
	var include := load(file) as ShaderInclude
	if include == null:
		push_error("CloudShaderPort: include missing: " + file)
		return ""
	var regex := RegEx.new()
	regex.compile("(?m)^[ \\t]*#include[ \\t]+\"([^\"]+)\"[^\\n]*$")
	var code := include.code
	var out := ""
	var at := 0
	for m in regex.search_all(code):
		out += code.substr(at, m.get_start() - at)
		var path := m.get_string(1)
		if not seen.has(path):
			seen[path] = true
			out += _inline(path, seen)
		at = m.get_end()
	return out + code.substr(at)


## std140: scalars 4 bytes, vec2 8, vec3 / vec4 16-aligned, mat3 three vec4 columns, mat4 four.
func _layout_block() -> String:
	var text := "layout(set = 0, binding = 0, std140) uniform CloudGlobals {\n"
	var offset := 0
	for m in members:
		var size_align := _std140(m["type"])
		offset = (offset + size_align.y - 1) / size_align.y * size_align.y
		m["offset"] = offset
		offset += size_align.x
		text += "\t%s %s;\n" % [m["type"], m["name"]]
	block_size = maxi((offset + 15) / 16 * 16, 16)
	return text + "};\n"


static func _std140(type: String) -> Vector2i:
	match type:
		"float", "int", "uint", "bool":
			return Vector2i(4, 4)
		"vec2", "ivec2", "uvec2":
			return Vector2i(8, 8)
		"vec3", "ivec3":
			return Vector2i(12, 16)
		"vec4", "ivec4":
			return Vector2i(16, 16)
		"mat3":
			return Vector2i(48, 16)
		"mat4":
			return Vector2i(64, 16)
	push_error("CloudShaderPort: no std140 rule for " + type)
	return Vector2i(16, 16)


## The uniform block's bytes from the current global shader parameters.
func pack() -> PackedByteArray:
	var bytes := PackedByteArray()
	bytes.resize(block_size)
	for m in members:
		var id: StringName = m["id"]
		var value: Variant
		if ShaderGlobals.has_value(id):
			value = ShaderGlobals.value(id)
		else:
			if not _defaults.has(id):
				_defaults[id] = ShaderGlobals.default_value(id)
			value = _defaults[id]
		var at: int = m["offset"]
		match String(m["type"]):
			"float":
				bytes.encode_float(at, float(value) if value != null else 0.0)
			"int", "uint", "bool":
				bytes.encode_s32(at, int(value) if value != null else 0)
			"vec2":
				var v: Vector2 = value if value is Vector2 else Vector2.ZERO
				bytes.encode_float(at, v.x)
				bytes.encode_float(at + 4, v.y)
			"ivec2":
				var v: Vector2i = value if value is Vector2i else Vector2i.ZERO
				bytes.encode_s32(at, v.x)
				bytes.encode_s32(at + 4, v.y)
			"vec3":
				var v: Vector3 = value if value is Vector3 else Vector3.ZERO
				_put_vec(bytes, at, Vector4(v.x, v.y, v.z, 0.0), 3)
			"vec4":
				var v := Vector4.ZERO
				if value is Vector4:
					v = value
				elif value is Color:
					v = Vector4(value.r, value.g, value.b, value.a)
				elif value is Plane:
					v = Vector4(value.x, value.y, value.z, value.d)
				_put_vec(bytes, at, v, 4)
			"mat3":
				var b: Basis = value if value is Basis else Basis.IDENTITY
				for c in range(3):
					var column := b[c]
					_put_vec(bytes, at + 16 * c, Vector4(column.x, column.y, column.z, 0.0), 3)
			"mat4":
				var p := Projection.IDENTITY
				if value is Projection:
					p = value
				elif value is Transform3D:
					p = Projection(value)
				for c in range(4):
					_put_vec(bytes, at + 16 * c, p[c], 4)
	return bytes


static func _put_vec(bytes: PackedByteArray, at: int, v: Vector4, count: int) -> void:
	for i in range(count):
		bytes.encode_float(at + 4 * i, v[i])


## The RD texture of each sampler global (in samplers order); RID() where the global holds no texture.
func sampler_textures() -> Array[RID]:
	var result: Array[RID] = []
	for s in samplers:
		var value: Variant = ShaderGlobals.value(s["id"])
		var rd_texture := RID()
		if value is Texture:
			rd_texture = RenderingServer.texture_get_rd_texture((value as Texture).get_rid())
		result.append(rd_texture)
	return result
