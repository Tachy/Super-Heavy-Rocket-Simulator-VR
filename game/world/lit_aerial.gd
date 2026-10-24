class_name LitAerial
extends RefCounted
## Materials of vehicles and structures (vehicles, catch towers, pads, drone ship) in the atmosphere's light and haze:
## shaders/lit_aerial.gdshader - the StandardMaterial3D parameters they used (albedo, roughness, metallic, emission)
## under Godot's lighting, plus the aerial perspective between the camera and the surface (M8 atmosphere, Task 8).

const SHADER := preload("res://shaders/lit_aerial.gdshader")


## A material with StandardMaterial3D's meaning of the parameters (albedo / emission as sRGB colours).
static func material(albedo: Color, roughness: float, metallic: float, emission := Color.BLACK) -> ShaderMaterial:
	var m := ShaderMaterial.new()
	m.shader = SHADER
	m.set_shader_parameter("albedo", albedo)
	m.set_shader_parameter("roughness", roughness)
	m.set_shader_parameter("metallic", metallic)
	m.set_shader_parameter("emission", emission)
	return m
