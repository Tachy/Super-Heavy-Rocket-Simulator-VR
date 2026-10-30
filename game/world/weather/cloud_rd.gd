class_name CloudRD
## RenderingDevice helpers the cloud compute passes share (CloudEffect, CloudBaked, CloudShadowEffect): the samplers
## keyed "linear|repeat|mipmap" (the sampler hints of the ported globals, CloudShaderPort), the fallback textures of an
## unset sampler global, set 0 of a ported kernel (its globals' block and samplers) and uniforms of set 1.


## The samplers of every key: nearest / linear x clamp / repeat without mipmaps, and the mipmapped repeat one (the
## shape prefilter's mips).
static func create_samplers(rd: RenderingDevice) -> Dictionary:
	var samplers := {}
	for linear in [false, true]:
		for repeat in [false, true]:
			var state := RDSamplerState.new()
			var filter := RenderingDevice.SAMPLER_FILTER_LINEAR if linear else RenderingDevice.SAMPLER_FILTER_NEAREST
			state.min_filter = filter
			state.mag_filter = filter
			state.mip_filter = RenderingDevice.SAMPLER_FILTER_NEAREST
			var mode := RenderingDevice.SAMPLER_REPEAT_MODE_REPEAT if repeat \
					else RenderingDevice.SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE
			state.repeat_u = mode
			state.repeat_v = mode
			state.repeat_w = mode
			samplers["%s|%s|false" % [linear, repeat]] = rd.sampler_create(state)
	samplers["true|true|true"] = rd.sampler_create(CloudShaderPort.mipmap_sampler_state())
	return samplers


## A 1 x 1 (x 1) zero RGBA8 texture per sampler type ("sampler2D", "sampler3D"): bound where a global holds none.
static func create_fallbacks(rd: RenderingDevice) -> Dictionary:
	var fallbacks := {}
	for type in ["sampler2D", "sampler3D"]:
		var tf := RDTextureFormat.new()
		tf.texture_type = RenderingDevice.TEXTURE_TYPE_2D if type == "sampler2D" else RenderingDevice.TEXTURE_TYPE_3D
		tf.width = 1
		tf.height = 1
		tf.depth = 1
		tf.format = RenderingDevice.DATA_FORMAT_R8G8B8A8_UNORM
		tf.usage_bits = RenderingDevice.TEXTURE_USAGE_SAMPLING_BIT | RenderingDevice.TEXTURE_USAGE_CAN_UPDATE_BIT
		fallbacks[type] = rd.texture_create(tf, RDTextureView.new(), [PackedByteArray([0, 0, 0, 0])])
	return fallbacks


## Set 0 of a ported kernel: its globals' block (`buffer`, filled now from the current globals) and its sampler globals'
## textures (the fallback where a global holds none).
static func set0(rd: RenderingDevice, port: CloudShaderPort, buffer: RID, samplers: Dictionary,
		fallbacks: Dictionary) -> Array[RDUniform]:
	rd.buffer_update(buffer, 0, port.block_size, port.pack())
	var uniforms: Array[RDUniform] = [ubo(buffer)]
	var textures := port.sampler_textures()
	for i in range(port.samplers.size()):
		var s: Dictionary = port.samplers[i]
		var u := RDUniform.new()
		u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
		u.binding = s["binding"]
		u.add_id(samplers["%s|%s|%s" % [s["linear"], s["repeat"], s["mipmap"]]])
		var texture: RID = textures[i]
		u.add_id(texture if texture.is_valid() else fallbacks.get(s["type"], fallbacks["sampler2D"]))
		uniforms.append(u)
	return uniforms


static func ubo(buffer: RID, binding := 0) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_UNIFORM_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u


static func tex(binding: int, sampler: RID, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_SAMPLER_WITH_TEXTURE
	u.binding = binding
	u.add_id(sampler)
	u.add_id(texture)
	return u


static func storage(binding: int, buffer: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_STORAGE_BUFFER
	u.binding = binding
	u.add_id(buffer)
	return u


static func image(binding: int, texture: RID) -> RDUniform:
	var u := RDUniform.new()
	u.uniform_type = RenderingDevice.UNIFORM_TYPE_IMAGE
	u.binding = binding
	u.add_id(texture)
	return u
