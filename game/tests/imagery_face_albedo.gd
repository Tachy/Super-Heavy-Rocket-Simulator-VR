extends RefCounted
## Reference for the ground bounce of a vertical face (U-4, for ground_albedo_weight_probe.gd and radiance_budget.gd): the
## exact view-factor weighted albedo of the imagery's ground for a wall element at height h with a horizontal normal of
## azimuth psi (from north, toward east). The ground element at the horizontal offset (r, phi) contributes
## dF = cos(theta_wall) cos(theta_ground) dA / (pi d^2) = r h cos(phi - psi) r dr dphi / (pi d^4); with r = h tan(beta) (beta:
## from the nadir) dF = sin^2(beta) max(cos(phi - psi), 0) dbeta dphi / pi, total 1/2 over the plane. Integrated on a polar
## grid around the eye over the imagery's pixels, each sample from the finest of nested crops (NAIP ~ 1 m near the eye out to
## Sentinel-2 ~ 40 m at 6 km), the given global albedo beyond R_MAX_M. Independent of GroundAlbedoNear (no rings, no sectors,
## its own crops and grid). Linear luminance.

const METRES_PER_DEGREE := 111132.0
## Nested crops: half-size (m) and requested texel (m); finest first.
const CROPS := [[120.0, 0.6], [500.0, 2.4], [2000.0, 10.0], [6000.0, 30.0]]
const R_MAX_M := 6000.0
const N_BETA := 600
const N_PHI := 360

var crops: Array = []  # [image, box_deg, half_m] finest first; empty: no imagery covers the site
var _lat := 0.0
var _lon := 0.0


func _init(imagery: ImagerySource, lat_deg: float, lon_deg: float, verbose := false) -> void:
	_lat = lat_deg
	_lon = lon_deg
	var cos_lat := cos(deg_to_rad(_lat))
	for spec in CROPS:
		var half_lat: float = spec[0] / METRES_PER_DEGREE
		var box := Rect2(_lon - half_lat / cos_lat, _lat - half_lat, 2.0 * half_lat / cos_lat, 2.0 * half_lat)
		var crop := imagery.crop(box, spec[1] / METRES_PER_DEGREE)
		if crop.is_empty() or not (crop["coverage_deg"] as Rect2).encloses(box):
			crops = []
			return
		var image: Image = crop["image"]
		var box_deg: Rect2 = crop["box_deg"]
		if verbose:
			print("  reference crop +-%.0f m: %dx%d px, %.2f m/px, source %s level %d" % [spec[0], image.get_width(),
					image.get_height(), box_deg.size.y * METRES_PER_DEGREE / image.get_height(), crop["source_id"], crop["level"]])
		crops.append([image, box_deg, half_lat * METRES_PER_DEGREE])


## Linear luminance of the imagery at the horizontal offset (east, north) in metres: the finest crop that holds it; -1 outside.
func sample(east_m: float, north_m: float) -> float:
	var lat := _lat + north_m / METRES_PER_DEGREE
	var lon := _lon + east_m / (METRES_PER_DEGREE * cos(deg_to_rad(_lat)))
	for c in crops:
		if maxf(absf(east_m), absf(north_m)) > c[2]:
			continue
		var image: Image = c[0]
		var box: Rect2 = c[1]
		var x := clampi(int((lon - box.position.x) / box.size.x * image.get_width()), 0, image.get_width() - 1)
		var y := clampi(int((box.end.y - lat) / box.size.y * image.get_height()), 0, image.get_height() - 1)
		var p := image.get_pixel(x, y)
		return luma(Vector3(_decode(p.r), _decode(p.g), _decode(p.b)))
	return -1.0


## The view-factor weighted ground albedo of a vertical face at height h, one value per azimuth (rad, from north toward east).
func face_albedo(h: float, azimuths: Array, global_luma: float) -> Array:
	var beta_max := atan(R_MAX_M / h)
	var d_beta := beta_max / N_BETA
	var d_phi := TAU / N_PHI
	var sums := []
	sums.resize(azimuths.size())
	sums.fill(0.0)
	var cos_psi := azimuths.map(func(x: float) -> float: return cos(x))
	var sin_psi := azimuths.map(func(x: float) -> float: return sin(x))
	for ib in range(N_BETA):
		var beta := (ib + 0.5) * d_beta
		var r := h * tan(beta)
		var w_beta := sin(beta) * sin(beta) * d_beta * d_phi / PI
		for ip in range(N_PHI):
			var phi := (ip + 0.5 + 0.5 * (ib % 2)) * d_phi
			var cp := cos(phi)
			var sp := sin(phi)
			var a := sample(r * sp, r * cp)
			for o in range(azimuths.size()):
				var c: float = cp * cos_psi[o] + sp * sin_psi[o]
				if c > 0.0:
					sums[o] += a * w_beta * c
	# beyond R_MAX_M: the global albedo; (2 / pi) int_{beta_max}^{pi/2} sin^2(beta) dbeta of the face's total 1/2
	var tail := (0.25 * PI - 0.5 * beta_max + 0.25 * sin(2.0 * beta_max)) * 2.0 / PI
	var out := []
	for o in range(azimuths.size()):
		out.append((sums[o] + global_luma * tail) / 0.5)
	return out


static func luma(rgb: Vector3) -> float:
	return 0.2126 * rgb.x + 0.7152 * rgb.y + 0.0722 * rgb.z


static func _decode(c: float) -> float:
	return c / 12.92 if c <= 0.04045 else pow((c + 0.055) / 1.055, 2.4)
