class_name MoonSunHorizon
extends RefCounted
## The Sun's visible fraction at a point near the Moon from the terrain's horizon (StarsimCore.
## horizon_elevation_rad: a full march from the finest data nearby) - the vehicles' direct Sun light and the eye
## adaptation, at every height. The same geometry as terrain_moon.gdshader (shadows fit the rest): the horizon in the
## Sun's azimuth is interpolated linearly between the two neighbouring sectors k * 2 pi / SECTOR_COUNT, the fraction is
## the exact circular segment of the solar disc above that straight horizon. An exact pre-check skips the march where no
## relief can reach the Sun (relief_horizon_bound_rad).

## The azimuth sectors of the terrain's horizon maps (terrain_moon.gdshader's 16).
const SECTOR_COUNT := 16
## A sector fraction below this is the sector itself (one march instead of two).
const SECTOR_EPSILON := 1e-9
## [E] A point stands at least this high above the terrain under it for the march (a vehicle origin in contact can lie
## below the surface sample; a march from inside the ground would see the ground itself as the horizon).
const MIN_HEIGHT_ABOVE_GROUND_M := 0.5


## Fraction of the solar disc (0..1) above the terrain horizon seen from point_fixed_m (Moon-fixed, from the Moon's
## centre, m) with the Sun at sun_fixed_m (Moon-fixed, from the Moon's centre, m); NAN where the core has no lunar
## terrain (the caller keeps the mean / ground sphere's occultation).
static func sun_visible_fraction(core: StarsimCore, point_fixed_m: Vector3, sun_fixed_m: Vector3) -> float:
	var max_height_m: float = core.terrain_max_height_m("moon")
	if not is_finite(max_height_m):
		return NAN
	var radius_m := point_fixed_m.length()
	var up := point_fixed_m / radius_m
	var to_sun_m := sun_fixed_m - point_fixed_m
	var distance_m := to_sun_m.length()
	var direction := to_sun_m / distance_m
	var elevation_rad := asin(clampf(direction.dot(up), -1.0, 1.0))
	var sun_radius_rad := asin(minf(Atmosphere.SUN_RADIUS_M / distance_m, 1.0))
	var height_m := radius_m - Atmosphere.MOON_RADIUS_M
	# Exact: above the bound no relief stands in front of the Sun's disc (nor does the mean sphere, which lies below it).
	if elevation_rad - sun_radius_rad > relief_horizon_bound_rad(height_m, max_height_m):
		return 1.0
	var longitude_rad := atan2(up.y, up.x)
	var east := Vector3(-sin(longitude_rad), cos(longitude_rad), 0.0)  # at a pole: the longitude fixes east
	var north := up.cross(east)
	var azimuth_rad := fposmod(atan2(direction.dot(east), direction.dot(north)), TAU)
	var sector := azimuth_rad * (SECTOR_COUNT / TAU)
	var k0 := int(floor(sector))
	var along := sector - k0
	var latitude_deg := rad_to_deg(asin(clampf(up.z, -1.0, 1.0)))
	var longitude_deg := rad_to_deg(longitude_rad)
	if height_m < max_height_m:  # a point may lie under the terrain: march from just above it
		var ground: Dictionary = core.surface_samples("moon", PackedFloat64Array([latitude_deg]),
				PackedFloat64Array([longitude_deg]))
		height_m = maxf(height_m, float(ground["height_m"][0]) + MIN_HEIGHT_ABOVE_GROUND_M)
	var horizon_rad: float = core.horizon_elevation_rad("moon", latitude_deg, longitude_deg, height_m,
			k0 * (TAU / SECTOR_COUNT))
	if along > SECTOR_EPSILON and is_finite(horizon_rad):
		horizon_rad = lerpf(horizon_rad, core.horizon_elevation_rad("moon", latitude_deg, longitude_deg, height_m,
				(k0 + 1) * (TAU / SECTOR_COUNT)), along)
	if not is_finite(horizon_rad):
		return NAN
	return disc_fraction_above(elevation_rad - horizon_rad, sun_radius_rad)


## The highest elevation (rad, above the local horizontal) any terrain can have from a point height_m over the reference
## sphere: the horizon of the sphere of radius R + max_height_m, -acos((R + max) / (R + h)). At or below the relief's
## top height relief may stand above the horizontal: INF (no bound, always march).
static func relief_horizon_bound_rad(height_m: float, max_height_m: float) -> float:
	if height_m <= max_height_m:
		return INF
	return -acos((Atmosphere.MOON_RADIUS_M + max_height_m) / (Atmosphere.MOON_RADIUS_M + height_m))


## The part of a disc of angular radius radius_rad whose centre stands centre_above_horizon_rad above a straight
## horizon: with d = the centre's signed height in disc radii, area = 1/2 + (d sqrt(1 - d^2) + asin d) / pi.
static func disc_fraction_above(centre_above_horizon_rad: float, radius_rad: float) -> float:
	var d := clampf(centre_above_horizon_rad / radius_rad, -1.0, 1.0)
	return clampf(0.5 + (d * sqrt(1.0 - d * d) + asin(d)) / PI, 0.0, 1.0)
