"""Radius of a stage's drawn hull over the station (no Blender dependency: the generators and the tests share it)."""
import math

NOSE_SAMPLES = 20  # [E] profile samples along the ogive (chord error < 5 mm on the 17 m / rho 35 m ogive)
MERGE_M = 1e-4  # profile stations closer than this are one
DOME_SAMPLES = 12  # segments of the booster's forward dome


class Profile:
    """Radius of the hull over the station: chamfer, plain barrel, tangent ogive nose with a blunt tip."""

    def __init__(self, stage, hull, aft_chamfer_m=None):
        self.L, self.R = stage["length_m"], stage["diameter_m"] / 2.0
        # the DRAWN ogive is the 1:100 model's (13.4 m), not the config's physics nose_length_m (17 m): a documented
        # deviation
        self.nose = hull["nose_drawn_length_m"] if stage["nose_length_m"] > 0.0 else 0.0
        self.chamfer = hull["base_chamfer_m"] if aft_chamfer_m is None else aft_chamfer_m
        self.tip_r = hull["nose_tip_radius_m"]
        self.cap_h = hull["nose_tip_cap_m"] if self.nose > 0.0 else 0.0
        if self.nose > 0.0:
            R, Ln = self.R, self.nose
            self.rho = (R * R + Ln * Ln) / (2.0 * R)
            # ogive from the tip: r(s) = sqrt(rho^2 - (Ln - s)^2) + R - rho; truncated where r = tip radius, the
            # remaining length is squeezed so that truncated ogive + cap ends exactly at X = length
            self.s_t = Ln - math.sqrt(self.rho ** 2 - (self.rho - R + self.tip_r) ** 2)
        self.x_barrel_end = self.L - self.nose
        # a booster's forward dome (1:100 model photo): the barrel ends at the truss shoulder, a half ellipse closes it
        # below the ring under the ship, inside the physics length
        self.dome_h = hull.get("dome_height_m", 0.0) if self.nose <= 0.0 else 0.0
        if self.dome_h > 0.0:
            self.x_barrel_end = self.L - hull["dome_start_m"]

    def base(self, x):
        """Radius of the shell."""
        if x < self.chamfer:
            return self.R - self.chamfer + x
        if self.dome_h > 0.0 and x > self.x_barrel_end:
            return self.R * math.sqrt(max(1.0 - ((x - self.x_barrel_end) / self.dome_h) ** 2, 0.0))
        if self.nose <= 0.0 or x <= self.x_barrel_end:
            return self.R
        s_new = self.L - x  # distance from the tip along the axis
        s = self.s_t + (s_new - self.cap_h) * (self.nose - self.s_t) / (self.nose - self.cap_h)
        return math.sqrt(max(self.rho ** 2 - (self.nose - s) ** 2, 0.0)) + self.R - self.rho

    def radius(self, x):
        return self.base(x)

    def stations(self):
        if self.dome_h > 0.0:
            return [0.0, self.chamfer, self.x_barrel_end]
        xs = {0.0, self.chamfer, self.x_barrel_end if self.nose > 0 else self.L}  # a straight barrel: its two ends only
        if self.nose > 0.0:
            x_end = self.L - self.cap_h
            for i in range(NOSE_SAMPLES + 1):  # denser toward the tip
                f = 1.0 - (1.0 - i / NOSE_SAMPLES) ** 1.6
                xs.add(self.x_barrel_end + f * (x_end - self.x_barrel_end))
            xs.add(x_end)
        xs = sorted(x for x in xs if 0.0 <= x <= (self.L - self.cap_h) + 1e-9)
        out = []
        for x in xs:
            if not out or x - out[-1] > MERGE_M:
                out.append(x)
        return out

    def runs(self):
        main = [(x, self.radius(x)) for x in self.stations()]
        if self.dome_h > 0.0:
            for i in range(1, DOME_SAMPLES + 1):
                t = 0.5 * math.pi * i / DOME_SAMPLES
                main.append((self.x_barrel_end + self.dome_h * math.sin(t), self.R * math.cos(t)))
            return [([(0.0, 0.0), (0.0, main[0][1])], (-1.0, 0.0, 0.0)), (main[:2], "out"), (main[1:], "out")]
        if self.nose > 0.0:  # blunt tip: a quarter ellipse (axial cap_h, radial tip_r) closes the shell at X = length
            x_e, r_e = main[-1]
            for i in range(1, 9):
                t = 0.5 * math.pi * i / 8.0
                main.append((x_e + self.cap_h * math.sin(t), r_e * math.cos(t)))
        # the chamfer is its own smooth group: one long barrel strip must not inherit the chamfer corner's normal
        runs = [(main[:2], "out"), (main[1:], "out")]  # a nose: the aft end is open (`engine_bay` closes it)
        if self.nose <= 0.0:
            runs.insert(0, ([(0.0, 0.0), (0.0, main[0][1])], (-1.0, 0.0, 0.0)))
            runs.append(([(self.L, main[-1][1]), (self.L, 0.0)], (1.0, 0.0, 0.0)))
        return runs
