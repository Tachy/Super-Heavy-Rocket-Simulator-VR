"""Known lengths usable as scale anchors (data). Tag: [S] sourced, [E] estimated.

anchor(name, label_a, label_b) builds a fit Anchor between two part points.
"""
from __future__ import annotations

from dataclasses import dataclass

from .fit import Anchor


@dataclass(frozen=True)
class KnownLength:
    length_m: float
    sigma_m: float
    source: str
    tag: str


KNOWN = {
    # IDSS Interface Definition Document revision E, Fig 3.1.1.1-2: transfer passageway (also config/vehicles/detail/crew_dragon.toml petal_tip_diameter_m)
    "idss_passage_m": KnownLength(0.800, 0.002, "IDSS IDD E Fig 3.1.1.1-2 transfer passageway 800 mm", "S"),
    # Apple iPad mini (7.9 in, 1st-3rd gen) body 200.1/203.2 x 134.7 mm: the cabin tablets in photos are iPad minis
    "ipad_mini_long_m": KnownLength(0.2032, 0.002, "Apple iPad mini 7.9in body 203.2 mm", "S"),
    "ipad_mini_short_m": KnownLength(0.1348, 0.002, "Apple iPad mini 7.9in body 134.8 mm", "S"),
    # Exterior window extent along the capsule axis: 2 x 0.255 (config/vehicles/detail/crew_dragon.toml windows, [E] M2 'okno')
    "window_axial_m": KnownLength(0.510, 0.05, "config/vehicles/detail/crew_dragon.toml windows half x 0.255 (exterior)", "E"),
    # ANSUR II male medians (data/anthropometry/ANSUR_II_MALE_Public.csv, mm / 1000); sigma = population std (a single person is not the median)
    "ansur_hand_length_m": KnownLength(0.193, 0.0099, "ANSUR II male median handlength 193 mm (sd 9.9)", "S"),
    "ansur_hand_breadth_m": KnownLength(0.088, 0.0044, "ANSUR II male median handbreadth 88 mm (sd 4.4)", "S"),
    "ansur_head_length_m": KnownLength(0.200, 0.0070, "ANSUR II male median headlength 200 mm (sd 7.0)", "S"),
    "ansur_head_breadth_m": KnownLength(0.154, 0.0055, "ANSUR II male median headbreadth 154 mm (sd 5.5)", "S"),
}

ANSUR_COLUMNS = {"ansur_hand_length_m": "handlength", "ansur_hand_breadth_m": "handbreadth",
                 "ansur_head_length_m": "headlength", "ansur_head_breadth_m": "headbreadth"}


def anchor(name: str, label_a: str, label_b: str, sigma_scale: float = 1.0) -> Anchor:
    k = KNOWN[name]
    return Anchor(label_a, label_b, k.length_m, k.sigma_m * sigma_scale, f"[{k.tag}] {k.source}")
