"""Heading and magnetic-state correction utilities."""

from __future__ import annotations
import json
import math
from pathlib import Path
import numpy as np


HARD_IRON_BIAS = np.array(
    [130.054348, -52.792277, -175.082933],
    dtype=float,
)

SOFT_IRON_MATRIX = np.array(
    [
        [0.002010925,  0.000005664,  0.000191615],
        [0.000005664,  0.002092747, -0.000034360],
        [0.000191615, -0.000034360,  0.002238688],
    ],
    dtype=float,
)


def wrap_360(angle_deg):
    return float(angle_deg % 360.0)


def angle_difference(target_deg, measured_deg):
    return float(((target_deg - measured_deg + 180.0) % 360.0) - 180.0)


def circular_signed_mean_deg(values):
    values = np.asarray(list(values), dtype=float)
    if values.size == 0:
        raise ValueError("No angles supplied")

    radians = np.radians(values)
    angle = math.degrees(
        math.atan2(
            float(np.mean(np.sin(radians))),
            float(np.mean(np.cos(radians))),
        )
    )
    return float(((angle + 180.0) % 360.0) - 180.0)


def apply_calibration(
    raw_xyz,
    bias=HARD_IRON_BIAS,
    matrix=SOFT_IRON_MATRIX,
):
    raw = np.asarray(raw_xyz, dtype=float)
    if raw.shape != (3,):
        raise ValueError("raw_xyz must contain X, Y, Z")
    return (raw - bias) @ matrix


def magnetic_heading_deg(calibrated_xyz):
    mx, my, _ = np.asarray(calibrated_xyz, dtype=float)
    return wrap_360(math.degrees(math.atan2(my, mx)))


def magnetic_elevation_deg(calibrated_xyz):
    mx, my, mz = np.asarray(calibrated_xyz, dtype=float)
    return math.degrees(math.atan2(mz, math.sqrt(mx * mx + my * my)))


def nearest_lut_correction(heading_deg, mag_elevation_deg, lut):
    if not lut:
        return 0.0, None

    best_distance = float("inf")
    best_cell = None
    best_correction = 0.0

    for (heading_bin, elevation_bin), correction in lut.items():
        dh = angle_difference(heading_deg, heading_bin)
        de = mag_elevation_deg - elevation_bin
        distance = dh * dh + de * de

        if distance < best_distance:
            best_distance = distance
            best_cell = (heading_bin, elevation_bin)
            best_correction = correction

    return float(best_correction), best_cell


def load_lut(path):
    data = json.loads(Path(path).read_text())
    lut = {}

    for key, value in data.items():
        heading_text, elevation_text = key.split(",")
        lut[(float(heading_text), float(elevation_text))] = float(value)

    return lut


def corrected_heading(raw_xyz, lut=None):
    calibrated = apply_calibration(raw_xyz)
    base_heading = magnetic_heading_deg(calibrated)
    elevation = magnetic_elevation_deg(calibrated)

    correction = 0.0
    cell = None

    if lut:
        correction, cell = nearest_lut_correction(
            base_heading,
            elevation,
            lut,
        )

    final = wrap_360(base_heading + correction)

    return final, {
        "calibrated_xyz": calibrated,
        "base_heading_deg": base_heading,
        "magnetic_elevation_deg": elevation,
        "correction_deg": correction,
        "lut_cell": cell,
    }
