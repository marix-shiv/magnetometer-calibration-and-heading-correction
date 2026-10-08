#!/usr/bin/env python3
import argparse
import csv
import json
from collections import defaultdict

from src.heading import (
    angle_difference,
    apply_calibration,
    circular_signed_mean_deg,
    magnetic_elevation_deg,
    magnetic_heading_deg,
)


def round_bin(value, step):
    return round(value / step) * step


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("output_json")
    parser.add_argument("--heading-step", type=float, default=5.0)
    parser.add_argument("--elevation-step", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=3)
    args = parser.parse_args()

    cells = defaultdict(list)
    rows = 0

    with open(args.csv_file, newline="") as handle:
        reader = csv.DictReader(handle)

        for row in reader:
            raw = [
                float(row["mag_x"]),
                float(row["mag_y"]),
                float(row["mag_z"]),
            ]
            actual_yaw = float(row["actual_yaw"])

            calibrated = apply_calibration(raw)
            heading = magnetic_heading_deg(calibrated)
            elevation = magnetic_elevation_deg(calibrated)
            correction = angle_difference(actual_yaw, heading)

            heading_bin = round_bin(heading, args.heading_step) % 360.0
            elevation_bin = round_bin(elevation, args.elevation_step)

            cells[(heading_bin, elevation_bin)].append(correction)
            rows += 1

    output = {}

    for (heading_bin, elevation_bin), corrections in sorted(cells.items()):
        if len(corrections) < args.min_samples:
            continue

        key = f"{heading_bin:g},{elevation_bin:g}"
        output[key] = circular_signed_mean_deg(corrections)

    with open(args.output_json, "w") as handle:
        json.dump(output, handle, indent=2, sort_keys=True)

    print(f"Rows processed: {rows}")
    print(f"Populated LUT cells: {len(output)}")
    print(f"Saved: {args.output_json}")


if __name__ == "__main__":
    main()
