#!/usr/bin/env python3
import argparse
import csv
import numpy as np
from src.calibration import fit_ellipsoid_calibration, magnitude_statistics


def load_xyz(path):
    rows = []
    with open(path, newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append([
                float(row["mag_x"]),
                float(row["mag_y"]),
                float(row["mag_z"]),
            ])
    return np.asarray(rows, dtype=float)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    args = parser.parse_args()

    samples = load_xyz(args.csv_file)
    bias, matrix = fit_ellipsoid_calibration(samples)
    stats = magnitude_statistics(samples, bias, matrix)

    np.set_printoptions(precision=9, suppress=True)

    print(f"Samples: {len(samples)}")
    print("\nHard-iron bias:")
    print(bias)
    print("\nSoft-iron matrix:")
    print(matrix)
    print("\nCalibrated magnitude statistics:")
    for key, value in stats.items():
        print(f"{key}: {value:.9f}")


if __name__ == "__main__":
    main()
