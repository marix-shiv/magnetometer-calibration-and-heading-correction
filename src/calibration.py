"""3D magnetometer hard-iron / soft-iron calibration."""

from __future__ import annotations
import numpy as np


def fit_ellipsoid_calibration(samples_xyz):
    raw = np.asarray(samples_xyz, dtype=float)

    if raw.ndim != 2 or raw.shape[1] != 3:
        raise ValueError("samples_xyz must have shape (N, 3)")

    if raw.shape[0] < 20:
        raise ValueError("Insufficient samples for a reliable 3D fit")

    origin = np.mean(raw, axis=0)
    spread = np.std(raw, axis=0)
    scale = float(np.max(spread))

    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError("Calibration data has insufficient spread")

    xyz = (raw - origin) / scale
    x, y, z = xyz.T

    design = np.column_stack(
        (
            x * x,
            y * y,
            z * z,
            2.0 * x * y,
            2.0 * x * z,
            2.0 * y * z,
            x,
            y,
            z,
        )
    )

    parameters, *_ = np.linalg.lstsq(
        design,
        np.ones(raw.shape[0]),
        rcond=None,
    )

    q_matrix = np.array(
        [
            [parameters[0], parameters[3], parameters[4]],
            [parameters[3], parameters[1], parameters[5]],
            [parameters[4], parameters[5], parameters[2]],
        ],
        dtype=float,
    )

    q_linear = np.array(
        [parameters[6], parameters[7], parameters[8]],
        dtype=float,
    )

    center_normalized = -0.5 * np.linalg.solve(q_matrix, q_linear)

    k = 1.0 + center_normalized @ q_matrix @ center_normalized
    if not np.isfinite(k) or k <= 0.0:
        raise ValueError("Invalid ellipsoid normalization")

    shape = q_matrix / k
    eigenvalues, eigenvectors = np.linalg.eigh(shape)

    if np.any(eigenvalues <= 0.0):
        raise ValueError("Ellipsoid shape is not positive definite")

    sqrt_shape = (
        eigenvectors
        @ np.diag(np.sqrt(eigenvalues))
        @ eigenvectors.T
    )

    bias = origin + scale * center_normalized
    matrix = sqrt_shape / scale

    return bias, matrix


def apply_calibration(raw_xyz, bias, matrix):
    raw = np.asarray(raw_xyz, dtype=float)
    return (raw - np.asarray(bias, dtype=float)) @ np.asarray(matrix, dtype=float)


def magnitude_statistics(raw_samples, bias, matrix):
    raw = np.asarray(raw_samples, dtype=float)
    calibrated = (raw - bias) @ matrix
    radius = np.linalg.norm(calibrated, axis=1)
    error = radius - 1.0

    return {
        "mean_magnitude": float(np.mean(radius)),
        "std_magnitude": float(np.std(radius)),
        "mean_abs_error": float(np.mean(np.abs(error))),
        "rms_error": float(np.sqrt(np.mean(error * error))),
        "max_abs_error": float(np.max(np.abs(error))),
    }
