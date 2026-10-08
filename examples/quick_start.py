import numpy as np
from src.heading import (
    apply_calibration,
    magnetic_elevation_deg,
    magnetic_heading_deg,
)

raw = np.array([130.0, -50.0, -400.0])

calibrated = apply_calibration(raw)
heading = magnetic_heading_deg(calibrated)
elevation = magnetic_elevation_deg(calibrated)

print("Raw XYZ:", raw)
print("Calibrated XYZ:", calibrated)
print(f"Base heading: {heading:.2f} deg")
print(f"Magnetic elevation: {elevation:.2f} deg")
