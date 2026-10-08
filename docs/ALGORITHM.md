# Algorithm Notes

## 1. Magnetometer calibration

\[
m_{cal}=(m_{raw}-b)A
\]

## 2. Magnetic heading

\[
\psi_m=atan2(M_y,M_x)
\]

## 3. Magnetic vector elevation

\[
\alpha_m=atan2(M_z,\sqrt{M_x^2+M_y^2})
\]

This is a magnetic-state feature, not mechanical pitch.

## 4. Circular error

\[
e=((\psi_{true}-\psi_{measured}+180)\bmod360)-180
\]

## 5. LUT

```text
(calibrated heading, magnetic elevation)
        -> correction
```

## 6. Encoder tilt compensation

For a pitch rotation about Y, one common frame-dependent form is:

\[
X_h=M_x\cos\theta+sM_z\sin\theta
\]

\[
Y_h=M_y
\]

\[
\psi=atan2(Y_h,X_h)
\]

The sign \(s\) must match the sensor frame.

## 7. IMU pitch

\[
\theta_{acc}=atan2(a_x,\sqrt{a_y^2+a_z^2})
\]

\[
\theta_{gyro,k}=\theta_{k-1}+\omega_y\Delta t
\]

\[
\theta_k=\alpha\theta_{gyro,k}+(1-\alpha)\theta_{acc,k}
\]

with \(\alpha=0.98\) in the validated experiment.

## 8. AHRS direction

The next stage estimates a quaternion from:

```text
gyroscope + accelerometer + calibrated magnetometer
```

using Madgwick or xioTechnologies Fusion, then extracts yaw as the tilt-compensated heading.
