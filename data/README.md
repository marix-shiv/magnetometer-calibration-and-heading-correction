# Dataset formats

## 3D magnetometer calibration

```csv
mag_x,mag_y,mag_z
131,-54,-403
129,-51,-399
```

## Known-yaw LUT training

```csv
mag_x,mag_y,mag_z,current_pitch,actual_yaw
120,-65,-401,40,90
122,-64,-399,40,90
```

`actual_yaw` is the physical yaw used as ground truth.
