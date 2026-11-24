# Geometry quick reference

Use this file for units, common calculations and fixed robot dimensions.
Full helper contracts are in `/work/geometry_interfaces.md`; reference measurements are
in `/work/geometry_evidence.md`. Read only the file needed for the current decision.
Each file fits in one tool response when printed alone.

The helpers use supplied observations and caller-selected pixels or points. They do
not locate objects or check task success. Current gripper state is command feedback,
not measured opening or contact. Robot pose accuracy does not establish object pose
or contact-point accuracy.

### Inputs, outputs and units

`np`, `Image`, `plt`, `obs` and `geom` are preloaded. Use `print()` to return values.
World positions/displacements are finite numeric three-vectors in metres; lists and
NumPy arrays are accepted. Pixel coordinates are `(u,v)=(x,y)` in the original image,
with origin at the upper left. NumPy indexing is `[y,x]`. Rotation vectors use radians.
Camera names: `cam_head`, `cam_left_wrist`, `cam_right_wrist`.

### Common calculations

Use `obs.state()` for full-precision robot state and calibration. Build a pose target
with `geom.target_from_state(obs.state()['state']['left'])` (or `'right'`), then use
`geom.translate_points` or `geom.rotate_points`; preserve the physical finger order.
Use `geom.grasp_reference_for` to place the intended finger region at a working point.

For image geometry, use `geom.camera_ray` and `geom.project_camera` with a camera name.
Use `geom.plane_pixel_scale` for a known surface; `geom.pixel_scale` depends on depth.
For two-view triangulation, match the same feature and verify its reprojection in
both images. A small ray residual alone does not validate the match.
For a historical image, pass the same `observation_id` to the camera helper.

## Fixed geometry and its source

These dimensions came from the compiled native Panthera-HT MuJoCo model (two stock 6-DoF arms with
parallel grippers) and robot-only probes in `scripts/q0_calibrate_panthera.py`
(`controller/calibration_panthera/gripper.json`). The native TCP site is the grasp reference; it is
0.165 m in front of the link6 flange along the tool axis. Geometry values are not per-scene object
measurements.

### Reference frames and dimensions

The grasp frame has axes `(+X tool, +Y jaw, +Z tool cross jaw)`. The `-Y` finger is
`gripper_R_link` (p0) and the `+Y` finger is `gripper_L_link` (p1), for either arm.
Offsets below are relative to the grasp reference in the same local axes:

| Finger-region midpoint | Offset in metres |
| --- | --- |
| Front tip | `[0.006051,0,0]` |
| Distal flat-pad bounding box | `[0.001051,0,0]` |

`grasp_reference_for` applies these offsets. Tip and pad references differ by 5.0 mm along the
tool axis; choose the region that fits the object's thickness and available support clearance.
The pad bounding-box centre is a geometric reference, not a contact-pressure estimate.

| Additional dimension | Approximate value |
| --- | --- |
| Flat inner pad face, tool axis relative to grasp reference | -3.95 to +6.05 mm |
| Flat-face axial length | 10.00 mm |
| Flat-face rear edge behind grasp reference | 3.95 mm |
| Flat-face width (tool cross jaw) | 18.06 mm |
| Complete finger extent: tool / jaw / lateral | 118.1 / 35.0 / 92.0 mm |

The wider rear finger structure is included in the complete extent above.

### Opening and table baseline

Full-open inner pad gap is 82.0 mm. Empty-close probes closed to 0.0 mm (the pads meet).
The measured empty-gripper inner gap for normalized command `g` is `82.0*clip(g,0,1)` mm
(linear within 0.001 mm over 11 probe points); `width_mm` commands use the inverse map.
No measured gap is available to the current policy; gripper state is a command echo.

The compiled lab table top is the plane `z=0.000` m. This is the support surface, not the
unknown top of an object resting on it.
