# CAD reference (read-only)

Two parts from the SO-101 CAD assembly, as triangle meshes in each part's own body frame, in metres:

| File | Part |
|---|---|
| `base.npz`, `base.stl` | Base (the blue part in the images) |
| `motor.npz`, `motor.stl` | M1, an STS3215 servo motor (the orange part in the images) |

`*.npz` holds `vertices` (N,3 float64, metres) and `faces` (M,3 int64); load with `np.load`.
The STL files hold the same meshes.

`assembly.json` gives `T_base_motor`, the 4x4 homogeneous transform of the M1 body frame expressed in the
Base body frame when the two parts are assembled (CAD design pose), and each part's mesh bounds.

A part's body frame is the frame its mesh vertices are written in. In the simulator the parts rest on the
table in some unknown pose; object poses are not supplied, so locate the parts from the images.
These are design meshes: printed parts and simulator colliders can differ from them by fractions of a millimetre.
