## Fixed robot geometry

Offsets from the grasp reference, in local `(tool, jaw, tool cross jaw)` axes and metres:
- front-tip midpoint: `[0.006051, 0, 0]`;
- distal-pad bounding-box midpoint: `[0.001051, 0, 0]`.
Use `geom.grasp_reference_for` with `region='front_tip'` or
`region='distal_pad_bbox_center'` for the intended working region. These are fixed
geometric references; actual pad contact depends on object shape and alignment.
The flat pad face is about 10 mm long along the tool axis and 18 mm wide; the rear finger structure is wider.
Full-open inner gap is about 82.0 mm; empty closed jaws close to 0 mm.

The fixed table plane is world `z=0.000` m. An object's upper surface is above it
by the object's thickness. Pixel scale depends on depth; use calibrated projection
or plane-scale helpers for the actual working point. For triangulation, match the
same identifiable feature and check reprojection; a small ray residual alone does
not establish a correct match.

Detailed dimensions are in `/work/geometry_reference.md`; reference measurements are in
`/work/geometry_evidence.md`. Historical tracking errors and settling times are
examples, not operating bounds. Robot pose accuracy does not establish object pose
or contact-point accuracy.
