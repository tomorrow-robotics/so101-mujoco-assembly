# Geometry reference measurements

Fixed robot dimensions and current units are in `/work/geometry_reference.md`.

The records below describe a small set of reference measurements. Preserve their numbers
as evidence, not universal accuracy bounds, fixed pixel scales, settling durations
or extra steps required for every action. Current task strategy remains in the main prompt.

The key, coin and bread records below come from the upstream ARX X5 benchmark probes. Their
lessons about verifying possession, ambiguous correspondences and signed point order apply here;
their pixel counts, gaps and timings are X5-specific and are not Panthera measurements.

### Key probe

In a key exploration, a small verified lift separated the two visual hypotheses clearly:
its bow stayed within about 2 pixels of the co-motion prediction; if left on the table,
it would have appeared roughly 46 pixels away in that wrist view. The probe compared "held" and "left behind" predictions after a small motion
using fresh pixels. Closed fingers alone are insufficient. A subsequent 10-degree yaw and
gentle return/release were also observed; these short tests do not establish robustness
for a full insertion or large transport.

### Tracking and image scale (Panthera, this simulator)

In a robot-only free-space move of about 20 cm to a top-down pose above the table, the native
joint-target controller arrived within 2.7 mm and 2.3 degrees after the standard convergence
phase. This is one observed example, not a universal bound; fine targets should be approached
with short final segments and checked in fresh images. Wrist cameras are 640x480 with focal
length about 392 px; head camera 640x480 with focal length about 604 px. Pixel scale depends on
depth; use calibrated projection.

### Gripper settling probes (Panthera)

Robot-only probes (arm held still) measured the inner pad gap at 11 commands from 0 to 1: it
followed `82.0*g` mm and settled within 0.05 mm in 0.08 s or less. An empty close reaches 0 mm.
Settling occurs during executed simulation steps, not rendering or API waiting. The commanded
gap does not distinguish an empty grasp from holding an object.

### Coin probe

A coin exploration confirmed the ambiguity: a coin remained held through a 23 mm lift
and a small yaw even with the same approximately 0.185 opening as an empty close.
A thin edge occupied only about 12 pixels in the head view; mismatched head/wrist features
produced millimetre-scale depth errors despite plausible geometry. Additional wrist views and comparisons with background motion helped identify
ambiguous correspondences; weak template matches remained unreliable. A projected-pixel agreement checks the correspondence locally;
it does not establish metric accuracy everywhere.

### Bread probe

A bread probe also exposed the sign sensitivity of the ordered points: reversing the
jaw-axis point order caused an unintended near-180-degree wrist roll. This demonstrated that the ordered points encode a signed jaw axis. Small translation/yaw reprojections
agreed within roughly 1–2 pixels in the tested wrist views. The bread was lifted and
returned upright between open fingers; final seating and lasting stability could not
be established from one image. The narrow rack also made sideways withdrawal a risk to seating and support.
