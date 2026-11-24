## Measured arm reach (this robot, this table)

Kinematics only (joint limits, no collision check), measured with the IK the planner uses for single poses. "Tilt" is the angle of the tool direction from straight down: 0 deg = tool pointing straight down, 90 deg = horizontal. Each table is one lean direction of the tilted tool, seen from above; the jaw axis is horizontal, across the lean, with one fixed sign (jaw = world up x lean direction). Only that jaw sign was probed: the table leaning back toward the arm base is unreachable even at 0 deg tilt, where only the jaw sign differs from the reachable away-leaning table, so the reversed jaw may reach poses marked none there. Each cell is the range of horizontal distance (m) between the tool point and the arm's base axis, along world -y (the work area in front of the arms), in which that pose is reachable; `none` = not reachable at any distance. Probed every 0.02 m from 0.10 to 0.70 m. Heights are above the table top (world z = 0). A pose outside these ranges is rejected by the planner with "Cartesian segment is not reachable".

### left arm, base axis at world (x, y) = (-0.332, 0.706) m

**left arm, tool leaning away from the arm base (toward world -y)**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.10-0.48 | 0.14-0.62 | 0.20-0.68 | 0.30-0.70 | none |
| 0.06 | 0.10-0.46 | 0.12-0.60 | 0.16-0.68 | 0.22-0.70 | none |
| 0.10 | 0.10-0.42 | 0.10-0.58 | 0.14-0.66 | 0.20-0.70 | 0.48-0.70 |
| 0.14 | 0.14-0.36 | 0.10-0.56 | 0.10-0.64 | 0.16-0.70 | 0.34-0.70 |
| 0.18 | none | 0.10-0.54 | 0.10-0.62 | 0.12-0.68 | 0.26-0.70 |
| 0.25 | none | 0.16-0.48 | 0.10-0.56 | 0.10-0.66 | 0.22-0.70 |

**left arm, tool leaning sideways toward world +x**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.14-0.48 | 0.14-0.46 | 0.12-0.44 | 0.14-0.38 | none |
| 0.06 | 0.14-0.46 | 0.14-0.44 | 0.14-0.40 | 0.12-0.32 | none |
| 0.10 | 0.12-0.42 | 0.12-0.40 | 0.14-0.36 | none | none |
| 0.14 | 0.14-0.36 | 0.20-0.32 | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |

**left arm, tool leaning sideways toward world -x**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.14-0.48 | 0.14-0.46 | 0.12-0.44 | 0.14-0.38 | none |
| 0.06 | 0.14-0.46 | 0.14-0.44 | 0.14-0.40 | 0.12-0.32 | none |
| 0.10 | 0.12-0.42 | 0.12-0.40 | 0.16-0.36 | none | none |
| 0.14 | 0.14-0.36 | 0.20-0.32 | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |

**left arm, tool leaning back toward the arm base (toward world +y)**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | none | none | none | none | none |
| 0.06 | none | none | none | none | none |
| 0.10 | none | none | none | none | none |
| 0.14 | none | none | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |

### right arm, base axis at world (x, y) = (0.218, 0.724) m

**right arm, tool leaning away from the arm base (toward world -y)**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.10-0.48 | 0.14-0.62 | 0.20-0.68 | 0.30-0.70 | none |
| 0.06 | 0.10-0.46 | 0.12-0.60 | 0.16-0.68 | 0.22-0.70 | none |
| 0.10 | 0.10-0.42 | 0.10-0.58 | 0.14-0.66 | 0.20-0.70 | 0.48-0.70 |
| 0.14 | 0.14-0.36 | 0.10-0.56 | 0.10-0.64 | 0.16-0.70 | 0.34-0.70 |
| 0.18 | none | 0.10-0.54 | 0.10-0.62 | 0.12-0.68 | 0.26-0.70 |
| 0.25 | none | 0.16-0.48 | 0.10-0.56 | 0.10-0.66 | 0.22-0.70 |

**right arm, tool leaning sideways toward world +x**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.14-0.48 | 0.14-0.46 | 0.14-0.44 | 0.14-0.38 | none |
| 0.06 | 0.14-0.46 | 0.14-0.44 | 0.14-0.40 | 0.12-0.32 | none |
| 0.10 | 0.12-0.42 | 0.12-0.40 | 0.14-0.36 | none | none |
| 0.14 | 0.14-0.36 | 0.20-0.32 | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |

**right arm, tool leaning sideways toward world -x**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | 0.14-0.48 | 0.14-0.46 | 0.12-0.44 | 0.14-0.38 | none |
| 0.06 | 0.14-0.46 | 0.14-0.44 | 0.14-0.40 | 0.12-0.32 | none |
| 0.10 | 0.12-0.42 | 0.12-0.40 | 0.16-0.36 | none | none |
| 0.14 | 0.14-0.36 | 0.20-0.32 | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |

**right arm, tool leaning back toward the arm base (toward world +y)**

| tool-point height above table (m) | tilt 0 deg | tilt 30 deg | tilt 45 deg | tilt 60 deg | tilt 90 deg |
|---|---|---|---|---|---|
| 0.02 | none | none | none | none | none |
| 0.06 | none | none | none | none | none |
| 0.10 | none | none | none | none | none |
| 0.14 | none | none | none | none | none |
| 0.18 | none | none | none | none | none |
| 0.25 | none | none | none | none | none |
