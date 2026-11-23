# Map-quality measurement worksheet

Record these once before the next moving capture. Rough tape-measure values are
more useful than visual guesses; use meters and note the measurement uncertainty.

## Robot and LiDAR

- Robot maximum width: 0.22 m (Waveshare manufacturer footprint, conservative)
- Robot maximum length: 0.22 m
- Desired extra clearance on each side: 0.05 m
- `base_link` physical reference point:
- LiDAR center forward (+) / backward (-) from `base_link`:
- LiDAR center left (+) / right (-) from `base_link`: approximately 0 m, provisional
- LiDAR height:
- LiDAR yaw relative to robot forward: 0 degrees provisional; verify scan convention
- Mount rigid when gently pushed: yes/no
- LiDAR approximately level: yes/no

The tested configuration currently assumes LiDAR x=0.00 m, y=0.00 m,
z=0.1778 m, yaw=pi rad. Do not change it from estimates; record measured values
first so a transform candidate can be replayed and independently tested.

For straight-passage screening, use a 0.32 m minimum (0.22 m width plus
0.05 m per side). This does not define turning clearance or turning radius.
Turning must be checked using the full rectangular footprint swept through the
turn; never approximate it as half the width.

## Test space

- Surface and transitions:
- First straight length:
- First corridor/room clear width:
- Corner angle:
- Second straight length:
- Second corridor/room clear width:
- Required doorway/opening widths:
- Glass, mirrors, shiny appliances, low furniture, or moving objects:
- Start and checkpoint tape locations retained: yes/no

Draw a rough top-down sketch, label the start and goals, and put the dimensions
beside the corresponding walls/openings. Take one photo from each leg if the
space can be photographed without including private material.

## Navigation acceptance values

- Smallest required opening:
- Required center clearance = robot width / 2 + side clearance:
- Preliminary localization tolerance:
- Preliminary heading tolerance:

Keep the existing trajectory gates unless evidence supports changing them:
final position <=0.10 m, final heading <=5 degrees, checkpoint SLAM/odometry
disagreement <=0.10 m, and timing/stream freshness PASS.
