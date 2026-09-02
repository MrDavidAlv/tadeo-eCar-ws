"""Single source of truth for the indoor factory world.

Emits two artefacts from the SAME geometry list, so they cannot drift apart:

  * ``tadeocar_gazebo/worlds/factory.world``                      - SDF for Gz Sim
  * ``tadeocar_navigation/maps/factory_ground_truth.{pgm,yaml}``  - exact plan

This is the flat world: the one where wheel odometry is right, SLAM Toolbox has
walls at every heading to match against, and Nav2 can be judged on planning
rather than on localisation. The yard world next door is the one that breaks
those assumptions on purpose.

Layout, 20.5 x 15.25 m of clear floor:

    +-------------------------------------------------+
    |  racks (north)                     machine NE    |
    |                                                  |
    | dock          main aisle, 8.6 m wide             |
    | bay                                              |
    |                                                  |
    |  racks (south)                     machine SE    |
    +-------------------------------------------------+

The robot is 1.16 m long and 0.65 m wide over the wheels, which is what sets
the aisle widths below: a 1.2 m gap that looks generous on a plan leaves a 4WS
robot with 0.27 m a side and no room to reorient, so the working aisles here
are 2 m or wider and the narrow gaps between rack rows are deliberately dead
ends the planner is expected to avoid.

Usage:
    python3 generate_factory_world.py            # world + ground truth map
    python3 generate_factory_world.py <path>     # world only, written to <path>
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from world_common import (  # noqa: E402
    CEMENT, CRATE, DRUM, MU_CEMENT, PAINT, RACK, STEEL, WALL_MAT,
    Layout, workspace_src)

# ---------------------------------------------------------------- geometry --
T = 0.25          # wall thickness
H = 2.5           # wall height
HALF_X = 10.0     # clear floor half-length
HALF_Y = 7.5      # clear floor half-width

SPAWN = (0.0, 0.0)

f = Layout('factory')

# --------------------------------------------------------------- perimeter --
f.wall('wall_north', 0.0, HALF_Y + T / 2, 2 * HALF_X + T, T, H)
f.wall('wall_south', 0.0, -(HALF_Y + T / 2), 2 * HALF_X + T, T, H)
f.wall('wall_east', HALF_X + T / 2, 0.0, T, 2 * HALF_Y + T, H)
f.wall('wall_west', -(HALF_X + T / 2), 0.0, T, 2 * HALF_Y + T, H)

# ------------------------------------------------------------------- racks --
# Two rows a side, 1.5 m tall, split by a cross aisle at x = -0.5 so the robot
# can cut between north and south without driving all the way to the wall.
# The 3 m gap at x in [-1.5, 1.5] is the cross aisle, and it is 3 m because
# the robot is 1.16 m long: a 4WS chassis that has to reorient inside a gap
# needs room for its diagonal, not just its width.
for sign, tag in ((1, 'n'), (-1, 's')):
    for row, y in ((1, 4.5), (2, 6.0)):
        f.box(f'rack_{tag}{row}a', -5.25, sign * y, 0.75, 7.5, 0.4, 1.5, RACK)
        f.box(f'rack_{tag}{row}b', 3.5, sign * y, 0.75, 4.0, 0.4, 1.5, RACK)

# ---------------------------------------------------------------- machines --
for sign, tag in ((1, 'ne'), (-1, 'se')):
    f.box(f'machine_{tag}', 7.75, sign * 3.5, 0.9, 3.5, 3.5, 1.8, STEEL)
    f.box(f'machine_{tag}_panel', 7.75, sign * 3.5, 1.85, 3.5, 3.5, 0.1, (0.2, 0.3, 0.4))

# ----------------------------------------------------------------- pillars --
f.box('pillar_nw', -5.0, 3.5, 1.5, 0.35, 0.35, 3.0, WALL_MAT)
f.box('pillar_sw', -5.0, -3.5, 1.5, 0.35, 0.35, 3.0, WALL_MAT)
f.box('pillar_ne', 4.5, 3.5, 1.5, 0.35, 0.35, 3.0, WALL_MAT)
f.box('pillar_se', 4.5, -3.5, 1.5, 0.35, 0.35, 3.0, WALL_MAT)

# ------------------------------------------------------------------ crates --
# 0.6 m cubes: 0.07 m of clearance over the 0.53 m LiDAR plane. They are the
# shortest thing in this world the scan can see, and the check in world_common
# is what guarantees nothing shorter than that ever gets added by accident.
# Out on the open floor between the aisle and the rack faces, where the robot
# has to plan around them. The original world tucked this cluster into the
# 1.1 m pocket between the two rack rows, which put two of the crates inside a
# rack and the rest somewhere no 0.65 m robot could ever reach.
for sign, tag in ((1, 'n'), (-1, 's')):
    f.box(f'crate_{tag}1', 2.0, sign * 3.0, 0.3, 0.6, 0.6, 0.6, CRATE)
    f.box(f'crate_{tag}2', 2.75, sign * 3.0, 0.3, 0.6, 0.6, 0.6, CRATE)
    f.box(f'crate_{tag}3', 2.0, sign * 3.75, 0.3, 0.6, 0.6, 0.6, CRATE)
f.box('crate_w1', -7.0, 3.0, 0.35, 0.7, 0.7, 0.7, CRATE)
f.box('crate_w2', -7.0, -3.0, 0.35, 0.7, 0.7, 0.7, CRATE)

# ----------------------------------------------------------------- barrels --
f.cyl('barrel_north', -3.0, 2.5, 0.0, 0.25, 0.8, DRUM)
f.cyl('barrel_south', -3.0, -2.5, 0.0, 0.25, 0.8, DRUM)
f.cyl('barrel_east', 4.5, 0.0, 0.0, 0.25, 0.8, DRUM)

# -------------------------------------------------------------- loading bay --
# The bay is painted floor, not a platform. It used to be a 0.2 m slab, which
# is the worst of both worlds: 0.1 m wheels cannot climb it and a LiDAR plane
# at 0.53 m cannot see it, so the robot drove into a step that existed in the
# physics and in no sensor. What makes a dock a dock for a navigation stack is
# somewhere to stop, so it is a marked target on the floor, with the structure
# that has to be avoided standing at full height behind it.
f.slab('dock_bay', -8.0, 0.0, 3.0, 4.0, 0.01, PAINT, MU_CEMENT)
f.box('dock_wall', -9.4, 0.0, 0.6, 0.3, 4.4, 1.2, STEEL)
f.box('dock_rail_north', -7.7, 2.2, 0.5, 2.4, 0.15, 1.0, STEEL)
f.box('dock_rail_south', -7.7, -2.2, 0.5, 2.4, 0.15, 1.0, STEEL)

# -------------------------------------------------------------------- roof --
# Steel deck at 3.2 m on trusses. The trusses matter as much as the deck: a
# flat ceiling is one more large uniform surface, while beams every 2.5 m put
# hard edges across the top of the camera's field of view.
f.ceiling('roof', 0.0, 0.0, 2 * HALF_X + T, 2 * HALF_Y + T, 3.2, STEEL)
for i in range(-4, 5):
    f.box(f'truss_{i + 4}', i * 2.5, 0.0, 3.05, 0.25, 2 * HALF_Y, 0.30, STEEL,
          MU_CEMENT, 0.0, 'ceiling')

# ------------------------------------------------------------------ checks --
# Lanes the planner is entitled to assume exist. The cross aisle at x = -0.5 is
# the one worth asserting on: it is the only way between the north and south
# halves that does not go around the machines.
f.check(corridors=(
    ('north lane', -9.0, 9.5, 0.5, 1.5),
    ('south lane', -9.0, 9.5, -1.5, -0.5),
    ('dock approach', -9.0, -5.0, -1.0, 1.0),
    ('cross aisle', -1.2, 1.2, -HALF_Y + T, HALF_Y - T),
))

# Bay luminaires under the roof, on the same 5 m grid as the trusses. Without
# them the building is lit only by whatever the sun reaches through the doorway
# gap, and a camera in a dark building is a camera with no features.
BAY_LIGHTS = [(x, y, 3.0) for x in (-7.5, -2.5, 2.5, 7.5) for y in (-4.5, 0.0, 4.5)]

world = f.sdf(ground_mu=MU_CEMENT, ground_mat=CEMENT, ambient=0.72,
              background=(0.70, 0.72, 0.75), lights=BAY_LIGHTS)

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    src = workspace_src(here)

    if len(sys.argv) > 1:
        world_path = sys.argv[1]
        map_dir = None
    else:
        world_path = os.path.join(src, 'tadeocar_gazebo', 'worlds', 'factory.world')
        map_dir = os.path.join(src, 'tadeocar_navigation', 'maps')

    open(world_path, 'w').write(world)
    print(f'world: {len(f.boxes)} boxes + {len(f.cylinders)} cylinders -> {world_path}')
    print(f'floor: {2*HALF_X:.2f} x {2*HALF_Y:.2f} m, spawn {SPAWN}')

    if map_dir:
        os.makedirs(map_dir, exist_ok=True)
        f.write_map(os.path.join(map_dir, 'factory_ground_truth.pgm'),
                    os.path.join(map_dir, 'factory_ground_truth.yaml'), SPAWN)
