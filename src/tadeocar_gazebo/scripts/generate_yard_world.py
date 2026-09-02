"""Single source of truth for the outdoor logistics yard.

Emits two artefacts from the SAME geometry list, so they cannot drift apart:

  * ``tadeocar_gazebo/worlds/yard.world``                     - SDF for Gz Sim
  * ``tadeocar_navigation/maps/yard_ground_truth.{pgm,yaml}`` - exact plan

---------------------------------------------------------------------------
What this world is for
---------------------------------------------------------------------------
The factory world is flat, and on flat ground wheel odometry is right. This one
exists to break two assumptions the rest of the stack makes, one per half:

  * **The ramps** break "the world is a plane". Forward kinematics integrates
    wheel rotation as horizontal travel because an encoder cannot tell forward
    from up: drive 2.8 m up a 7 degree ramp and the wheels report 2.8 m of x
    and 0 m of z, when the truth is 2.78 m of x and 0.35 m of z. That is not
    noise, it is a question the sensor cannot answer, and it is why the EKF in
    ``tadeocar_perception`` takes attitude from the ZED 2i's IMU instead.

  * **The apron patches** break "friction is a constant". Asphalt, gravel and
    sand carry genuinely different ``<mu>``/``<mu2>``, not just different
    colours. On a 4WS robot that matters twice over: a crab manoeuvre asks all
    four tyres to roll in a direction the chassis is not pointing, so lateral
    grip decides whether the commanded direction is the one travelled.

Layout, 30 x 20 m, driven as a loop rather than a dead end:

    apron (z=0, asphalt)  --north ramp up-->  dock platform (z=0.35)
             ^                                        |
             +------------- south ramp down ----------+

No stairs anywhere, by design: 0.1 m wheels on a rigid chassis with no
suspension do not climb a riser, they hit it. The ramps are 6.7 degrees, which
that geometry takes without the chassis grounding out - the breakover angle
over a 1.058 m wheelbase at 0.1 m of clearance is about 21 degrees.

Usage:
    python3 generate_yard_world.py            # world + ground truth map
    python3 generate_yard_world.py <path>     # world only, written to <path>
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from world_common import (  # noqa: E402
    ASPHALT, CEMENT, CRATE, DRUM, GRAVEL, MU_ASPHALT, MU_CEMENT, MU_GRAVEL,
    MU_SAND, PAINT, SAND, STEEL, WALL_MAT, Layout, workspace_src)

# ---------------------------------------------------------------- geometry --
T = 0.20           # fence thickness
FENCE_H = 1.8      # fence height
HALF_X = 15.0
HALF_Y = 10.0

RISE = 0.35        # dock platform height above the apron
RUN = 2.8          # ramp horizontal length
RAMP_W = 2.5       # ramp width; the robot is 0.65 m wide and 1.16 m long
RAMP_X0 = 0.2      # ramp foot, apron side
RAMP_X1 = RAMP_X0 + RUN
RAMP_Y = 4.5       # north ramp centreline; the south one mirrors it

PLATFORM_X0 = RAMP_X1
PLATFORM_X1 = 13.5

SPAWN = (-9.0, RAMP_Y)     # lined up with the north ramp, facing +x

y = Layout('yard')

# ----------------------------------------------------------------- fencing --
y.wall('wall_west', -(HALF_X + T / 2), 0.0, T, 2 * HALF_Y + T, FENCE_H)
y.wall('wall_east', HALF_X + T / 2, 0.0, T, 2 * HALF_Y + T, FENCE_H)
y.wall('wall_north', 0.0, HALF_Y + T / 2, 2 * HALF_X + T, T, FENCE_H)
y.wall('wall_south', 0.0, -(HALF_Y + T / 2), 2 * HALF_X + T, T, FENCE_H)

# ------------------------------------------------------------------- ramps --
THETA, LIP = y.ramp('ramp_north', RAMP_X0, RAMP_X1, RAMP_Y, RAMP_W, RISE,
                    mat=CEMENT, mu=MU_CEMENT)
y.ramp('ramp_south', RAMP_X0, RAMP_X1, -RAMP_Y, RAMP_W, RISE,
       mat=CEMENT, mu=MU_CEMENT)

# ---------------------------------------------------------- dock platform ---
y.slab('platform', (PLATFORM_X0 + PLATFORM_X1) / 2, 0.0,
       PLATFORM_X1 - PLATFORM_X0, 2 * HALF_Y, RISE, CEMENT, MU_CEMENT,
       thickness=RISE)

# Barrier along the platform's west face, everywhere the two ramps do not open
# onto it. Without it the edge is a 0.35 m drop, and 0.35 m is the worst height
# an obstacle can be here: too tall for 0.1 m wheels to climb back out of and
# too short for a LiDAR plane at 0.53 m to see, so the robot would find the
# edge with a wheel and nothing in the stack would have warned it. The barrier
# is 0.7 m for exactly that reason, and world_common's height check is what
# refuses to emit the world if a later edit lowers it back into the blind band.
BARRIER_T = 0.25
BARRIER_H = 0.7
edges = [(-HALF_Y, -RAMP_Y - RAMP_W / 2),
         (-RAMP_Y + RAMP_W / 2, RAMP_Y - RAMP_W / 2),
         (RAMP_Y + RAMP_W / 2, HALF_Y)]
for i, (y0, y1) in enumerate(edges):
    y.box(f'barrier_{i}', PLATFORM_X0 + BARRIER_T / 2, (y0 + y1) / 2,
          RISE + BARRIER_H / 2, BARRIER_T, y1 - y0, BARRIER_H, PAINT, MU_CEMENT)

# --------------------------------------------------------- apron surfaces ---
# Three patches, three friction coefficients, laid as lanes the robot drives
# along rather than as decoration off to one side.
y.slab('patch_gravel', -8.0, 0.0, 10.0, 3.2, 0.01, GRAVEL, MU_GRAVEL)
y.slab('patch_sand', -8.0, -6.0, 10.0, 3.2, 0.01, SAND, MU_SAND)
y.slab('lane_paint_north', -6.0, RAMP_Y, 14.0, 0.12, 0.012, PAINT, MU_ASPHALT)
y.slab('lane_paint_south', -6.0, -RAMP_Y, 14.0, 0.12, 0.012, PAINT, MU_ASPHALT)

# --------------------------------------------------- containers on the dock --
# 20 ft containers, 2.44 m wide and 2.59 m tall, standing on the platform.
for i, cy in enumerate((7.0, -7.5)):
    y.box(f'container_{i}', 9.5, cy, RISE + 1.295, 6.06, 2.44, 2.59,
          (0.55, 0.25, 0.20) if i == 0 else (0.20, 0.40, 0.55))
y.box('dock_office', 12.0, 0.0, RISE + 1.4, 2.6, 4.0, 2.8, WALL_MAT)

# ------------------------------------------------------- apron furniture ----
for i, (cx, cy) in enumerate(((-12.0, 8.0), (-9.0, 8.0), (-6.0, 8.0))):
    y.box(f'pallet_stack_{i}', cx, cy, 0.55, 1.2, 1.0, 1.1, CRATE)
y.box('trailer', -3.0, -8.2, 1.25, 8.0, 2.5, 2.5, (0.75, 0.75, 0.78))
y.cyl('light_pole_nw', -13.0, 8.5, 0.0, 0.12, 5.0, STEEL)
y.cyl('light_pole_sw', -13.0, -8.5, 0.0, 0.12, 5.0, STEEL)
y.cyl('drum_a', -5.0, 1.6, 0.0, 0.28, 0.9, DRUM)
y.cyl('drum_b', -5.6, 1.6, 0.0, 0.28, 0.9, DRUM)
y.cyl('drum_c', -5.3, 2.3, 0.0, 0.28, 0.9, DRUM)

# ------------------------------------------------------------------ checks --
# Each ramp is only usable if the robot can reach its foot. That is worth
# asserting rather than eyeballing: a pallet parked across an approach lane
# reads as perfectly reasonable on a plan and turns the loop into a dead end.
y.check(corridors=(
    ('north ramp approach', -HALF_X + T, RAMP_X0,
     RAMP_Y - RAMP_W / 2, RAMP_Y + RAMP_W / 2),
    ('south ramp approach', -HALF_X + T, RAMP_X0,
     -RAMP_Y - RAMP_W / 2, -RAMP_Y + RAMP_W / 2),
    ('platform lane', PLATFORM_X0 + BARRIER_T, 10.5,
     -RAMP_Y + RAMP_W / 2, RAMP_Y - RAMP_W / 2),
    ('west return lane', -HALF_X + T, RAMP_X0, -1.2, 1.2),
))

world = y.sdf(ground_mu=MU_ASPHALT, ground_mat=ASPHALT, ambient=0.5,
              background=(0.55, 0.68, 0.85))

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    src = workspace_src(here)

    if len(sys.argv) > 1:
        world_path = sys.argv[1]
        map_dir = None
    else:
        world_path = os.path.join(src, 'tadeocar_gazebo', 'worlds', 'yard.world')
        map_dir = os.path.join(src, 'tadeocar_navigation', 'maps')

    open(world_path, 'w').write(world)
    print(f'world: {len(y.boxes)} boxes + {len(y.cylinders)} cylinders -> {world_path}')
    print(f'apron {2*HALF_X:.1f} x {2*HALF_Y:.1f} m, platform x in '
          f'[{PLATFORM_X0:.2f}, {PLATFORM_X1:.2f}] at z={RISE} m')
    print(f'ramps: {math.degrees(THETA):.2f} deg, rise {RISE} m over {RUN} m, '
          f'width {RAMP_W} m, lip at the foot {LIP*1000:.1f} mm')
    print(f'apron friction: asphalt mu={MU_ASPHALT}, gravel mu={MU_GRAVEL}, '
          f'sand mu={MU_SAND}')

    if map_dir:
        os.makedirs(map_dir, exist_ok=True)
        y.write_map(os.path.join(map_dir, 'yard_ground_truth.pgm'),
                    os.path.join(map_dir, 'yard_ground_truth.yaml'), SPAWN)
