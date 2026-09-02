"""A flat, featureless world, emitted with the same plugin set as the other two.

It exists for measurement, not for demos: odometry error, steering response and
the friction of a single surface are all easier to read when there is nothing
for the robot to hit and nothing for SLAM to latch onto. Regenerating it here
rather than keeping a hand-written copy is what guarantees it carries the same
physics settings and the same IMU system plugin as the worlds the results are
meant to transfer to.

Usage:
    python3 generate_empty_world.py            # writes worlds/empty.world
    python3 generate_empty_world.py <path>
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from world_common import CEMENT, MU_CEMENT, Layout, workspace_src  # noqa: E402

e = Layout('empty')
world = e.sdf(ground_mu=MU_CEMENT, ground_mat=CEMENT, ambient=0.6,
              background=(0.70, 0.75, 0.82))

if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        world_path = sys.argv[1]
    else:
        world_path = os.path.join(workspace_src(here), 'tadeocar_gazebo',
                                  'worlds', 'empty.world')
    open(world_path, 'w').write(world)
    print(f'world: flat ground plane, mu={MU_CEMENT} -> {world_path}')
