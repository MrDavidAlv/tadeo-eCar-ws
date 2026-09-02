"""The URDF and the SDF must describe the same robot.

Two files hold the geometry: tadeocar_description/urdf/tadeocar.urdf.xacro,
which TF and RViz read, and tadeocar_gazebo/models/tadeocar_v1/model.sdf, which
the physics reads. Neither can be dropped - one has no mass, the other has no
package:// paths - so the only protection against them drifting apart is a test
that reads both and compares the numbers.

This is not hypothetical. Before this test existed the pair disagreed about:

  * the wheel radius, 0.1 m against a 0.125 m mesh, in both files;
  * the LiDAR height, 0.33 m in one URDF and 0.43 m in another;
  * whether the steering joints had limits at all.

Run directly with ``python3 -m pytest`` or through ``colcon test``.
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(os.path.dirname(HERE))
XACRO = os.path.join(SRC, 'tadeocar_description', 'urdf', 'tadeocar.urdf.xacro')
SDF = os.path.join(SRC, 'tadeocar_gazebo', 'models', 'tadeocar_v1', 'model.sdf')

TOL = 1e-6


def _urdf():
    out = subprocess.run(['xacro', XACRO], capture_output=True, check=True)
    return ET.fromstring(out.stdout)


def _sdf():
    return ET.parse(SDF).getroot().find('model')


@pytest.fixture(scope='module')
def urdf():
    return _urdf()


@pytest.fixture(scope='module')
def sdf():
    return _sdf()


def _urdf_joint(root, name):
    for j in root.findall('joint'):
        if j.get('name') == name:
            return j
    raise AssertionError(f'URDF has no joint {name}')


def _sdf_pose(model, link_name):
    for link in model.findall('link'):
        if link.get('name') == link_name:
            pose = link.find('pose')
            vals = [float(v) for v in (pose.text if pose is not None else '0 0 0 0 0 0').split()]
            return vals + [0.0] * (6 - len(vals))
    raise AssertionError(f'SDF has no link {link_name}')


def _urdf_origin(root, joint_name):
    o = _urdf_joint(root, joint_name).find('origin')
    xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
    return xyz + rpy


# --------------------------------------------------------------------------
# Link placement. SDF link poses are absolute in the model frame, where z = 0
# is the ground; URDF joint origins are relative to their parent, and
# base_footprint -> base_link lifts everything by one wheel radius.
# --------------------------------------------------------------------------
def test_base_link_sits_at_wheel_radius(urdf, sdf):
    radius = _urdf_origin(urdf, 'base_footprint_joint')[2]
    assert abs(_sdf_pose(sdf, 'base_link')[2] - radius) < TOL


@pytest.mark.parametrize('link,joint', [
    ('base_scan', 'lidar_joint'),
    ('zed2i_camera_link', 'zed2i_camera_joint'),
    ('sensor_mast_link', 'sensor_mast_joint'),
])
def test_sensor_mounts_agree(urdf, sdf, link, joint):
    base_z = _sdf_pose(sdf, 'base_link')[2]
    u = _urdf_origin(urdf, joint)
    s = _sdf_pose(sdf, link)
    assert abs(s[0] - u[0]) < TOL, f'{link} x'
    assert abs(s[1] - u[1]) < TOL, f'{link} y'
    assert abs(s[2] - (base_z + u[2])) < TOL, f'{link} z'


@pytest.mark.parametrize('prefix', ['front_left', 'front_right',
                                    'rear_left', 'rear_right'])
def test_steering_axes_agree(urdf, sdf, prefix):
    base_z = _sdf_pose(sdf, 'base_link')[2]
    u = _urdf_origin(urdf, f'{prefix}_steering_joint')
    s = _sdf_pose(sdf, f'{prefix}_steering_link')
    assert abs(s[0] - u[0]) < TOL
    assert abs(s[1] - u[1]) < TOL
    assert abs(s[2] - base_z) < TOL


@pytest.mark.parametrize('prefix', ['front_left', 'front_right',
                                    'rear_left', 'rear_right'])
def test_wheel_centres_agree(urdf, sdf, prefix):
    """The wheel sits one mechanical trail off its steering axis."""
    steer = _urdf_origin(urdf, f'{prefix}_steering_joint')
    trail = _urdf_origin(urdf, f'{prefix}_wheel_joint')
    s = _sdf_pose(sdf, f'{prefix}_wheel_link')
    assert abs(s[0] - (steer[0] + trail[0])) < 1e-4
    assert abs(s[1] - (steer[1] + trail[1])) < TOL


# --------------------------------------------------------------------------
# Wheel size. The one number that turns rotation into distance.
# --------------------------------------------------------------------------
def test_wheel_radius_agrees(urdf, sdf):
    u = None
    for link in urdf.findall('link'):
        if link.get('name') == 'front_left_wheel_link':
            cyl = link.find('collision/geometry/cylinder')
            u = (float(cyl.get('radius')), float(cyl.get('length')))
    assert u is not None, 'URDF wheel has no cylinder collision'

    for link in sdf.findall('link'):
        if link.get('name') == 'front_left_wheel_link':
            cyl = link.find('collision/geometry/cylinder')
            s = (float(cyl.find('radius').text), float(cyl.find('length').text))
    assert abs(u[0] - s[0]) < TOL, f'radius: urdf {u[0]} vs sdf {s[0]}'
    assert abs(u[1] - s[1]) < TOL, f'width: urdf {u[1]} vs sdf {s[1]}'


def test_wheel_radius_matches_the_mesh(urdf):
    """0.125 m, measured off wheel.dae. See the note in the xacro."""
    for link in urdf.findall('link'):
        if link.get('name') == 'front_left_wheel_link':
            cyl = link.find('collision/geometry/cylinder')
            assert abs(float(cyl.get('radius')) - 0.125) < 1e-9


# --------------------------------------------------------------------------
# Joint limits. A description that promises more travel than the servo has is
# a planner failure waiting to happen.
# --------------------------------------------------------------------------
@pytest.mark.parametrize('prefix', ['front_left', 'front_right',
                                    'rear_left', 'rear_right'])
def test_steering_limits_agree(urdf, sdf, prefix):
    lim = _urdf_joint(urdf, f'{prefix}_steering_joint').find('limit')
    lo, hi = float(lim.get('lower')), float(lim.get('upper'))

    for joint in sdf.findall('joint'):
        if joint.get('name') == f'{prefix}_steering_joint':
            axis = joint.find('axis/limit')
            assert abs(float(axis.find('lower').text) - lo) < 1e-4
            assert abs(float(axis.find('upper').text) - hi) < 1e-4
            break
    else:
        raise AssertionError(f'SDF has no joint {prefix}_steering_joint')

    # 270 degree servos, centred on straight ahead.
    assert abs(hi - math.radians(135)) < 2e-3


# --------------------------------------------------------------------------
# The LiDAR has to clear the deck, or a 360 degree scan reads the robot.
# --------------------------------------------------------------------------
def test_lidar_clears_the_deck(sdf):
    deck_top = None
    for link in sdf.findall('link'):
        if link.get('name') == 'base_link':
            box = link.find('collision/geometry/box/size')
            pose = [float(v) for v in link.find('collision/pose').text.split()]
            base_z = float(link.find('pose').text.split()[2])
            deck_top = base_z + pose[2] + float(box.text.split()[2]) / 2
    scan_z = _sdf_pose(sdf, 'base_scan')[2]
    assert deck_top is not None
    assert scan_z > deck_top + 0.05, (
        f'scan plane {scan_z:.3f} m only clears the deck top {deck_top:.3f} m '
        f'by {1000*(scan_z-deck_top):.0f} mm')


def test_lidar_sweeps_a_full_circle(sdf):
    for link in sdf.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type') != 'gpu_lidar':
                continue
            h = sensor.find('lidar/scan/horizontal')
            lo = float(h.find('min_angle').text)
            hi = float(h.find('max_angle').text)
            assert abs((hi - lo) - 2 * math.pi) < 1e-3, 'scan is not 360 degrees'
            rng = sensor.find('lidar/range')
            assert abs(float(rng.find('min').text) - 0.12) < 1e-9
            assert abs(float(rng.find('max').text) - 8.0) < 1e-9
            return
    raise AssertionError('SDF has no gpu_lidar')
