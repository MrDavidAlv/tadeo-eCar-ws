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
    return _origin6(_urdf_joint(root, joint_name).find('origin'))


def _origin6(o):
    if o is None:
        return [0.0] * 6
    xyz = [float(v) for v in (o.get('xyz') or '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy') or '0 0 0').split()]
    return xyz + rpy


def _pose6(el):
    pose = el.find('pose')
    vals = [float(v) for v in (pose.text if pose is not None else '0 0 0 0 0 0').split()]
    return vals + [0.0] * (6 - len(vals))


def _urdf_link(root, name):
    for link in root.findall('link'):
        if link.get('name') == name:
            return link
    raise AssertionError(f'URDF has no link {name}')


def _sdf_link(model, name):
    for link in model.findall('link'):
        if link.get('name') == name:
            return link
    raise AssertionError(f'SDF has no link {name}')


def _rot(r, p, y):
    """Rotation matrix of fixed-axis roll, pitch, yaw: the convention of both
    URDF rpy and SDF poses. Compared as matrices, so that (pi, 0, pi) and
    (0, pi, 0) are recognised as the same rotation."""
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _same_pose(a, b, z_offset=0.0, tol=TOL):
    """a from the SDF, b from the URDF, which sits z_offset lower."""
    if any(abs(a[k] - b[k] - (z_offset if k == 2 else 0.0)) > tol for k in range(3)):
        return False
    ra, rb = _rot(*a[3:]), _rot(*b[3:])
    return all(abs(ra[i][j] - rb[i][j]) < 1e-6 for i in range(3) for j in range(3))


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
    ('base_scan_rear', 'lidar_rear_joint'),
    ('sensor_mast_link', 'sensor_mast_joint'),
    ('sensor_mast_rear_link', 'sensor_mast_rear_joint'),
    ('zed2i_camera_link', 'zed2i_camera_joint'),
    ('deck_plate_link', 'deck_plate_joint'),
    ('upper_chassis_link', 'upper_chassis_joint'),
    ('lamps_link', 'lamps_joint'),
])
def test_fixed_links_agree(urdf, sdf, link, joint):
    """Position and rotation: the camera is pitched and the rear LiDAR faces
    back, and a test of xyz alone passes either of them turned the wrong way."""
    base_z = _sdf_pose(sdf, 'base_link')[2]
    u = _urdf_origin(urdf, joint)
    s = _sdf_pose(sdf, link)
    assert _same_pose(s, u, z_offset=base_z), f'{link}: sdf {s} vs urdf {u}'


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
# The parts whose visuals and collisions are written out twice.
# --------------------------------------------------------------------------
def test_lamps_agree(urdf, sdf):
    """Every lamp housing, lens and bracket, in the same place. Both lamp
    links sit at base_link, so their visual poses compare directly."""
    u = {v.get('name'): _origin6(v.find('origin'))
         for v in _urdf_link(urdf, 'lamps_link').findall('visual')}
    s = {v.get('name'): _pose6(v)
         for v in _sdf_link(sdf, 'lamps_link').findall('visual')}
    assert set(u) == set(s), f'only in urdf {set(u) - set(s)}, only in sdf {set(s) - set(u)}'
    for name in u:
        assert _same_pose(s[name], u[name]), f'{name}: sdf {s[name]} vs urdf {u[name]}'


def test_upper_chassis_collision_agrees(urdf, sdf):
    """One box per tube, the same boxes in both files."""
    def boxes(link, sdf_side):
        out = {}
        for c in link.findall('collision'):
            if sdf_side:
                size = [float(v) for v in c.find('geometry/box/size').text.split()]
                out[c.get('name')] = (_pose6(c), size)
            else:
                size = [float(v) for v in c.find('geometry/box').get('size').split()]
                out[c.get('name')] = (_origin6(c.find('origin')), size)
        return out
    u = boxes(_urdf_link(urdf, 'upper_chassis_link'), False)
    s = boxes(_sdf_link(sdf, 'upper_chassis_link'), True)
    assert set(u) == set(s), f'only in urdf {set(u) - set(s)}, only in sdf {set(s) - set(u)}'
    for name in u:
        assert _same_pose(s[name][0], u[name][0]), f'{name} pose'
        assert all(abs(a - b) < TOL for a, b in zip(s[name][1], u[name][1])), f'{name} size'


def test_urdf_carries_no_mass(urdf):
    """Mass lives in model.sdf alone. A URDF with inertials on some links
    and not on others describes a robot of some other weight, and anything
    that sums it - a URDF-based simulator, a dynamics library - gets that
    robot instead of this one."""
    heavy = [link.get('name') for link in urdf.findall('link')
             if link.find('inertial') is not None]
    assert not heavy, f'URDF links with inertials: {heavy}'


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
# Both LiDARs have to clear what they sit over: the deck, and the tread plate
# on it. Lower, the scan reads the robot.
# --------------------------------------------------------------------------
def _box_top(link):
    """Absolute height of the top of a link's first box collision."""
    c = link.find('collision')
    return (_pose6(link)[2] + _pose6(c)[2]
            + float(c.find('geometry/box/size').text.split()[2]) / 2)


@pytest.mark.parametrize('scan_link', ['base_scan', 'base_scan_rear'])
def test_lidar_clears_the_deck(sdf, scan_link):
    top = max(_box_top(_sdf_link(sdf, 'base_link')),
              _box_top(_sdf_link(sdf, 'deck_plate_link')))
    scan_z = _sdf_pose(sdf, scan_link)[2]
    assert scan_z > top + 0.05, (
        f'{scan_link} scan plane {scan_z:.3f} m only clears the top of the deck '
        f'and plate, {top:.3f} m, by {1000*(scan_z-top):.0f} mm')


# --------------------------------------------------------------------------
# Each LiDAR uses the half it faces. The other half looks across the deck and
# would only ever see the upper chassis, the other sensor and the load.
# --------------------------------------------------------------------------
def _lidars(sdf):
    out = {}
    for link in sdf.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type') == 'gpu_lidar':
                out[sensor.get('name')] = (link.get('name'), sensor)
    return out


def test_each_lidar_scans_the_half_it_faces(sdf):
    lidars = _lidars(sdf)
    assert set(lidars) == {'lidar', 'lidar_rear'}
    for name, (_, sensor) in lidars.items():
        h = sensor.find('lidar/scan/horizontal')
        lo = float(h.find('min_angle').text)
        hi = float(h.find('max_angle').text)
        assert abs(lo + math.pi / 2) < 1e-3 and abs(hi - math.pi / 2) < 1e-3, (
            f'{name} scans {math.degrees(lo):.0f} to {math.degrees(hi):.0f} '
            'degrees, not the 180 it faces')
        rng = sensor.find('lidar/range')
        assert abs(float(rng.find('min').text) - 0.12) < 1e-9
        assert abs(float(rng.find('max').text) - 8.0) < 1e-9


def test_the_two_halves_face_opposite_ways(sdf):
    yaw = {name: _sdf_pose(sdf, link)[5] for name, (link, _) in _lidars(sdf).items()}
    assert abs(yaw['lidar']) < 1e-6
    assert abs(abs(yaw['lidar_rear']) - math.pi) < 1e-6
