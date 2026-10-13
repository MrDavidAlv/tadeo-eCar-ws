#!/usr/bin/env python3
"""Render what each sensor covers, from the robot's own files.

Top view: the half each LiDAR scans, the strip beside the robot that neither
does, and the ZED 2i's horizontal field of view. Side view: the two scan
planes and the camera's vertical field of view, down to the first floor it
can see. The robot in both is its visual meshes, projected, and every mount,
angle and range is read from ``tadeocar.urdf.xacro`` and ``model.sdf`` at
render time, as in ``render_diagram.py``.

    source /opt/ros/humble/setup.bash
    python3 docs/mathematical-model/render_sensors.py [out.png]

Default output: images/sensor-coverage.png
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import xacro
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon, Rectangle, Wedge

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_diagram import load_dae, load_stl, rot  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'src')
MESH = os.path.join(SRC, 'tadeocar_description', 'meshes')

INK = '#1b2430'
MUTED = '#5a6675'
GRID = '#e6e8ec'
GREEN = '#2d8a4e'
ORANGE = '#d9822b'
BLUE = '#1d4e89'
BLIND = '#8a8f98'
MONO = 'DejaVu Sans Mono'


# ============================================================== sources =====
def read_model():
    urdf = ET.fromstring(xacro.process_file(
        os.path.join(SRC, 'tadeocar_description/urdf/tadeocar.urdf.xacro')).toxml())
    sdf = ET.parse(os.path.join(SRC, 'tadeocar_gazebo/models/tadeocar_v1/model.sdf')).getroot()
    model = sdf.find('model')

    joints = {}
    for j in urdf.findall('joint'):
        o = j.find('origin')
        if o is not None:
            joints[j.get('name')] = (
                np.array([float(v) for v in (o.get('xyz') or '0 0 0').split()]),
                [float(v) for v in (o.get('rpy') or '0 0 0').split()])

    lidars = {}
    camera = None
    for link in model.findall('link'):
        for s in link.findall('sensor'):
            if s.get('type') == 'gpu_lidar':
                h = s.find('lidar/scan/horizontal')
                rng = s.find('lidar/range')
                lidars[s.get('name')] = dict(
                    lo=float(h.find('min_angle').text), hi=float(h.find('max_angle').text),
                    samples=int(h.find('samples').text), rmin=float(rng.find('min').text),
                    rmax=float(rng.find('max').text), rate=float(s.find('update_rate').text))
            elif s.get('type') == 'rgbd_camera':
                c = s.find('camera')
                camera = dict(hfov=float(c.find('horizontal_fov').text),
                              w=int(c.find('image/width').text), h=int(c.find('image/height').text),
                              near=float(c.find('depth_camera/clip/near').text),
                              far=float(c.find('depth_camera/clip/far').text))
    camera['vfov'] = 2 * math.atan(math.tan(camera['hfov'] / 2) * camera['h'] / camera['w'])

    radius = float(joints['base_footprint_joint'][0][2])
    return dict(joints=joints, lidars=lidars, camera=camera, radius=radius)


def model_parts(J):
    """(name, triangles in base_link) for the parts worth drawing."""
    def at(tris, xyz, rpy=(0, 0, 0), scale=1.0):
        return (tris * scale) @ rot(rpy).T + np.asarray(xyz)

    parts = [('deck', at(load_dae(os.path.join(MESH, 'chassis/chasis_base.dae')), (0, 0, 0.155))),
             ('plate', load_dae(os.path.join(MESH, 'chassis/deck_plate.dae')))]
    arc_xyz, _ = J['upper_chassis_joint']
    arc = at(load_dae(os.path.join(MESH, 'chassis/chasis.dae')),
             (-0.4354, 0.3925, 0), (math.pi / 2, 0, math.pi), 0.001)
    parts.append(('arc', arc + arc_xyz))
    wheel = load_dae(os.path.join(MESH, 'wheels/wheel.dae'))
    strut = load_dae(os.path.join(MESH, 'suspension/suspension.dae'))
    for w in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        sx, _ = J[f'{w}_steering_joint']
        tx, _ = J[f'{w}_wheel_joint']
        parts.append(('wheel', at(wheel, sx + tx, (math.pi / 2, 0, 0))))
        flip = math.pi if w.startswith('front') else 0.0
        parts.append(('strut', at(strut, sx + np.array([0, 0, 0.227]), (0, math.pi, flip))))
    zxyz, zrpy = J['zed2i_camera_joint']
    parts.append(('zed', at(load_stl(os.path.join(MESH, 'sensors/zed2i.stl')), zxyz, zrpy)))
    return parts


COLOURS = {'deck': '#3a3f47', 'plate': '#b9bec6', 'arc': '#2b2f36', 'wheel': '#16181c',
           'strut': '#9aa1ab', 'zed': BLUE}


def silhouette(ax, parts, axes, lift, order):
    """Every part's triangles projected onto one plane, drawn flat."""
    i, j = axes
    for name in order:
        tris = [t for n, t in parts if n == name]
        if not tris:
            continue
        t = np.vstack(tris)
        poly = np.stack([t[:, :, i], t[:, :, j] + (lift if j == 2 else 0.0)], axis=-1)
        ax.add_collection(PolyCollection(poly, facecolors=COLOURS[name], edgecolors='none',
                                         antialiased=False, zorder=4))


def sensor_marks(ax, M, top):
    J, r = M['joints'], M['radius']
    for joint, colour in (('lidar_joint', GREEN), ('lidar_rear_joint', ORANGE)):
        x, y, z = J[joint][0]
        if top:
            ax.add_patch(plt.Circle((x, y), 0.0375, fc=colour, ec='white', lw=1.0, zorder=7))
        else:
            ax.add_patch(Rectangle((x - 0.0375, z + r - 0.015), 0.075, 0.030, fc=colour,
                                   ec='white', lw=0.8, zorder=7))
            ax.add_patch(Rectangle((x - 0.035, z + r - 0.105), 0.07, 0.09, fc='#c9ccd1',
                                   ec='none', zorder=6))


# ================================================================ panels =====
def top_view(ax, M, parts):
    J, L, C = M['joints'], M['lidars'], M['camera']
    fx = J['lidar_joint'][0][0]
    rx = J['lidar_rear_joint'][0][0]
    zx = J['zed2i_camera_joint'][0][0]
    half_w = 0.4075
    X0, X1, Y0, Y1 = -4.3, 5.1, -2.9, 2.9

    # coverage, back to front: camera, LiDAR halves, the blind strip
    hf = math.degrees(C['hfov'] / 2)
    ax.add_patch(Wedge((zx, 0), 12, -hf, hf, fc=BLUE, alpha=0.07, ec='none', zorder=1))
    for s in (-1, 1):
        a = math.radians(hf) * s
        ax.plot([zx, zx + 12 * math.cos(a)], [0, 12 * math.sin(a)], color=BLUE, lw=1.0,
                ls=(0, (6, 4)), zorder=2)
    span_f = math.degrees(L['lidar']['hi'] - L['lidar']['lo'])
    span_r = math.degrees(L['lidar_rear']['hi'] - L['lidar_rear']['lo'])
    ax.add_patch(Wedge((fx, 0), L['lidar']['rmax'], -span_f / 2, span_f / 2, fc=GREEN,
                       alpha=0.13, ec='none', zorder=1))
    ax.add_patch(Wedge((rx, 0), L['lidar_rear']['rmax'], 180 - span_r / 2, 180 + span_r / 2,
                       fc=ORANGE, alpha=0.13, ec='none', zorder=1))
    for x, c in ((fx, GREEN), (rx, ORANGE)):
        ax.plot([x, x], [Y0, Y1], color=c, lw=1.2, ls=(0, (2, 3)), zorder=2)
    for s in (-1, 1):
        y_in, y_out = s * half_w, s * Y1
        ax.add_patch(Polygon([(rx, y_in), (fx, y_in), (fx, y_out), (rx, y_out)], closed=True,
                             fc='none', ec=BLIND, hatch='////', lw=0, zorder=2))

    silhouette(ax, parts, (0, 1), 0.0, ('deck', 'plate', 'strut', 'wheel', 'arc', 'zed'))
    sensor_marks(ax, M, top=True)

    # labels
    def tag(x, y, text, c, ha='center'):
        ax.text(x, y, text, color=c, fontsize=9.6, fontweight='bold', ha=ha, va='center',
                zorder=9, bbox=dict(boxstyle='round,pad=0.3', fc='white', ec=c, lw=0.9))
    tag(3.45, -2.25, f'front LiDAR  ·  {span_f:.0f}° ahead\n'
        f'{L["lidar"]["rmin"]:.2f} to {L["lidar"]["rmax"]:.0f} m  ·  {L["lidar"]["rate"]:.0f} Hz',
        GREEN)
    tag(-2.95, -2.25, f'rear LiDAR  ·  {span_r:.0f}° behind\nsame unit, turned round', ORANGE)
    tag(-0.04, 2.25, f'seen by neither LiDAR\nthe {fx - rx:.3f} m between them', BLIND)
    tag(3.70, 1.20, f'ZED 2i  ·  {2 * hf:.0f}° horizontal', BLUE)
    ax.annotate('', xy=(4.95, 0), xytext=(4.25, 0),
                arrowprops=dict(arrowstyle='-|>', color=INK, lw=1.4))
    ax.text(4.95, 0.16, 'forward', color=INK, fontsize=9, ha='right')

    ax.set_xlim(X0, X1)
    ax.set_ylim(Y0, Y1)
    ax.set_aspect('equal')
    ax.set_xlabel('x in base_link (m)', color=MUTED)
    ax.set_ylabel('y (m)', color=MUTED)
    ax.grid(color=GRID, lw=0.7, zorder=0)
    ax.set_axisbelow(True)


def side_view(ax, M, parts):
    J, C, r = M['joints'], M['camera'], M['radius']
    fx, _, lz = J['lidar_joint'][0]
    rx = J['lidar_rear_joint'][0][0]
    zx, _, zz = J['zed2i_camera_joint'][0]
    pitch = J['zed2i_camera_joint'][1][1]
    X0, X1, Z0, Z1 = -1.35, 3.25, -0.06, 1.95
    cam = np.array([zx, zz + r])
    plane = lz + r

    # camera frustum, clipped at the floor
    half = C['vfov'] / 2
    down = pitch + half                      # lower edge, below horizontal
    up = half - pitch                        # upper edge, above horizontal
    floor_x = cam[0] + cam[1] / math.tan(down)
    far = X1 + 1
    top_end = (far, cam[1] + (far - cam[0]) * math.tan(up))
    ax.add_patch(Polygon([tuple(cam), top_end, (far, 0), (floor_x, 0)], closed=True,
                         fc=BLUE, alpha=0.08, ec='none', zorder=1))
    ax.plot([cam[0], top_end[0]], [cam[1], top_end[1]], color=BLUE, lw=1.0, ls=(0, (6, 4)))
    ax.plot([cam[0], floor_x], [cam[1], 0], color=BLUE, lw=1.0, ls=(0, (6, 4)))
    axis_end = (cam[0] + 0.9 * math.cos(pitch), cam[1] - 0.9 * math.sin(pitch))
    ax.annotate('', xy=axis_end, xytext=tuple(cam),
                arrowprops=dict(arrowstyle='-|>', color=BLUE, lw=1.3))

    # scan planes
    ax.plot([fx, X1], [plane, plane], color=GREEN, lw=2.4, solid_capstyle='butt', zorder=3)
    ax.plot([X0, rx], [plane, plane], color=ORANGE, lw=2.4, solid_capstyle='butt', zorder=3)

    ax.axhline(0, color=INK, lw=1.2, zorder=5)
    silhouette(ax, parts, (0, 2), r, ('plate', 'deck', 'strut', 'wheel', 'arc', 'zed'))
    sensor_marks(ax, M, top=False)

    deck_front = max(t[:, :, 0].max() for n, t in parts if n == 'deck')
    overall = max(t[:, :, 2].max() for n, t in parts if n == 'arc') + r

    def note(x, y, text, c, ha='left', va='center'):
        ax.text(x, y, text, color=c, fontsize=9.4, ha=ha, va=va, zorder=9,
                bbox=dict(boxstyle='round,pad=0.28', fc='white', ec='none', alpha=0.85))
    note(1.62, plane + 0.05, f'both scan planes  {plane:.3f} m', INK, va='bottom')
    note(cam[0] + 0.25, cam[1] + 0.42,
         f'ZED 2i  {cam[1]:.3f} m up, {math.degrees(pitch):.0f}° down\n'
         f'{math.degrees(C["vfov"]):.1f}° vertical  ·  depth {C["near"]:.1f} to {C["far"]:.0f} m',
         BLUE)
    note(floor_x + 0.05, 0.10, f'first floor it sees:\n{floor_x - deck_front:.2f} m '
         f'beyond the deck', BLUE, va='bottom')
    ax.annotate('', xy=(X0 + 0.18, overall), xytext=(X0 + 0.18, 0),
                arrowprops=dict(arrowstyle='<|-|>', color=MUTED, lw=1.0))
    note(X0 + 0.26, overall - 0.10, f'{overall:.3f} m overall', MUTED, va='top')

    ax.set_xlim(X0, X1)
    ax.set_ylim(Z0, Z1)
    ax.set_aspect('equal')
    ax.set_xlabel('x in base_link (m)', color=MUTED)
    ax.set_ylabel('height above ground (m)', color=MUTED)
    ax.grid(color=GRID, lw=0.7, zorder=0)
    ax.set_axisbelow(True)
    return floor_x


def render(out):
    M = read_model()
    parts = model_parts(M['joints'])
    fig = plt.figure(figsize=(17.0, 6.2), dpi=120)
    fig.patch.set_facecolor('white')
    plt.rcParams.update({'xtick.color': MUTED, 'ytick.color': MUTED, 'axes.edgecolor': '#c9ccd1'})
    a1 = fig.add_axes([0.035, 0.07, 0.52, 0.75])
    a2 = fig.add_axes([0.600, 0.07, 0.39, 0.75])
    top_view(a1, M, parts)
    side_view(a2, M, parts)
    a1.set_title('Top view: what each sensor covers', loc='left', fontsize=12.5,
                 fontweight='bold', color=INK, pad=8)
    a2.set_title('Side view: scan planes and camera field of view', loc='left',
                 fontsize=12.5, fontweight='bold', color=INK, pad=8)
    fig.text(0.035, 0.955, 'TadeoeCar  ·  Sensor coverage', fontsize=19, fontweight='bold',
             color=INK)
    fig.text(0.035, 0.912, 'every mount, angle and range is read from tadeocar.urdf.xacro and '
             'model.sdf at render time; the robot is its own visual meshes',
             fontsize=10.5, style='italic', color=MUTED)
    fig.savefig(out, dpi=120, facecolor='white', bbox_inches='tight')
    print(f'wrote {out}')


if __name__ == '__main__':
    render(sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'images', 'sensor-coverage.png'))
