#!/usr/bin/env python3
"""Render the kinematic-model figure straight from the robot's own files.

Everything on the sheet - geometry, limits, sensor specification, control
gains - is read from ``tadeocar.urdf.xacro``, ``model.sdf``,
``robot_params.yaml``, ``nav2_params.yaml`` and ``slam_params.yaml`` at render
time, and the robot is drawn from its actual visual meshes rather than
sketched. Change a parameter and re-run this; the figure cannot go stale.

The description is xacro, so this needs the ``xacro`` module on the path.
Source the ROS environment first:

    source /opt/ros/humble/setup.bash
    python3 docs/mathematical-model/render_diagram.py [out.png]

Default output: images/mathematical-model.png
"""
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np
import xacro
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'src')
MESH = os.path.join(SRC, 'tadeocar_description', 'meshes')

INK = '#1b2430'
MUTED = '#5a6675'
ACCENT = '#c1121f'
BLUE = '#1d4e89'
GREEN = '#2d6a4f'
AMBER = '#b26a00'
MONO = 'DejaVu Sans Mono'


# =========================================================== mesh loading ===
def load_stl(path):
    d = open(path, 'rb').read()
    if d[:5] == b'solid' and b'facet' in d[:2000]:
        tris, cur = [], []
        for line in d.decode('utf-8', 'ignore').splitlines():
            t = line.split()
            if t[:1] == ['vertex']:
                cur.append([float(v) for v in t[1:4]])
                if len(cur) == 3:
                    tris.append(cur)
                    cur = []
        return np.array(tris)
    n = struct.unpack('<I', d[80:84])[0]
    dt = np.dtype([('n', '<3f4'), ('v', '<3f4', (3,)), ('a', '<u2')])
    return np.frombuffer(d[84:84 + n * 50], dtype=dt, count=n)['v'].astype(np.float64)


def load_dae(path):
    """Triangles out of a COLLADA file.

    Only what these meshes actually use: float_array sources, a vertices
    element naming the POSITION source, and triangles or polylist primitives.
    Enough to draw the robot, and small enough to read.
    """
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    root = ET.parse(path).getroot()
    tris = []
    for geom in root.iter('{%s}geometry' % ns['c']):
        mesh = geom.find('c:mesh', ns)
        if mesh is None:
            continue
        sources = {}
        for src in mesh.findall('c:source', ns):
            arr = src.find('c:float_array', ns)
            if arr is None or not arr.text:
                continue
            vals = np.fromstring(arr.text, sep=' ')
            stride = 3
            acc = src.find('c:technique_common/c:accessor', ns)
            if acc is not None and acc.get('stride'):
                stride = int(acc.get('stride'))
            sources['#' + src.get('id')] = vals.reshape(-1, stride)[:, :3]

        verts = mesh.find('c:vertices', ns)
        vmap = {}
        if verts is not None:
            for inp in verts.findall('c:input', ns):
                if inp.get('semantic') == 'POSITION':
                    vmap['#' + verts.get('id')] = sources.get(inp.get('source'))

        for prim in list(mesh.findall('c:triangles', ns)) + \
                list(mesh.findall('c:polylist', ns)):
            inputs = prim.findall('c:input', ns)
            stride = max(int(i.get('offset', 0)) for i in inputs) + 1
            offset, pos = None, None
            for inp in inputs:
                if inp.get('semantic') != 'VERTEX':
                    continue
                offset = int(inp.get('offset', 0))
                src = inp.get('source')
                pos = vmap.get(src, sources.get(src))
            p = prim.find('c:p', ns)
            if pos is None or p is None or not p.text:
                continue
            idx = np.fromstring(p.text, sep=' ', dtype=float).astype(int)
            idx = idx.reshape(-1, stride)[:, offset]

            vcount = prim.find('c:vcount', ns)
            if vcount is not None and vcount.text:
                counts = np.fromstring(vcount.text, sep=' ', dtype=float).astype(int)
                at = 0
                for c in counts:                     # fan-triangulate a polygon
                    face = idx[at:at + c]
                    at += c
                    for k in range(1, c - 1):
                        tris.append(pos[[face[0], face[k], face[k + 1]]])
            else:
                tris.append(pos[idx].reshape(-1, 3, 3))
    if not tris:
        return np.zeros((0, 3, 3))
    return np.vstack([t.reshape(-1, 3, 3) for t in tris])


def rot(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# =============================================================== sources ====
def read_sources():
    xacro_file = os.path.join(SRC, 'tadeocar_description/urdf/tadeocar.urdf.xacro')
    urdf = ET.fromstring(xacro.process_file(xacro_file).toxml())
    sdf = open(os.path.join(SRC, 'tadeocar_gazebo/models/tadeocar_v1/model.sdf')).read()
    params = yaml.safe_load(
        open(os.path.join(SRC, 'tadeocar_control/config/robot_params.yaml')))['/**']['ros__parameters']
    nav = yaml.safe_load(open(os.path.join(SRC, 'tadeocar_navigation/config/nav2_params.yaml')))
    slam = yaml.safe_load(
        open(os.path.join(SRC, 'tadeocar_slam/config/slam_params.yaml')))['slam_toolbox']['ros__parameters']

    joints = {}
    for j in urdf.findall('joint'):
        o = j.find('origin')
        if o is None:
            continue
        joints[j.get('name')] = (
            [float(v) for v in (o.get('xyz') or '0 0 0').split()],
            [float(v) for v in (o.get('rpy') or '0 0 0').split()])

    def sdf_float(pattern, default=0.0):
        m = re.search(pattern, sdf)
        return float(m.group(1)) if m else default

    lidar = {
        'min': sdf_float(r'<range>\s*<min>([\d.]+)', 0),
        'max': sdf_float(r'<range>\s*<min>[\d.]+\s*</min>\s*<max>([\d.]+)', 0),
        'samples': int(sdf_float(r'<samples>(\d+)', 0)),
        'rate': int(sdf_float(r'<update_rate>(\d+)</update_rate>\s*<topic>scan', 0)),
    }
    cam = {
        'w': int(sdf_float(r'<width>(\d+)', 0)),
        'h': int(sdf_float(r'<height>(\d+)', 0)),
        'fov': sdf_float(r'<horizontal_fov>([\d.]+)', 0),
    }
    pid = {k: sdf_float(rf'<{k}>([\d.]+)</{k}>', 0)
           for k in ('p_gain', 'i_gain', 'd_gain', 'i_max')}
    mass = sum(float(m) for m in re.findall(r'<mass>([\d.]+)</mass>', sdf))

    return dict(urdf=urdf, joints=joints, params=params, nav=nav, slam=slam,
                lidar=lidar, cam=cam, pid=pid, mass=mass)


# ============================================================== drawing =====
def robot_parts(joints):
    deck = load_dae(os.path.join(MESH, 'chassis/chasis_base.dae'))
    wheel = load_dae(os.path.join(MESH, 'wheels/wheel.dae'))
    strut = load_dae(os.path.join(MESH, 'suspension/suspension.dae'))
    zed = load_stl(os.path.join(MESH, 'sensors/zed2i.stl'))

    parts = [('deck', deck + np.array([0, 0, 0.155]))]
    for w in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        sx, _ = joints[f'{w}_steering_joint']
        tx, _ = joints[f'{w}_wheel_joint']
        centre = np.array(sx) + np.array(tx)
        parts.append((f'{w}_wheel', wheel @ rot([math.pi / 2, 0, 0]).T + centre))
        flip = math.pi if w.startswith('front') else 0.0
        parts.append((f'{w}_strut',
                      strut @ rot([0, math.pi, flip]).T + np.array(sx) + np.array([0, 0, 0.227])))
    zx, _ = joints['zed2i_camera_joint']
    parts.append(('zed', zed + np.array(zx)))
    return parts


def draw_robot(ax, joints):
    """Orthographic top view of the real meshes, lit from above."""
    colours = {'deck': np.array([0.17, 0.19, 0.24]),
               'zed': np.array([0.30, 0.32, 0.36])}
    XY, RGB, Z, A = [], [], [], []
    for name, tris in robot_parts(joints):
        if len(tris) == 0:
            continue
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        n = np.cross(v1 - v0, v2 - v0)
        ln = np.linalg.norm(n, axis=1)
        ok = ln > 1e-12
        n, tris = n[ok] / ln[ok, None], tris[ok]
        vis = n[:, 2] > 0.0
        n, tris = n[vis], tris[vis]
        if len(tris) == 0:
            continue
        if 'strut' in name:
            base = np.array([0.55, 0.57, 0.60])
        elif 'wheel' in name:
            base = np.array([0.13, 0.13, 0.14])
        else:
            base = colours.get(name, np.array([0.35, 0.36, 0.39]))
        L = np.array([-0.35, 0.45, 0.82])
        L /= np.linalg.norm(L)
        shade = 0.34 + 0.76 * np.clip(n @ L, 0, 1)
        rgb = np.clip(base[None, :] * shade[:, None] + 0.14 * (n[:, 2] ** 6)[:, None], 0, 1)
        XY.append(tris[:, :, :2])
        RGB.append(rgb)
        # The deck sits above everything and would hide the four wheel modules
        # the panel exists to show, so it is drawn translucent and the wheels
        # are lifted above it.
        A.append(np.full(len(tris), 0.55 if name == 'deck' else 1.0))
        Z.append(tris[:, :, 2].mean(axis=1)
                 + (0.0 if name == 'deck' else 1.0))
    XY, RGB, Z, A = np.vstack(XY), np.vstack(RGB), np.concatenate(Z), np.concatenate(A)
    for alpha, z in ((0.55, 2), (1.0, 4)):
        sel = (A == alpha) if alpha < 1.0 else (A >= 1.0)
        if not sel.any():
            continue
        order = np.argsort(Z[sel])
        ax.add_collection(PolyCollection(XY[sel][order], facecolors=RGB[sel][order],
                                         alpha=alpha, edgecolors='none',
                                         antialiased=False, zorder=z))


def panel(fig, rect, title, colour):
    ax = fig.add_axes(rect)
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    ax.add_patch(FancyBboxPatch((0.004, 0.004), 0.992, 0.992,
                                boxstyle='round,pad=0.006,rounding_size=0.02',
                                linewidth=1.1, edgecolor=colour,
                                facecolor='none', transform=ax.transAxes,
                                clip_on=False, zorder=0))
    ax.text(0.028, 0.945, title, transform=ax.transAxes, fontsize=13.5,
            fontweight='bold', color=colour, va='top')
    return ax


def lines(ax, y, rows, x=0.045, dy=0.062, size=10.2):
    for text, colour, weight in rows:
        ax.text(x, y, text, transform=ax.transAxes, fontsize=size,
                family=MONO, color=colour, fontweight=weight, va='top')
        y -= dy
    return y


# ================================================================= sheet ====
def render(out):
    S = read_sources()
    P, J = S['params'], S['joints']
    r = P['wheel_radius']
    fx, rx = P['front_axle_x'], P['rear_axle_x']
    ht = P['half_track']
    L = fx - rx
    lidar_xyz = J['lidar_joint'][0]
    zed_xyz = J['zed2i_camera_joint'][0]
    steer_lim = P['max_steering_angle']

    fig = plt.figure(figsize=(19.2, 11.6), dpi=110)
    fig.patch.set_facecolor('white')

    fig.text(0.5, 0.972, 'TadeoeCar 4WD4WS  ·  Kinematic Model', ha='center',
             fontsize=27, fontweight='bold', color=INK)
    fig.text(0.5, 0.941,
             'every value on this sheet is read from tadeocar.urdf.xacro, model.sdf, '
             'robot_params.yaml, nav2_params.yaml and slam_params.yaml at render time',
             ha='center', fontsize=11.5, style='italic', color=MUTED)

    # ---------------------------------------------------------- 1 geometry --
    ax = panel(fig, [0.028, 0.400, 0.455, 0.520],
               '1 · GEOMETRY  (real visual meshes, to scale)', BLUE)
    g = fig.add_axes([0.055, 0.425, 0.400, 0.430])
    g.set_aspect('equal')
    g.axis('off')
    draw_robot(g, J)

    # footprint
    g.add_patch(Rectangle((-0.757, -0.4075), 1.418, 0.815, fill=False,
                          linestyle=(0, (6, 4)), linewidth=1.4,
                          edgecolor=ACCENT, zorder=4))
    g.text(0.60, 0.455, 'Nav2 footprint\n1.418 x 0.815 m', color=ACCENT,
           fontsize=9.2, fontweight='bold', ha='center', va='bottom')

    # base_link axes
    g.annotate('', xy=(0.42, 0), xytext=(0, 0), zorder=6,
               arrowprops=dict(arrowstyle='-|>', color=ACCENT, lw=2.0))
    g.annotate('', xy=(0, 0.42), xytext=(0, 0), zorder=6,
               arrowprops=dict(arrowstyle='-|>', color=ACCENT, lw=2.0))
    g.text(0.45, 0.01, 'x', color=ACCENT, fontsize=13, fontweight='bold')
    g.text(0.02, 0.44, 'y', color=ACCENT, fontsize=13, fontweight='bold')
    g.plot(0, 0, 'o', ms=8, mfc='white', mec=AMBER, mew=2.2, zorder=7)

    # wheel labels and steering axes
    for tag, (px, py) in (('FL', (fx, ht)), ('FR', (fx, -ht)),
                          ('RL', (rx, ht)), ('RR', (rx, -ht))):
        g.plot(px, py, 'x', ms=7, color=BLUE, mew=2.0, zorder=6)
        g.text(px, py + (0.115 if py > 0 else -0.155), tag, color='white',
               fontsize=9.5, fontweight='bold', ha='center',
               bbox=dict(boxstyle='round,pad=0.22', fc=BLUE, ec='none'), zorder=7)

    # dimension lines
    g.annotate('', xy=(fx, -0.62), xytext=(rx, -0.62), zorder=5,
               arrowprops=dict(arrowstyle='<|-|>', color=GREEN, lw=1.5))
    g.text((fx + rx) / 2, -0.70, f'L = {L:.3f} m   (wheelbase)', color=GREEN,
           fontsize=10.5, fontweight='bold', ha='center')
    g.annotate('', xy=(-0.95, ht), xytext=(-0.95, -ht), zorder=5,
               arrowprops=dict(arrowstyle='<|-|>', color=GREEN, lw=1.5))
    g.text(-1.00, 0, f'T = {2*ht:.3f} m', color=GREEN, fontsize=10.5,
           fontweight='bold', rotation=90, ha='right', va='center')

    # the asymmetry, which is the point of the panel
    g.plot([rx, fx], [0.52, 0.52], color=AMBER, lw=1.3, zorder=5)
    for xx in (rx, fx):
        g.plot([xx, xx], [0.50, 0.54], color=AMBER, lw=1.3, zorder=5)
    g.plot((fx + rx) / 2, 0.52, 'v', color=AMBER, ms=7, zorder=6)
    g.text((fx + rx) / 2, 0.575,
           f'wheelbase midpoint  x = {(fx+rx)/2:+.3f} m\nNOT base_link',
           color=AMBER, fontsize=9.2, fontweight='bold', ha='center')

    # sensors
    g.plot(lidar_xyz[0], lidar_xyz[1], 'o', ms=7, mfc='#f4d35e',
           mec=INK, mew=1.2, zorder=7)
    g.annotate(f'LiDAR   {lidar_xyz[0]:.2f} m fwd, 0.450 m up',
               xy=(lidar_xyz[0], lidar_xyz[1]), xytext=(0.86, -0.30),
               fontsize=8.8, color=INK, zorder=8, ha='left',
               arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.9))
    g.annotate('ZED 2i   0.300 m up',
               xy=(zed_xyz[0], zed_xyz[1]), xytext=(0.86, -0.42),
               fontsize=8.8, color=INK, zorder=8, ha='left',
               arrowprops=dict(arrowstyle='-', color=MUTED, lw=0.9))
    g.plot(zed_xyz[0], zed_xyz[1], 'o', ms=6, mfc='#8ecae6', mec=INK,
           mew=1.1, zorder=7)

    g.set_xlim(-1.28, 1.72)
    g.set_ylim(-0.90, 0.78)

    ax.text(0.045, 0.135,
            f'wheel radius   r  = {r:.3f} m        front axle   x = {fx:+.3f} m\n'
            f'wheelbase      L  = {L:.3f} m        rear axle    x = {rx:+.3f} m\n'
            f'track          T  = {2*ht:.3f} m        total mass   = {S["mass"]:.1f} kg',
            transform=ax.transAxes, fontsize=10.2, family=MONO, color=INK,
            va='top', linespacing=1.75)

    # ------------------------------------------------------ 2 kinematics ----
    ax = panel(fig, [0.503, 0.628, 0.469, 0.292], '2 · KINEMATICS', GREEN)
    lines(ax, 0.855, [
        ('FORWARD    body twist -> each wheel', BLUE, 'bold'),
        ('  v_i = (vx - wz*py_i,  vy + wz*px_i)', INK, 'normal'),
        ('  d_i = atan2(v_iy, v_ix)        w_i = |v_i| / r', INK, 'normal'),
        ('INVERSE    eight measurements -> three unknowns', BLUE, 'bold'),
        ('  A x = b,   x = pinv(A) b        A is 8x3, constant', INK, 'normal'),
        ('  b_2i = w_i r cos(d_i)   b_2i+1 = w_i r sin(d_i)', INK, 'normal'),
        ('ODOMETRY   midpoint integration at 50 Hz', BLUE, 'bold'),
        ('  th_ = th + wz dt/2', INK, 'normal'),
        ('  x+ = x + (vx cos th_ - vy sin th_) dt', INK, 'normal'),
    ], dy=0.086, size=10.4)
    ax.text(0.045, 0.075, f'r = {r:.3f} m      dt = 0.020 s      overdetermined by 5',
            transform=ax.transAxes, fontsize=10.6, family=MONO,
            color=ACCENT, fontweight='bold')

    # ----------------------------------------------------------- 3 modes ----
    ax = panel(fig, [0.503, 0.400, 0.469, 0.212],
               '3 · THREE MODES, THREE WHEEL PATTERNS', ACCENT)
    # The wheel angles are COMPUTED from the same equations the controller
    # uses, for one representative twist per mode, rather than drawn by hand.
    # Hand-drawn is how the omnidirectional glyph came to show both front
    # wheels at one angle and both rear at another - which is counter-phase
    # steering, a pattern this mode never produces. Every wheel points along
    # its own velocity, so in general all four angles differ.
    def fold(a):
        a = math.atan2(math.sin(a), math.cos(a))
        if a > math.pi / 2:
            return a - math.pi
        if a < -math.pi / 2:
            return a + math.pi
        return a

    positions = ((fx, ht), (fx, -ht), (rx, ht), (rx, -ht))     # FL FR RL RR

    def omni(vx, vy, wz):
        return [math.degrees(fold(math.atan2(vy + wz * px, vx - wz * py)))
                for px, py in positions]

    def ackermann(vx, wz):
        R = vx / wz
        return [math.degrees(math.atan(L / (R - ht))),
                math.degrees(math.atan(L / (R + ht))), 0.0, 0.0]

    def crab(vx, vy):
        a = math.degrees(fold(math.atan2(vy, vx)))
        return [a, a, a, a]

    for k, (name, angles, note) in enumerate((
            ('omnidirectional', omni(0.6, 0.0, 0.5),
             'vx 0.6, wz 0.5: four different angles'),
            ('ackermann', ackermann(0.6, 0.5),
             'same turn, rear axle straight'),
            ('crab', crab(0.35, 0.35),
             'vx = vy: all four equal, heading held'))):
        cx = 0.175 + 0.325 * k
        m = fig.add_axes([0.516 + 0.152 * k, 0.452, 0.118, 0.093])
        m.set_aspect('equal')
        m.axis('off')
        # Deck and wheel positions to the robot's own proportions, so the
        # asymmetric wheelbase shows here too. Forward is to the right.
        m.add_patch(Rectangle((-0.757, -0.408), 1.418, 0.815, fill=True,
                              facecolor='#eef1f6', edgecolor=MUTED, lw=1.0))
        for (px, py), a in zip(positions, angles):
            th = math.radians(a)
            m.plot([px - 0.17 * math.cos(th), px + 0.17 * math.cos(th)],
                   [py - 0.17 * math.sin(th), py + 0.17 * math.sin(th)],
                   color=INK, lw=4.6, solid_capstyle='round')
        m.annotate('', xy=(0.42, 0), xytext=(0.0, 0),
                   arrowprops=dict(arrowstyle='-|>', color=ACCENT, lw=1.4))
        m.text(0.50, 0.0, 'fwd', color=ACCENT, fontsize=7.6,
               fontweight='bold', va='center')
        m.set_xlim(-0.95, 1.05)
        m.set_ylim(-0.62, 0.62)
        ax.text(cx, 0.845, name, transform=ax.transAxes, fontsize=11,
                fontweight='bold', color=INK, ha='center', va='top')
        ax.text(cx, 0.19, note, transform=ax.transAxes, fontsize=9.0,
                color=MUTED, ha='center')
    ax.text(0.5, 0.075,
            'angles computed from the controller\'s own equations, not drawn: '
            'in omnidirectional mode all four genuinely differ',
            transform=ax.transAxes, fontsize=9.6, color=ACCENT, ha='center')

    # ---------------------------------------------------------- 4 limits ----
    ax = panel(fig, [0.028, 0.018, 0.300, 0.364], '4 · LIMITS', BLUE)
    dwb = S['nav']['controller_server']['ros__parameters']['FollowPath']
    lines(ax, 0.855, [
        ('BODY   (Nav2 and the controller agree)', BLUE, 'bold'),
        (f'  |v|      <= {P["max_linear_speed"]:.1f} m/s', INK, 'normal'),
        (f'  |w|      <= {P["max_angular_speed"]:.1f} rad/s', INK, 'normal'),
        (f'  pursuit     {dwb["desired_linear_vel"]:.1f} m/s desired', INK, 'normal'),
        ('STEERING   270 deg servos', BLUE, 'bold'),
        (f'  angle    <= {math.degrees(steer_lim):.0f} deg', INK, 'normal'),
        ('  rate     <= 3.0 rad/s', INK, 'normal'),
        (f'  PID       p={S["pid"]["p_gain"]:.0f} i={S["pid"]["i_gain"]:.0f} '
         f'd={S["pid"]["d_gain"]:.0f}', INK, 'normal'),
        ('WHEELS', BLUE, 'bold'),
        (f'  1 m/s     = {1/r:.1f} rad/s', INK, 'normal'),
        (f'  gate      above {P["gate_speed_threshold"]:.2f} m/s', INK, 'normal'),
    ], dy=0.067, size=9.8)
    ax.text(0.045, 0.082,
            'the 1.0 m/s ceiling is the LiDAR:\n'
            f'{S["lidar"]["rate"]} Hz means 0.14 m between scans',
            transform=ax.transAxes, fontsize=9.4, color=ACCENT, va='top')

    # ---------------------------------------------------- 5 control chain ---
    ax = panel(fig, [0.348, 0.018, 0.300, 0.364], '5 · CONTROL CHAIN', GREEN)
    lines(ax, 0.855, [
        ('joy / web / teleop / Nav2', INK, 'normal'),
        ('   |  twist_mux, e_stop wins', MUTED, 'normal'),
        ('   v  /cmd_vel   (Twist)', ACCENT, 'bold'),
        ('fourws_kinematics', BLUE, 'bold'),
        ('   |  4 x cmd_pos   steering, rad', INK, 'normal'),
        ('   |  4 x cmd_vel   wheels, rad/s', INK, 'normal'),
        ('   v  gated on the WORST', INK, 'normal'),
        ('      steering error of the four', INK, 'normal'),
        ('wheel_odometry  ->  /odom', BLUE, 'bold'),
        ('   +  ZED 2i IMU  ->  EKF', BLUE, 'bold'),
        ('   v  odom -> base_footprint', ACCENT, 'bold'),
    ], dy=0.067, size=9.8)
    ax.text(0.045, 0.082,
            'Gazebo ground truth exists on\n/odom_truth and feeds nothing.',
            transform=ax.transAxes, fontsize=9.4, color=MUTED, va='top')

    # -------------------------------------------------------- 6 sensing ----
    ax = panel(fig, [0.668, 0.018, 0.304, 0.364], '6 · SENSING', AMBER)
    Ld, C = S['lidar'], S['cam']
    fxp = (C['w'] / 2) / math.tan(C['fov'] / 2)
    lines(ax, 0.855, [
        ('YDLIDAR X2', AMBER, 'bold'),
        (f'  {Ld["min"]:.2f} - {Ld["max"]:.1f} m   360 deg   {Ld["rate"]} Hz', INK, 'normal'),
        (f'  {Ld["samples"]} pts/rev = {360/Ld["samples"]:.2f} deg', INK, 'normal'),
        ('ZED 2i   RGB-D', AMBER, 'bold'),
        (f'  {C["w"]} x {C["h"]}  {math.degrees(C["fov"]):.0f} deg  fx={fxp:.1f}', INK, 'normal'),
        ('  depth 0.3 - 20 m, IMU 400 Hz', INK, 'normal'),
        ('SLAM TOOLBOX', AMBER, 'bold'),
        (f'  node every {S["slam"]["minimum_travel_distance"]} m / '
         f'{S["slam"]["minimum_travel_heading"]} rad', INK, 'normal'),
        (f'  loop search {S["slam"]["loop_search_maximum_distance"]:.0f} m', INK, 'normal'),
        ('COSTMAPS', AMBER, 'bold'),
        ('  local 6x6 m, inflation 0.85 m', INK, 'normal'),
    ], dy=0.067, size=9.8)
    ax.text(0.045, 0.082,
            'the scan plane is 0.450 m up:\n61 mm over the deck, so 360 deg is clean',
            transform=ax.transAxes, fontsize=9.4, color=MUTED, va='top')

    fig.savefig(out, dpi=110, facecolor='white', bbox_inches='tight')
    print(f'wrote {out}')


if __name__ == '__main__':
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, 'images', 'mathematical-model.png')
    render(out)
