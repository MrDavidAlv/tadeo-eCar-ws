#!/usr/bin/env python3
"""Generate the aluminium tread plate that covers the deck, and its texture.

    python3 src/tadeocar_description/scripts/generate_deck_plate.py

Writes meshes/chassis/deck_plate.dae and meshes/chassis/tread_plate.png in
tadeocar_description, and copies both into tadeocar_gazebo's model, which
resolves its meshes from its own directory.

The outline is not typed in. It is the convex hull of the deck mesh's top face,
read from chasis_base.dae, so the plate follows the deck's rounded corners and
cannot overhang them. It is inset 5 mm so its edge sits on the rim rather than
on the radius.

The deck's top is a perimeter rim with two shallow end bays and an open centre
bay about 0.8 x 0.7 m. A 3 mm plate cannot span that under load - a strip of it
sags more than a centimetre under 100 kg spread over the deck - so the plate
carries two transverse ribs underneath, 40 x 20 mm aluminium tube resting on
the rim. They are part of the mesh so the model shows what has to be built.
"""

import math
import os
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
MESHES = os.path.join(PKG, 'meshes', 'chassis')
GZ_MESHES = os.path.join(SRC, 'tadeocar_gazebo', 'models', 'tadeocar_v1',
                         'meshes', 'chassis')

DECK_Z = 0.155          # chasis_base.dae visual origin above base_link
PLATE_T = 0.003         # 3 mm five-bar tread plate
INSET = 0.005
TILE = 0.10             # metres of plate per texture repeat
RIB_W, RIB_H = 0.040, 0.020
RIB_X = (-0.25, 0.15)   # across the open centre bay, measured from base_link


def deck_vertices():
    """Every vertex position in chasis_base.dae, in base_link."""
    import re
    text = open(os.path.join(MESHES, 'chasis_base.dae')).read()
    arrays = re.findall(r'<float_array[^>]*id="([^"]*)"[^>]*>([^<]*)</float_array>', text)
    pos = [np.array(a.split(), float).reshape(-1, 3)
           for name, a in arrays if 'position' in name.lower()]
    v = np.concatenate(pos)
    v[:, 2] += DECK_Z
    return v


def convex_hull(points):
    pts = sorted(map(tuple, points))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h
    lower, upper = half(pts), half(reversed(pts))
    return np.array(lower[:-1] + upper[:-1])


def inset(poly, d):
    """Move each edge of a counter-clockwise convex polygon inward by d."""
    n = len(poly)
    lines = []
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        t = (b - a) / np.linalg.norm(b - a)
        nrm = np.array([-t[1], t[0]])           # inward normal for CCW
        lines.append((a + nrm * d, t))
    out = []
    for i in range(n):
        (p1, t1), (p2, t2) = lines[i - 1], lines[i]
        m = np.array([t1, -t2]).T
        if abs(np.linalg.det(m)) < 1e-9:
            out.append(p2)
            continue
        s = np.linalg.solve(m, p2 - p1)
        out.append(p1 + t1 * s[0])
    return np.array(out)


def prism(poly, z0, z1):
    """Closed prism over a convex polygon: positions, normals, uvs, triangles."""
    P, N, UV, T = [], [], [], []
    def add(p, n):
        P.append(p); N.append(n); UV.append((p[0] / TILE, p[1] / TILE))
        return len(P) - 1
    c = poly.mean(0)
    for z, nz in ((z1, 1.0), (z0, -1.0)):
        ci = add((c[0], c[1], z), (0, 0, nz))
        ring = [add((p[0], p[1], z), (0, 0, nz)) for p in poly]
        for i in range(len(poly)):
            a, b = ring[i], ring[(i + 1) % len(poly)]
            T.append((ci, a, b) if nz > 0 else (ci, b, a))
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        e = b - a
        n = (e[1], -e[0], 0.0)
        n = tuple(np.array(n) / np.linalg.norm(n))
        q = [add((a[0], a[1], z0), n), add((b[0], b[1], z0), n),
             add((b[0], b[1], z1), n), add((a[0], a[1], z1), n)]
        T += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
    return P, N, UV, T


def box(cx, cy, cz, sx, sy, sz):
    poly = np.array([(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2),
                     (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)])
    return prism(poly, cz - sz / 2, cz + sz / 2)


def collada(parts):
    """parts: list of (name, material, (P, N, UV, T))."""
    geoms, nodes = [], []
    for name, mat, (P, N, UV, T) in parts:
        f = lambda a: ' '.join(f'{v:.6f}' for row in a for v in row)
        idx = ' '.join(f'{i} {i} {i}' for tri in T for i in tri)
        geoms.append(f'''
    <geometry id="{name}-mesh" name="{name}">
      <mesh>
        <source id="{name}-pos"><float_array id="{name}-pos-array" count="{3*len(P)}">{f(P)}</float_array>
          <technique_common><accessor source="#{name}-pos-array" count="{len(P)}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
        <source id="{name}-nrm"><float_array id="{name}-nrm-array" count="{3*len(N)}">{f(N)}</float_array>
          <technique_common><accessor source="#{name}-nrm-array" count="{len(N)}" stride="3">
            <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
        <source id="{name}-uv"><float_array id="{name}-uv-array" count="{2*len(UV)}">{f(UV)}</float_array>
          <technique_common><accessor source="#{name}-uv-array" count="{len(UV)}" stride="2">
            <param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
        <vertices id="{name}-vtx"><input semantic="POSITION" source="#{name}-pos"/></vertices>
        <triangles material="{mat}" count="{len(T)}">
          <input semantic="VERTEX" source="#{name}-vtx" offset="0"/>
          <input semantic="NORMAL" source="#{name}-nrm" offset="1"/>
          <input semantic="TEXCOORD" source="#{name}-uv" offset="2" set="0"/>
          <p>{idx}</p>
        </triangles>
      </mesh>
    </geometry>''')
        nodes.append(f'''
      <node id="{name}" name="{name}"><instance_geometry url="#{name}-mesh">
        <bind_material><technique_common><instance_material symbol="{mat}" target="#{mat}">
          <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
        </instance_material></technique_common></bind_material></instance_geometry></node>''')
    return f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="tread-img"><init_from>tread_plate.png</init_from></image></library_images>
  <library_effects>
    <effect id="tread-fx"><profile_COMMON>
      <newparam sid="tread-surf"><surface type="2D"><init_from>tread-img</init_from></surface></newparam>
      <newparam sid="tread-samp"><sampler2D><source>tread-surf</source></sampler2D></newparam>
      <technique sid="common"><phong>
        <diffuse><texture texture="tread-samp" texcoord="UVSET0"/></diffuse>
        <specular><color>0.6 0.6 0.6 1</color></specular><shininess><float>40</float></shininess>
      </phong></technique></profile_COMMON></effect>
    <effect id="rib-fx"><profile_COMMON><technique sid="common"><phong>
        <diffuse><color>0.55 0.56 0.58 1</color></diffuse></phong></technique></profile_COMMON></effect>
  </library_effects>
  <library_materials>
    <material id="tread" name="tread"><instance_effect url="#tread-fx"/></material>
    <material id="rib" name="rib"><instance_effect url="#rib-fx"/></material>
  </library_materials>
  <library_geometries>{''.join(geoms)}
  </library_geometries>
  <library_visual_scenes><visual_scene id="Scene" name="Scene">{''.join(nodes)}
  </visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''


def tread_texture(path, px=512):
    """Five-bar tread plate: raised bars alternating direction on brushed aluminium."""
    rng = np.random.default_rng(7)
    base = 168 + rng.normal(0, 6, (px, px)).cumsum(axis=1) * 0.02
    base += rng.normal(0, 4, (px, px))
    img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8)).convert('RGB')
    img = img.filter(ImageFilter.GaussianBlur(0.6))
    draw = ImageDraw.Draw(img)
    cell = px // 8
    for gy in range(8):
        for gx in range(8):
            cx, cy = (gx + 0.5) * cell, (gy + 0.5) * cell
            ang = math.radians(45 if (gx + gy) % 2 else -45)
            for k in range(-2, 3):
                ox, oy = -math.sin(ang) * k * cell * 0.14, math.cos(ang) * k * cell * 0.14
                L, W = cell * 0.30, cell * 0.045
                dx, dy = math.cos(ang) * L, math.sin(ang) * L
                nx, ny = -math.sin(ang) * W, math.cos(ang) * W
                poly = [(cx + ox - dx - nx, cy + oy - dy - ny), (cx + ox + dx - nx, cy + oy + dy - ny),
                        (cx + ox + dx + nx, cy + oy + dy + ny), (cx + ox - dx + nx, cy + oy - dy + ny)]
                draw.polygon([(x + 1.5, y + 1.5) for x, y in poly], fill=(118, 120, 124))
                draw.polygon(poly, fill=(214, 216, 220))
    img.save(path)


def main():
    v = deck_vertices()
    # The rim is not machined flat to the millimetre: its top face spans about
    # a centimetre of height across the deck. Everything within 10 mm of the
    # highest point is rim top, and its hull is the outline.
    top = v[v[:, 2] > v[:, 2].max() - 0.010][:, :2]
    hull = convex_hull(top)
    area = 0.5 * np.sum(hull[:, 0] * np.roll(hull[:, 1], -1) - np.roll(hull[:, 0], -1) * hull[:, 1])
    if area < 0:                                   # inset() expects counter-clockwise
        hull = hull[::-1]
    outline = inset(hull, INSET)
    z0 = v[:, 2].max()
    plate = prism(outline, z0, z0 + PLATE_T)

    # Ribs span the open centre bay from rim to rim, under the plate.
    y_span = 2 * (outline[:, 1].max() - 0.010)
    parts = [('deck_plate', 'tread', plate)]
    for i, x in enumerate(RIB_X):
        parts.append((f'deck_rib_{i}', 'rib',
                      box(x, 0.0, z0 - RIB_H / 2, RIB_W, y_span, RIB_H)))

    os.makedirs(MESHES, exist_ok=True)
    with open(os.path.join(MESHES, 'deck_plate.dae'), 'w') as f:
        f.write(collada(parts))
    tread_texture(os.path.join(MESHES, 'tread_plate.png'))
    for name in ('deck_plate.dae', 'tread_plate.png'):
        shutil.copy(os.path.join(MESHES, name), os.path.join(GZ_MESHES, name))

    x0, x1 = outline[:, 0].min(), outline[:, 0].max()
    y0, y1 = outline[:, 1].min(), outline[:, 1].max()
    print(f'plate: {len(outline)}-gon, x [{x0:.3f}, {x1:.3f}], y [{y0:.3f}, {y1:.3f}], '
          f'z [{z0:.4f}, {z0 + PLATE_T:.4f}] in base_link')
    print(f'ribs: {RIB_W*1000:.0f} x {RIB_H*1000:.0f} mm at x = {RIB_X}, {y_span:.3f} m long')


if __name__ == '__main__':
    main()
