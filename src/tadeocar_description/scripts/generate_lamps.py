#!/usr/bin/env python3
"""Generate the lamp meshes and lens textures.

    python3 src/tadeocar_description/scripts/generate_lamps.py

Writes meshes/lamps/*.dae and *.png in tadeocar_description and copies them
into tadeocar_gazebo's model.

Every lamp is two separate meshes: a housing, black and chamfered at the front
edge, and a lens that sits 1 mm proud of it. They are separate so that each
lens can be its own visual, lit or dark on its own, when the lamps are driven
from the controller's state.

Each mesh has its origin at the centre of the housing's front face, with +x
pointing where the light goes. A rear lamp is placed with a half turn about z.

Sizes are those of real sealed LED units of the kind bolted to trailers and
utility vehicles:

    headlamp       90 x 40 mm face, 45 mm deep, two projector lenses
    turn lamp      55 x 32 mm face, 30 mm deep, amber
    tail combo    150 x 40 mm face, 32 mm deep, three lenses across the
                   140 mm inside a 5 mm border, in the proportions of
                   TAIL_LENSES: reversing (white, 32.7 mm), stop and
                   position (red, 65.3 mm), direction (amber, 37.3 mm),
                   with 2.3 mm black dividers
"""

import math
import os
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.dirname(PKG)
OUT = os.path.join(PKG, 'meshes', 'lamps')
GZ_OUT = os.path.join(SRC, 'tadeocar_gazebo', 'models', 'tadeocar_v1', 'meshes', 'lamps')

TEX = 256


# ------------------------------------------------------------------ geometry

def rounded_rect(w, h, r, seg=6):
    """Counter-clockwise outline in the (y, z) plane, centred on the origin."""
    r = min(r, w / 2, h / 2)
    pts = []
    for cy, cz, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90),
                       (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for k in range(seg + 1):
            a = math.radians(a0 + 90 * k / seg)
            pts.append((cy + r * math.cos(a), cz + r * math.sin(a)))
    return np.array(pts)


class Mesh:
    def __init__(self):
        self.P, self.N, self.UV, self.T = [], [], [], []

    def add(self, p, n, uv=(0.0, 0.0)):
        self.P.append(tuple(p)); self.N.append(tuple(n)); self.UV.append(tuple(uv))
        return len(self.P) - 1

    def cap(self, ring_yz, x, facing, uv_box=None):
        """Flat polygon at depth x, normal +-x, optional UVs over its box."""
        n = (facing, 0.0, 0.0)
        c = ring_yz.mean(0)
        def uv(y, z):
            if uv_box is None:
                return (0.0, 0.0)
            y0, y1, z0, z1 = uv_box
            u = (y - y0) / (y1 - y0)
            return (1 - u if facing > 0 else u, (z - z0) / (z1 - z0))
        ci = self.add((x, c[0], c[1]), n, uv(*c))
        idx = [self.add((x, y, z), n, uv(y, z)) for y, z in ring_yz]
        for i in range(len(idx)):
            a, b = idx[i], idx[(i + 1) % len(idx)]
            self.T.append((ci, a, b) if facing > 0 else (ci, b, a))

    def band(self, ring_a, xa, ring_b, xb):
        """Side wall between two outlines at depths xa < xb."""
        for i in range(len(ring_a)):
            j = (i + 1) % len(ring_a)
            q = [np.array((xa, *ring_a[i])), np.array((xa, *ring_a[j])),
                 np.array((xb, *ring_b[j])), np.array((xb, *ring_b[i]))]
            nrm = np.cross(q[1] - q[0], q[3] - q[0])
            if np.linalg.norm(nrm) < 1e-12:
                nrm = np.cross(q[2] - q[1], q[3] - q[1])
            nrm = nrm / np.linalg.norm(nrm)
            k = [self.add(p, nrm) for p in q]
            self.T += [(k[0], k[1], k[2]), (k[0], k[2], k[3])]


def housing(w, h, depth, r, chamfer):
    """Black body: back face at -depth, front face at 0, front edge chamfered."""
    m = Mesh()
    full = rounded_rect(w, h, r)
    inset = rounded_rect(w - 2 * chamfer, h - 2 * chamfer, max(r - chamfer, 0.001))
    m.cap(full, -depth, -1.0)
    m.band(full, -depth, full, -chamfer)
    m.band(full, -chamfer, inset, 0.0)
    m.cap(inset, 0.0, +1.0)
    return m


def lens(w, h, r, thick=0.002, y_off=0.0):
    """Lens plate 1 mm proud of the housing face, UV-mapped over its face."""
    m = Mesh()
    ring = rounded_rect(w, h, r) + np.array([y_off, 0.0])
    box = (ring[:, 0].min(), ring[:, 0].max(), ring[:, 1].min(), ring[:, 1].max())
    m.cap(ring, 0.001, -1.0)
    m.band(ring, 0.001, ring, 0.001 + thick)
    m.cap(ring, 0.001 + thick, +1.0, uv_box=box)
    return m


def write_dae(path, mesh, color, texture=None, emission=None, shininess=40):
    f = lambda a: ' '.join(f'{v:.6f}' for row in a for v in row)
    n = len(mesh.P)
    idx = ' '.join(f'{i} {i} {i}' for tri in mesh.T for i in tri)
    tex_lib = ''
    diffuse = f'<color>{color[0]} {color[1]} {color[2]} 1</color>'
    params = ''
    if texture:
        tex_lib = f'<library_images><image id="img"><init_from>{texture}</init_from></image></library_images>'
        params = ('<newparam sid="surf"><surface type="2D"><init_from>img</init_from></surface></newparam>'
                  '<newparam sid="samp"><sampler2D><source>surf</source></sampler2D></newparam>')
        diffuse = '<texture texture="samp" texcoord="UVSET0"/>'
    emis = f'<emission><color>{emission[0]} {emission[1]} {emission[2]} 1</color></emission>' if emission else ''
    xml = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  {tex_lib}
  <library_effects><effect id="fx"><profile_COMMON>{params}<technique sid="common"><phong>
    {emis}<diffuse>{diffuse}</diffuse>
    <specular><color>0.5 0.5 0.5 1</color></specular><shininess><float>{shininess}</float></shininess>
  </phong></technique></profile_COMMON></effect></library_effects>
  <library_materials><material id="mat" name="mat"><instance_effect url="#fx"/></material></library_materials>
  <library_geometries><geometry id="g" name="g"><mesh>
    <source id="pos"><float_array id="pos-a" count="{3*n}">{f(mesh.P)}</float_array>
      <technique_common><accessor source="#pos-a" count="{n}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="nrm"><float_array id="nrm-a" count="{3*n}">{f(mesh.N)}</float_array>
      <technique_common><accessor source="#nrm-a" count="{n}" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
    <source id="uv"><float_array id="uv-a" count="{2*n}">{f(mesh.UV)}</float_array>
      <technique_common><accessor source="#uv-a" count="{n}" stride="2"><param name="S" type="float"/><param name="T" type="float"/></accessor></technique_common></source>
    <vertices id="vtx"><input semantic="POSITION" source="#pos"/></vertices>
    <triangles material="mat" count="{len(mesh.T)}">
      <input semantic="VERTEX" source="#vtx" offset="0"/><input semantic="NORMAL" source="#nrm" offset="1"/>
      <input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>{idx}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="S"><node id="n"><instance_geometry url="#g">
    <bind_material><technique_common><instance_material symbol="mat" target="#mat">
      <bind_vertex_input semantic="UVSET0" input_semantic="TEXCOORD" input_set="0"/>
    </instance_material></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#S"/></scene>
</COLLADA>
'''
    open(path, 'w').write(xml)


# ------------------------------------------------------------------ textures

def _canvas(base, w, h):
    return Image.new('RGB', (w, h), base)


def tex_headlamp(lit, W, H):
    """Two projector modules in chrome bowls, under a light-guide brow."""
    img = _canvas((14, 14, 16), W, H); d = ImageDraw.Draw(img)
    R = H * 0.30
    for cx in (W * 0.27, W * 0.73):
        cy = H * 0.58
        for k in range(14, 0, -1):                          # chrome bowl
            g = int(60 + 150 * (1 - k / 14) ** 2)
            rr = R * 1.45 * k / 14
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], fill=(g, g, g + 6))
        lens_c = (255, 255, 250) if lit else (70, 76, 86)
        d.ellipse([cx - R * 0.72, cy - R * 0.72, cx + R * 0.72, cy + R * 0.72], fill=lens_c)
    d.rectangle([W * 0.06, H * 0.08, W * 0.94, H * 0.17], fill=(235, 240, 255) if lit else (40, 42, 46))
    return img.filter(ImageFilter.GaussianBlur(1.2 if lit else 0.6))


def _led_grid(color_off, color_on, lit, rows, cols, base, W, H):
    img = _canvas(base, W, H); d = ImageDraw.Draw(img)
    for i in range(0, W, 6):                                 # moulded prism lines
        d.line([(i, 0), (i, H)], fill=tuple(min(255, c + 12) for c in base), width=1)
    pitch = min(W / cols, H / rows)
    for r in range(rows):
        for c in range(cols):
            cx = (c + 0.5) * W / cols; cy = (r + 0.5) * H / rows
            R = pitch * 0.28
            d.ellipse([cx - R * 1.6, cy - R * 1.6, cx + R * 1.6, cy + R * 1.6], fill=tuple(min(255, c + 25) for c in base))
            d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=color_on if lit else color_off)
    return img.filter(ImageFilter.GaussianBlur(1.0 if lit else 0.5))


def tex_amber(lit, W, H):
    return _led_grid((150, 70, 0), (255, 200, 60), lit, 2, max(2, round(2 * W / H)), (190, 95, 5) if lit else (120, 58, 4), W, H)


def tex_red(lit, W, H):
    return _led_grid((110, 8, 8), (255, 70, 60), lit, 2, max(2, round(2 * W / H)), (200, 20, 20) if lit else (95, 8, 10), W, H)


def tex_white(lit, W, H):
    return _led_grid((150, 150, 155), (255, 255, 255), lit, 2, max(2, round(2 * W / H)), (225, 228, 232) if lit else (130, 132, 138), W, H)


# ------------------------------------------------------------------ assemble

PARTS = {
    # name: (housing w, h, depth, corner r, chamfer)
    'headlamp': (0.090, 0.040, 0.045, 0.010, 0.004),
    'turn': (0.055, 0.032, 0.030, 0.008, 0.003),
    'tail_combo': (0.150, 0.040, 0.032, 0.008, 0.003),
}
# Tail combo lenses, from the inner edge outwards, y offsets across the face.
TAIL_LENSES = (('reverse', 0.035), ('stop', 0.070), ('turn', 0.040))
DIVIDER = 0.0025


def tail_lens_layout():
    """(name, width, centre offset) for the combo's three lenses, inner edge at -w/2."""
    w = PARTS['tail_combo'][0] - 2 * 0.005
    total = sum(lw for _, lw in TAIL_LENSES) + DIVIDER * (len(TAIL_LENSES) - 1)
    scale = w / total
    y = -w / 2
    out = []
    for name, lw in TAIL_LENSES:
        lw *= scale
        out.append((name, lw, y + lw / 2))
        y += lw + DIVIDER * scale
    return out


def main():
    os.makedirs(OUT, exist_ok=True)
    os.makedirs(GZ_OUT, exist_ok=True)
    black = (0.03, 0.03, 0.035)
    for name, (w, h, depth, r, ch) in PARTS.items():
        write_dae(os.path.join(OUT, f'{name}_housing.dae'), housing(w, h, depth, r, ch), black, shininess=60)
    lenses = {
        'headlamp_lens': (lens(0.090 - 0.010, 0.040 - 0.010, 0.006), 'lens_headlamp'),
        'turn_lens': (lens(0.055 - 0.008, 0.032 - 0.008, 0.005), 'lens_amber'),
    }
    for lname, lw, off in tail_lens_layout():
        lenses[f'tail_{lname}_lens'] = (lens(lw, 0.040 - 0.010, 0.004, y_off=off),
                                        {'reverse': 'lens_white', 'stop': 'lens_red', 'turn': 'lens_amber'}[lname])
    makers = {'lens_headlamp': tex_headlamp, 'lens_amber': tex_amber, 'lens_red': tex_red, 'lens_white': tex_white}
    for name, (mesh, kind) in lenses.items():
        P = np.array(mesh.P)
        w, h = np.ptp(P[:, 1]), np.ptp(P[:, 2])          # lens face in metres
        H = 128; W = max(32, int(round(H * w / h)))      # same aspect in pixels
        for lit in (False, True):
            makers[kind](lit, W, H).save(os.path.join(OUT, f'{name}{"_lit" if lit else ""}.png'))
        write_dae(os.path.join(OUT, f'{name}.dae'), mesh, (1, 1, 1), texture=f'{name}_lit.png', shininess=90)
    for f in os.listdir(OUT):
        shutil.copy(os.path.join(OUT, f), os.path.join(GZ_OUT, f))
    print('meshes:', ', '.join(sorted(f for f in os.listdir(OUT) if f.endswith('.dae'))))
    print('tail lenses (name, width, offset):', [(n, round(w, 4), round(o, 4)) for n, w, o in tail_lens_layout()])


if __name__ == '__main__':
    main()
