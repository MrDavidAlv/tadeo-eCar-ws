"""Generate the world textures.

They are generated rather than downloaded for the same reason the worlds are:
so that everything in this package can be rebuilt from source, and so no binary
arrives in the repository without a recipe that explains it.

What they are for is narrower than "making the simulation look nicer". A flat
shaded box has an intensity gradient nowhere except at its silhouette, and a
corner detector needs gradients. Measured on the untextured factory world,
RTAB-Map's visual odometry ran at 0 to 9 inliers against a threshold of 10 and
lost tracking within seconds of the robot moving. Every texture here therefore
carries hard edges - slab joints, panel seams, plank lines, rivets - at a scale
the camera resolves from a few metres away, rather than smooth noise.

Usage:
    python3 generate_textures.py            # writes models/materials/textures
    python3 generate_textures.py <dir>

Needs Pillow. Nothing else in the workspace does, which is why it is a script
run by hand rather than a build step.
"""

import math
import os
import random
import sys

from PIL import Image, ImageDraw

SIZE = 512


def grain(img, amount, seed):
    """Per-pixel noise, so no region is perfectly uniform."""
    rnd = random.Random(seed)
    px = img.load()
    w, h = img.size
    for y in range(h):
        for x in range(w):
            r, g, b = px[x, y][:3]
            n = rnd.randint(-amount, amount)
            px[x, y] = (max(0, min(255, r + n)), max(0, min(255, g + n)),
                        max(0, min(255, b + n)))
    return img


def concrete(size=SIZE):
    random.seed(11)
    im = Image.new('RGB', (size, size), (150, 150, 146))
    d = ImageDraw.Draw(im)
    step = size // 4
    for i in range(0, size + 1, step):
        d.line([(i, 0), (i, size)], fill=(112, 112, 110), width=3)
        d.line([(0, i), (size, i)], fill=(112, 112, 110), width=3)
    for _ in range(220):
        x, y = random.randrange(size), random.randrange(size)
        r = random.randrange(2, 7)
        c = random.randrange(125, 172)
        d.ellipse([x - r, y - r, x + r, y + r], fill=(c, c, c - 3))
    return grain(im, 9, 1)


def painted_wall(size=SIZE):
    random.seed(12)
    im = Image.new('RGB', (size, size), (203, 202, 196))
    d = ImageDraw.Draw(im)
    for i in range(0, size, size // 8):
        d.line([(0, i), (size, i)], fill=(188, 187, 182), width=2)
    d.rectangle([0, 0, size - 1, size - 1], outline=(150, 150, 145), width=4)
    for _ in range(120):
        d.point((random.randrange(size), random.randrange(size)),
                fill=(170, 170, 166))
    return grain(im, 6, 2)


def steel(size=SIZE):
    random.seed(13)
    im = Image.new('RGB', (size, size), (118, 126, 136))
    d = ImageDraw.Draw(im)
    for i in range(0, size, 24):
        d.line([(i, 0), (i, size)], fill=(96, 104, 114), width=2)
    for i in range(0, size, size // 4):
        d.line([(0, i), (size, i)], fill=(78, 86, 96), width=5)
        for x in range(12, size, size // 8):
            d.ellipse([x - 4, i - 4, x + 4, i + 4], fill=(150, 158, 168))
    return grain(im, 8, 3)


def crate(size=SIZE):
    random.seed(14)
    im = Image.new('RGB', (size, size), (150, 108, 62))
    d = ImageDraw.Draw(im)
    for i in range(0, size, size // 6):
        d.line([(0, i), (size, i)], fill=(118, 82, 44), width=4)
    for _ in range(400):
        y = random.randrange(size)
        x0 = random.randrange(size)
        d.line([(x0, y), (x0 + random.randrange(20, 120), y)],
               fill=(132, 94, 52), width=1)
    d.rectangle([0, 0, size - 1, size - 1], outline=(96, 66, 34), width=6)
    return grain(im, 10, 4)


def asphalt(size=SIZE):
    random.seed(15)
    im = Image.new('RGB', (size, size), (74, 74, 78))
    d = ImageDraw.Draw(im)
    for _ in range(2600):
        x, y = random.randrange(size), random.randrange(size)
        r = random.randrange(1, 4)
        c = random.randrange(58, 108)
        d.ellipse([x - r, y - r, x + r, y + r], fill=(c, c, c + 3))
    return grain(im, 12, 5)


def gravel(size=SIZE):
    random.seed(16)
    im = Image.new('RGB', (size, size), (116, 110, 100))
    d = ImageDraw.Draw(im)
    for _ in range(4200):
        x, y = random.randrange(size), random.randrange(size)
        r = random.randrange(2, 7)
        c = random.randrange(86, 168)
        d.ellipse([x - r, y - r, x + r, y + r],
                  fill=(c, int(c * 0.95), int(c * 0.86)))
    return grain(im, 14, 6)


def sand(size=SIZE):
    random.seed(17)
    im = Image.new('RGB', (size, size), (198, 174, 118))
    d = ImageDraw.Draw(im)
    for y in range(size):
        shade = int(10 * math.sin(y / 18.0))
        d.line([(0, y), (size, y)], fill=(198 + shade, 174 + shade, 118 + shade))
    return grain(im, 11, 7)


TEXTURES = {
    'concrete': concrete,
    'painted_wall': painted_wall,
    'steel': steel,
    'crate': crate,
    'asphalt': asphalt,
    'gravel': gravel,
    'sand': sand,
}


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    # Next to the worlds, not next to the models. Gazebo resolves a relative
    # texture URI against the directory of the file that names it, NOT against
    # GZ_SIM_RESOURCE_PATH: with the textures under models/ the server logged
    # "Unable to find file [materials/textures/concrete.png]" for every one of
    # them and rendered the whole world flat-shaded, which is precisely the
    # condition the textures exist to remove.
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(here), 'worlds', 'materials', 'textures')
    os.makedirs(out, exist_ok=True)
    for name, fn in TEXTURES.items():
        path = os.path.join(out, f'{name}.png')
        fn().save(path)
        print(f'{name:14s} {SIZE}x{SIZE} -> {path}')
