"""Render the converted mesh the same way the board does (z-buffer, Gouraud, one light).

  python tools/preview.py mesh.npz out_dir

Writes still images and a turntable GIF for the README.
"""
import math
import os
import sys

import numpy as np
from PIL import Image

W, H = 368, 448


def rot(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    return rx @ ry


def render(m, yaw, pitch, zoom=1.0):
    v = m["v"].astype(np.float64) / 32767
    n = m["n"].astype(np.float64) / 127
    f = m["f"].astype(np.int64)
    col = m["c"].astype(np.float64)
    R = rot(yaw, pitch)
    v = v @ R.T
    n = n @ R.T
    light = np.array([-0.4, 0.6, 0.7])
    light /= np.linalg.norm(light)
    # two-sided: Tripo meshes have mixed winding, so flip each normal to face the camera
    n = n * np.where(n[:, 2:3] < 0, -1, 1)
    inten = 0.35 + 0.9 * np.clip(n @ light, 0, 1)
    # same perspective as the sketch: camera at z = +3 looking down -z
    cam = 3.0
    s = 470 * zoom  # FOCAL in the sketch
    z = cam - v[:, 2]
    sx = W / 2 + s * v[:, 0] / z
    sy = 236 - s * v[:, 1] / z  # CENTER_Y in the sketch
    img = np.zeros((H, W, 3))
    zb = np.full((H, W), np.inf)
    for (a, b, c), fc in zip(f, col):
        x0, y0, x1, y1, x2, y2 = sx[a], sy[a], sx[b], sy[b], sx[c], sy[c]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if area == 0:
            continue
        mnx, mxx = max(int(min(x0, x1, x2)), 0), min(int(max(x0, x1, x2)) + 1, W - 1)
        mny, mxy = max(int(min(y0, y1, y2)), 0), min(int(max(y0, y1, y2)) + 1, H - 1)
        if mnx > mxx or mny > mxy:
            continue
        px, py = np.meshgrid(np.arange(mnx, mxx + 1) + 0.5, np.arange(mny, mxy + 1) + 0.5)
        w0 = ((x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)) / area
        w1 = ((x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)) / area
        w2 = 1 - w0 - w1
        inside = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
        if not inside.any():
            continue
        zz = w0 * z[a] + w1 * z[b] + w2 * z[c]
        sub = zb[mny:mxy + 1, mnx:mxx + 1]
        ok = inside & (zz < sub)
        sub[ok] = zz[ok]
        li = w0 * inten[a] + w1 * inten[b] + w2 * inten[c]
        img[mny:mxy + 1, mnx:mxx + 1][ok] = np.clip(fc[None, :] * li[ok][:, None], 0, 255)
    return Image.fromarray(img.astype(np.uint8))


def main():
    m = np.load(sys.argv[1])
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    for name, yaw in [("side", -math.pi / 2), ("three_quarter", -math.pi / 4 - 0.2), ("front", 0.0)]:
        render(m, yaw, 0.22).save(os.path.join(out, name + ".png"))
    if "--gif" in sys.argv:
        frames = [render(m, -math.pi / 2 + i * 2 * math.pi / 36, 0.22) for i in range(36)]
        frames[0].save(os.path.join(out, "turntable.gif"), save_all=True, append_images=frames[1:],
                       duration=70, loop=0, optimize=True)
    print("wrote previews to", out)


if __name__ == "__main__":
    main()
