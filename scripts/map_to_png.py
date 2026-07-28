#!/usr/bin/env python3
"""Render a saved Nav2/SLAM occupancy map (.pgm) as a readable PNG.

The raw .pgm is greyscale and low-contrast; this colourises the three occupancy
classes (free / occupied / unknown) and upscales with nearest-neighbour so the
cell structure stays crisp. Run with the Windows robotics venv.
"""
import sys
from pathlib import Path

import numpy as np
import imageio.v3 as iio


def read_pgm(path: Path) -> np.ndarray:
    data = path.read_bytes()
    parts, idx = [], 0
    while len(parts) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":               # comment line
            while idx < len(data) and data[idx] != 0x0A:
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        parts.append(data[start:idx])
    idx += 1                                        # single whitespace after maxval
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[idx:idx + w * h], dtype=np.uint8).reshape(h, w)


def colourise(img: np.ndarray, scale: int = 6) -> np.ndarray:
    out = np.zeros((*img.shape, 3), dtype=np.uint8)
    unknown = (img > 100) & (img < 250)
    free = img >= 250
    occ = img <= 100
    out[free] = (245, 245, 245)      # free space
    out[occ] = (20, 20, 30)          # walls / obstacles
    out[unknown] = (130, 140, 155)   # never observed
    return np.kron(out, np.ones((scale, scale, 1), dtype=np.uint8))


def main():
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else src.with_suffix(".png")
    img = read_pgm(src)
    free = int((img >= 250).sum())
    occ = int((img <= 100).sum())
    unknown = img.size - free - occ
    iio.imwrite(dst, colourise(img))
    print(f"{src.name}: {img.shape[1]}x{img.shape[0]} cells")
    print(f"  free     {free:6d} ({100*free/img.size:5.1f}%)")
    print(f"  occupied {occ:6d} ({100*occ/img.size:5.1f}%)")
    print(f"  unknown  {unknown:6d} ({100*unknown/img.size:5.1f}%)")
    print(f"wrote {dst}")


if __name__ == "__main__":
    main()
