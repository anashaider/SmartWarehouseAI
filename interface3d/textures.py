"""Procedural textures generated with NumPy (deterministic, no asset files)."""

from __future__ import annotations

import numpy as np
from panda3d.core import SamplerState, Texture


def _to_texture(arr: np.ndarray, name: str, repeat: bool = True, mipmap: bool = True) -> Texture:
    """``arr``: H x W x 3/4 uint8, row 0 = top of the image."""
    arr = np.ascontiguousarray(np.flipud(arr))  # Panda3D stores images bottom-up
    h, w, ch = arr.shape
    tex = Texture(name)
    fmt = Texture.F_rgba if ch == 4 else Texture.F_rgb
    tex.setup2dTexture(w, h, Texture.T_unsigned_byte, fmt)
    order = "RGBA" if ch == 4 else "RGB"
    tex.setRamImageAs(arr.tobytes(), order)
    wrap = SamplerState.WM_repeat if repeat else SamplerState.WM_clamp
    tex.setWrapU(wrap)
    tex.setWrapV(wrap)
    if mipmap:
        tex.setMinfilter(SamplerState.FT_linear_mipmap_linear)
        tex.setAnisotropicDegree(4)
    else:
        tex.setMinfilter(SamplerState.FT_linear)
    tex.setMagfilter(SamplerState.FT_linear)
    return tex


def _value_noise(size: int, cells: int, rng: np.random.Generator) -> np.ndarray:
    """Tileable smooth value noise in [0, 1]."""
    grid = rng.random((cells, cells))
    x = np.linspace(0, cells, size, endpoint=False)
    xi = x.astype(int)
    xf = x - xi
    xf = xf * xf * (3 - 2 * xf)
    x0, x1 = xi % cells, (xi + 1) % cells
    a = grid[np.ix_(x0, x0)]
    b = grid[np.ix_(x0, x1)]
    c = grid[np.ix_(x1, x0)]
    d = grid[np.ix_(x1, x1)]
    fx, fy = xf[None, :], xf[:, None]
    top = a * (1 - fx) + b * fx
    bot = c * (1 - fx) + d * fx
    return top * (1 - fy) + bot * fy


def concrete_floor(size: int = 256, seed: int = 7) -> Texture:
    """Polished concrete tile: one texture repeat = one grid cell, with seams."""
    rng = np.random.default_rng(seed)
    n = 0.55 * _value_noise(size, 8, rng) + 0.30 * _value_noise(size, 32, rng) + 0.15 * rng.random((size, size))
    base = np.array([62, 68, 78], dtype=float)
    img = base[None, None, :] * (0.82 + 0.30 * n[..., None])
    # subtle stains
    stains = _value_noise(size, 4, rng)
    img *= (0.94 + 0.08 * stains[..., None])
    # tile seams
    seam = 2
    img[:seam, :, :] *= 0.62
    img[:, :seam, :] *= 0.62
    img[seam:seam + 1, :, :] *= 1.12
    img[:, seam:seam + 1, :] *= 1.12
    return _to_texture(np.clip(img, 0, 255).astype(np.uint8), "concrete")


def hazard_stripes(size: int = 128) -> Texture:
    """Diagonal yellow / black safety stripes."""
    yy, xx = np.mgrid[0:size, 0:size]
    band = ((xx + yy) // (size // 4)) % 2
    yellow = np.array([235, 182, 30], dtype=np.uint8)
    black = np.array([28, 28, 30], dtype=np.uint8)
    img = np.where(band[..., None] == 0, yellow, black).astype(np.uint8)
    return _to_texture(img, "hazard")


def dash_texture(width: int = 64, height: int = 16) -> Texture:
    """Chevron/dash pattern with alpha, scrolled along navigation paths."""
    img = np.zeros((height, width, 4), dtype=np.uint8)
    xx = np.arange(width)[None, :]
    yy = np.abs(np.arange(height)[:, None] - (height - 1) / 2)
    chevron = ((xx - yy * 1.6) % width) < width * 0.42
    img[..., :3] = 255
    img[..., 3] = np.where(chevron, 235, 50)
    return _to_texture(img, "dash")


def corrugated_wall(size: int = 128, seed: int = 3) -> Texture:
    rng = np.random.default_rng(seed)
    xx = np.arange(size)[None, :]
    ridge = 0.5 + 0.5 * np.sin(xx / size * 2 * np.pi * 8)
    n = _value_noise(size, 8, rng)
    base = np.array([44, 52, 66], dtype=float)
    img = base[None, None, :] * (0.75 + 0.25 * ridge[..., None] + 0.1 * n[..., None])
    img = np.broadcast_to(img, (size, size, 3))
    return _to_texture(np.clip(img, 0, 255).astype(np.uint8), "wall")


def conveyor_belt(size: int = 64) -> Texture:
    yy = np.arange(size)[:, None]
    ridges = (yy % 8) < 2
    img = np.where(ridges[..., None], np.array([18, 18, 22]), np.array([46, 48, 54]))
    img = np.broadcast_to(img, (size, size, 3)).astype(np.uint8)
    return _to_texture(np.ascontiguousarray(img), "belt")


def soft_glow(size: int = 64) -> Texture:
    """Radial white glow with alpha falloff (for markers and effects)."""
    yy, xx = np.mgrid[0:size, 0:size]
    r = np.hypot(xx - (size - 1) / 2, yy - (size - 1) / 2) / (size / 2)
    alpha = np.clip(1 - r, 0, 1) ** 2
    img = np.zeros((size, size, 4), dtype=np.uint8)
    img[..., :3] = 255
    img[..., 3] = (alpha * 255).astype(np.uint8)
    return _to_texture(img, "glow", repeat=False)


def label_texture(text_rows: list[str] | None = None, size: int = 64) -> Texture:
    """Cardboard label: off-white rectangle with barcode-like bars."""
    img = np.full((size, size, 3), (214, 168, 112), dtype=np.uint8)
    img[size // 4: size * 3 // 4, size // 6: size * 5 // 6] = (238, 234, 222)
    rng = np.random.default_rng(11)
    x = size // 6 + 4
    while x < size * 5 // 6 - 4:
        w = int(rng.integers(1, 3))
        img[size // 2 - 2: size * 3 // 4 - 4, x:x + w] = (30, 30, 30)
        x += w + int(rng.integers(1, 3))
    return _to_texture(img, "label")
