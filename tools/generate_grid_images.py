"""
Generate ODV grid PNG images from grid definitions in vehicle_odv.py.

Usage (from project root):
    python tools/generate_grid_images.py

Output:
    images/ODV_GRID_DEFAULT.png
    images/ODV_GRID_EX1.png
    images/ODV_GRID_EX2.png
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tools.compile_pybricks_files as _compile
from PIL import Image, ImageDraw, ImageFont
from modules.vehicle_odv import ODV_GRID_DEFAULT, ODV_GRID_EX1, ODV_GRID_EX2, ODV_GRID_EX3

# ── layout ────────────────────────────────────────────────────────────────────
TILE   = 36   # px per tile (square)
GAP    = 3    # px gap between tiles
PAD    = 6    # px outer padding

# ── colours ───────────────────────────────────────────────────────────────────
BG           = (255, 255, 255)
TRACK_FILL   = (208, 208, 208)
LOAD_FILL    = ( 92, 184,  92)   # green
UNLOAD_FILL  = (224, 120,   0)   # orange
BORDER       = (  0,   0,   0)
LABEL_COLOR  = (255, 255, 255)
ARROW_COLOR  = ( 80,  80,  80)

TILE_COLORS = {
    '#': TRACK_FILL,
    'L': LOAD_FILL,
    'U': UNLOAD_FILL,
    '<': TRACK_FILL,
    '>': TRACK_FILL,
}

GRIDS = [
    ('ODV_GRID_DEFAULT', ODV_GRID_DEFAULT),
    ('ODV_GRID_EX1', ODV_GRID_EX1),
    ('ODV_GRID_EX2', ODV_GRID_EX2),
    ('ODV_GRID_EX3', ODV_GRID_EX3),
]


def _load_font(size: int) -> ImageFont.ImageFont:
    candidates = [
        'arialbd.ttf',
        'arial.ttf',
        'DejaVuSans-Bold.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
    ]
    for path in candidates:
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            pass
    return ImageFont.load_default()


def _draw_arrow(draw: ImageDraw.ImageDraw, px: int, py: int, char: str) -> None:
    """Draw a filled arrow centred in the tile at pixel origin (px, py)."""
    cx = px + TILE // 2
    cy = py + TILE // 2
    hw = TILE // 4   # half-width of arrow body
    ah = TILE // 5   # half-height of arrowhead

    if char == '<':   # pointing left
        pts = [(cx + hw, cy - ah), (cx - hw, cy), (cx + hw, cy + ah)]
    else:             # '>' pointing right
        pts = [(cx - hw, cy - ah), (cx + hw, cy), (cx - hw, cy + ah)]

    draw.polygon(pts, fill=ARROW_COLOR)


def render_grid(grid: list[str]) -> Image.Image:
    rows = len(grid)
    cols = max(len(row.rstrip()) for row in grid)

    img_w = PAD * 2 + cols * TILE + (cols - 1) * GAP
    img_h = PAD * 2 + rows * TILE + (rows - 1) * GAP

    img  = Image.new('RGB', (img_w, img_h), BG)
    draw = ImageDraw.Draw(img)
    font = _load_font(TILE // 2)

    for row_idx, row in enumerate(grid):
        for col_idx, char in enumerate(row.rstrip()):
            if char == 'X':
                continue   # wall — leave white background

            px = PAD + col_idx * (TILE + GAP)
            py = PAD + row_idx * (TILE + GAP)
            x1, y1 = px, py
            x2, y2 = px + TILE - 1, py + TILE - 1

            fill = TILE_COLORS.get(char, TRACK_FILL)
            draw.rectangle([x1, y1, x2, y2], fill=fill, outline=BORDER, width=1)

            if char in ('L', 'U'):
                cx, cy = px + TILE // 2, py + TILE // 2
                draw.text((cx, cy), char, fill=LABEL_COLOR, font=font, anchor='mm')
            elif char in ('<', '>'):
                _draw_arrow(draw, px, py, char)

    return img


def main() -> None:
    _compile.main()

    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'images')
    os.makedirs(out_dir, exist_ok=True)

    for name, grid in GRIDS:
        path = os.path.join(out_dir, f'{name}.png')
        img  = render_grid(grid)
        img.save(path)
        print(f'  {name}.png  {img.size[0]}×{img.size[1]}px')


if __name__ == '__main__':
    main()
