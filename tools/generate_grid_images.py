"""
Generate ODV grid PNG images from grid definitions in vehicle_odv.py.

Usage (from project root):
    python tools/generate_grid_images.py

Output:
    docs/images/ODV_GRID_DEFAULT.png
    docs/images/ODV_GRID_EX1.png
    docs/images/ODV_GRID_EX2.png
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tools.compile_pybricks_files as _compile
from PIL import Image, ImageDraw, ImageFont
from modules.vehicle_odv import ODV_GRID_DEFAULT, ODV_GRID_EX1, ODV_GRID_EX2, ODV_GRID_EX3

# ── layout ────────────────────────────────────────────────────────────────────
TILE        = 72    # px per tile (square) — render at 2x so the HTML scale-down stays crisp on Retina
TILE_LONG   = round(TILE * 1.15)  # px for Load/Unload tiles (15% longer in X)
GAP         = 6    # px gap between tiles
PAD         = 12   # px outer padding

# ── colours ───────────────────────────────────────────────────────────────────
BG           = (255, 255, 255)
TRACK_FILL   = (230, 227, 224)  #	Very Light Bluish Gray	E6E3E0
LOAD_FILL    = ( 187, 223,  11)   # Lime	BBE90B
UNLOAD_FILL  = (254, 138,   24)   # Orange	FE8A18
BORDER       = (  0,   0,   0)
LABEL_COLOR  = (255, 255, 255)
ARROW_COLOR  = (201,  26,  9)   # Red	C91A09

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

# Single-tile previews for the docs' grid-legend. Re-uses render_grid so the
# tile colours and arrow style stay byte-for-byte identical to the full grid
# PNGs above; HTML scales them down via CSS for inline-icon display.
SINGLE_TILES = [
    ('tile_load',   ['L']),
    ('tile_unload', ['U']),
    ('tile_arrow',  ['<']),
]


def _load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
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


def _rline(draw: ImageDraw.ImageDraw, p1: tuple, p2: tuple, fill: tuple, width: int) -> None:
    """Line with round caps — draws the line then circles at both endpoints."""
    draw.line([p1, p2], fill=fill, width=width)
    r = width // 2
    for x, y in (p1, p2):
        draw.ellipse([x - r, y - r, x + r, y + r], fill=fill)


def _draw_arrow(draw: ImageDraw.ImageDraw, px: int, py: int, char: str) -> None:
    """Draw a thin arrow (<- or ->) centred in the tile at pixel origin (px, py).
    Shaft runs full-width to the tip; two chevron arms branch from the tip."""
    cx = px + TILE // 2
    cy = py + TILE // 2
    hw = TILE // 3       # half total arrow span
    aw = TILE // 4       # chevron arm horizontal reach
    ah = TILE // 4       # chevron arm half-height
    lw = max(2, TILE // 12)  # line width

    if char == '<':   # pointing left  (<-)
        tip_x = cx - hw
        _rline(draw, (cx + hw, cy), (tip_x, cy),        ARROW_COLOR, lw)  # shaft
        _rline(draw, (tip_x, cy),   (tip_x + aw, cy - ah), ARROW_COLOR, lw)  # top arm
        _rline(draw, (tip_x, cy),   (tip_x + aw, cy + ah), ARROW_COLOR, lw)  # bottom arm
    else:             # pointing right  (->)
        tip_x = cx + hw
        _rline(draw, (cx - hw, cy), (tip_x, cy),        ARROW_COLOR, lw)  # shaft
        _rline(draw, (tip_x, cy),   (tip_x - aw, cy - ah), ARROW_COLOR, lw)  # top arm
        _rline(draw, (tip_x, cy),   (tip_x - aw, cy + ah), ARROW_COLOR, lw)  # bottom arm


def _col_widths(grid: list[str]) -> list[int]:
    """Return per-column pixel width: TILE_LONG if any row has L or U, else TILE."""
    cols = max(len(row.rstrip()) for row in grid)
    widths = []
    for col_idx in range(cols):
        wide = any(
            col_idx < len(row) and row[col_idx] in ('L', 'U')
            for row in grid
        )
        widths.append(TILE_LONG if wide else TILE)
    return widths


def render_grid(grid: list[str]) -> Image.Image:
    rows = len(grid)
    col_w = _col_widths(grid)
    cols  = len(col_w)

    img_w = PAD * 2 + sum(col_w) + (cols - 1) * GAP
    img_h = PAD * 2 + rows * TILE + (rows - 1) * GAP

    img  = Image.new('RGB', (img_w, img_h), BG)
    draw = ImageDraw.Draw(img)
    font = _load_font(TILE // 2)

    # pre-compute left pixel edge of each column
    col_x = []
    x = PAD
    for w in col_w:
        col_x.append(x)
        x += w + GAP

    for row_idx, row in enumerate(grid):
        for col_idx, char in enumerate(row.rstrip()):
            if char == 'X':
                continue   # wall — leave white background

            px = col_x[col_idx]
            py = PAD + row_idx * (TILE + GAP)
            w  = col_w[col_idx]
            x1, y1 = px, py
            x2, y2 = px + w - 1, py + TILE - 1

            fill = TILE_COLORS.get(char, TRACK_FILL)
            draw.rectangle([x1, y1, x2, y2], fill=fill, outline=BORDER, width=1)

            if char in ('L', 'U'):
                cx, cy = px + w // 2, py + TILE // 2
                draw.text((cx, cy), char, fill=LABEL_COLOR, font=font, anchor='mm')
            elif char in ('<', '>'):
                _draw_arrow(draw, px, py, char)

    return img


def main() -> None:
    _compile.main()

    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'docs', 'images')
    os.makedirs(out_dir, exist_ok=True)

    for name, grid in GRIDS:
        path = os.path.join(out_dir, f'{name}.png')
        img  = render_grid(grid)
        img.save(path)
        print(f'  {name}.png  {img.size[0]}×{img.size[1]}px')

    for name, grid in SINGLE_TILES:
        path = os.path.join(out_dir, f'{name}.png')
        img  = render_grid(grid)
        img.save(path)
        print(f'  {name}.png  {img.size[0]}×{img.size[1]}px')


if __name__ == '__main__':
    main()
