"""
Compile the per-vehicle PyBricks source files into standalone lego_vehicle_timer_*.py files.

Usage (from project root):
    python tools/compile_pybricks_files.py
"""
import os
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.resolve()
MODULES_DIR  = PROJECT_ROOT / 'modules'
OUT_DIR      = PROJECT_ROOT / 'docs' / 'pybricks'

VEHICLES = ['servo', 'train', 'skid_steer', 'odv']
MARKERS  = ['IMPORTS', 'VARS', 'MODULE', 'DRIVE_SETUP']


class CompileError(Exception):
    pass


def _locate_markers(path: Path, lines: list[str]) -> dict[str, tuple[int, int]]:
    """Return {marker: (start_line, end_line)} after validating every marker pair
    appears exactly once, START precedes END, and pairs do not overlap.
    Raise CompileError with a clear message on any violation."""
    raw: dict[str, dict[str, int]] = {}
    for i, line in enumerate(lines):
        stripped = line.lstrip()
        if not stripped.startswith('#'):
            continue
        body = stripped[1:].strip()
        for marker in MARKERS:
            for suffix in ('START', 'END'):
                tag = f'{marker}_{suffix}'
                if body == tag or body.startswith(tag + ' '):
                    slot = raw.setdefault(marker, {})
                    if suffix in slot:
                        raise CompileError(
                            f'{path}: duplicate marker "# {tag}" '
                            f'at lines {slot[suffix] + 1} and {i + 1}'
                        )
                    slot[suffix] = i

    pairs: dict[str, tuple[int, int]] = {}
    for marker in MARKERS:
        slot = raw.get(marker, {})
        if 'START' not in slot:
            raise CompileError(f'{path}: missing marker "# {marker}_START"')
        if 'END' not in slot:
            raise CompileError(f'{path}: missing marker "# {marker}_END"')
        if slot['START'] >= slot['END']:
            raise CompileError(
                f'{path}: "# {marker}_START" (line {slot["START"] + 1}) must precede '
                f'"# {marker}_END" (line {slot["END"] + 1})'
            )
        pairs[marker] = (slot['START'], slot['END'])

    ordered = sorted((s, e, m) for m, (s, e) in pairs.items())
    for (s1, e1, m1), (s2, e2, m2) in zip(ordered, ordered[1:]):
        if s2 <= e1:
            raise CompileError(
                f'{path}: marker pairs "{m1}" (lines {s1 + 1}-{e1 + 1}) and '
                f'"{m2}" (lines {s2 + 1}-{e2 + 1}) overlap'
            )
    return pairs


def _extract_sections(path: Path) -> dict[str, str]:
    """Return the inner content of each marker pair (exclusive of the marker lines)."""
    with open(path) as f:
        lines = f.readlines()
    pairs = _locate_markers(path, lines)
    return {m: ''.join(lines[s + 1:e]) for m, (s, e) in pairs.items()}


def _compile_one(vehicle: str) -> Path:
    print(f'Compiling {vehicle}')

    base_path = MODULES_DIR / 'lego_vehicle_timer_base.py'
    with open(base_path) as f:
        base_content = f.read()

    vehicle_path = MODULES_DIR / f'vehicle_{vehicle}.py'
    sections = _extract_sections(vehicle_path)

    new_content = base_content.replace('# IMPORT_SECTION',  sections['IMPORTS'])
    new_content = new_content.replace('# VARS_SECTION',     sections['VARS'])
    new_content = new_content.replace('# VEHICLE_SECTION',  sections['MODULE'])
    new_content = new_content.replace(
        'drive_motors = MotorHelper(False, False)', sections['DRIVE_SETUP']
    )

    out_path = OUT_DIR / f'lego_vehicle_timer_{vehicle}.py'
    if out_path.exists():
        os.remove(out_path)
    with open(out_path, 'w') as f:
        f.write(new_content)
    return out_path


def _run_ruff() -> None:
    print('Linting compiled files with ruff')
    result = subprocess.run(
        [sys.executable, '-m', 'ruff', 'check', str(OUT_DIR)],
        cwd=str(PROJECT_ROOT),
    )
    if result.returncode != 0:
        raise CompileError(f'ruff reported lint errors in {OUT_DIR}')


def main() -> None:
    for vehicle in VEHICLES:
        _compile_one(vehicle)
    _run_ruff()


if __name__ == '__main__':
    main()
