#!/usr/bin/env python3
"""
apply_kfs_layout.py

Reads config/kfs_layout.yaml and regenerates the KFS model section
inside worlds/meihua_forest.sdf between the markers:
    <!-- KFS_SECTION_START -->
    <!-- KFS_SECTION_END -->

Usage (from the package root):
    python3 scripts/apply_kfs_layout.py
    python3 scripts/apply_kfs_layout.py --config /path/to/kfs_layout.yaml
    python3 scripts/apply_kfs_layout.py --sdf /path/to/meihua_forest.sdf
"""

import argparse
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not installed.  Run:  pip install pyyaml")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Field geometry — platform top surface (metres)
# ---------------------------------------------------------------------------
_X = {
    'lf': {'c1': -4.2, 'c2': -3.0, 'c3': -1.8},
    'rf': {'c1':  1.8, 'c2':  3.0, 'c3':  4.2},
}
_Y = {'r1': 1.8, 'r2': 0.6, 'r3': -0.6, 'r4': -1.8}

_TOP = {
    'lf': {
        'r1': {'c1': 0.40, 'c2': 0.20, 'c3': 0.40},
        'r2': {'c1': 0.20, 'c2': 0.40, 'c3': 0.60},
        'r3': {'c1': 0.40, 'c2': 0.60, 'c3': 0.40},
        'r4': {'c1': 0.20, 'c2': 0.40, 'c3': 0.20},
    },
    'rf': {
        'r1': {'c1': 0.40, 'c2': 0.20, 'c3': 0.40},
        'r2': {'c1': 0.60, 'c2': 0.40, 'c3': 0.20},
        'r3': {'c1': 0.40, 'c2': 0.60, 'c3': 0.40},
        'r4': {'c1': 0.20, 'c2': 0.40, 'c3': 0.20},
    },
}

# ---------------------------------------------------------------------------
# KFS geometry & appearance
# ---------------------------------------------------------------------------
KFS_SIDE = 0.35                          # 350 mm cube
KFS_HALF = KFS_SIDE / 2                  # 0.175 m
KFS_MASS = 0.2                           # kg
KFS_I    = round(KFS_MASS * KFS_SIDE**2 / 6, 5)  # 0.00408 (uniform cube)

COLORS = {
    'r1':   '0.80 0.15 0.10 1',   # red
    'r2':   '0.10 0.20 0.85 1',   # blue
    'fake': '0.55 0.55 0.55 1',   # grey
}

VALID_LABELS  = set(COLORS.keys())
FOREST_KEYS   = [('left_forest', 'lf'), ('right_forest', 'rf')]
ROWS          = ['r1', 'r2', 'r3', 'r4']
COLS          = ['c1', 'c2', 'c3']

START_TAG = '<!-- KFS_SECTION_START -->'
END_TAG   = '<!-- KFS_SECTION_END -->'

PKG_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# SDF snippet builder
# ---------------------------------------------------------------------------
def _model_sdf(name: str, x: float, y: float, z: float, label: str) -> str:
    color = COLORS[label]
    s = KFS_SIDE
    I = KFS_I
    return (
        f'    <model name="{name}"><!-- {label} -->\n'
        f'      <pose>{x} {y} {z:.4f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <visual name="visual">\n'
        f'          <geometry><box><size>{s} {s} {s}</size></box></geometry>\n'
        f'          <material><ambient>{color}</ambient>'
        f'<diffuse>{color}</diffuse></material>\n'
        f'        </visual>\n'
        f'        <collision name="collision">\n'
        f'          <geometry><box><size>{s} {s} {s}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <inertial>\n'
        f'          <mass>{KFS_MASS}</mass>\n'
        f'          <inertia>'
        f'<ixx>{I}</ixx><ixy>0</ixy><ixz>0</ixz>'
        f'<iyy>{I}</iyy><iyz>0</iyz><izz>{I}</izz>'
        f'</inertia>\n'
        f'        </inertial>\n'
        f'      </link>\n'
        f'    </model>'
    )


def generate_section(layout: dict) -> str:
    lines = [
        '    <!-- KFS — cube 350x350x350mm | r1=red  r2=blue  fake=grey -->',
        '    <!-- Edit config/kfs_layout.yaml then re-run apply_kfs_layout.py -->',
    ]
    placed = 0
    errors = []

    for yaml_key, forest in FOREST_KEYS:
        forest_cfg = layout.get(yaml_key) or {}
        for row in ROWS:
            row_cfg = forest_cfg.get(row)
            if not row_cfg:
                continue
            for i, col in enumerate(COLS):
                raw = row_cfg[i] if i < len(row_cfg) else None
                if raw is None:
                    continue
                label = str(raw).strip().lower()
                if label in ('none', 'null', '~', ''):
                    continue
                if label not in VALID_LABELS:
                    errors.append(
                        f'  Unknown label "{raw}" at {yaml_key}/{row}/col{i+1} '
                        f'— use r1, r2, fake, or ~ (skipped)'
                    )
                    continue
                x   = _X[forest][col]
                y   = _Y[row]
                z   = _TOP[forest][row][col] + KFS_HALF
                name = f'kfs_{forest}_{row}{col}'
                lines.append('')
                lines.append(_model_sdf(name, x, y, z, label))
                placed += 1

    return '\n'.join(lines), placed, errors


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--config', default=str(PKG_ROOT / 'config' / 'kfs_layout.yaml'),
        help='Path to kfs_layout.yaml  (default: config/kfs_layout.yaml)',
    )
    parser.add_argument(
        '--sdf', default=str(PKG_ROOT / 'worlds' / 'meihua_forest.sdf'),
        help='Path to meihua_forest.sdf  (default: worlds/meihua_forest.sdf)',
    )
    args = parser.parse_args()

    config_path = Path(args.config)
    sdf_path    = Path(args.sdf)

    if not config_path.exists():
        print(f'ERROR: config not found: {config_path}')
        sys.exit(1)
    if not sdf_path.exists():
        print(f'ERROR: SDF not found: {sdf_path}')
        sys.exit(1)

    with open(config_path) as f:
        layout = yaml.safe_load(f) or {}

    sdf = sdf_path.read_text()

    if START_TAG not in sdf or END_TAG not in sdf:
        print(f'ERROR: KFS section markers not found in {sdf_path}')
        print(f'  Expected: {START_TAG}  ...  {END_TAG}')
        sys.exit(1)

    section, placed, errors = generate_section(layout)

    new_sdf = re.sub(
        re.escape(START_TAG) + r'.*?' + re.escape(END_TAG),
        START_TAG + '\n' + section + '\n    ' + END_TAG,
        sdf,
        flags=re.DOTALL,
    )

    sdf_path.write_text(new_sdf)

    print(f'Updated: {sdf_path}')
    print(f'  KFS placed: {placed}')
    if errors:
        print('  Warnings:')
        for e in errors:
            print(e)


if __name__ == '__main__':
    main()
