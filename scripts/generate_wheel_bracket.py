#!/usr/bin/env python3
"""Generate wheel-mount bracket STL for R2 rocker-bogie mecanum robot.

Based on real robot photo: a large motor housing (black cylinder, nearly
the same diameter as the wheel) sits between the rocker arm and the wheel.
A bracket plate on top connects the motor to the arm.

Coordinate system (part-local, mm):
  - Origin at wheel axle center
  - +Y toward parent arm (inboard)
  - +Z up
  - +X forward

Output: meshes/wheel_bracket.stl
"""

import struct
import math
import os

# ── Dimensions (mm) — tuned to match real robot photo ────────────────────────
MOTOR_RADIUS = 38.0       # motor housing radius (~76mm dia, close to wheel dia 101mm)
MOTOR_LENGTH = 30.0       # motor housing length along Y
MOTOR_Y_START = 2.0       # small clearance from wheel face

BRACKET_THICKNESS = 4.0   # plate thickness
BRACKET_WIDTH = 52.0      # X-direction width of bracket plates

# Motor flange (slightly larger ring at the wheel-side end)
FLANGE_RADIUS = 42.0
FLANGE_LENGTH = 4.0

# ── Mesh generation ──────────────────────────────────────────────────────────

def cylinder_triangles(radius, y_start, y_end, n_seg=32):
    """Generate triangles for a cylinder along Y axis."""
    tris = []
    for i in range(n_seg):
        a0 = 2 * math.pi * i / n_seg
        a1 = 2 * math.pi * (i + 1) / n_seg
        c0, s0 = math.cos(a0), math.sin(a0)
        c1, s1 = math.cos(a1), math.sin(a1)

        x0, z0 = radius * c0, radius * s0
        x1, z1 = radius * c1, radius * s1

        # Side quad (2 tris)
        tris.append(((x0, y_start, z0), (x1, y_start, z1), (x1, y_end, z1)))
        tris.append(((x0, y_start, z0), (x1, y_end, z1), (x0, y_end, z0)))
        # End caps
        tris.append(((0, y_start, 0), (x1, y_start, z1), (x0, y_start, z0)))
        tris.append(((0, y_end, 0), (x0, y_end, z0), (x1, y_end, z1)))

    return tris


def box_triangles(x0, x1, y0, y1, z0, z1):
    """Generate 12 triangles for an axis-aligned box."""
    v = [
        (x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
        (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1),
    ]
    faces = [
        (0,3,2,1),(4,5,6,7),  # -Z, +Z
        (0,1,5,4),(2,3,7,6),  # -Y, +Y
        (0,4,7,3),(1,2,6,5),  # -X, +X
    ]
    tris = []
    for f in faces:
        tris.append((v[f[0]], v[f[1]], v[f[2]]))
        tris.append((v[f[0]], v[f[2]], v[f[3]]))
    return tris


def compute_normal(v0, v1, v2):
    ux, uy, uz = v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]
    vx, vy, vz = v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    L = math.sqrt(nx*nx + ny*ny + nz*nz)
    return (nx/L, ny/L, nz/L) if L > 1e-12 else (0,0,0)


def write_binary_stl(filepath, triangles):
    with open(filepath, 'wb') as f:
        f.write(b'\x00' * 80)
        f.write(struct.pack('<I', len(triangles)))
        for v0, v1, v2 in triangles:
            n = compute_normal(v0, v1, v2)
            f.write(struct.pack('<fff', *n))
            f.write(struct.pack('<fff', *v0))
            f.write(struct.pack('<fff', *v1))
            f.write(struct.pack('<fff', *v2))
            f.write(struct.pack('<H', 0))
    print(f"  {filepath}: {len(triangles)} triangles")


def generate_bracket():
    tris = []
    half_w = BRACKET_WIDTH / 2
    t = BRACKET_THICKNESS
    motor_y_end = MOTOR_Y_START + MOTOR_LENGTH

    # 1. Motor housing — large cylinder (main visual element)
    tris.extend(cylinder_triangles(MOTOR_RADIUS, MOTOR_Y_START, motor_y_end))

    # 2. Flange ring at wheel-side end (slightly wider)
    tris.extend(cylinder_triangles(
        FLANGE_RADIUS, MOTOR_Y_START, MOTOR_Y_START + FLANGE_LENGTH))

    # 3. Top bracket plate — horizontal, connects motor top to parent arm
    plate_z_bot = MOTOR_RADIUS + 1
    plate_z_top = plate_z_bot + t
    tris.extend(box_triangles(
        -half_w, half_w,
        MOTOR_Y_START - 2, motor_y_end + 6,
        plate_z_bot, plate_z_top))

    # 4. Back plate — vertical at parent-arm end, drops from top plate to motor center
    tris.extend(box_triangles(
        -half_w, half_w,
        motor_y_end, motor_y_end + 6,
        -MOTOR_RADIUS * 0.3, plate_z_top))

    # 5. Front plate — partial vertical at wheel side (holds motor from above)
    tris.extend(box_triangles(
        -half_w, half_w,
        MOTOR_Y_START - 2, MOTOR_Y_START,
        MOTOR_RADIUS * 0.4, plate_z_top))

    # 6. Two side gussets (reinforcement ribs)
    for xs, xe in [(-half_w, -half_w + t), (half_w - t, half_w)]:
        tris.extend(box_triangles(
            xs, xe,
            motor_y_end - t, motor_y_end + 6,
            0, plate_z_bot))

    return tris


def mirror_y(triangles):
    """Mirror triangles in Y axis and reverse winding order for correct normals."""
    mirrored = []
    for v0, v1, v2 in triangles:
        # Negate Y, swap v1/v2 to fix winding
        mirrored.append(
            ((v0[0], -v0[1], v0[2]),
             (v2[0], -v2[1], v2[2]),
             (v1[0], -v1[1], v1[2])))
    return mirrored


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    mesh_dir = os.path.join(script_dir, '..', 'meshes')
    os.makedirs(mesh_dir, exist_ok=True)

    print("Generating wheel bracket STLs...")
    tris = generate_bracket()

    # Left: bracket extends +Y (toward parent for left wheels)
    write_binary_stl(os.path.join(mesh_dir, 'wheel_bracket_left.stl'), tris)

    # Right: mirror in Y (bracket extends -Y, toward parent for right wheels)
    write_binary_stl(os.path.join(mesh_dir, 'wheel_bracket_right.stl'), mirror_y(tris))
    print("Done!")


if __name__ == '__main__':
    main()
