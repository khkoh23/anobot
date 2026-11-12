#!/usr/bin/env python3
"""Propagate a re-taught marker-to-grasp transform to shared tank/rod CAD poses.

Rigid correction:
    C = T_new_grasp @ inverse(T_old_grasp)
    T_new_scene = C @ T_old_scene

The script prints replacement YAML values for both workstation_scene.yaml and
rod_lifecycle.yaml. It does not modify files.

RPY convention: degrees, fixed-axis roll/pitch/yaw represented as Rz(yaw) @
Ry(pitch) @ Rx(roll), matching the usual ROS quaternion_from_euler convention.
"""

import argparse
import math
import numpy as np


def rotation_from_rpy_deg(roll_deg, pitch_deg, yaw_deg):
    roll, pitch, yaw = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=float)


def transform_from_xyz_rpy(values):
    x, y, z, roll, pitch, yaw = values
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation_from_rpy_deg(roll, pitch, yaw)
    transform[:3, 3] = [x, y, z]
    return transform


def rpy_deg_from_rotation(rotation):
    # ZYX extraction. Near pitch = +/-90 degrees, equivalent Euler triples can
    # look very different; keep the full triple together.
    pitch = math.asin(max(-1.0, min(1.0, -float(rotation[2, 0]))))

    if abs(math.cos(pitch)) > 1.0e-9:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0

    return np.degrees([roll, pitch, yaw])


def format_values(transform):
    x, y, z = transform[:3, 3]
    roll, pitch, yaw = rpy_deg_from_rotation(transform[:3, :3])
    return x, y, z, roll, pitch, yaw


def add_pose_argument(parser, name, help_text):
    parser.add_argument(
        name,
        nargs=6,
        type=float,
        required=True,
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help=help_text + " as x y z roll_deg pitch_deg yaw_deg",
    )


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Propagate a changed marker mounting from an old taught grasp "
            "transform to tank and rod poses that share one CAD origin."
        )
    )
    add_pose_argument(
        parser,
        "--old-grasp",
        "Previously aligned marker-to-grasp pose",
    )
    add_pose_argument(
        parser,
        "--new-grasp",
        "New physically taught marker-to-grasp pose",
    )
    add_pose_argument(
        parser,
        "--old-scene",
        "Previously aligned marker-to-shared-CAD-origin pose",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=6,
        help="Decimal places for printed YAML values (default: 6)",
    )
    args = parser.parse_args()

    old_grasp = transform_from_xyz_rpy(args.old_grasp)
    new_grasp = transform_from_xyz_rpy(args.new_grasp)
    old_scene = transform_from_xyz_rpy(args.old_scene)

    correction = new_grasp @ np.linalg.inv(old_grasp)
    new_scene = correction @ old_scene

    cx, cy, cz, cr, cp, cyaw = format_values(correction)
    x, y, z, roll, pitch, yaw = format_values(new_scene)
    p = args.precision

    print("RIGID MARKER-MOUNT CORRECTION")
    print("=" * 60)
    print("Correction applied in workstation_marker_acquired coordinates:")
    print(f"  translation [m]: x={cx:+.{p}f}, y={cy:+.{p}f}, z={cz:+.{p}f}")
    print(f"  RPY [deg]:       roll={cr:+.{p}f}, pitch={cp:+.{p}f}, yaw={cyaw:+.{p}f}")

    print("\nNEW SHARED TANK/ROD CAD-ORIGIN POSE")
    print(f"  x={x:+.{p}f} m")
    print(f"  y={y:+.{p}f} m")
    print(f"  z={z:+.{p}f} m")
    print(f"  roll={roll:+.{p}f} deg")
    print(f"  pitch={pitch:+.{p}f} deg")
    print(f"  yaw={yaw:+.{p}f} deg")

    print("\nworkstation_scene.yaml replacement")
    print(f"    x: {x:.{p}f}")
    print(f"    y: {y:.{p}f}")
    print(f"    z: {z:.{p}f}")
    print(f"    roll_deg: {roll:.{p}f}")
    print(f"    pitch_deg: {pitch:.{p}f}")
    print(f"    yaw_deg: {yaw:.{p}f}")

    print("\nrod_lifecycle.yaml replacement")
    print(f"    rod_x: {x:.{p}f}")
    print(f"    rod_y: {y:.{p}f}")
    print(f"    rod_z: {z:.{p}f}")
    print(f"    rod_roll_deg: {roll:.{p}f}")
    print(f"    rod_pitch_deg: {pitch:.{p}f}")
    print(f"    rod_yaw_deg: {yaw:.{p}f}")

    if abs(abs(pitch) - 90.0) < 5.0:
        print(
            "\nNOTE: The result is near an RPY singularity. Large-looking "
            "roll/yaw changes can represent a small physical rotation. Copy "
            "all three angles together and do not tune them independently."
        )


if __name__ == "__main__":
    main()
