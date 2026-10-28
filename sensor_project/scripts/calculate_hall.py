"""Calculate three-axis Hall fields from Abaqus magnet pose trajectories.

Run from the sensor project root:
    abaqus python scripts/calculate_hall.py poses.csv hall_readings.csv
        --magnets config/magnets.json --hall config/hall_sensors.json
"""

from __future__ import print_function

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np


from magnet_field import finite_cylinder_field

PROJECT_DIR = Path(__file__).resolve().parent.parent

def read_config(path):
    with open(path, "r", encoding="utf-8-sig") as stream:
        return json.load(stream)


def rotate_vector(vector, rotation_vector):
    rotation = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(rotation))
    vector = np.asarray(vector, dtype=float)
    if angle < 1.0e-12:
        return vector.copy()
    axis = rotation / angle
    return (vector * math.cos(angle)
            + np.cross(axis, vector) * math.sin(angle)
            + axis * np.dot(axis, vector) * (1.0 - math.cos(angle)))


def validate_sensor(sensor):
    position = np.asarray(sensor["position"], dtype=float)
    axes = sensor.get("axes_global", {
        "x": [1.0, 0.0, 0.0],
        "y": [0.0, 1.0, 0.0],
        "z": [0.0, 0.0, 1.0],
    })
    rotation = np.column_stack([
        np.asarray(axes[axis], dtype=float) for axis in ("x", "y", "z")
    ])
    if position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("Sensor %s position and axes must be 3D" % sensor["id"])
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-5):
        raise ValueError("Sensor %s axes must be orthonormal" % sensor["id"])
    return position, rotation


def load_pose(row, magnet, line_number):
    name = str(magnet["id"])
    try:
        position = np.array([float(row["%s_%s" % (name, axis)])
                             for axis in ("x", "y", "z")])
        rotation = np.array([float(row["%s_ur%d_rad" % (name, axis)])
                             for axis in (1, 2, 3)])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("CSV line %d: invalid pose for %s: %s" %
                         (line_number, name, error))
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
        raise ValueError("CSV line %d: non-finite pose for %s" %
                         (line_number, name))
    return position, rotation


def output_columns(sensors):
    columns = ["step", "frame", "step_time", "total_time"]
    for sensor in sensors:
        name = str(sensor["id"])
        columns.extend("%s_%s" % (name, axis)
                       for axis in ("Bx_mT", "By_mT", "Bz_mT", "Bmag_mT",
                                    "delta_Bx_mT", "delta_By_mT", "delta_Bz_mT",
                                    "delta_Bmag_mT"))
    return columns


def calculate(input_csv, output_csv, magnet_config, hall_config,
              nr=30, nphi=120):
    geometry = magnet_config.get("magnet_geometry", {})
    radius = float(geometry["diameter"]) / 2.0
    height = float(geometry["height"])
    default_br = float(geometry["Br_mT"])
    magnets = magnet_config["magnets"]
    sensors = hall_config["sensors"]
    sensor_frames = [validate_sensor(sensor) for sensor in sensors]
    baseline = None
    output_rows = []

    with open(input_csv, "r", newline="", encoding="utf-8-sig") as source:
        reader = csv.DictReader(source)
        required = {"step", "frame", "step_time", "total_time"}
        for magnet in magnets:
            name = str(magnet["id"])
            required.update("%s_%s" % (name, axis) for axis in ("x", "y", "z"))
            required.update("%s_ur%d_rad" % (name, axis) for axis in (1, 2, 3))
        missing = sorted(required - set(reader.fieldnames or []))
        if missing:
            raise ValueError("Pose CSV missing columns: " + ", ".join(missing))

        for line_number, row in enumerate(reader, start=2):
            moved_magnets = []
            for magnet in magnets:
                position, rotation = load_pose(row, magnet, line_number)
                direction = rotate_vector(magnet["direction"], rotation)
                direction *= int(magnet.get("magnetization_sign", 1))
                moved_magnets.append((position, direction,
                                      float(magnet.get("Br_mT", default_br))))

            hall_readings = []
            for sensor, (sensor_position, sensor_axes) in zip(sensors, sensor_frames):
                field_global = np.zeros(3, dtype=float)
                for position, direction, remanence in moved_magnets:
                    field_global += finite_cylinder_field(
                        point_global=sensor_position,
                        magnet_position=position,
                        magnet_n=direction,
                        radius=radius,
                        height=height,
                        Br=remanence,
                        nr=nr,
                        nphi=nphi,
                    )
                field_local = sensor_axes.T @ field_global
                hall_readings.append((field_local, float(np.linalg.norm(field_local))))

            if baseline is None:
                baseline = [(field.copy(), magnitude)
                            for field, magnitude in hall_readings]
            result = [row["step"], row["frame"], row["step_time"], row["total_time"]]
            for (field, magnitude), (initial_field, initial_magnitude) in zip(
                    hall_readings, baseline):
                result.extend(["%.2f" % value for value in field])
                result.append("%.2f" % magnitude)
                result.extend("%.2f" % value for value in field - initial_field)
                result.append("%.2f" % (magnitude - initial_magnitude))
            output_rows.append(result)

    if not output_rows:
        raise ValueError("Pose CSV has no rows")
    with open(output_csv, "w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(output_columns(sensors))
        writer.writerows(output_rows)
    return len(output_rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("poses", help="CSV from export_magnet_poses.py")
    parser.add_argument("output", nargs="?", default="hall_readings.csv")
    parser.add_argument("--magnets", default=str(PROJECT_DIR / "config" / "magnets.json"))
    parser.add_argument("--hall", default=str(PROJECT_DIR / "config" / "hall_sensors.json"))
    parser.add_argument("--nr", type=int, default=30,
                        help="radial field integration subdivisions")
    parser.add_argument("--nphi", type=int, default=120,
                        help="angular field integration subdivisions")
    args = parser.parse_args()
    if args.nr < 1 or args.nphi < 1:
        parser.error("--nr and --nphi must be positive")

    magnet_config = read_config(args.magnets)
    hall_config = read_config(args.hall)
    count = calculate(args.poses, args.output, magnet_config, hall_config,
                      nr=args.nr, nphi=args.nphi)
    print("Wrote Hall readings for %d frames to %s" % (count, args.output))
    print("Br = %.3f mT; quadrature = %d x %d" % (
        float(magnet_config["magnet_geometry"]["Br_mT"]), args.nr, args.nphi))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("Error: %s" % error, file=sys.stderr)
        sys.exit(1)
