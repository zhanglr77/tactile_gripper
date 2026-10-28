"""Export configured magnet reference-point poses from an Abaqus ODB.

Run with Abaqus Python, for example:
    abaqus python export_magnet_poses.py model.odb poses.csv --config magnets.json
"""

from __future__ import print_function

import argparse
import csv
import json
import os
import sys

from odbAccess import openOdb

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def one_field_value(frame, field_name, node_set, description):
    if field_name not in frame.fieldOutputs.keys():
        raise ValueError("Frame %s is missing field %s" %
                         (str(frame.frameValue), field_name))
    field = frame.fieldOutputs[field_name]
    values = field.getSubset(region=node_set).values
    if len(values) != 1:
        raise ValueError("%s must have one %s value; found %d" %
                         (description, field_name, len(values)))
    return values[0].data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("odb", help="input Abaqus ODB")
    parser.add_argument("output", nargs="?", default="mag_poses.csv")
    parser.add_argument("--config", default=os.path.join(PROJECT_DIR, "config", "magnets.json"),
                        help="magnet configuration (default: project config/magnets.json)")
    parser.add_argument("--step", action="append", dest="steps",
                        help="step to export; repeat to select several (default: all)")
    args = parser.parse_args()

    with open(args.config, "r", encoding="utf-8-sig") as stream:
        config = json.load(stream)
    magnets = config.get("magnets", [])
    if not magnets:
        raise ValueError("No magnets are defined in " + args.config)

    odb = openOdb(path=os.path.abspath(args.odb), readOnly=True)
    try:
        selected_steps = args.steps or list(odb.steps.keys())
        missing_steps = [name for name in selected_steps if name not in odb.steps]
        if missing_steps:
            raise ValueError("ODB is missing step(s): " + ", ".join(missing_steps))

        sets = odb.rootAssembly.nodeSets
        mag_sets = {}
        for magnet in magnets:
            name = str(magnet["id"])
            set_names = magnet.get("odb_node_set")
            if set_names is None:
                set_names = ["SET-%s-RP" % name.upper(),
                             "SET-%s" % name.upper()]
            elif isinstance(set_names, str):
                set_names = [set_names]
            set_name = next((candidate for candidate in set_names
                             if candidate in sets), None)
            if set_name is None:
                raise ValueError("ODB is missing node set for %s (checked %s)" %
                                 (name, ", ".join(set_names)))
            mag_sets[name] = sets[set_name]

        columns = ["step", "frame", "step_time", "total_time"]
        for magnet in magnets:
            name = str(magnet["id"])
            columns.extend("%s_%s" % (name, axis) for axis in ("x", "y", "z"))
            columns.extend("%s_ur%d_rad" % (name, axis) for axis in (1, 2, 3))

        rows = []
        elapsed_before = 0.0
        for step_name in odb.steps.keys():
            step = odb.steps[step_name]
            if step_name not in selected_steps:
                if step.frames:
                    elapsed_before += float(step.frames[-1].frameValue)
                continue
            if not step.frames:
                raise ValueError("Step %s contains no frames" % step_name)
            for frame_index, frame in enumerate(step.frames):
                step_time = float(frame.frameValue)
                row = [step_name, frame_index, step_time,
                       elapsed_before + step_time]
                for magnet in magnets:
                    name = str(magnet["id"])
                    region = mag_sets[name]
                    u = one_field_value(frame, "U", region, name)
                    ur = one_field_value(frame, "UR", region, name)
                    initial = magnet["position"]
                    row.extend(float(initial[i]) + float(u[i])
                               for i in range(3))
                    row.extend(float(value) for value in ur[:3])
                rows.append(row)
            elapsed_before += float(step.frames[-1].frameValue)
    finally:
        odb.close()

    if not rows:
        raise ValueError("No frames were exported")
    with open(args.output, "w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(columns)
        writer.writerows(rows)
    print("Wrote %d frame poses from %d step(s) to %s" %
          (len(rows), len(selected_steps), args.output))


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("Error: %s" % error, file=sys.stderr)
        sys.exit(1)
