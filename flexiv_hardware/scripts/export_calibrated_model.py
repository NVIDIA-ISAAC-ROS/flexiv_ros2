#!/usr/bin/env python3
"""Export Flexiv robot calibrated kinematics URDF for use with isaac_ros_manipulation.

Connects to the robot, calls Model.SyncURDF() to write the factory-calibrated
joint origins into a copy of the nominal kinematics URDF, then saves the
result to $ISAAC_ROS_WS/isaac_ros_assets/<robot-sn>/.

The output file (<robot-sn>_calibrated_kinematics.urdf) is consumed by the
isaac_ros_manipulation launch pipeline (FlexivDriverUtils.apply_real_cumotion_urdf_prefix):
its calibrated joint origins are spliced into the full cuMotion URDF (arm + gripper)
and the result is prefixed with <robot-sn>_ before being passed to cuMotion.
The exported file is NOT fed to cuMotion directly.

Run once per robot. Re-run only after Flexiv performs a recalibration.

Prerequisites:
  - flexivrdk Python bindings installed (built from flexiv_hardware/rdk)
  - Robot powered on and reachable at its serial-number hostname
  - $ISAAC_ROS_WS set (or pass --output-dir explicitly)

Example:
  python3 export_calibrated_model.py --robot-sn Rizon4s-063459
"""

import argparse
import csv
import json
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


JOINT_NAMES = [f"joint{i}" for i in range(1, 8)] + ["link7_to_flange"]

# Nominal kinematics URDF shipped with the RDK, relative to this script.
_SCRIPT_DIR = Path(__file__).resolve().parent
_RDK_TEMPLATE = _SCRIPT_DIR.parent / "rdk" / "resources" / "flexiv_Rizon4s_kinematics.urdf"


def float_list(value):
    if value is None:
        return []
    return [float(item) for item in value.split()]


def parse_urdf_kinematics(path):
    root = ET.parse(path).getroot()
    joints = {}
    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        if name not in JOINT_NAMES:
            continue
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        joints[name] = {
            "xyz": float_list(origin.attrib.get("xyz")) if origin is not None else [],
            "rpy": float_list(origin.attrib.get("rpy")) if origin is not None else [],
            "axis": float_list(axis.attrib.get("xyz")) if axis is not None else [],
            "limit": {k: float(v) for k, v in limit.attrib.items()} if limit is not None else {},
        }
    return joints


def vector_delta(after, before):
    return [a - b for a, b in zip(after or [], before or [])]


def make_delta(before, after):
    delta = {}
    max_abs_xyz = 0.0
    max_abs_rpy = 0.0
    for name in JOINT_NAMES:
        if name not in before or name not in after:
            continue
        xyz_delta = vector_delta(after[name].get("xyz"), before[name].get("xyz"))
        rpy_delta = vector_delta(after[name].get("rpy"), before[name].get("rpy"))
        max_abs_xyz = max(max_abs_xyz, *(abs(x) for x in xyz_delta), 0.0)
        max_abs_rpy = max(max_abs_rpy, *(abs(x) for x in rpy_delta), 0.0)
        delta[name] = {
            "xyz_delta_m": xyz_delta,
            "rpy_delta_rad": rpy_delta,
        }
    return {
        "per_joint": delta,
        "max_abs_xyz_delta_m": max_abs_xyz,
        "max_abs_rpy_delta_rad": max_abs_rpy,
    }


def robot_info_to_dict(info):
    fields = [
        "serial_num",
        "software_ver",
        "model_name",
        "license_type",
        "DoF_e",
        "DoF_m",
        "DoF",
        "K_x_nom",
        "K_q_nom",
        "q_min",
        "q_max",
        "dq_max",
        "tau_max",
        "has_FT_sensor",
    ]
    out = {}
    for field in fields:
        if hasattr(info, field):
            value = getattr(info, field)
            if isinstance(value, tuple):
                value = list(value)
            out[field] = value
    return out


def write_joint_limits_csv(path, info):
    q_min = list(getattr(info, "q_min", []))
    q_max = list(getattr(info, "q_max", []))
    dq_max = list(getattr(info, "dq_max", []))
    tau_max = list(getattr(info, "tau_max", []))
    n = max(len(q_min), len(q_max), len(dq_max), len(tau_max), 7)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["joint", "q_min_rad", "q_max_rad", "dq_max_rad_s", "tau_max_nm"])
        for i in range(n):
            writer.writerow(
                [
                    f"joint{i + 1}",
                    q_min[i] if i < len(q_min) else "",
                    q_max[i] if i < len(q_max) else "",
                    dq_max[i] if i < len(dq_max) else "",
                    tau_max[i] if i < len(tau_max) else "",
                ]
            )


def find_template_urdf(args):
    if args.template_urdf:
        path = Path(args.template_urdf).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"Template URDF does not exist: {path}")
        return path

    candidates = [
        # Preferred: RDK resources bundled alongside this script in the repo
        _RDK_TEMPLATE,
        # Installed flexiv_description (available after colcon build)
        Path(f"/opt/ros/jazzy/share/flexiv_description/urdf/{args.robot_sn}.urdf"),
        Path("/opt/ros/jazzy/share/flexiv_description/urdf/Rizon4s.urdf"),
    ]
    for path in candidates:
        if path.exists():
            return path
    raise FileNotFoundError(
        "Could not find a template URDF. Pass --template-urdf explicitly, "
        "or ensure the RDK submodule is initialised (git submodule update --init --recursive)."
    )


def default_output_dir():
    isaac_ros_ws = os.environ.get("ISAAC_ROS_WS")
    if isaac_ros_ws:
        return str(Path(isaac_ros_ws) / "isaac_ros_assets")
    return None


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Export Flexiv calibrated kinematics URDF for use with isaac_ros_manipulation. "
            "Output goes to <output-dir>/<robot-sn>/."
        )
    )
    parser.add_argument("--robot-sn", required=True, help="Robot serial number, e.g. Rizon4s-063459")
    parser.add_argument("--template-urdf", default=None, help="Override the nominal kinematics URDF template")
    parser.add_argument(
        "--output-dir",
        default=default_output_dir(),
        help=(
            "Root assets directory. Defaults to $ISAAC_ROS_WS/isaac_ros_assets. "
            "The calibrated model is written to <output-dir>/<robot-sn>/."
        ),
    )
    parser.add_argument("--yes", action="store_true", help="Skip the connect confirmation prompt.")
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.output_dir:
        print(
            "ERROR: --output-dir not set and $ISAAC_ROS_WS is not in the environment.\n"
            "Set ISAAC_ROS_WS to your isaac_ros workspace root or pass --output-dir explicitly.",
            file=sys.stderr,
        )
        return 1

    template_urdf = find_template_urdf(args)
    output_dir = Path(args.output_dir).expanduser() / args.robot_sn
    output_dir.mkdir(parents=True, exist_ok=True)

    nominal_urdf = output_dir / "template_before_sync.urdf"
    calibrated_urdf = output_dir / f"{args.robot_sn}_calibrated_kinematics.urdf"
    shutil.copy2(template_urdf, nominal_urdf)
    shutil.copy2(template_urdf, calibrated_urdf)

    print("This will connect to the Flexiv robot and read/sync model data.")
    print("It will not enable the robot or send motion commands.")
    print(f"Robot:    {args.robot_sn}")
    print(f"Template: {template_urdf}")
    print(f"Output:   {output_dir}")
    if not args.yes:
        answer = input("Proceed? [y/N]: ").strip().lower()
        if answer not in ("y", "yes"):
            print("Cancelled.")
            return 1

    try:
        import flexivrdk

        robot = flexivrdk.Robot(args.robot_sn)
        info = robot.info()
        model = flexivrdk.Model(robot)
        model.SyncURDF(str(calibrated_urdf))
    except Exception as exc:
        print(f"[Flexiv export] ERROR: {exc}", file=sys.stderr)
        return 1

    before = parse_urdf_kinematics(nominal_urdf)
    after = parse_urdf_kinematics(calibrated_urdf)
    delta = make_delta(before, after)

    (output_dir / "robot_info.json").write_text(json.dumps(robot_info_to_dict(info), indent=2))
    (output_dir / "kinematics_before_sync.json").write_text(json.dumps(before, indent=2))
    (output_dir / "kinematics_calibrated.json").write_text(json.dumps(after, indent=2))
    (output_dir / "kinematics_delta.json").write_text(json.dumps(delta, indent=2))
    write_joint_limits_csv(output_dir / "joint_limits.csv", info)

    print("\nExport complete.")
    print(f"Calibrated URDF: {calibrated_urdf}")
    print(f"Robot info:      {output_dir / 'robot_info.json'}")
    print(f"Joint limits:    {output_dir / 'joint_limits.csv'}")
    print(f"Kinematics JSON: {output_dir / 'kinematics_calibrated.json'}")
    print(f"Max xyz delta:   {delta['max_abs_xyz_delta_m']:.9g} m")
    print(f"Max rpy delta:   {delta['max_abs_rpy_delta_rad']:.9g} rad")
    print()
    print("The isaac_ros_manipulation launch pipeline will merge this file's calibrated")
    print("joint origins into the full cuMotion URDF at launch time. No further action needed.")
    print("Verify the path matches cumotion_calibrated_kinematics_urdf in your workflow params:")
    print(f"  {calibrated_urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
