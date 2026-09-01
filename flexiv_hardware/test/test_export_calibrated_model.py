#!/usr/bin/env python3

import importlib.util
from pathlib import Path
from types import SimpleNamespace


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_DIR / "scripts" / "export_calibrated_model.py"
SPEC = importlib.util.spec_from_file_location("export_calibrated_model", SCRIPT_PATH)
EXPORTER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORTER)


def test_bundled_template_contains_required_unprefixed_joints():
    template = EXPORTER.find_template_urdf(SimpleNamespace(template_urdf=None))
    kinematics = EXPORTER.parse_urdf_kinematics(template)

    assert set(kinematics) == set(EXPORTER.JOINT_NAMES)
    for joint in kinematics.values():
        assert len(joint["xyz"]) == 3
        assert len(joint["rpy"]) == 3
