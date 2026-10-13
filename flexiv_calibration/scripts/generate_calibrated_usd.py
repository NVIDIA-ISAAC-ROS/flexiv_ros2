#!/usr/bin/env python3
"""Reproduce the uploaded calibrated Rizon4s USD from the public USD + URDF.

Requires OpenUSD: python -m pip install usd-core==26.8
Only arm physics:localPos0 and physics:localRot0 are patched. The public USD's
saved poses, fixed joints, gripper, meshes and dynamics are retained as authored.
An optional --verify-against file is read only for comparison, never generation.
"""
import argparse
import filecmp
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

from pxr import Gf, Usd, UsdGeom, UsdPhysics

ARM = [f'joint{i}' for i in range(1, 8)]
ROOT = '/Rizon4s'
# Keep the original float32 quaternion representation when differences are just
# rounding (e.g. joint1: +/-pi represented by the source importer). This default
# is part of the reproduction algorithm and is reported with the output.
ROTATION_COMPONENT_EPSILON = 1e-7


def require(value, message):
    if not value:
        raise ValueError(message)


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def vector(text):
    values = tuple(map(float, text.split()))
    require(len(values) == 3 and all(map(math.isfinite, values)), 'Expected three finite XYZ/RPY values')
    return Gf.Vec3d(*values)


def rpy_quat(rpy):
    r, p, y = (float(v) / 2 for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    # URDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll), scalar-first quaternion.
    return Gf.Quatd(cr*cp*cy + sr*sp*sy, Gf.Vec3d(
        sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy)).GetNormalized()


def components(q):
    return (q.GetReal(), *q.GetImaginary())


def matrix(pos, quat):
    return Gf.Matrix4d(1).SetRotate(Gf.Quatd(quat)).SetTranslateOnly(Gf.Vec3d(pos))


def check_fixed_joint(stage, node):
    origin = node.find('origin')
    expected = matrix(vector(origin.get('xyz', '0 0 0')), rpy_quat(vector(origin.get('rpy', '0 0 0'))))
    if node.get('name') == 'base_joint':
        actual = Gf.Matrix4d(1)
        require(node.find('parent').get('link') == 'world' and node.find('child').get('link') == 'base_link', 'Unexpected base joint')
    else:
        require(node.find('parent').get('link') == 'link7' and node.find('child').get('link') == 'flange', 'Unexpected flange joint')
        p = stage.GetPrimAtPath(ROOT + '/joints/link7_to_flange')
        require(p.IsA(UsdPhysics.FixedJoint), 'Missing fixed flange joint')
        frames = [matrix(p.GetAttribute(f'physics:localPos{k}').Get(), p.GetAttribute(f'physics:localRot{k}').Get()) for k in (0,1)]
        actual = frames[1].GetInverse() * frames[0]
    require(max(abs(actual[i][j]-expected[i][j]) for i in range(4) for j in range(4)) < 1e-6,
            f"URDF changes {node.get('name')}; this reproduction tool preserves fixed joints")


def patch(stage, urdf):
    root = ET.parse(urdf).getroot()
    require(root.tag == 'robot', 'Input must be a URDF robot')
    nodes = root.findall('joint')
    joints = {n.get('name'): n for n in nodes}
    require(len(nodes) == len(joints) and set(joints) == set(ARM + ['base_joint', 'link7_to_flange']),
            'Expected base_joint, joint1..7, and link7_to_flange only')
    require(str(stage.GetDefaultPrim().GetPath()) == ROOT, 'Expected /Rizon4s default prim')
    require(UsdGeom.GetStageMetersPerUnit(stage) == 1 and UsdGeom.GetStageUpAxis(stage) == 'Z', 'Expected meter units and Z-up')
    # This tool targets the single-layer public asset, including internal mesh
    # instances. Reject external layers to avoid silently patching an assembly.
    require(len([l for l in stage.GetUsedLayers() if l != stage.GetSessionLayer()]) == 1,
            'Expected a single-layer public reference USD')
    for name in ('base_joint', 'link7_to_flange'):
        require(joints[name].get('type') == 'fixed', 'Expected fixed joint')
        check_fixed_joint(stage, joints[name])
    changes = []
    for i, name in enumerate(ARM):
        node = joints[name]
        require(node.get('type') == 'revolute', f'{name} must be revolute')
        parent, child = ('base_link' if i == 0 else f'link{i}'), f'link{i+1}'
        require(node.find('parent').get('link') == parent and node.find('child').get('link') == child,
                f'Unexpected URDF topology at {name}')
        p = stage.GetPrimAtPath(f'{ROOT}/joints/{name}')
        require(p.IsA(UsdPhysics.RevoluteJoint), f'Missing revolute joint {name}')
        for k, body in enumerate((parent, child)):
            require([str(x) for x in p.GetRelationship(f'physics:body{k}').GetTargets()] == [f'{ROOT}/{body}'],
                    f'Unexpected USD topology at {name}')
        axis = vector(node.find('axis').get('xyz'))
        token = p.GetAttribute('physics:axis').Get()
        expected_axis = {'X': Gf.Vec3d(1,0,0), 'Y': Gf.Vec3d(0,1,0), 'Z': Gf.Vec3d(0,0,1)}[token]
        require((axis - expected_axis).GetLength() < 1e-12, f'URDF axis differs for {name}')
        require(p.GetAttribute('physics:localPos1').Get() == Gf.Vec3f(0) and
                p.GetAttribute('physics:localRot1').Get() == Gf.Quatf(1), f'Unexpected child joint frame for {name}')
        for attr, limit in [('lowerLimit', 'lower'), ('upperLimit', 'upper')]:
            angle = float(node.find('limit').get(limit))
            require(math.isfinite(angle) and abs(p.GetAttribute('physics:'+attr).Get()-math.degrees(angle)) < 0.001,
                    f'URDF joint limits differ for {name}')
        origin = node.find('origin')
        q = Gf.Quatf(rpy_quat(vector(origin.get('rpy', '0 0 0'))))
        a = p.GetAttribute('physics:localRot0')
        require(not a.GetNumTimeSamples(), 'Animated joint frames are unsupported')
        old = a.Get()
        # q and -q represent the same orientation; retain the source hemisphere.
        if Gf.Dot(old, q) < 0:
            q = -q
        if max(abs(x-y) for x,y in zip(components(old), components(q))) > ROTATION_COMPONENT_EPSILON:
            a.Set(q)
            changes.append(str(a.GetPath()))
        a = p.GetAttribute('physics:localPos0')
        require(not a.GetNumTimeSamples(), 'Animated joint frames are unsupported')
        pos = Gf.Vec3f(vector(origin.get('xyz', '0 0 0')))
        if a.Get() != pos:
            a.Set(pos)
            changes.append(str(a.GetPath()))
    return changes


def generate(reference, urdf, output, verify_against=None, force=False):
    reference, urdf, output = [Path(p).expanduser().resolve() for p in (reference, urdf, output)]
    verification = Path(verify_against).expanduser().resolve() if verify_against else None
    require(output not in (reference, urdf, verification), 'Output must not overwrite any input')
    require(output.suffix == '.usd', 'Use a .usd output to preserve the source crate format')
    require(force or not output.exists(), 'Output exists; choose another name or pass --force')
    with open(reference, 'rb') as f:
        require(f.read(8) == b'PXR-USDC', 'Reference must be a binary USD crate')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.calibration-', dir=output.parent) as tmp:
        candidate = Path(tmp) / 'calibrated.usd'
        # Copy the original crate, edit, then Save in place. Export/flatten would
        # rewrite its binary layout and defeat byte-for-byte reproduction.
        shutil.copyfile(reference, candidate)
        stage = Usd.Stage.Open(str(candidate))
        require(bool(stage), 'Unable to open reference')
        changes = patch(stage, urdf)
        require(stage.GetRootLayer().Save(), 'USD save failed')
        report = dict(reference_sha256=sha256(reference), urdf_sha256=sha256(urdf),
                      output_sha256=sha256(candidate), bytes=candidate.stat().st_size,
                      usd_version=list(Usd.GetVersion()), changed_properties=changes,
                      rotation_component_epsilon=ROTATION_COMPONENT_EPSILON)
        if verification:
            # This file contributes no values to the generated asset.
            report['target_sha256'] = sha256(verification)
            report['bitwise_equal_to_target'] = filecmp.cmp(candidate, verification, shallow=False)
            require(report['bitwise_equal_to_target'], 'Generated USD is not bitwise equal to verification target; output was not published')
        if force:
            os.replace(candidate, output)
        else:
            os.link(candidate, output)  # Atomic, and refuses to clobber existing output.
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--reference-usd', required=True, type=Path)
    p.add_argument('--urdf', required=True, type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--verify-against', type=Path, help='Optional byte-comparison target; never used to generate the USD')
    p.add_argument('--force', action='store_true')
    a = p.parse_args()
    try:
        result = generate(a.reference_usd, a.urdf, a.output, a.verify_against, a.force)
    except (ValueError, OSError, RuntimeError, ET.ParseError, AttributeError, KeyError, TypeError) as e:
        p.exit(1, f'Error: {e}\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
