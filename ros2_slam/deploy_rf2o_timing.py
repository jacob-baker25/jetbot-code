#!/usr/bin/env python3
"""Guarded RF2O source deployment; requires a subsequent colcon build. Python 3.6."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile


FILES = ['src/CLaserOdometry2DNode.cpp',
         'include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp',
         'include/rf2o_laser_odometry/TimingDiagnostics.hpp',
         'CMakeLists.txt', 'package.xml']


def digest(path):
    return hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest()


def validate(source, target, baseline):
    errors = []
    for rel, old_hash in baseline.items():
        current = target / rel
        if not current.is_file():
            errors.append('Missing installed source: ' + rel)
        elif digest(current) not in (old_hash, digest(source / rel)):
            errors.append('Unexpected local changes: ' + rel)
    for rel in FILES:
        if not (source / rel).is_file():
            errors.append('Missing candidate source: ' + rel)
        current = target / rel
        if rel not in baseline and current.exists() and digest(current) != digest(source / rel):
            errors.append('Unexpected existing helper: ' + rel)
    if errors:
        raise RuntimeError('\n'.join(errors) + '\nNo installed files changed.')


def deploy(source, target, backup_root, baseline):
    validate(source, target, baseline)
    backup_root.mkdir(parents=True, exist_ok=True)
    # Backups include package.xml. Exclude the entire backup tree from colcon's
    # default recursive workspace discovery before copying any package files.
    (backup_root / 'COLCON_IGNORE').touch(exist_ok=True)
    backup = Path(tempfile.mkdtemp(prefix='rf2o_timing_', dir=str(backup_root)))
    manifest = {'files': {}, 'build_required': True}
    # Finish all backups before replacing any file.
    for rel in FILES:
        current = target / rel
        saved = backup / rel
        if current.exists():
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(str(current), str(saved))
        manifest['files'][rel] = {'previous_sha256': digest(current) if current.exists() else None,
                                  'candidate_sha256': digest(source / rel)}
    (backup / 'deployment.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print('Backup prepared: ' + str(backup), flush=True)
    for rel in FILES:
        current = target / rel
        current.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(str(source / rel), str(current))
        print('Installed ' + str(current))
    return backup


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.home() / 'autochair_ros2_ws')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    source = root / 'rf2o_laser_odometry'
    target = args.workspace / 'src/rf2o_laser_odometry'
    baseline = json.loads((root / 'rf2o_timing_baseline.json').read_text())
    validate(source, target, baseline)
    # Compile and exercise the portable diagnostics before modifying ROS sources.
    with tempfile.TemporaryDirectory(prefix='rf2o_timing_test_') as directory:
        executable = str(Path(directory) / 'test_rf2o_timing')
        subprocess.check_call(['c++', '-std=c++14', '-Wall', '-Wextra', '-pedantic',
            '-I' + str(source / 'include'), str(root / 'analysis/test_rf2o_timing.cpp'),
            '-o', executable])
        output = subprocess.check_output([executable], universal_newlines=True)
        summary = json.loads(output.split('RF2O_TIMING_SUMMARY ', 1)[1])
        if summary['event_count'] != 102 or len(summary['events']) != 64:
            raise RuntimeError('Diagnostics C++ test failed; no source files changed')
    print('Diagnostics C++14 test: PASS')
    backup = deploy(source, target, args.workspace / 'backups', baseline)
    print('Backup: {}\nBuild RF2O before running the trial. No other package needs rebuilding.'.format(backup))


if __name__ == '__main__':
    main()
