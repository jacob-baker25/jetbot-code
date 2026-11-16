#!/usr/bin/env python3
"""Guarded RF2O source deployment; requires a subsequent colcon build. Python 3.6."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import sys


FILES = ['src/CLaserOdometry2DNode.cpp',
         'src/CLaserOdometry2D.cpp',
         'include/rf2o_laser_odometry/CLaserOdometry2D.hpp',
         'include/rf2o_laser_odometry/CLaserOdometry2DNode.hpp',
         'include/rf2o_laser_odometry/TimingDiagnostics.hpp',
         'CMakeLists.txt', 'package.xml']


def digest(path):
    return hashlib.sha256(path.read_bytes().replace(b'\r\n', b'\n')).hexdigest()


def validate(source, target, baseline):
    errors = []
    for rel, old_hash in baseline.items():
        accepted = old_hash if isinstance(old_hash, list) else [old_hash]
        current = target / rel
        if not current.is_file():
            errors.append('Missing installed source: ' + rel)
        elif digest(current) not in accepted + [digest(source / rel)]:
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


def main_scope_fixture(source):
    # Compile the REAL main() at global scope, as in the legacy translation unit.
    # Only the ROS interface is stubbed; the diagnostics header is real. The
    # original header-only test did not exercise these namespace references.
    cpp = (source / 'src/CLaserOdometry2DNode.cpp').read_text()
    signature = 'int main(int argc, char** argv)'
    if cpp.count(signature) != 1:
        raise RuntimeError('Could not isolate RF2O main() for scope regression')
    return '''#include <memory>
#include <iostream>
#include <stdexcept>
#include "rf2o_laser_odometry/TimingDiagnostics.hpp"
namespace rf2o {
class CLaserOdometry2DNode {
public:
  TimingDiagnostics timing;
  double freq=10.;
  void process() {}
};
}
namespace rclcpp {
inline void init(int, char**) {}
inline bool ok() { static int remaining=2; return remaining-- > 0; }
struct Rate { explicit Rate(double) {} void sleep() {} };
template<typename T> void spin_some(const std::shared_ptr<T>&) {}
}
''' + cpp[cpp.index(signature):]


def check_main_scope(source, directory, compiler='c++'):
    fixture = Path(directory) / 'main_scope.cpp'
    executable = str(Path(directory) / 'main_scope_test.exe')
    fixture.write_text(main_scope_fixture(source))
    command = [compiler, '-std=c++14', '-Wall', '-Wextra', '-pedantic',
               '-I' + str(source / 'include'), str(fixture), '-o', executable]
    subprocess.check_call(command)
    output = subprocess.check_output([executable], stderr=subprocess.STDOUT,
                                     universal_newlines=True)
    summary = json.loads(output.split('RF2O_TIMING_SUMMARY ', 1)[1])
    if summary['metrics']['spin']['count'] != 2 or summary['metrics']['rate_sleep']['count'] != 2:
        raise RuntimeError('Main scope execution test failed')
    # Demonstrate that this test detects the exact previously missed defect.
    original = fixture.read_text()
    # Expired-reference errors must return failure AND destroy the node so its
    # shutdown diagnostics survive. An uncaught rethrow may skip destruction.
    failure_case = original.replace('void process() {}',
        'void process() { throw std::runtime_error("test_expiry"); }\n'
        '  ~CLaserOdometry2DNode() { std::cerr << "NODE_DESTROYED"; }')
    fixture.write_text(failure_case)
    subprocess.check_call(command)
    failed = subprocess.run([executable], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, universal_newlines=True)
    if failed.returncode != 1 or 'RF2O_FATAL test_expiry' not in failed.stderr or 'NODE_DESTROYED' not in failed.stderr:
        raise RuntimeError('Fatal exit did not preserve shutdown diagnostics')
    fixture.write_text(original.replace('rf2o::TimingScope', 'TimingScope')
                      .replace('rf2o::TimingDiagnostics', 'TimingDiagnostics'))
    rejected = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if rejected.returncode == 0:
        raise RuntimeError('Regression test unexpectedly accepted unqualified main types')


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
        check_main_scope(source, directory)
    # Exercise the actual Eigen calculation and node process body before copying
    # any sources. Uses the Jetson's already installed compiler/Eigen headers.
    subprocess.check_call([sys.executable, str(root/'analysis/run_rf2o_core_tests.py')])
    print('Diagnostics C++14 test: PASS')
    print('Actual main() scope regression: PASS (ROS interfaces stubbed; colcon build still required)')
    backup = deploy(source, target, args.workspace / 'backups', baseline)
    print('Backup: {}\nBuild RF2O before running the trial. No other package needs rebuilding.'.format(backup))


if __name__ == '__main__':
    main()
