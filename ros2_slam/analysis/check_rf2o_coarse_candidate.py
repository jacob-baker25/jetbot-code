#!/usr/bin/env python3
"""Offline saved-pair comparison; Python 3.6. No ROS/hardware/deployment.

Compile the real core against the existing minimal ROS stubs. The experimental
change exists only in a temporary source copy. Results qualify a hypothesis,
not a live robot or mapping accuracy. Output directory must be new.
"""
import argparse
import hashlib
import io
import json
import math
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile

from run_rf2o_core_tests import eigen_include_flags


def make_candidate(source):
    changes = [
        ('  unsigned int solved_levels = 0;',
         '  unsigned int solved_levels = 0;\n  bool finest_solved = false;'),
        ('    if (!solveSystemNonLinear()) return reject(failure_reason);',
         '''    if (!solveSystemNonLinear()) {
      // Experimental offline candidate: never solve an unobservable system.
      // Leave this level's identity transform and try the finer scan instead.
      if (std::string(failure_reason) == "rank_deficient_system" && i+1 < ctf_levels) {
        failure_reason = "none";
        continue;
      }
      return reject(failure_reason);
    }'''),
        ('    ++solved_levels;',
         '    ++solved_levels;\n    if (image_level == 0) finest_solved = true;'),
        ('  if (!solved_levels) return reject("insufficient_returns");',
         '  if (!solved_levels) return reject("insufficient_returns");\n'
         '  if (!finest_solved) return reject("finest_level_unsolved");')]
    for old, new in changes:
        if source.count(old) != 1:
            raise ValueError('Unexpected core source; candidate was not generated: ' + old)
        source = source.replace(old, new)
    return source


def validate_pair(pair):
    for name in ('reference', 'candidate'):
        scan = pair[name]
        if len(scan['ranges']) != 360:
            raise ValueError('This recorded-pair fixture requires 360 bins')
        if not isinstance(scan['sec'], int) or not isinstance(scan['nanosec'], int):
            raise ValueError('Noninteger stamp')
        if not 0 <= scan['nanosec'] < 1000000000:
            raise ValueError('Invalid nanoseconds')
        if not math.isfinite(scan['angle_increment']) or scan['angle_increment'] <= 0:
            raise ValueError('Invalid increment')
        if any(v is not None and (not isinstance(v, (int, float)) or
                                  not math.isfinite(v) or v <= 0) for v in scan['ranges']):
            raise ValueError('Unexpected finite range')
    prior = pair['motion_prior']
    if len(prior) != 3 or any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in prior):
        raise ValueError('Invalid motion prior')


def load_pairs(bundle):
    pairs = []

    def visit(archive, depth):
        for member in archive.getmembers():
            if not member.isfile():
                continue
            if member.name.endswith('/rf2o.log'):
                if member.size > 8*1024**2:
                    raise ValueError('RF2O log exceeds diagnostic bound')
                content = archive.extractfile(member).read().decode('utf-8')
                for line in content.splitlines():
                    if 'RF2O_REJECT_INPUT ' in line:
                        pair = json.loads(line.split('RF2O_REJECT_INPUT ', 1)[1])
                        validate_pair(pair)
                        pairs.append({'log': member.name, 'input': pair})
            elif depth == 0 and member.name.endswith('_bundle.tar.gz'):
                if member.size > 32*1024**2:
                    raise ValueError('Nested bundle exceeds diagnostic bound')
                with tarfile.open(fileobj=io.BytesIO(archive.extractfile(member).read()), mode='r:gz') as nested:
                    visit(nested, 1)
    with tarfile.open(str(bundle), 'r:gz') as archive:
        visit(archive, 0)
    if not pairs or len(pairs) > 30:
        raise ValueError('Expected 1..30 saved rejected pairs')
    return pairs


def pair_text(pair):
    validate_pair(pair)
    values = []
    for name in ('reference', 'candidate'):
        scan = pair[name]
        values.extend([str(scan['sec']), str(scan['nanosec']), str(scan['angle_increment'])])
        values.extend('inf' if v is None else str(v) for v in scan['ranges'])
    values.extend(str(v) for v in pair['motion_prior'])
    return ' '.join(values) + '\n'


HARNESS = r'''
#include "rf2o_laser_odometry/CLaserOdometry2D.hpp"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <stdexcept>
sensor_msgs::msg::LaserScan read_scan(std::istream &in) {
  sensor_msgs::msg::LaserScan scan;
  if (!(in >> scan.header.stamp.sec >> scan.header.stamp.nanosec >> scan.angle_increment))
    throw std::runtime_error("scan header read failed");
  for (int i=0;i<360;++i) {
    std::string value;
    if (!(in >> value)) throw std::runtime_error("range read failed");
    scan.ranges.push_back(std::stof(value));
  }
  return scan;
}
int main(int argc,char **argv) {
  try {
    for (int i=1;i<argc;++i) {
      std::ifstream input(argv[i]);
      auto reference=read_scan(input), candidate=read_scan(input);
      rf2o::MatrixS31 prior;
      for (int j=0;j<3;++j)
        if (!(input >> prior(j))) throw std::runtime_error("prior read failed");
      rf2o::CLaserOdometry2D core;
      geometry_msgs::msg::Pose pose; pose.orientation.w=1;
      core.suppress_small_motion=false;
      core.init(reference,pose); core.kai_loc_old_=prior;
      bool accepted=core.odometryCalculation(candidate);
      double translation=core.getPose().translation().norm();
      double yaw=rf2o::getYaw(core.getPose().rotation());
      if (!std::isfinite(translation) || !std::isfinite(yaw))
        throw std::runtime_error("nonfinite replay pose");
      std::cout << std::setprecision(12)
        << "{\"pair\":" << i-1 << ",\"accepted\":" << (accepted?"true":"false")
        << ",\"reason\":\"" << core.failure_reason << "\",\"level\":" << core.level
        << ",\"columns\":" << core.cols_i << ",\"valid_returns\":" << core.num_valid_range
        << ",\"translation_m\":" << translation << ",\"yaw_rad\":" << yaw << "}\n";
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
'''


def execute(command, log, timeout=600):
    with log.open('w', encoding='utf-8') as stream:
        stream.write('COMMAND ' + json.dumps(command) + '\n'); stream.flush()
        subprocess.run(command, stdout=stream, stderr=subprocess.STDOUT,
                       check=True, timeout=timeout)


def compare(bundle, output, compiler, eigen_parent):
    here = Path(__file__).resolve().parent
    core = here.parent/'rf2o_laser_odometry'
    source = (here/'fixtures/rf2o_coarse_baseline.cpp').read_text()
    candidate = make_candidate(source)
    pairs = load_pairs(bundle)
    report = {'status': 'running', 'hardware_started': False, 'installed_files_modified': False,
              'archive_sha256': hashlib.sha256(bundle.read_bytes()).hexdigest(),
              'source_sha256': hashlib.sha256(source.encode()).hexdigest(),
              'candidate_sha256': hashlib.sha256(candidate.encode()).hexdigest(),
              'pairs': [{'log': p['log'], 'recorded_reason': p['input']['reason']} for p in pairs],
              'variants': {},
              'interpretation': 'Pair replay with restored motion prior; not full original trajectory state. '
                                'PASS is not drift, live timing, motion or map qualification.'}
    report_path = output/'report.json'
    def save():
        report_path.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    save()
    try:
        if not shutil.which(compiler):
            raise RuntimeError('Compiler unavailable: '+compiler)
        if not (eigen_parent/'eigen3/Eigen/Core').is_file():
            raise RuntimeError('Eigen headers unavailable: '+str(eigen_parent))
        # Normal /tmp is executable; /dev/shm may be mounted noexec. No package
        # files are changed, and all compiled temporary files are cleaned up.
        with tempfile.TemporaryDirectory(prefix='rf2o_offline_') as temporary:
            temp = Path(temporary)
            fixtures = []
            for i, pair in enumerate(pairs):
                path = temp/('pair_'+str(i)+'.txt'); path.write_text(pair_text(pair['input']))
                fixtures.append(str(path))
            harness = temp/'pairs.cpp'; harness.write_text(HARNESS)
            flags = [compiler, '-std=c++14', '-O1'] + eigen_include_flags(eigen_parent) + [
                '-I'+str(here/'rf2o_test_stubs'), '-I'+str(core/'include')]
            for name, text in [('baseline', source), ('skip_coarse', candidate)]:
                print('Offline '+name+': compiling actual core; robot remains untouched.', flush=True)
                cpp = temp/(name+'.cpp'); cpp.write_text(text)
                obj = temp/(name+'.o')
                execute(flags+['-c', str(cpp), '-o', str(obj)], output/(name+'_compile.log'))
                pair_exe = temp/(name+'_pairs')
                execute(flags+[str(harness), str(obj), '-o', str(pair_exe)], output/(name+'_link.log'))
                execute([str(pair_exe)]+fixtures, output/(name+'_pairs.log'), 60)
                rows = [json.loads(line) for line in (output/(name+'_pairs.log')).read_text().splitlines()
                        if line.startswith('{')]
                if len(rows) != len(pairs):
                    raise RuntimeError('Incomplete pair replay output')
                report['variants'][name] = {'pairs': rows, 'core_regressions_passed': False}
                save()
                tests = temp/(name+'_regressions')
                execute(flags+[str(here/'test_rf2o_core.cpp'), str(obj), '-o', str(tests)],
                        output/(name+'_regression_link.log'))
                execute([str(tests)], output/(name+'_regressions.log'), 60)
                if 'RF2O_CORE_TEST_PASS' not in (output/(name+'_regressions.log')).read_text():
                    raise RuntimeError('Core regression completion marker missing')
                report['variants'][name]['core_regressions_passed'] = True
                save()
        baseline = report['variants']['baseline']['pairs']
        modified = report['variants']['skip_coarse']['pairs']
        report['hypothesis_supported'] = (
            all(not row['accepted'] and row['reason'] == pair['input']['reason']
                for row, pair in zip(baseline, pairs)) and
            all(row['accepted'] and row['reason'] == 'none' for row in modified))
        report['status'] = 'complete'
        save()
        print('Comparison complete. Hypothesis supported: '+str(report['hypothesis_supported']), flush=True)
        return 0
    except Exception as error:
        report.update(status='failed', error=repr(error)); save()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--compiler', default='g++')
    parser.add_argument('--eigen-parent', type=Path, default=Path('/usr/include'))
    args = parser.parse_args()
    if not args.bundle.is_file():
        parser.error('Input bundle missing')
    args.output.mkdir(parents=True, exist_ok=False)
    try:
        return compare(args.bundle, args.output, args.compiler, args.eigen_parent)
    except Exception as error:
        # Preserve a report even for fixture/source validation before compilation.
        if not (args.output/'report.json').exists():
            (args.output/'report.json').write_text(json.dumps({'status':'failed','error':repr(error)},indent=2)+'\n')
        print('Offline check failed: '+str(error), flush=True)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
