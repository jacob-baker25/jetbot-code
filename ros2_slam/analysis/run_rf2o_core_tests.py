#!/usr/bin/env python3
"""Compile the actual RF2O calculation with Eigen and ROS message/clock stubs.

Python 3.6; no ROS installation needed. This does not replace a full ROS build.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


def run(source, directory, eigen_parent=Path('/usr/include'), compiler='c++'):
    here = Path(__file__).resolve().parent
    executable = str(Path(directory) / 'rf2o_core_test')
    subprocess.check_call([compiler, '-std=c++14', '-O1', '-Wall', '-Wextra',
        '-isystem', str(eigen_parent), '-isystem', str(eigen_parent/'eigen3'),
        '-I'+str(here/'rf2o_test_stubs'), '-I'+str(source/'include'),
        str(here/'test_rf2o_core.cpp'), str(source/'src/CLaserOdometry2D.cpp'),
        '-o', executable])
    subprocess.check_call([executable])
    check_process(source, directory, compiler)


def check_process(source, directory, compiler='c++'):
    cpp = (source/'src/CLaserOdometry2DNode.cpp').read_text()
    start = cpp.index('void CLaserOdometry2DNode::process()')
    method = cpp[start:cpp.index('/**', start)]
    fixture = Path(directory)/'rf2o_process_test.cpp'
    fixture.write_text('''#include <stdexcept>
#include <string>
#include <cassert>
#include <vector>
#include "rf2o_laser_odometry/TimingDiagnostics.hpp"
#define RCLCPP_WARN(...) ((void)0)
namespace rf2o {
struct Core {
  bool valid=true;
  unsigned long long rejected_estimates=0;
  const char *failure_reason="invalid";
  int kai_loc_old_=0;
  bool is_initialized() { return true; }
  bool odometryCalculation(int) { if (!valid) ++rejected_estimates; return valid; }
};
struct CLaserOdometry2DNode {
  Core rf2o_ref; TimingDiagnostics timing;
  bool new_scan_available=true;
  int last_scan=0, publications=0;
  int last_valid_scan=0;
  struct Failure { std::string reason; int reference, candidate, prior; };
  std::vector<Failure> failure_inputs;
  bool scan_available() { return new_scan_available; }
  void publish() { ++publications; }
  void process();
};
''' + method + '''
}
int main() {
  rf2o::CLaserOdometry2DNode good;
  good.process(); assert(good.publications==1 && !good.new_scan_available);
  good.process(); assert(good.publications==1);
  rf2o::CLaserOdometry2DNode bad; bad.rf2o_ref.valid=false;
  bad.process(); assert(bad.publications==0 && !bad.new_scan_available);
  assert(bad.failure_inputs.size()==1);
  bad.process(); assert(bad.rf2o_ref.rejected_estimates==1);
  for (int i=0;i<8;++i) { bad.new_scan_available=true; bad.process(); }
  assert(bad.failure_inputs.size()==3 && bad.publications==0);
  rf2o::CLaserOdometry2DNode expired; expired.rf2o_ref.valid=false;
  expired.rf2o_ref.failure_reason="reference_expired";
  bool stopped=false;
  try { expired.process(); } catch (const std::runtime_error &) { stopped=true; }
  assert(stopped && expired.publications==0 && !expired.new_scan_available);
}
''')
    executable = str(Path(directory)/'rf2o_process_test')
    subprocess.check_call([compiler, '-std=c++14', '-I'+str(source/'include'),
                           str(fixture), '-o', executable])
    subprocess.check_call([executable])
    print('RF2O_PROCESS_TEST_PASS valid-only publication and expired-reference stop')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--eigen-parent', type=Path, default=Path('/usr/include'))
    parser.add_argument('--compiler', default='c++')
    args = parser.parse_args()
    source = Path(__file__).resolve().parents[1]/'rf2o_laser_odometry'
    with tempfile.TemporaryDirectory(prefix='rf2o_core_test_') as directory:
        run(source, directory, args.eigen_parent, args.compiler)


if __name__ == '__main__':
    main()
