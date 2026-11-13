"""Run: python3 -m unittest discover -s ros2_slam/analysis -p 'test_*.py' -v

The C++ test compiles the production configure(), helper, and Karto accessors
against a minimal Node test double. It is not a ROS/Jetson integration build.
"""
import ast
import contextlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "local_nodes"))
from matcher_profiles import BASELINE, PROFILE_FILES, check_karto_config, expected_parameters
from analyze_experiments import analyze_directory, markdown_report


def log_for(profile):
    return "\n".join("KARTO_CONFIG {}={:.12g}".format(k, v)
                     for k, v in expected_parameters(profile).items())


def scalar_yaml(path):
    # These checked-in files only have scalar values; no external dependency.
    out = {}
    for line in path.read_text().splitlines():
        match = re.match(r"\s+(\w+):\s+([^#]+)", line)
        if match:
            raw = match.group(2).strip()
            out[match.group(1)] = {"true": 1.0, "false": 0.0}.get(raw, raw)
    return out


class ProfilesTest(unittest.TestCase):
    def test_profiles_match_effective_defaults_and_only_width_differs(self):
        profiles = {}
        for name, filename in PROFILE_FILES.items():
            values = scalar_yaml(ROOT / "slam_toolbox/config" / filename)
            for key, expected in expected_parameters(name).items():
                self.assertEqual(float(values[key]), expected, key)
            profiles[name] = values
        differing = {key for key in profiles["baseline"]
                     if profiles["baseline"][key] != profiles["narrow"][key]}
        self.assertEqual(differing, {"correlation_search_space_dimension"})

    def test_valid_internal_values_pass(self):
        for profile in PROFILE_FILES:
            actual, errors = check_karto_config(log_for(profile), profile)
            self.assertEqual(errors, [])
            self.assertEqual(len(actual), 28)

    def test_old_binary_or_wrong_profile_fails(self):
        self.assertTrue(check_karto_config("", "baseline")[1])
        self.assertTrue(check_karto_config(log_for("baseline"), "narrow")[1])
        text = log_for("narrow").replace(
            "distance_variance_penalty=0.5", "distance_variance_penalty=0.09")
        self.assertTrue(check_karto_config(text, "narrow")[1])

    def test_nonfinite_conflicting_missing_and_malformed_fail(self):
        for value in ("nan", "inf", "garbage"):
            text = log_for("narrow").replace(
                "minimum_distance_penalty=0.05", "minimum_distance_penalty=" + value)
            self.assertTrue(check_karto_config(text, "narrow")[1])
        self.assertTrue(check_karto_config(log_for("narrow") +
                        "\nKARTO_CONFIG minimum_distance_penalty=0.5", "narrow")[1])
        self.assertTrue(check_karto_config(log_for("narrow").replace(
                        "KARTO_CONFIG minimum_distance_penalty=0.05", ""), "narrow")[1])

    def test_session_guard_uses_internal_log_and_saves_failure(self):
        # Compile only the real guard method, without importing JetBot or ROS.
        tree = ast.parse((ROOT / "local_nodes/mapping_session.py").read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
                   and n.name == "MappingSession")
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == "verify_karto_config")
        module = ast.parse("")
        module.body = [method]
        for text, expected in [(log_for("narrow"), True),
                               (log_for("baseline"), False), ("", False)]:
            ticks = iter([0.0, 0.0, 6.0])
            namespace = {
                "check_karto_config": check_karto_config,
                "expected_parameters": expected_parameters,
                "time": types.SimpleNamespace(monotonic=lambda: next(ticks)),
                "rclpy": types.SimpleNamespace(spin_once=lambda *a, **k: None),
            }
            exec(compile(ast.fix_missing_locations(module), "session_guard", "exec"), namespace)
            with tempfile.TemporaryDirectory() as folder:
                log = Path(folder) / "slam.log"
                log.write_text(text)
                session = types.SimpleNamespace(matcher_profile="narrow",
                    process_log_paths={"slam": str(log)}, preflight_results={})
                with contextlib.redirect_stdout(io.StringIO()):
                    result = namespace["verify_karto_config"](session)
                self.assertEqual(result, expected)
                self.assertEqual(session.preflight_results["karto_config"]["passed"], expected)

    def test_analyzer_rejects_failed_guard_and_separates_profiles(self):
        rows = []
        for profile in PROFILE_FILES:
            for passed in (True, False):
                with tempfile.TemporaryDirectory() as folder:
                    pose = {"x": 0, "y": 0, "yaw_deg": 0}
                    results = {"configuration": {"matcher_profile": profile,
                               "scan_distance_gate_mode": "euclidean_heading"},
                               "runtime_gate_mode": "euclidean_heading",
                               "run_status": "complete",
                               "preflight": {"karto_config": {"passed": passed}},
                               "checkpoints": [{"label": label, "rf2o": pose, "slam": pose}
                                               for label in ("start", "final")]}
                    (Path(folder) / "results.json").write_text(json.dumps(results))
                    row = analyze_directory(folder)
                    self.assertEqual(row["validity"], "valid_complete" if passed
                                     else "exclude_karto_config")
                    rows.append(row)
        report = markdown_report(rows, "test")
        self.assertIn("| euclidean_heading | baseline | rf2o | 1 |", report)
        self.assertIn("| euclidean_heading | narrow | rf2o | 1 |", report)

    def test_analyzer_separates_odometry_modes(self):
        rows = []
        for odom_mode in ("rf2o", "imu_yaw"):
            with tempfile.TemporaryDirectory() as folder:
                pose = {"x": 0, "y": 0, "yaw_deg": 0}
                results = {
                    "configuration": {
                        "matcher_profile": "narrow",
                        "scan_distance_gate_mode": "euclidean_heading",
                        "odom_mode": odom_mode,
                    },
                    "runtime_gate_mode": "euclidean_heading",
                    "run_status": "complete",
                    "preflight": {
                        "karto_config": {"passed": True},
                        "odom_source": {"passed": True},
                    },
                    "checkpoints": [
                        {"label": label, "rf2o": pose, "slam": pose}
                        for label in ("start", "final")
                    ],
                }
                (Path(folder) / "results.json").write_text(json.dumps(results))
                rows.append(analyze_directory(folder))
        report = markdown_report(rows, "test")
        self.assertIn("| euclidean_heading | narrow | imu_yaw | 1 |", report)
        self.assertIn("| euclidean_heading | narrow | rf2o | 1 |", report)

    def test_python36_syntax(self):
        # Newer Python can validate against the Jetson's Python 3.6 grammar.
        if sys.version_info >= (3, 8):
            for filename in ("mapping_session.py", "matcher_profiles.py"):
                ast.parse((ROOT / "local_nodes" / filename).read_text(), feature_version=(3, 6))
            ast.parse((ROOT / "slam_toolbox/launch/online_sync_launch.py").read_text(),
                      feature_version=(3, 6))

    @unittest.skipUnless(shutil.which("bash"), "bash required for deployment test")
    def test_deploy_copies_repair_profiles_helper_and_preserves_backup(self):
        script = (ROOT / "deploy_scan_gate_diagnostics.sh").read_text()
        block = script.split("FILES=(", 1)[1].split(")", 1)[0]
        files = re.findall(r'"([^"]+)"', block)
        self.assertIn("slam_toolbox/src/slam_mapper.cpp", files)
        self.assertIn("local_nodes/matcher_profiles.py", files)
        self.assertIn("slam_toolbox/config/mapper_params_online_sync_narrow.yaml", files)
        with tempfile.TemporaryDirectory() as folder:
            staging = Path(folder) / "repo"
            workspace = Path(folder) / "workspace"
            (workspace / "src/slam_toolbox").mkdir(parents=True)
            (workspace / "local_nodes").mkdir()
            old_mapper = workspace / "src/slam_toolbox/src/slam_mapper.cpp"
            old_mapper.parent.mkdir()
            old_mapper.write_text("previous mapper source")
            for filename in files:
                destination = staging / filename
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(str(ROOT / filename), str(destination))
            deployed_script = staging / "deploy_scan_gate_diagnostics.sh"
            # Bash fixtures need Unix line endings and paths on Windows too.
            with deployed_script.open("w", newline="\n") as stream:
                stream.write(script)
            env = dict(os.environ, AUTOCHAIR_WS=workspace.as_posix())
            bash = "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
            subprocess.run([bash, deployed_script.as_posix()], env=env, check=True,
                           stdout=subprocess.PIPE)
            for filename in files:
                target = workspace / filename if filename.startswith("local_nodes/") else workspace / "src" / filename
                self.assertEqual(target.read_bytes(), (ROOT / filename).read_bytes())
            backups = list((workspace / "backups").glob("*/slam_toolbox/src/slam_mapper.cpp"))
            self.assertEqual(len(backups), 1)
            self.assertEqual(backups[0].read_text(), "previous mapper source")


@unittest.skipUnless(shutil.which("g++"), "g++ required for C++ loading regression")
class CppLoadingTest(unittest.TestCase):
    def test_real_configure_and_accessors_with_ros_parameter_semantics(self):
        source = (ROOT / "slam_toolbox/src/slam_mapper.cpp").read_text()
        helper = source[source.index("template<typename T>"):
                        source.index("}  // namespace")]
        configure = source[source.index("void SMapper::configure"):
                           source.index("void SMapper::Reset")]
        # Strip the trailing separator comment, preserving the complete function.
        configure = configure[:configure.rfind("/*****")]
        implementation = (ROOT / "slam_toolbox/lib/karto_sdk/src/Mapper.cpp").read_text()
        accessors = re.findall(
            r"(?:bool|int|double|void) Mapper::(?:get|set)Param\w+\([^)]*\)\s*\{[^}]*\}",
            implementation)
        accessor_text = "\n".join(accessors)
        members = sorted(set(re.findall(r"m_p\w+(?=->)", accessor_text)))
        inline_accessors = accessor_text.replace("Mapper::", "")
        code = r'''
#include <map>
#include <string>
#include <memory>
#include <stdexcept>
#include <cmath>
#include <cstdio>
#include <cassert>
using kt_double = double;
using kt_int32u = unsigned int;
using kt_int32s = int;
using kt_bool = bool;
namespace rclcpp {
struct Node {
  using SharedPtr = std::shared_ptr<Node>;
  std::map<std::string, double> declared, overrides;
  bool has_parameter(const std::string &n) { return declared.count(n); }
  template<class T> T declare_parameter(const std::string &n, const T &d) {
    if (has_parameter(n)) throw std::runtime_error("duplicate declaration");
    declared[n] = overrides.count(n) ? overrides[n] : double(d);
    return T(declared[n]);
  }
  template<class T> bool get_parameter(const std::string &n, T &v) {
    if (!has_parameter(n)) return false;
    v = T(declared[n]); return true;
  }
  int get_logger() { return 0; }
};
}
#define RCLCPP_INFO(logger, fmt, ...) std::printf(fmt "\n", __VA_ARGS__)
namespace karto {
namespace math {
template<class T> T Square(T x) { return x*x; }
double RadiansToDegrees(double x) { return x*180.0/3.141592653589793; }
double DegreesToRadians(double x) { return x*3.141592653589793/180.0; }
}
struct Cell {
  double value = 0;
  double GetValue() { return value; }
  void SetValue(double x) { value = x; }
};
struct Mapper {
'''
        code += "\n".join("std::unique_ptr<Cell> " + m + "{new Cell};" for m in members)
        code += "\n" + inline_accessors + "\n};\n}\n"
        code += helper
        code += '''
struct SMapper {
  std::unique_ptr<karto::Mapper> mapper_{new karto::Mapper};
  void configure(const rclcpp::Node::SharedPtr &node);
};
'''
        code += configure
        code += '''
int main(int argc, char **argv) {
  auto node = std::make_shared<rclcpp::Node>();
  std::string mode = argc > 1 ? argv[1] : "baseline";
  if (mode == "narrow") node->overrides["correlation_search_space_dimension"] = 0.3;
  if (mode == "edge") {
    node->declared["correlation_search_space_dimension"] = 0.3;
    node->overrides["distance_variance_penalty"] = 0.09;
    node->overrides["use_scan_matching"] = 0;
    node->overrides["do_loop_closing"] = 0;
    node->overrides["link_match_minimum_response_fine"] = 0;
  }
  SMapper mapper;
  mapper.configure(node);
  if (mode == "edge") {
    assert(mapper.mapper_->getParamCorrelationSearchSpaceDimension() == 0.3);
    assert(std::abs(mapper.mapper_->getParamDistanceVariancePenalty() - 0.09) < 1e-10);
    assert(!mapper.mapper_->getParamUseScanMatching());
    assert(!mapper.mapper_->getParamDoLoopClosing());
    assert(mapper.mapper_->getParamLinkMatchMinimumResponseFine() == 0);
  }
}
'''
        with tempfile.TemporaryDirectory() as folder:
            src = Path(folder) / "test.cpp"
            exe = Path(folder) / "test"
            src.write_text(code)
            subprocess.run(["g++", "-std=c++14", "-Wall", "-Wextra", "-Werror",
                            str(src), "-o", str(exe)], check=True)
            for profile in PROFILE_FILES:
                output = subprocess.check_output([str(exe), profile]).decode()
                self.assertEqual(check_karto_config(output, profile)[1], [])
            subprocess.run([str(exe), "edge"], check=True, stdout=subprocess.PIPE)


if __name__ == "__main__":
    unittest.main()
