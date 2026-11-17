import ast
import io
import json
from pathlib import Path
import sys
import tarfile
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_rf2o_coarse_candidate as check


def fixture():
    reference = {'sec': 100, 'nanosec': 0, 'angle_increment': 0.01745329238474369,
                 'ranges': [1.0, None]*180}
    candidate = dict(reference, nanosec=100000000)
    return {'reason':'rank_deficient_system', 'reference':reference,
            'candidate':candidate, 'motion_prior':[0.01, -0.02, 0.003]}


def archive_bytes(name, data):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode='w:gz') as archive:
        info = tarfile.TarInfo(name); info.size=len(data)
        archive.addfile(info, io.BytesIO(data))
    return stream.getvalue()


class OfflineComparisonTests(unittest.TestCase):
    def test_nested_archive_without_extraction(self):
        content = ('RF2O_REJECT_INPUT '+json.dumps(fixture())+'\n').encode()
        nested = archive_bytes('trial/sessions/run/rf2o.log', content)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'input.tar.gz'
            path.write_bytes(archive_bytes('batch/still_bundle.tar.gz', nested))
            pairs = check.load_pairs(path)
            self.assertEqual(pairs[0]['input'], fixture())
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_preserves_missing_ranges_and_signed_prior(self):
        values = check.pair_text(fixture()).split()
        self.assertEqual(values[4], 'inf')
        self.assertEqual(values[-3:], ['0.01','-0.02','0.003'])
        self.assertEqual(len(values), 729)

    def test_rejects_invalid_prior_and_geometry(self):
        for change in ('prior', 'count', 'stamp'):
            pair=fixture()
            if change=='prior': pair['motion_prior'][0]=None
            elif change=='count': pair['candidate']['ranges']=[1.]*359
            else: pair['candidate']['nanosec']=1000000000
            with self.assertRaises(ValueError): check.validate_pair(pair)

    def test_rejects_unknown_source(self):
        with self.assertRaises(ValueError): check.make_candidate('unrelated source')

    def test_only_generates_candidate_from_current_core(self):
        path=Path(__file__).resolve().parent/'fixtures/rf2o_coarse_baseline.cpp'
        before=path.read_bytes(); source=path.read_text()
        result=check.make_candidate(source)
        self.assertNotEqual(result, source)
        self.assertEqual(path.read_bytes(), before)
        self.assertIn('if (age > max_reference_age_s)', result)
        self.assertIn('!AtA.fullPivLu().isInvertible()', result)
        with self.assertRaises(ValueError): check.make_candidate(result)

    def test_compiler_failure_is_recorded_not_passed(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory); bundle=root/'input.tar.gz'; output=root/'out'; output.mkdir()
            bundle.write_bytes(archive_bytes('trial/rf2o.log',
                ('RF2O_REJECT_INPUT '+json.dumps(fixture())+'\n').encode()))
            with self.assertRaises(RuntimeError):
                check.compare(bundle, output, 'nonexistent_rf2o_test_compiler', root)
            report=json.loads((output/'report.json').read_text())
            self.assertEqual(report['status'], 'failed')
            self.assertFalse(report['installed_files_modified'])
            self.assertEqual(report['variants'], {})

    def test_python36_syntax(self):
        ast.parse(Path(check.__file__).read_text(), feature_version=(3,6))

    def test_integrated_core_matches_verified_candidate(self):
        here=Path(__file__).resolve().parent
        baseline=(here/'fixtures/rf2o_coarse_baseline.cpp').read_text()
        integrated=(here.parent/'rf2o_laser_odometry/src/CLaserOdometry2D.cpp').read_text()
        self.assertEqual(integrated, check.make_candidate(baseline))

    def test_runtime_marker_rejects_old_binary(self):
        import importlib.util
        path=Path(__file__).resolve().parents[1]/'run_hallway_trial.py'
        spec=importlib.util.spec_from_file_location('pyramid_runner', str(path))
        runner=importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
        with self.assertRaises(RuntimeError):
            runner.require_pyramid_policy('RF2O_TIMING_ENABLED version=1')
        runner.require_pyramid_policy('RF2O_PYRAMID_POLICY version=1 skip_rank_deficient_coarse=true require_finest=true\n')


if __name__ == '__main__':
    unittest.main()
