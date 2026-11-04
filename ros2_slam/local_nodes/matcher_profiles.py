"""Verified matcher profiles. Standard library only; compatible with Python 3.6."""
import math
import re

PROFILE_FILES = {
    "baseline": "mapper_params_online_sync.yaml",
    "narrow": "mapper_params_online_sync_narrow.yaml",
}

# Effective C++ defaults used before the parameter-loading repair.
BASELINE = {
    "use_scan_matching": 1.0,
    "use_scan_barycenter": 1.0,
    "minimum_travel_distance": 0.5,
    "minimum_travel_heading": 0.5,
    "scan_buffer_size": 10,
    "scan_buffer_maximum_scan_distance": 10,
    "link_match_minimum_response_fine": 0.1,
    "link_scan_maximum_distance": 1.5,
    "loop_search_maximum_distance": 3.0,
    "do_loop_closing": 1.0,
    "loop_match_minimum_chain_size": 10,
    "loop_match_maximum_variance_coarse": 3.0,
    "loop_match_minimum_response_coarse": 0.35,
    "loop_match_minimum_response_fine": 0.45,
    "correlation_search_space_dimension": 0.5,
    "correlation_search_space_resolution": 0.01,
    "correlation_search_space_smear_deviation": 0.1,
    "loop_search_space_dimension": 8.0,
    "loop_search_space_resolution": 0.05,
    "loop_search_space_smear_deviation": 0.03,
    "distance_variance_penalty": 0.5,
    "angle_variance_penalty": 1.0,
    "fine_search_angle_offset": 0.00349,
    "coarse_search_angle_offset": 0.349,
    "coarse_angle_resolution": 0.0349,
    "minimum_angle_penalty": 0.9,
    "minimum_distance_penalty": 0.05,
    "use_response_expansion": 1.0,
}

def expected_parameters(profile):
    if profile not in PROFILE_FILES:
        raise ValueError("Unknown matcher profile: " + profile)
    expected = dict(BASELINE)
    if profile == "narrow":
        expected["correlation_search_space_dimension"] = 0.3
    return expected


def check_karto_config(log_text, profile):
    """Fail closed on missing, conflicting, nonfinite or incorrect internal values."""
    expected = expected_parameters(profile)
    actual = {}
    errors = []
    for name, raw in re.findall(r"KARTO_CONFIG\s+(\w+)=(\S+)", log_text):
        if name not in expected:
            continue
        try:
            value = float(raw)
        except ValueError:
            errors.append("invalid value for " + name)
            continue
        if name in actual and actual[name] != value:
            errors.append("conflicting values for " + name)
        actual[name] = value
    for name, value in sorted(expected.items()):
        got = actual.get(name)
        if got is None:
            errors.append("missing " + name)
        elif not math.isfinite(got) or not math.isclose(got, value, rel_tol=1e-8, abs_tol=1e-10):
            errors.append("{}: expected {}, Karto {}".format(name, value, got))
    return actual, errors
