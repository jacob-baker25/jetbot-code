#!/usr/bin/env python3
"""Build one temporal-median LaserScan payload per stationary stop.

This module has no ROS imports.  It operates on the JSON payloads stored by
``replay_recorder.py`` so episode selection and median construction can be
tested offline.  Odometry and TF are never modified; the replay player merely
replaces a run of stationary scans with one aggregate emitted at the final
scan's original sequence and timestamp.
"""
import copy
import math
import statistics


NANOSECONDS = 1000000000


class StationaryAggregationError(RuntimeError):
    pass


def stamp_seconds(header):
    return int(header["sec"]) + int(header["nanosec"]) / float(NANOSECONDS)


def angle_difference(first, second):
    return math.atan2(math.sin(second - first), math.cos(second - first))


def finite_number(value):
    return value is not None and math.isfinite(float(value))


class StationaryMedianScanSelector(object):
    """Select stationary episodes and construct one median scan per episode."""

    def __init__(self, timeline, maximum_scan_translation=0.003,
                 maximum_scan_rotation=math.radians(1.0),
                 maximum_episode_translation=0.03,
                 maximum_episode_rotation=math.radians(2.0),
                 maximum_gap=0.30, minimum_episode_scans=8,
                 minimum_observation_fraction=0.60):
        values = (
            maximum_scan_translation,
            maximum_scan_rotation,
            maximum_episode_translation,
            maximum_episode_rotation,
            maximum_gap,
            minimum_observation_fraction,
        )
        if any(not math.isfinite(float(value)) or float(value) <= 0
               for value in values):
            raise ValueError("Stationary aggregation thresholds must be positive")
        if int(minimum_episode_scans) != minimum_episode_scans \
                or minimum_episode_scans < 2:
            raise ValueError("Minimum episode scans must be an integer >= 2")
        if minimum_observation_fraction > 1:
            raise ValueError("Minimum observation fraction must not exceed 1")
        self.timeline = timeline
        self.maximum_scan_translation = float(maximum_scan_translation)
        self.maximum_scan_rotation = float(maximum_scan_rotation)
        self.maximum_episode_translation = float(maximum_episode_translation)
        self.maximum_episode_rotation = float(maximum_episode_rotation)
        self.maximum_gap = float(maximum_gap)
        self.minimum_episode_scans = int(minimum_episode_scans)
        self.minimum_observation_fraction = float(
            minimum_observation_fraction)
        self.selected = {}
        self.observed_scans = 0
        self.stationary_scans = 0
        self.scans_without_coverage = 0
        self.episode_candidates = 0
        self.short_episodes_rejected = 0
        self.published_episodes = 0
        self.published_episode_scan_counts = []
        self.output_valid_bins = []

    def scan_state(self, payload):
        end = stamp_seconds(payload["header"])
        duration = float(payload["scan_time"])
        if not math.isfinite(duration) or duration <= 0:
            raise StationaryAggregationError(
                "Laser scan_time must be finite and positive")
        start = end - duration
        if not self.timeline.covers(start, end):
            self.scans_without_coverage += 1
            return None
        start_pose = self.timeline.interpolate(start)
        end_pose = self.timeline.interpolate(end)
        translation = math.hypot(
            end_pose[0] - start_pose[0], end_pose[1] - start_pose[1])
        rotation = abs(angle_difference(start_pose[2], end_pose[2]))
        return {
            "end": end,
            "pose": end_pose,
            "stationary": (
                translation <= self.maximum_scan_translation
                and rotation <= self.maximum_scan_rotation),
        }

    def same_episode(self, first_state, previous_state, current_state):
        first_pose = first_state["pose"]
        pose = current_state["pose"]
        return (
            current_state["end"] - previous_state["end"] <= self.maximum_gap
            and math.hypot(
                pose[0] - first_pose[0], pose[1] - first_pose[1])
            <= self.maximum_episode_translation
            and abs(angle_difference(first_pose[2], pose[2]))
            <= self.maximum_episode_rotation
        )

    @staticmethod
    def validate_geometry(payloads):
        first = payloads[0]
        count = len(first.get("ranges", []))
        if count < 2 or len(first.get("intensities", [])) != count:
            raise StationaryAggregationError("Invalid LaserScan array lengths")
        fields = (
            "angle_min", "angle_max", "angle_increment",
            "range_min", "range_max",
        )
        for payload in payloads[1:]:
            if (len(payload.get("ranges", [])) != count
                    or len(payload.get("intensities", [])) != count):
                raise StationaryAggregationError(
                    "Stationary episode changed LaserScan array lengths")
            for field in fields:
                if not math.isclose(
                        float(payload[field]), float(first[field]),
                        rel_tol=0.0, abs_tol=1e-12):
                    raise StationaryAggregationError(
                        "Stationary episode changed LaserScan geometry")
        return count

    def aggregate(self, episode):
        payloads = [item[2] for item in episode]
        count = self.validate_geometry(payloads)
        required = int(math.ceil(
            self.minimum_observation_fraction * len(payloads)))
        ranges = []
        intensities = []
        for index in range(count):
            valid = [
                (float(payload["ranges"][index]),
                 payload["intensities"][index])
                for payload in payloads
                if finite_number(payload["ranges"][index])
            ]
            if len(valid) < required:
                ranges.append(None)
                intensities.append(0.0)
                continue
            ranges.append(statistics.median([item[0] for item in valid]))
            quality = [float(item[1]) for item in valid
                       if finite_number(item[1])]
            intensities.append(statistics.median(quality) if quality else 0.0)
        result = copy.deepcopy(payloads[-1])
        result["ranges"] = ranges
        result["intensities"] = intensities
        return result

    def finish_episode(self, episode):
        if not episode:
            return
        self.episode_candidates += 1
        if len(episode) < self.minimum_episode_scans:
            self.short_episodes_rejected += 1
            return
        sequence = int(episode[-1][0])
        if sequence in self.selected:
            raise StationaryAggregationError("Duplicate selected scan sequence")
        aggregate = self.aggregate(episode)
        self.selected[sequence] = aggregate
        self.published_episodes += 1
        self.published_episode_scan_counts.append(len(episode))
        self.output_valid_bins.append(sum(
            1 for value in aggregate["ranges"] if finite_number(value)))

    def prepare(self, records):
        if self.selected or self.observed_scans:
            raise StationaryAggregationError(
                "Stationary selector may only be prepared once")
        current = []
        states = []
        previous_sequence = None
        for sequence, payload in records:
            sequence = int(sequence)
            if previous_sequence is not None and sequence <= previous_sequence:
                raise StationaryAggregationError(
                    "Scan sequences must increase")
            previous_sequence = sequence
            self.observed_scans += 1
            state = self.scan_state(payload)
            if state is None or not state["stationary"]:
                self.finish_episode(current)
                current = []
                states = []
                continue
            self.stationary_scans += 1
            if current and not self.same_episode(states[0], states[-1], state):
                self.finish_episode(current)
                current = []
                states = []
            current.append((sequence, state, payload))
            states.append(state)
        self.finish_episode(current)
        if not self.selected:
            raise StationaryAggregationError(
                "No stationary episode met the dwell requirement")
        return self

    @classmethod
    def from_connection(cls, connection, timeline, **kwargs):
        selector = cls(timeline, **kwargs)
        records = []
        import json
        for sequence, encoded in connection.execute(
                "SELECT sequence, payload FROM messages "
                "WHERE topic='/scan_slam' ORDER BY sequence"):
            records.append((sequence, json.loads(encoded)))
        return selector.prepare(records)

    def payload_for(self, sequence):
        return self.selected.get(int(sequence))

    def summary(self):
        return {
            "maximum_scan_translation_m": self.maximum_scan_translation,
            "maximum_scan_rotation_deg": math.degrees(
                self.maximum_scan_rotation),
            "maximum_episode_translation_m": (
                self.maximum_episode_translation),
            "maximum_episode_rotation_deg": math.degrees(
                self.maximum_episode_rotation),
            "maximum_gap_s": self.maximum_gap,
            "minimum_episode_scans": self.minimum_episode_scans,
            "minimum_observation_fraction": (
                self.minimum_observation_fraction),
            "observed_scans": self.observed_scans,
            "stationary_scans": self.stationary_scans,
            "scans_without_coverage": self.scans_without_coverage,
            "episode_candidates": self.episode_candidates,
            "short_episodes_rejected": self.short_episodes_rejected,
            "published_episodes": self.published_episodes,
            "published_episode_scan_counts": (
                self.published_episode_scan_counts),
            "output_valid_bins": self.output_valid_bins,
        }
