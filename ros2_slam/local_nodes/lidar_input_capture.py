#!/usr/bin/env python3
"""Bounded passive capture of pre-binning LiDAR input, flushed after shutdown.

This does not filter, re-bin, timestamp, publish or command anything. There is
no disk I/O in record(). Missing output ranges become JSON null; original beam
angles (including fractional degrees), quality and distance are retained.
"""
import json
import math
from pathlib import Path


class LidarInputCapture:
    def __init__(self, path, limit):
        if not isinstance(limit,int) or not 0<=limit<=1200:
            raise ValueError('LiDAR input capture limit must be 0..1200 scans')
        if limit and not path:
            raise ValueError('LiDAR input capture needs an output path')
        self.path=Path(path) if path else None
        if limit and self.path.exists():
            raise ValueError('Refusing to overwrite LiDAR input capture')
        self.limit=limit
        self.rows=[]
        self.skipped_oversize=0
        self.after_limit=0
        self.saved=False
        self.raw_points=0
        self.budget_drops=0

    def record(self, raw, ranges, stamp, increment, receipt_unix_s):
        if not self.limit: return
        if len(self.rows)>=self.limit:
            self.after_limit+=1
            return
        # Bound malformed/unexpected input without truncating a scan invisibly.
        if len(raw)>2000 or len(ranges)!=360:
            self.skipped_oversize+=1
            return
        if self.raw_points+len(raw)>250000:
            self.budget_drops+=1
            return
        self.raw_points+=len(raw)
        self.rows.append(dict(schema=1,receipt_unix_s=receipt_unix_s,
            stamp=dict(sec=stamp.sec,nanosec=stamp.nanosec),
            angle_increment=increment,
            raw_clockwise_quality_angle_deg_distance_mm=[list(p) for p in raw],
            published_ranges_m=[v if math.isfinite(v) else None for v in ranges]))

    def save(self):
        if not self.limit or self.saved: return
        # Exclusive creation preserves any pre-existing experiment.
        with self.path.open('x') as stream:
            for row in self.rows:
                stream.write(json.dumps(row,allow_nan=False,separators=(',',':'))+'\n')
        self.saved=True

    def summary(self):
        return dict(version=1,limit=self.limit,recorded=len(self.rows),
                    skipped_oversize=self.skipped_oversize,after_limit=self.after_limit,
                    raw_points=self.raw_points,budget_drops=self.budget_drops,
                    saved=self.saved,path=str(self.path) if self.path else None)
