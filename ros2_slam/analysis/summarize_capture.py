#!/usr/bin/env python3
"""Summarize capture_sensors JSONL without ROS; no ground-truth accuracy claims."""
import argparse
import collections
import json
import math
import statistics


def stamp(header):
    value = header['stamp']
    return value['sec'] + value['nanosec'] / 1e9


def summarize(path):
    groups = collections.defaultdict(list)
    malformed = 0
    with open(path) as stream:
        for line in stream:
            try:
                record = json.loads(line)
                groups[record['topic']].append(record)
            except (ValueError, KeyError):
                malformed += 1
    report = {'malformed_records': malformed, 'topics': {}}
    for topic, rows in sorted(groups.items()):
        span = rows[-1]['elapsed_s'] - rows[0]['elapsed_s']
        info = {'messages': len(rows), 'receipt_span_s': span,
                'receipt_rate_hz': (len(rows)-1)/span if span > 0 else None}
        report['topics'][topic] = info
        if topic == 'phidget_raw':
            rates = [r['msg']['angular_rate_deg_s'][2] for r in rows]
            valid = [v for v in rates if isinstance(v,(int,float)) and math.isfinite(v)]
            integrated = 0.0
            skipped = 0
            for a,b in zip(rows,rows[1:]):
                dt = (b['msg']['device_timestamp_ms']-a['msg']['device_timestamp_ms'])/1000.0
                va,vb = a['msg']['angular_rate_deg_s'][2],b['msg']['angular_rate_deg_s'][2]
                if 0 < dt <= .25 and all(isinstance(v,(int,float)) and math.isfinite(v)
                                        for v in (va,vb)):
                    integrated += (va+vb)*.5*dt
                else:
                    skipped += 1
            info.update({'raw_sensor_z_integral_deg':integrated,
                         'integration_skipped_intervals':skipped,
                         'gyro_z_nonfinite':len(rates)-len(valid),
                         'gyro_z_mean_deg_s':statistics.mean(valid) if valid else None,
                         'gyro_z_std_deg_s':statistics.pstdev(valid) if valid else None})
        header_rows = [r for r in rows if 'header' in r['msg']]
        if header_rows:
            stamps = [stamp(r['msg']['header']) for r in header_rows]
            ages = [r['receipt_ros_s'] - s for r, s in zip(header_rows, stamps)]
            info.update({'frames': sorted(set(r['msg']['header']['frame_id']
                                              for r in header_rows)),
                         'mean_receipt_age_s': statistics.mean(ages),
                         'max_receipt_age_s': max(ages),
                         'duplicate_stamps': sum(b == a for a,b in zip(stamps, stamps[1:])),
                         'backward_stamps': sum(b < a for a,b in zip(stamps, stamps[1:]))})
        if 'ranges' in rows[0]['msg']:
            residuals = []
            fractions = []
            for row in rows:
                m = row['msg']
                n = len(m['ranges'])
                residuals.append(m['angle_max'] - m['angle_min']
                                 - (n-1)*m['angle_increment'])
                valid = sum(isinstance(v, (int,float)) and math.isfinite(v)
                            and m['range_min'] <= v <= m['range_max']
                            for v in m['ranges'])
                fractions.append(valid/n if n else 0)
            info.update({'bins': sorted(set(len(r['msg']['ranges']) for r in rows)),
                         'max_abs_angle_endpoint_residual_rad': max(map(abs,residuals)),
                         'mean_valid_range_fraction': statistics.mean(fractions)})
        if 'angular_velocity' in rows[0]['msg']:
            values = [r['msg']['angular_velocity']['z'] for r in rows]
            finite = [v for v in values if isinstance(v,(int,float)) and math.isfinite(v)]
            info['gyro_z_nonfinite'] = len(values)-len(finite)
            if finite:
                info['gyro_z_mean_rad_s'] = statistics.mean(finite)
                info['gyro_z_std_rad_s'] = statistics.pstdev(finite)
            angle = 0.0
            skipped = 0
            for a,b in zip(rows, rows[1:]):
                dt = stamp(b['msg']['header'])-stamp(a['msg']['header'])
                va,vb = a['msg']['angular_velocity']['z'], b['msg']['angular_velocity']['z']
                if (0 < dt <= 0.25 and all(isinstance(v,(int,float)) and math.isfinite(v)
                                           for v in (va,vb))):
                    angle += 0.5*(va+vb)*dt
                else:
                    skipped += 1
            info.update({'integrated_gyro_z_deg': math.degrees(angle),
                         'integration_skipped_intervals': skipped,
                         'orientation_covariance_first': rows[0]['msg']['orientation_covariance'],
                         'angular_velocity_covariance_first': rows[0]['msg']['angular_velocity_covariance']})
    report['interpretation'] = (
        'Missing topics were not received, not proven absent. Best-effort capture can drop data. '
        'phidget_raw integral is native sensor Z; ROS Imu integral is expressed in its header frame. '
        'Neither is an external ground-truth angle; no additional bias correction is applied. '
        'Gyro mean measures bias only in an entirely stationary capture. '
        'Age uses publisher and recorder ROS clocks; acquisition time may differ. '
        'Endpoint residual flags metadata inconsistency, not its effect on this legacy mapper.')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture')
    args = parser.parse_args()
    print(json.dumps(summarize(args.capture), indent=2, allow_nan=False))
