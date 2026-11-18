#!/usr/bin/env python3
"""Offline RF2O sequence A/B, Python 3.6. No ROS or hardware required.

Feeds every captured /scan, from zero initial pose/prior. This is not byte-exact
live scheduling, IMU fusion or SLAM replay. Preserve rejection/expiry behavior.
Output directory must be new; originals are never modified.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from run_rf2o_core_tests import eigen_include_flags
from scan_gap_candidate import fill_single_bin_gaps


def load_scans(path):
    scans=[]
    with path.open() as stream:
        for line in stream:
            row=json.loads(line)
            if row.get('topic') != '/scan':
                continue
            msg=row['msg']; stamp=msg['header']['stamp']
            sec, ns=stamp['sec'], stamp['nanosec']
            if not isinstance(sec,int) or not isinstance(ns,int) or not 0<=ns<10**9:
                raise ValueError('Invalid measurement stamp')
            ranges=[float(v) if v is not None else float('inf') for v in msg['ranges']]
            ranges=[v if math.isfinite(v) and v>0 else float('inf') for v in ranges]
            inc=float(msg['angle_increment'])
            if len(ranges)!=360 or not math.isfinite(inc) or abs(inc-math.pi/180)>1e-7:
                raise ValueError('Expected 360 one-degree bins')
            scan=dict(sec=sec, ns=ns, increment=inc, ranges=ranges)
            if scans and stamp_ns(scan)<=stamp_ns(scans[-1]):
                raise ValueError('Nonincreasing measurement stamp')
            scans.append(scan)
    if len(scans)<2: raise ValueError('Need two or more scans')
    return scans


def stamp_ns(scan): return scan['sec']*10**9+scan['ns']


def scan_statistics(scans):
    masks=[[math.isfinite(v) for v in s['ranges']] for s in scans]
    deltas=sorted(abs(a-b) for first,second in zip(scans,scans[1:])
                  for a,b in zip(first['ranges'],second['ranges'])
                  if math.isfinite(a) and math.isfinite(b))
    def quantile(q):
        if not deltas: return None
        index=q*(len(deltas)-1);lo=int(index);hi=min(lo+1,len(deltas)-1)
        return deltas[lo]+(deltas[hi]-deltas[lo])*(index-lo)
    return dict(scans=len(scans), span_s=(stamp_ns(scans[-1])-stamp_ns(scans[0]))*1e-9,
                valid_fraction=sum(sum(m) for m in masks)/(360.0*len(masks)),
                mean_validity_changes=sum(sum(a!=b for a,b in zip(x,y))
                    for x,y in zip(masks,masks[1:]))/(len(masks)-1),
                common_bin_delta_m=dict(median=quantile(.5),p90=quantile(.9),p99=quantile(.99)))


def replay(executable, scans, prefix, mode):
    inferred=0;lines=[str(len(scans))]
    for scan in scans:
        ranges=scan['ranges']
        if mode=='single_bin':
            ranges,n=fill_single_bin_gaps(ranges,scan['increment']);inferred+=n
        lines.append(' '.join([str(scan['sec']),str(scan['ns']),str(scan['increment'])]+
                              [str(v) for v in ranges]))
    path=prefix.with_suffix('.txt');path.write_text('\n'.join(lines)+'\n')
    result=subprocess.check_output([str(executable),str(path)],universal_newlines=True,timeout=120)
    prefix.with_suffix('.csv').write_text('index,accepted,reason,x_m,y_m,yaw_rad\n'+result)
    rows=[line.split(',') for line in result.splitlines()]
    poses=[(float(r[3]),float(r[4]),float(r[5])) for r in rows]
    if not rows: raise RuntimeError('No replay output')
    return dict(processed=len(rows),expected=len(scans)-1,
                accepted=sum(r[1]=='1' for r in rows),rejected=sum(r[1]!='1' for r in rows),
                last_reason=rows[-1][2],complete=len(rows)==len(scans)-1,
                endpoint_m=math.hypot(*poses[-1][:2]),
                max_excursion_m=max(math.hypot(*p[:2]) for p in poses),
                final_x_m=poses[-1][0],final_y_m=poses[-1][1],
                final_yaw_deg=math.degrees(poses[-1][2]),inferred_bins=inferred)


def synthetic(scans, dx, dyaw, count=100):
    """Ray-cast fixed rectangular room; use recorded masks only, not ranges.

    An artificial observability/small-motion regression, not hallway truth.
    RF2O's centered scan convention is used for comparison to its existing tests.
    """
    result=[]
    for n in range(count+1):
        ranges=[];x=n*dx;yaw=n*dyaw
        for i,original in enumerate(scans[n%len(scans)]['ranges']):
            a=math.radians(-179.5+i)+yaw;c=math.cos(a);s=math.sin(a)
            tx=((5 if c>0 else -3)-x)/c if abs(c)>1e-10 else 1e10
            ty=(3 if s>0 else -2)/s if abs(s)>1e-10 else 1e10
            ranges.append(min(tx,ty) if math.isfinite(original) else float('inf'))
        result.append(dict(sec=1+n//10,ns=(n%10)*100000000,increment=math.pi/180,ranges=ranges))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures',nargs='+',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--eigen-parent',type=Path,default=Path('/usr/include'))
    parser.add_argument('--compiler',default='c++')
    args=parser.parse_args()
    loaded=[load_scans(p) for p in args.captures]
    args.output.mkdir(parents=True,exist_ok=False)
    here=Path(__file__).resolve().parent;source=here.parent/'rf2o_laser_odometry'
    exe=args.output.resolve()/'replay'
    command=[args.compiler,'-std=c++14','-O1']+eigen_include_flags(args.eigen_parent)+[
        '-I'+str(here/'rf2o_test_stubs'),'-I'+str(source/'include'),
        str(here/'replay_scan_sequence.cpp'),str(source/'src/CLaserOdometry2D.cpp'),'-o',str(exe)]
    with (args.output/'build.log').open('w') as log:
        subprocess.run(command,check=True,stdout=log,stderr=subprocess.STDOUT,timeout=600)
    report=dict(interpretation='Offline sequence screening, not live timing, fused odometry or mapping qualification.',
                source_sha256={str(p.relative_to(here.parent)):hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [source/'src/CLaserOdometry2D.cpp',here/'replay_scan_sequence.cpp',
                              here/'replay_scan_sequence.py',here/'scan_gap_candidate.py']},
                captures=[],synthetic=[])
    for k,(path,scans) in enumerate(zip(args.captures,loaded)):
        entry=dict(path=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),statistics=scan_statistics(scans))
        for mode in ['none','single_bin']:
            entry[mode]=replay(exe,scans,args.output/('capture_{}_{}'.format(k,mode)),mode)
        report['captures'].append(entry)
        for label,dx,dyaw in [('still',0,0),('forward',.001,0),('backward',-.001,0),
                              ('left',0,.001),('right',0,-.001)]:
            seq=synthetic(scans,dx,dyaw)
            for mode in ['none','single_bin']:
                r=replay(exe,seq,args.output/('synthetic_{}_{}_{}'.format(k,label,mode)),mode)
                r.update(mask_source=k,case=label,mode=mode,expected_x_m=100*dx,
                         expected_yaw_deg=math.degrees(100*dyaw))
                r['passed']=r['complete'] and r['rejected']==0 and abs(r['final_x_m']-100*dx)<.02 and abs(r['final_y_m'])<.02 and abs(r['final_yaw_deg']-math.degrees(100*dyaw))<1.2
                report['synthetic'].append(r)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report,indent=2))
    # Baseline is a control, not a gate. Candidate must preserve small real motion.
    return 0 if all(r['passed'] for r in report['synthetic'] if r['mode']=='single_bin') else 1


if __name__=='__main__': raise SystemExit(main())
