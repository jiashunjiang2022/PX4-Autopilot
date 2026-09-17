#!/usr/bin/env python3
"""Frozen 9.16/9.17 cohort only: causal export and summary, never model fitting.

Usage: script export|summarize OUTPUT_DIRECTORY
Export reads explicit nine old logs. Run unit-TailTrimLearningReview with
TAIL_TRIM_REVIEW_DIR pointing to OUTPUT_DIRECTORY before summarize.
"""
import csv
import hashlib
import json
import pathlib
import subprocess
import sys

import numpy as np
from pyulog import ULog

ROOT = pathlib.Path('/Users/jiangjiashun/Documents/门控数据分析')
COHORT = [("9.16数据", i) for i in (2, 3, 4)] + [("9.17数据", i) for i in (7, 8, 9, 10, 11, 12)]
TOPICS = ['rate_ctrl_status', 'vehicle_status', 'vehicle_attitude_setpoint',
          'vehicle_rates_setpoint', 'actuator_servos', 'vehicle_land_detected', 'vehicle_control_mode']


def export(out):
    inventory = []
    with (out / 'replay-input.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow('flight,t,now,servo_stamp,dt,g,i,s,imax,phi,p,left,right,armed,landed,control_valid,setpoint_valid,mapping,safety,mission,epoch,obs,actual_s,phi_valid,rate_valid,tau,gwin,gstd,gsign,ewin'.split(','))
        for index, (day, number) in enumerate(COHORT):
            paths = list((ROOT / day).glob(f'log_{number}_*.ulg'))
            assert len(paths) == 1, (day, number, paths)
            path = paths[0]
            u = ULog(str(path), message_name_filter_list=TOPICS)
            data = {d.name: d.data for d in u.data_list if d.multi_id == 0}
            assert all(name in data for name in TOPICS), path
            status = data['rate_ctrl_status']
            grid = np.arange(int(status['timestamp'][0]), int(status['timestamp'][-1])+1, 20000, dtype=np.int64)
            sampled = {}
            for name, d in data.items():
                assert np.all(np.diff(d['timestamp'].astype(np.int64)) >= 0), (path, name)
                ix = np.searchsorted(d['timestamp'], grid, side='right') - 1
                sampled[name] = ({k: v[np.maximum(ix, 0)] for k, v in d.items()}, ix >= 0)
            params = dict(u.initial_parameters)
            changes = sorted(u.changed_parameters, key=lambda x: x[0])
            cp = 0
            previous_nav = None
            for n, now in enumerate(grid):
                while cp < len(changes) and changes[cp][0] <= now:
                    _, key, value = changes[cp]; params[key] = value; cp += 1
                def value(topic, key):
                    return float(sampled[topic][0][key][n])
                def fresh(topic, age):
                    return sampled[topic][1][n] and 0 <= now-value(topic, 'timestamp') <= age
                rc = lambda key: value('rate_ctrl_status', key)
                vs = lambda key: value('vehicle_status', key)
                g, i, s = rc('flap_b2b_g_current'), rc('rollspeed_integ'), rc('flap_b2b_transferred_raw')
                q = [value('vehicle_attitude_setpoint', f'q_d[{k}]') for k in range(4)]
                norm = np.linalg.norm(q)
                qvalid = np.all(np.isfinite(q)) and abs(norm-1) < .01
                phi = np.arctan2(2*(q[0]*q[1]+q[2]*q[3]), 1-2*(q[1]**2+q[2]**2)) if qvalid else 0.
                p = value('vehicle_rates_setpoint', 'roll')
                spvalid = qvalid and fresh('vehicle_attitude_setpoint', 200000) and fresh('vehicle_rates_setpoint', 200000) and np.isfinite(p)
                control = fresh('vehicle_status', 1000000) and fresh('vehicle_control_mode', 1000000) and fresh('rate_ctrl_status', 50000)
                control = control and vs('vehicle_type') == 2 and not vs('is_vtol') and not vs('in_transition_mode')
                control = control and value('vehicle_control_mode','flag_control_rates_enabled') and value('vehicle_control_mode','flag_control_attitude_enabled')
                # Frozen controller logged its exact airframe verifier; do not guess generic mapping.
                mapping = bool(rc('flap_slow_config_valid'))
                nav = int(vs('nav_state'))
                safety = bool(vs('failsafe')) or (previous_nav == 3 and nav == 15) # VehicleStatus.msg: STAB=15
                previous_nav = nav
                writer.writerow([index, (now-grid[0])*1e-6, now, value('actuator_servos','timestamp'), .02, g,i,s,
                    params['FW_RR_IMAX'],phi,p,value('actuator_servos','control[0]'),value('actuator_servos','control[1]'),
                    int(vs('arming_state') == 2), int(not fresh('vehicle_land_detected',1500000) or bool(value('vehicle_land_detected','landed'))),
                    int(bool(control)),int(spvalid),int(mapping),int(safety),int(nav == 3),int(rc('flap_b2b_reset_epoch')),
                    g*(i+s)/1.1,g*s/1.1,int(qvalid),int(np.isfinite(p)),
                    params['FLAP_B2B_TAU'],params['FLAP_B2B_GWIN'],params['FLAP_B2B_GSTD'],params['FLAP_B2B_GSIGN'],params['FLAP_B2B_EWIN']])
            version = u.msg_info_dict.get('ver_sw', '')
            blob = subprocess.run(['git','rev-parse',f'{version}:src/modules/fw_rate_control/BumplessRollITransfer.hpp'],capture_output=True,text=True)
            inventory.append(dict(flight=index, path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                firmware=version, b2b_blob=blob.stdout.strip() if blob.returncode == 0 else 'UNKNOWN', rows=len(grid),
                parameters={k:v for k,v in u.initial_parameters.items() if k.startswith('FLAP_B2B') or k=='FW_RR_IMAX'},
                parameter_changes=[(int(t),k,v) for t,k,v in u.changed_parameters if k.startswith('FLAP_B2B') or k=='FW_RR_IMAX']))
    (out/'replay-inventory.json').write_text(json.dumps(inventory,indent=2,ensure_ascii=False))
    print('EXPORTED', len(inventory), 'frozen cohort logs')


def first(times, mask):
    ix = np.flatnonzero(mask)
    return float(times[ix[0]]) if len(ix) else None


def summarize(out):
    rows = np.genfromtxt(out/'synthetic.csv',delimiter=',',names=True,dtype=None,encoding='utf8')
    summary = []
    for case in dict.fromkeys(rows['scenario']):
        d = rows[rows['scenario']==case]; t = d['t']; mission = t >= 0
        for name in ('old','old_mission_start','candidate','v3'):
            b=d[name+'_b']; gate=d[name+'_gate'].astype(bool)
            zero=first(t,(t>=20)&d[name+'_zero'].astype(bool))
            release=first(t,(t>=20)&(b < b[np.flatnonzero(t<20)[-1]]-1e-5))
            signmask=mission&(d['maneuver']==0)&(np.abs(b)>1e-6)&(np.abs(d['burden'])>1e-6)
            summary.append(dict(scenario=case,learner=name,first_gate=first(t,gate),
                first_gate_after_mission=first(t,mission&gate),time_to_b002=first(t,mission&(b>=.02)),
                final_target=float(d[name+'_target'][-1]),final_b=float(b[-1]),
                sign_correct_fraction=float(np.mean(np.sign(b[signmask])==np.sign(d['burden'][signmask]))) if np.any(signmask) else None,
                transient_max_b=float(np.max(np.abs(b))) if case=='transient' else None,
                release_latency=release-20 if release is not None and case in ('decrease','reversal') else None,
                reversal_zero_time=zero if case=='reversal' else None,
                fresh_gate_delay=(first(t,(t>zero)&gate)-zero) if zero is not None and first(t,(t>zero)&gate) is not None and case=='reversal' else None))
    (out/'synthetic-summary.json').write_text(json.dumps(summary,indent=2))
    if not (out/'replay-output.csv').exists(): return
    rows=np.genfromtxt(out/'replay-output.csv',delimiter=',',names=True)
    reports=[]
    for flight in np.unique(rows['flight']):
        d=rows[rows['flight']==flight]; trusted=d['trusted']>.5; t=d['t']; obs=d['obs']
        for learner in ('old','candidate'):
            b=d[learner+'_b']; mask=trusted&(d[learner+'_valid']>.5)
            median=float(np.median(obs[mask])) if np.any(mask) else None
            threshold=.5*min(.08,max(abs(median)-.03,0)) if median is not None else 0
            target_sign=np.sign(median) if median is not None else 0
            t0=first(t,mask); half=first(t,mask&(target_sign*b>=threshold)) if threshold>1e-6 else None
            active=mask&(np.abs(b)>1e-6)&(np.abs(obs)>1e-6)
            reset=d[learner+'_reset']>.5; maneuver=d['maneuver' if learner=='candidate' else 'old_maneuver']>.5
            normal_turn=maneuver&(d['armed']>.5)&~reset
            db=np.r_[0,np.diff(b)]
            reports.append(dict(flight=int(flight),learner=learner,trusted_rows=int(mask.sum()),
                first_gate_time=first(t,mask&(d[learner+'_gate']>.5)),median_B_obs=median,
                median_actual_V3_tail_S=float(np.median(d['actual_s'][mask])) if np.any(mask) else None,
                median_candidate_b=float(np.median(b[mask])) if np.any(mask) else None,
                sign_agreement=float(np.mean(np.sign(b[active])==np.sign(obs[active]))) if np.any(active) else None,
                sign_coverage=float(active.sum()/mask.sum()) if np.any(mask) else None,
                time_to_50pct=half-t0 if half is not None else None,
                reset_count=int(np.count_nonzero(reset&~np.r_[False,reset[:-1]])),reset_rows=int(reset.sum()),
                maneuver_freeze_fraction=float(np.mean(np.abs(db[normal_turn])<1e-7)) if np.any(normal_turn) else None))
    (out/'replay-summary.json').write_text(json.dumps(reports,indent=2))
    inputs=np.genfromtxt(out/'replay-input.csv',delimiter=',',names=True)
    exclusions=[]
    for flight in np.unique(rows['flight']):
        d=rows[rows['flight']==flight]; i=inputs[inputs['flight']==flight]
        eligible=(i['armed']>.5)&(i['landed']<.5)&(i['mission']>.5)
        trust=d['trusted']>.5
        runs=np.diff(np.r_[0,trust.astype(int),0]); starts=np.flatnonzero(runs==1); ends=np.flatnonzero(runs==-1)
        g=i['g'][eligible & np.isfinite(i['g'])]
        exclusions.append(dict(flight=int(flight),mission_airborne_rows=int(eligible.sum()),
            invalid_control_rows=int(np.count_nonzero(eligible&(i['control_valid']<.5))),
            invalid_setpoint_rows=int(np.count_nonzero(eligible&(i['setpoint_valid']<.5))),
            invalid_mapping_rows=int(np.count_nonzero(eligible&(i['mapping']<.5))),
            maneuver_rows=int(np.count_nonzero(eligible&(d['maneuver']>.5))),
            max_trusted_run_s=float(np.max(ends-starts)*.02) if len(starts) else 0,
            g_p05_p50_p95=np.quantile(g,[.05,.5,.95]).tolist() if len(g) else []))
    (out/'replay-exclusions.json').write_text(json.dumps(exclusions,indent=2))
    print('SUMMARIZED', len(reports), 'flight/learner comparisons')


if __name__ == '__main__':
    destination=pathlib.Path(sys.argv[2]); destination.mkdir(parents=True,exist_ok=True)
    {'export':export,'summarize':summarize}[sys.argv[1]](destination)
