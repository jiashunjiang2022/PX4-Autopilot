#!/usr/bin/env python3
"""Replay the actual production header against immutable authoritative CSVs.

Binary64 roundoff threshold 2e-12; no trajectory exclusions. Hard numerical
faults intentionally reset immediately, and are checked separately against
the production safety requirement, rather than hidden by a wider tolerance.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DEFAULT = Path('/Users/jiangjiashun/Documents/pitch_pff_handoff_20261010')
FIELDS = ['coeff_cos_1f','coeff_sin_1f','coeff_cos_2f_internal','coeff_sin_2f_internal',
          'baseline','qhat_1f','qhat_2f_internal','qhat_1f_applied','estimation_error',
          'alpha_effective','fade','warm_cycles','warmup_complete','active','fading','reset_reason']
REASONS = {'NONE':0,'DISABLED':1,'NONFINITE_Q_SP':9,'PHASE_INVALID':7,
           'NONFINITE_PHASE':10,'INVALID_FREQUENCY':11,'INVALID_DT':12}

def truth(value):
    return int(value.lower() in ('true','1'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--reference-dir',type=Path,default=DEFAULT)
    parser.add_argument('--json',type=Path)
    args=parser.parse_args()
    metrics={}
    with tempfile.TemporaryDirectory(prefix='pitch-pff-replay-') as tmp:
        binary=Path(tmp)/'replay'
        subprocess.run(['c++','-std=c++17','-O2','-Wall','-Wextra','-Werror','-I.',
                        'Tools/pitch_ff_phase_shaper/pitch_ff_reference_replay.cpp','-o',str(binary)],cwd=ROOT,check=True)
        for name in ['PITCH_PFF_SMOKE_REFERENCE.csv','PITCH_PFF_PRODUCTION_REFERENCE.csv']:
            path=args.reference_dir/name
            rows=list(csv.DictReader(path.open()))
            trace=''.join(' '.join([str(truth(r['reset_before_case'])),r['q_sp'],r['wing_phase_rad'],
                      str(truth(r['phase_valid'])),r['flap_frequency_hz'],r['dt_s'],
                      str(truth(r['enable'])),r['alpha'],r['adapt_cycles']])+'\n' for r in rows)
            output=subprocess.check_output([str(binary)],input=trace,text=True,cwd=ROOT)
            actual=list(csv.reader(output.splitlines()))
            assert len(actual)==len(rows)
            max_errors={f:0. for f in FIELDS[:12]}
            q_actual=[];q_expected=[];safety_count=0
            for idx,(r,o) in enumerate(zip(rows,actual)):
                reason=r['expected_reset_reason']
                # The package contains excessive-dt faults with zero expected
                # residual. The hard reset and reference agree on these rows.
                hard_fault=reason in ('INVALID_DT','NONFINITE_Q_SP','NONFINITE_PHASE')
                if hard_fault:
                    safety_count+=1
                    assert float(o[7])==0. and int(o[13])==0, (name,idx,'hard fault')
                for j,field in enumerate(FIELDS):
                    if j<12:
                        expected=float(r['expected_'+field]);got=float(o[j])
                        err=abs(got-expected)
                        max_errors[field]=max(max_errors[field],err)
                        assert err<=2e-12,(name,idx,field,got,expected)
                    elif j<15:
                        assert int(o[j])==truth(r['expected_'+field]),(name,idx,field)
                    else:
                        assert int(o[j])==REASONS[reason],(name,idx,field,o[j],reason)
                q_actual.append(float(o[5]));q_expected.append(float(r['expected_qhat_1f']))
            errors=[a-b for a,b in zip(q_actual,q_expected)]
            ma=sum(q_actual)/len(rows);mb=sum(q_expected)/len(rows)
            corr=sum((a-ma)*(b-mb) for a,b in zip(q_actual,q_expected))/math.sqrt(
                 sum((a-ma)**2 for a in q_actual)*sum((b-mb)**2 for b in q_expected))
            metrics[name]={'samples':len(rows),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                'qhat_rmse':math.sqrt(sum(e*e for e in errors)/len(rows)),
                'qhat_max_abs':max(map(abs,errors)),'qhat_corr':corr,
                'max_errors':max_errors,'active_parity':'PASS','reset_parity':'PASS',
                'hard_numeric_fault_samples':safety_count}
            print(name,json.dumps(metrics[name],sort_keys=True))
    prod=metrics['PITCH_PFF_PRODUCTION_REFERENCE.csv']
    for label,key in [('QHAT_RMSE','qhat_rmse'),('QHAT_MAX_ABS','qhat_max_abs'),('QHAT_CORR','qhat_corr'),
                      ('ACTIVE_PARITY','active_parity'),('RESET_PARITY','reset_parity')]:
        print('OFFLINE_PRODUCTION_'+label+'='+str(prod[key]))
    print('OFFLINE_PRODUCTION_COEFF_COS_MAX_ABS='+str(prod['max_errors']['coeff_cos_1f']))
    print('OFFLINE_PRODUCTION_COEFF_SIN_MAX_ABS='+str(prod['max_errors']['coeff_sin_1f']))
    print('OFFLINE_TO_PRODUCTION_REPLAY_PARITY=PASS')
    if args.json:
        args.json.write_text(json.dumps(metrics,indent=2)+'\n')

if __name__=='__main__': main()
