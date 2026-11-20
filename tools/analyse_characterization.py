"""Export measured calm-air throttle points and approximate model-angle onset."""
import argparse,csv,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser=argparse.ArgumentParser();parser.add_argument('runs',nargs='+',type=Path);parser.add_argument('--output',required=True,type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for run in args.runs:
        path=run/'characterization_result.json'
        if not path.exists() or 'passed' not in json.loads(path.read_text()):path=run/'autotune_result.json'
        result=json.loads(path.read_text())
        rows=list(csv.DictReader((run/'characterization.csv').open()))
        limit=result.get('limit');onset=None
        # Merlin wing alpha = body alpha + a0 (0.065 rad). Main wing
        # alpha_stall=0.16 rad; this neglects rotation-induced CP velocity.
        if limit:
            for row in rows:
                if (float(row['stage_pwm'])==limit['stage_pwm'] and abs(float(row['vz_mps']))<.5
                        and abs(float(row['height_error_m']))<10
                        and float(row['alpha_body_deg'])>=math.degrees(.16-.065)):
                    onset={k:float(v) for k,v in row.items()};break
        results.append(dict(run=str(run),rtl_confirmed=result.get('rtl_confirmed'),points=result['points'],
            approximate_model_stall_onset=onset))
    (args.output/'results.json').write_text(json.dumps(dict(runs=results,
        assumptions='Calm air, near-level, current Merlin mass/configuration; body angle plus wing a0; CP angular-velocity correction neglected'),indent=2))
    with (args.output/'throttle_map.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=['run','throttle_pct','speed_mps','speed_std_mps','vertical_speed_mps','steady'])
        writer.writeheader()
        for r in results:
            for p in r['points']:writer.writerow(dict(run=r['run'],**{k:p[k] for k in writer.fieldnames[1:]}))
    fig,ax=plt.subplots(figsize=(8,5))
    for i,r in enumerate(results):
        points=sorted([p for p in r['points'] if p['steady']],key=lambda p:p['throttle_pct'])
        ax.errorbar([p['throttle_pct'] for p in points],[p['speed_mps'] for p in points],
            yerr=[p['speed_std_mps'] for p in points],marker='o',linestyle='none',capsize=4,label=f'Flight {i+1}: accepted near-level segments')
    onsets=[r['approximate_model_stall_onset']['speed_mps'] for r in results if r['approximate_model_stall_onset']]
    if onsets:ax.axhspan(min(onsets),max(onsets),color='orange',alpha=.2,label='Approximate model stall-onset observations')
    ax.set(xlabel='Reported throttle (%)',ylabel='Velocity-derived speed (m/s)',title='Merlin SITL: calm-air throttle characterization')
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(args.output/'throttle_map.png',dpi=160)
    print(args.output)
if __name__=='__main__':main()
