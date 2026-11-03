#!/usr/bin/env python3
"""Recalculate rig motion and continuous events using the final VPR alignment."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def execute(script, *args):
    command = [sys.executable, str(HERE/script), *map(str,args)]
    print('[STEP] '+' '.join(command),flush=True)
    subprocess.run(command,check=True,env={**os.environ,'MPLBACKEND':'Agg'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage',choices=('motion','complete'))
    parser.add_argument('--experiments-root',type=Path,default=Path.home()/'Documents/code/Experiments')
    parser.add_argument('--vpr-results',type=Path,required=True,help='Final completed VPR results directory')
    parser.add_argument('--output-root',type=Path,required=True,help='New output directory')
    args = parser.parse_args()
    output = args.output_root.expanduser().resolve()
    if output.exists():
        raise FileExistsError('Choose a new output directory; previous evaluations are preserved')
    vpr = args.vpr_results.expanduser().resolve()
    if not (vpr/'run_complete.json').is_file():
        raise ValueError('Expected a completed final VPR results directory')
    extra = ['--events'] if args.stage=='complete' else []
    execute('evaluate_rig.py','--experiments-root',args.experiments_root,
        '--vpr-results',vpr,'--output-root',output,
        '--ramp-seconds',8,'--ramp-reference-rpm',500,*extra)
    if args.stage == 'complete':
        execute('sad_vpr.py','stationary','--source-results',vpr,
            '--duration-s',.5,'--guard-s',.2,'--startup-s',.2,'--bin-ms',.5,
            '--output-root',output/'stationary')
        required = [output/'rig_evaluation_complete.json',output/'stationary/stationary_complete.json']
        for path in required:
            if not path.is_file():
                raise ValueError(f'Missing completion marker: {path}')
        stamp = dict(completed_utc=datetime.now(timezone.utc).isoformat(),
            accepted_vpr_results=str(vpr),
            vpr_protocol_sha256=hashlib.sha256((vpr/'protocol_snapshot.json').read_bytes()).hexdigest(),
            completion_files={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in required})
        (output/'rig_event_pipeline_complete.json').write_text(json.dumps(stamp,indent=2)+'\n',encoding='utf-8')
    print(f'[DONE] {args.stage}: {output}',flush=True)


if __name__ == '__main__':
    try:
        main()
    except (OSError,ValueError,KeyError,subprocess.CalledProcessError) as exc:
        sys.exit(f'[ERROR] {exc}')
