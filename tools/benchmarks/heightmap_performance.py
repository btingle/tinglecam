#!/usr/bin/env python3
"""Measure surface finishing, grid resolution, retained payload and process RSS."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=root / 'build/tinglecam')
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--runs', type=int, default=1)
    parser.add_argument('--use-user-config', action='store_true')
    parser.add_argument('--ball-tool', default='6mm-ball-alu')
    parser.add_argument('--rough-tool', default='3.175mm-square')
    parser.add_argument('--tracks', type=Path, default=root / 'tracks_step_dxf')
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    out = (args.output or Path(tempfile.mkdtemp(prefix='tinglecam-heightmap-'))).resolve()
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    if not args.use_user_config:
        env['XDG_CONFIG_HOME'] = str(out / 'config')
    rough = ['--part', 'tests/fixtures/meshes/LsJig1.obj', '--box-stock', '5',
             '--mode', 'rough', '--tool', args.rough_tool, '--machine', 'makera-z1',
             '--material', 'hdpe', '--stepdown', '3', '--stepover', '.2',
             '--finishing-subdivisions', '4', '--path-pitch', '.2']
    cases = [('rough', rough + ['--simulation-pitch', '.1']),
             ('rough_fine', rough + ['--simulation-pitch', '.025'])]
    for model, stepover, pitch in [('s', '2', '.1'), ('L', '4', '.1'), ('s', '.3', '.025')]:
        path = args.tracks / f'{model}.step'
        if not path.exists():
            print(f'Skipping missing generated track: {path}', flush=True)
            continue
        name = f'finish_{model}' if pitch == '.1' else 'finish_fine'
        cases.append((name, ['--part', str(path.resolve()), '--box-stock', '0',
                             '--mode', 'surface', '--face', '8', '--tool', args.ball_tool,
                             '--machine', 'grbl-router', '--material', 'hdpe',
                             '--surface-stepover', stepover, '--simulation-pitch', pitch]))
    rows = []
    for binary, label in [(args.baseline, 'before'), (args.binary, 'after')]:
        if binary is None:
            continue
        for run in range(1, args.runs + 1):
            for name, options in cases:
                # A fine-grid baseline can be prohibitively slow; compare the
                # same coarse cases and measure fine resolution on the new engine.
                if label == 'before' and 'fine' in name:
                    continue
                stem = out / f'{label}-{name}-{run}'
                command = [str(binary.resolve()), *options, '--profile-planning',
                           '--gcode', str(stem.with_suffix('.nc'))]
                timed = ['/usr/bin/time', '-f', '%e %U %S %M', '-o',
                         str(stem.with_suffix('.time')), *command]
                with stem.with_suffix('.log').open('w') as log:
                    result = subprocess.run(timed, cwd=root, env=env, stdout=log, stderr=log)
                result.check_returncode()
                wall, user, system, rss = stem.with_suffix('.time').read_text().split()
                log = stem.with_suffix('.log').read_text()
                row = dict(version=label, case=name, run=run, wall_s=float(wall),
                           cpu_s=float(user)+float(system), max_rss_kib=int(rss),
                           sha256=hashlib.sha256(stem.with_suffix('.nc').read_bytes()).hexdigest())
                for field in ('planning_profile_ms', 'memory_profile'):
                    match = re.search(field + r' ([^\n\r]+)', log)
                    if match:
                        row.update({key: float(value) for key, value in re.findall(r'(\w+)=([0-9.]+)', match[1])})
                rows.append(row)
                fields = list(dict.fromkeys(key for item in rows for key in item))
                with (out / 'results.csv').open('w', newline='') as stream:
                    writer = csv.DictWriter(stream, fields)
                    writer.writeheader()
                    writer.writerows(rows)
                print(json.dumps(row), flush=True)
    print(f'Artifacts: {out}', flush=True)


if __name__ == '__main__':
    main()
