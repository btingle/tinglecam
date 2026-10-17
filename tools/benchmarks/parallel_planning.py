#!/usr/bin/env python3
"""Compare worker counts on identical LsJig roughing inputs and G-code."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import statistics
import subprocess
import tempfile
import time


def main():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, default=root / 'build/tinglecam')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--tool', default='3.175mm-square')
    parser.add_argument('--profile-workers', action='store_true',
                        help='Record batch and individual layer elapsed times')
    parser.add_argument('--config', type=Path,
                        help='XDG_CONFIG_HOME; otherwise use isolated shipped defaults')
    args = parser.parse_args()
    if args.runs < 1:
        parser.error('--runs must be positive')
    out = args.output or Path(tempfile.mkdtemp(prefix='tinglecam-workers-'))
    out.mkdir(parents=True, exist_ok=True)
    config = args.config or out / 'config'
    config.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, XDG_CONFIG_HOME=str(config.resolve()))
    if args.profile_workers:
        env['TINGLECAM_PROFILE_WORKERS'] = '1'
    rows = []
    expected_hash = None
    print(f'Benchmark artifacts: {out}', flush=True)
    for run in range(1, args.runs + 1):
        # Alternate order to reduce systematic warmup/temperature bias.
        counts = [1, 2, 4, 8] if run % 2 else [8, 4, 2, 1]
        for workers in counts:
            stem = out / f'workers-{workers}-{run}'
            cmd = [str(args.binary.resolve()), '--part',
                   str(root / 'tests/fixtures/meshes/LsJig1.obj'),
                   '--box-stock', '5', '--mode', 'rough', '--tool', args.tool,
                   '--machine', 'makera-z1', '--material', 'hdpe',
                   '--stepdown', '3', '--stepover', '0.2',
                   '--finishing-subdivisions', '4', '--path-pitch', '0.2',
                   '--simulation-method', 'local_area', '--profile-planning',
                   '--gcode', str(stem.with_suffix('.nc').resolve())]
            resources = stem.with_suffix('.resources')
            cmd = ['/usr/bin/time', '-f', 'max_rss_kb=%M minor_faults=%R major_faults=%F',
                   '-o', str(resources.resolve()), *cmd]
            before = resource.getrusage(resource.RUSAGE_CHILDREN)
            start = time.monotonic()
            result = subprocess.run(cmd, env=dict(env, TINGLECAM_PLANNING_WORKERS=str(workers)),
                                    capture_output=True, text=True, cwd=root)
            wall = time.monotonic() - start
            after = resource.getrusage(resource.RUSAGE_CHILDREN)
            stem.with_suffix('.out').write_text(result.stdout)
            stem.with_suffix('.err').write_text(result.stderr)
            result.check_returncode()
            match = re.search(r'planning_profile_ms (.*)', result.stderr)
            if not match:
                raise RuntimeError(f'Missing profile in {stem}.err')
            metrics = dict(re.findall(r'(\w+)=([0-9.]+)', match.group(1)))
            digest = hashlib.sha256(stem.with_suffix('.nc').read_bytes()).hexdigest()
            if expected_hash is None:
                expected_hash = digest
            if digest != expected_hash:
                raise RuntimeError(f'G-code differs with {workers} workers: {stem}')
            memory = {key: int(value) for key, value in
                      re.findall(r'(\w+)=(\d+)', resources.read_text())}
            row = dict(workers=workers, run=run, wall_s=wall,
                       cpu_s=after.ru_utime + after.ru_stime - before.ru_utime - before.ru_stime,
                       **{key + '_s': float(value) / 1000 for key, value in metrics.items()},
                       **memory, sha256=digest)
            rows.append(row)
            with (out / 'results.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=row.keys())
                writer.writeheader()
                writer.writerows(rows)
            print(json.dumps(row), flush=True)
    for workers in [1, 2, 4, 8]:
        samples = [row for row in rows if row['workers'] == workers]
        print(workers, {key: statistics.median(row[key] for row in samples)
                        for key in rows[0] if key.endswith('_s')}, flush=True)


if __name__ == '__main__':
    main()
