"""Compare recorded (-vx,+vy) IMU fusion with (+vx,+vy), using the production C KF.

This replays recorded paired, gravity/lever-arm-corrected navigation inputs only.
It does not replay CAN, wheel pairing, attitude estimation or center compensation.
CSV inputs are never modified. Distance uses navigation axes, not rotating body axes.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import sys

from vofa_csv import _decode, integer, schemas, continuous_attempt_segments
from vofa_fusion import FusionBridge, analyze_fusion

ROOT = Path(__file__).resolve().parents[1]
Q_FIELDS = ('imu_qw', 'imu_qx', 'imu_qy', 'imu_qz')


def read_original(path):
    """Accept the known schema-zero first row only; reject every other malformed row."""
    path = Path(path).resolve()
    with path.open(encoding='utf-8-sig', newline='') as stream:
        table = list(csv.reader(stream))
    if not table or table[0] != [f'I{i}' for i in range(96)]:
        raise ValueError(f'{path}: expected the original I0..I95 schema-5 header')
    rows, excluded = [], []
    definition = schemas()[5]
    for line, values in enumerate(table[1:], 2):
        try:
            if len(values) != 96:
                raise ValueError('row width is not 96')
            schema = integer(values[0])
            if schema != 5:
                if line == 2 and schema == 0:
                    excluded.append(dict(line=line, reason='initial schema-zero row'))
                    continue
                raise ValueError(f'unsupported schema {schema}')
            row = _decode(values, 5, definition)
            if rows and (row['tick_ms'], row['tx_sequence']) == (rows[-1]['tick_ms'], rows[-1]['tx_sequence']):
                if row != {k: v for k, v in rows[-1].items() if k != 'line'}:
                    raise ValueError('duplicate frame has inconsistent content')
                continue
            row['line'] = line
            rows.append(row)
        except (ValueError, OverflowError) as error:
            raise ValueError(f'{path}: line {line}: {error}') from error
    if not rows:
        raise ValueError(f'{path}: no valid samples')
    return rows, dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                     data_rows=len(table)-1, decoded_rows=len(rows), excluded=excluded)


def latest_motion_segment(rows):
    chunks, chunk = [], []
    for row in rows:
        if chunk:
            previous = chunk[-1]
            if (not 0 < row['tick_ms']-previous['tick_ms'] <= 200 or
                    row['tx_sequence']-previous['tx_sequence'] != 1 or
                    row['config_fingerprint'] != previous['config_fingerprint']):
                chunks.append(chunk)
                chunk = []
        chunk.append(row)
    chunks.append(chunk)
    moving = [c for c in chunks if any(math.hypot(r['target_vx_mps'], r['target_vy_mps']) > .05 for r in c)]
    if not moving:
        raise ValueError('no continuous segment contains commanded translation >0.05 m/s')
    selected = moving[-1]
    return selected, [dict(first_line=c[0]['line'], last_line=c[-1]['line'],
        first_tick_ms=c[0]['tick_ms'], last_tick_ms=c[-1]['tick_ms'], rows=len(c)) for c in chunks]


class Orientations:
    def __init__(self, rows):
        self.values = {}
        for row in rows:
            if not row['valid'] & 8:
                raise ValueError(f"line {row['line']}: IMU validity bit is clear")
            q = [row[k] for k in Q_FIELDS]
            norm = math.sqrt(sum(v*v for v in q))
            if not .98 < norm < 1.02:
                raise ValueError(f"line {row['line']}: quaternion norm outside tolerance")
            self.values[row['imu_tick_ms']] = [v/norm for v in q]
        self.times = sorted(self.values)

    def at(self, tick):
        if tick in self.values:
            return self.values[tick], 'exact'
        index = bisect_left(self.times, tick)
        if index == 0:
            return self.values[self.times[0]], 'nearest_start_unknown_history'
        if index == len(self.times):
            raise ValueError('middle attitude is beyond recorded history')
        t0, t1 = self.times[index-1:index+1]
        if t1-t0 > 100:
            raise ValueError('attitude interpolation would cross a >100 ms gap')
        a, b = self.values[t0], self.values[t1]
        dot = sum(x*y for x, y in zip(a, b))
        if dot < 0:
            b, dot = [-v for v in b], -dot
        fraction = (tick-t0)/(t1-t0)
        if dot > .99999:
            return [(1-fraction)*x+fraction*y for x, y in zip(a, b)], 'slerp'
        angle = math.acos(min(1., dot))
        return [(math.sin((1-fraction)*angle)*x+math.sin(fraction*angle)*y)/math.sin(angle)
                for x, y in zip(a, b)], 'slerp'


def horizontal_matrix(q):
    norm = math.sqrt(sum(v*v for v in q))
    w, x, y, z = [v/norm for v in q]
    return (1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*y+z*w), 1-2*(x*x+z*z))


def rotate(matrix, vector, inverse=False):
    a, b, c, d = matrix
    x, y = vector
    if inverse:
        determinant = a*d-b*c
        if determinant <= .5:
            raise ValueError('horizontal attitude projection fails the production det>0.5 gate')
        return [(d*x-b*y)/determinant, (a*y-c*x)/determinant]
    return [a*x+b*y, c*x+d*y]


def changed_wheel(matrix, old_navigation):
    body = rotate(matrix, old_navigation, inverse=True)
    return rotate(matrix, [-body[0], body[1]])


def replay(rows, bridge):
    if any(abs(r[k]-v) > 2e-6 for r in rows
           for k,v in zip(('mount_qw','mount_qx','mount_qy','mount_qz'),(1,0,0,0))):
        raise ValueError('X-only comparison requires the recorded identity mounting quaternion')
    capture = dict(schema=5, rows=rows)
    baseline = analyze_fusion(capture, bridge)
    segments=continuous_attempt_segments(capture)
    starts={s[0]['line']:not bool(s[0]['attempt_flags'] & 8) for s in segments}
    calls = [r for segment in segments for r in segment]
    orientations, sources = Orientations(rows), Counter()
    state, trace, previous = bridge.new_state(), [], None
    baseline_state=None
    validation={key:dict(calls=0,max_error_mps=None,mismatches=0)
                for key in ('known_exact','warm_converged')}
    explained_resets=0
    for row in calls:
        if row['wheel_us'] != row['imu_us']:
            raise ValueError('this comparison requires the recorded paired wheel/IMU epoch')
        if previous and row['attempt_sequence']-previous['attempt_sequence'] != 1:
            state = bridge.new_state()
        if previous and row['resets_before'] != previous['resets_before']:
            if row['resets_before']-previous['resets_before']!=1 or previous['attempt_flags'] & 32:
                raise ValueError('recorded reset is not explained by the preceding KF rejection')
            explained_resets+=1
        if row['line'] in starts:
            baseline_state=bridge.new_state()
            exact, segment_index=starts[row['line']],0
        reproduced=bridge.update(baseline_state,row)
        if exact or segment_index>=100:
            metric=validation['known_exact' if exact else 'warm_converged']
            error=max(abs(reproduced['velocity_n_mps'][i]-row[k])
                      for i,k in enumerate(('output_n_x_mps','output_n_y_mps')))
            metric['calls']+=1
            metric['max_error_mps']=max(metric['max_error_mps'] or 0,error)
            metric['mismatches']+=error>2e-5 or reproduced['fused']!=bool(row['attempt_flags']&32) or \
                reproduced['valid']!=bool(row['attempt_flags']&16) or reproduced['sample_us']!=row['output_sample_us']
        segment_index+=1
        q, source = orientations.at(row['middle_receiver_ms'])
        sources[source] += 1
        matrix = horizontal_matrix(q)
        old_wheel = [row['wheel_n_x_mps'], row['wheel_n_y_mps']]
        new_wheel = changed_wheel(matrix, old_wheel)
        candidate = dict(row, wheel_n_x_mps=new_wheel[0], wheel_n_y_mps=new_wheel[1])
        output = bridge.update(state, candidate)
        if not output['valid']:
            raise ValueError(f"line {row['line']}: candidate output became invalid")
        old = [row['output_n_x_mps'], row['output_n_y_mps']]
        old_body = rotate(matrix, old, inverse=True)
        new_body = rotate(matrix, output['velocity_n_mps'], inverse=True)
        trace.append(dict(line=row['line'], time=row['middle_receiver_ms']*.001,
            old=old, candidate=output['velocity_n_mps'], wheel_old=old_wheel, wheel_new=new_wheel,
            old_public=[-old_body[0], old_body[1]], candidate_public=new_body,
            old_fused=bool(row['attempt_flags'] & 32), candidate_fused=output['fused']))
        previous = row
    if len(trace) < 2 or any(b['time'] <= a['time'] for a, b in zip(trace, trace[1:])):
        raise ValueError('insufficient or nonmonotonic paired samples')
    baseline['validation']=validation
    baseline['explained_recorded_resets']=explained_resets
    return baseline, trace, dict(sources)


def integral(trace, key, start, end):
    if not trace[0]['time'] <= start < end <= trace[-1]['time']:
        raise ValueError('requested integration window is not fully recorded')
    total = [0., 0.]
    for a, b in zip(trace, trace[1:]):
        left, right = max(start, a['time']), min(end, b['time'])
        if left >= right:
            continue
        width = b['time']-a['time']
        for axis in range(2):
            slope = (b[key][axis]-a[key][axis])/width
            v0 = a[key][axis]+slope*(left-a['time'])
            v1 = a[key][axis]+slope*(right-a['time'])
            total[axis] += .5*(v0+v1)*(right-left)
    return total


def vector_stats(vectors):
    return dict(samples=len(vectors), mean=[statistics.mean(v[i] for v in vectors) for i in range(2)],
        rms_norm=math.sqrt(statistics.mean(sum(x*x for x in v) for v in vectors)),
        max_norm=max(math.hypot(*v) for v in vectors)) if vectors else None


def analyze_file(path, kind, bridge, window=None):
    rows, metadata = read_original(path)
    selected, chunks = latest_motion_segment(rows)
    if any(not r['valid'] & 2 for r in selected):
        raise ValueError('selected segment contains invalid targets; command-based window is unknown')
    active = [r for r in selected if math.hypot(r['target_vx_mps'], r['target_vy_mps']) > .05]
    groups = [[active[0]]]
    for row in active[1:]:
        if row['tick_ms']-groups[-1][-1]['tick_ms'] > 700:
            groups.append([])
        groups[-1].append(row)
    first, last = active[0]['tick_ms']*.001, active[-1]['tick_ms']*.001
    baseline, trace, q_sources = replay(selected, bridge)
    main_start, main_end = window if window else (first-.5, last+1.)
    if not main_start < main_end:
        raise ValueError('integration window start must precede end')
    if len(trace)<=100 or main_start<trace[99]['time']:
        raise ValueError('evaluation window starts before the 100-call/nominal-2s candidate warmup ends')
    scored = [t for t in trace[100:] if main_start <= t['time'] <= main_end]
    comparison = {}
    for key, wheel in (('old', 'wheel_old'), ('candidate', 'wheel_new')):
        comparison[key] = dict(rejected_calls=sum(not t[key+'_fused'] for t in trace),
            same_time_wheel_discrepancy=vector_stats([[t[key][i]-t[wheel][i] for i in range(2)] for t in scored]))
    windows = []
    if kind != 'spin':
        definitions=[('primary',main_start,main_end),('short_padding',first-.2,last+.5),
                     ('long_padding',first-1.,last+2.)]
        if len(groups)>1:
            main=max(groups,key=lambda g:g[-1]['tick_ms']-g[0]['tick_ms'])
            definitions.append(('longest_move_only',main[0]['tick_ms']*.001-.5,main[-1]['tick_ms']*.001+1.))
        for name, start, end in definitions:
            distances = {}
            for key in ('old', 'candidate', 'wheel_old', 'wheel_new'):
                displacement = integral(trace, key, start, end)
                length = math.hypot(*displacement)
                distances[key] = dict(navigation_displacement_m=displacement, net_distance_m=length,
                    error_m=length-3.2, error_percent=(length/3.2-1)*100)
            windows.append(dict(name=name, explicit_primary=bool(window) if name=='primary' else False,
                start_s=start, end_s=end, values=distances))
    tail_start = max(last+1., selected[-1]['tick_ms']*.001-2.)
    tail_rows = [r for r in selected if r['tick_ms']*.001 >= tail_start]
    tail = [t for t in trace if t['time'] >= tail_start]
    tail_stationary = bool(tail_rows) and all(math.hypot(r['target_vx_mps'], r['target_vy_mps']) <= .05 and
        math.hypot(r['wheel_vx_mps'], r['wheel_vy_mps']) < .03 and abs(r['imu_gz_radps']) < .05 for r in tail_rows)
    return dict(kind=kind, capture=metadata, continuous_segments=chunks,
        selected=dict(first_line=selected[0]['line'], last_line=selected[-1]['line'], rows=len(selected),
            start_s=selected[0]['tick_ms']*.001, end_s=selected[-1]['tick_ms']*.001,
            command_first_line=active[0]['line'], command_last_line=active[-1]['line'],
            command_start_s=first, command_end_s=last,
            command_groups=[dict(first_line=g[0]['line'],last_line=g[-1]['line'],
                start_s=g[0]['tick_ms']*.001,end_s=g[-1]['tick_ms']*.001) for g in groups]),
        actual_config={k: selected[-1][k] for k in ('config_fingerprint', 'fusion_wheel_variance',
            'fusion_accel_variance', 'fusion_jerk_density', 'fusion_max_residual_mps',
            'mount_qw', 'mount_qx', 'mount_qy', 'mount_qz', 'offset_x_m', 'offset_y_m', 'offset_z_m')},
        baseline_replay=baseline['replay'], baseline_segments=baseline['segments'],
        baseline_validation=baseline['validation'], explained_recorded_resets=baseline['explained_recorded_resets'],
        baseline_limitations=baseline['limitations'], orientation_sources=q_sources,
        scoring_window_s=[main_start,main_end], candidate_warmup_end_s=trace[99]['time'],
        max_paired_gap_s=max(b['time']-a['time'] for a,b in zip(trace,trace[1:])), comparison=comparison,
        distance_windows=windows, stationary_tail=dict(start_s=tail_start,
            sensor_conditions_consistent_with_stationary=tail_stationary,
            physical_stationarity_independently_verified=False,
            old_public=vector_stats([t['old_public'] for t in tail]),
            candidate_public=vector_stats([t['candidate_public'] for t in tail])))


def markdown(report):
    lines = ['# IMU X mapping replay', '',
        'Recorded paired navigation inputs replayed through the production C KF. Q/R and rejection limits unchanged.',
        'Old input mapping: (-vx,+vy); candidate: (+vx,+vy). Candidate public output uses (+X,+Y).',
        'CAN, pairing, attitude estimation and center compensation are not replayed. No hardware accuracy claim.', '',
        '| Trial | Selected CSV lines | Old / candidate rejects | Replay max error (known / warm) |',
        '|---|---:|---:|---:|']
    for r in report['trials']:
        s, c, b = r['selected'], r['comparison'], r['baseline_replay']
        validation=r['baseline_validation']
        errors=['n/a (0 calls)' if not validation[k]['calls'] else
                f"{validation[k]['max_error_mps']:.3g} m/s ({validation[k]['calls']} calls)"
                for k in ('known_exact','warm_converged')]
        lines.append(f"| {r['kind']} | {s['first_line']}–{s['last_line']} | {c['old']['rejected_calls']} / {c['candidate']['rejected_calls']} | {' / '.join(errors)} |")
    lines += ['', 'Net distance: navigation-vector integral magnitude, compared with the stated 3.2 m.',
        'Default window includes all translation groups: first command minus 0.5 s, last command plus 1 s.',
        'Explicit trial windows override that default; exact bounds and longest-move-only sensitivity are in JSON.', '',
        '| Trial | Old recorded KF | Candidate KF | Corrected wheel | Candidate error |', '|---|---:|---:|---:|---:|']
    for r in report['trials']:
        if r['distance_windows']:
            v=r['distance_windows'][0]['values']
            lines.append(f"| {r['kind']} | {v['old']['net_distance_m']:.4f} m | {v['candidate']['net_distance_m']:.4f} m | {v['wheel_new']['net_distance_m']:.4f} m | {v['candidate']['error_percent']:+.2f}% |")
    lines += ['', 'The JSON preserves hashes, line/time ranges, parameters, exact/SLERP counts, window sensitivity and tail speed.',
        'Unknown baseline initial states exclude the first 100 calls (nominal 2 s); only known reset states support exact comparisons.',
        'Candidate scoring and distance windows start after the same initial 100-call warmup.',
        'Recorded resets reset the baseline. The candidate retains its own state across baseline rejections; missing sequence calls reset it.',
        'SLERP fills missing intermediate attitudes after baseline reinitialization. The first prior attitude may be unavailable.',
        'Wheel discrepancy measures internal consistency, not independent velocity accuracy.', '']
    return '\n'.join(lines)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for kind in ('forward', 'left', 'right', 'spin'):
        parser.add_argument('--'+kind, type=Path, required=True)
        if kind!='spin':
            parser.add_argument('--'+kind+'-window',type=float,nargs=2,metavar=('START_S','END_S'),
                help='explicit integration bounds in receiver-clock seconds, including any end correction')
    parser.add_argument('--output-dir', type=Path, required=True)
    args=parser.parse_args(argv)
    if args.output_dir.exists():
        parser.error('output directory exists; choose a new result directory')
    args.output_dir.mkdir(parents=True)
    bridge=None
    try:
        bridge=FusionBridge(args.output_dir/'c-build')
        report=dict(schema='imu-x-mapping-replay/1', hardware_validated=False,
            method='paired-input KF counterfactual; navigation net distance; original Q/R and gates',
            trials=[analyze_file(getattr(args,k), k, bridge, getattr(args,k+'_window',None))
                for k in ('forward','left','right','spin')],
            sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in bridge.sources+[Path(__file__).resolve(), ROOT/'scripts/vofa_csv.py',ROOT/'scripts/vofa_fusion.py']})
        (args.output_dir/'summary.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf8')
        (args.output_dir/'report.md').write_text(markdown(report),encoding='utf8')
    except (ValueError, OSError, subprocess.CalledProcessError) as error:
        print(str(error),file=sys.stderr)
        return 2
    finally:
        if bridge:
            bridge.close()
    print(args.output_dir/'report.md')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
