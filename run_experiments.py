#!/usr/bin/env python3
"""Run headless line-follower experiments automatically and summarise them in docs/results.csv.

Run from a terminal where the workspace is sourced:
    cd ~/line_follower_ws && source install/setup.bash
    python3 src/line_follower/scripts/run_experiments.py --speeds 0.15 0.25 0.35 0.45
    python3 src/line_follower/scripts/run_experiments.py --runs 0.45,2.0,0.05 0.45,3.0,0.1

Each run:
  1. kills any leftover simulation processes,
  2. writes a parameter YAML (base_speed, kp, kd) and starts `sim.launch.py headless:=true follow:=true`,
  3. waits until lap_logger_node reports a finished lap, or the time limit is reached,
  4. stops the simulation and computes the metrics from the run's CSV.

Columns of docs/results.csv:
  run, base_speed, kp, ki, kd, completed, lap_time_s, mean_abs_error, max_abs_error, line_lost_count,
  mean_linear_speed, max_deviation_m, direction, wall_time_s, csv_file
    completed       1 if a lap was detected AND it was driven counter-clockwise (the normal direction)
    lap_time_s      simulation time of the first lap (empty if not completed)
    errors / speed  computed over the first lap (or the whole run if no lap)
    max_deviation_m largest distance between the robot centre and the line centre
    direction       ccw / cw / ? : driving direction around the track
    lap_distance_m  distance driven during lap 1
    pose_source     ground_truth (true pose from Gazebo) or odom (wheel odometry, drifts when wheels slip)
    mean_error_in_curves  mean error while the robot centre is on a curve (|x| > straight/2 + 0.2 m)
    steering_sign_flips   how often angular.z changes sign with a jump > 0.3 rad/s (oscillation)
"""
import argparse
import csv
import glob
import math
import os
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import track_geometry as tg  # noqa: E402

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
WS_DIR = os.path.abspath(os.path.join(PKG_DIR, '..', '..'))
RESULTS = os.path.join(PKG_DIR, 'docs', 'results.csv')
EXP_DIR = os.path.join(WS_DIR, 'logs', 'experiments')
FIELDS = ['run', 'base_speed', 'kp', 'ki', 'kd', 'completed', 'lap_time_s', 'mean_abs_error',
          'max_abs_error', 'line_lost_count', 'mean_linear_speed', 'max_deviation_m', 'direction',
          'lap_distance_m', 'pose_source', 'mean_error_in_curves', 'steering_sign_flips',
          'wall_time_s', 'csv_file']
SIM_PATTERNS = re.compile(r'ros2 launch line_follower|ign gazebo|ruby /usr/bin/ign|parameter_bridge|'
                          r'lib/line_follower/|ros_gz_sim/create')


def kill_leftovers():
    """Kill any simulation process still alive (by PID, never this script)."""
    me = {os.getpid(), os.getppid()}
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        out = subprocess.run(['ps', '-eo', 'pid=,args='], capture_output=True, text=True).stdout
        pids = [int(line.split(None, 1)[0]) for line in out.splitlines()
                if SIM_PATTERNS.search(line) and int(line.split(None, 1)[0]) not in me]
        if not pids:
            return 0
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
        time.sleep(3)
    return len(pids)


def write_params(path, base_speed, kp, ki, kd, log_dir):
    with open(path, 'w') as f:
        f.write(f"""line_follower_node:
  ros__parameters:
    kp: {kp}
    ki: {ki}
    kd: {kd}
    base_speed: {base_speed}
    min_speed: 0.05
    max_angular: 2.0
    search_speed: 1.0
    line_threshold: 0.4
    integral_limit: 1.0
    sensor_timeout: 0.5
lap_logger_node:
  ros__parameters:
    log_dir: "{log_dir}"
    log_rate: 10.0
    lap_radius: 0.15
    min_lap_distance: 5.0
""")


def analyse(run_dir, stdout_log):
    csvs = sorted(glob.glob(os.path.join(run_dir, 'run_*[0-9].csv')))
    if not csvs:
        return None
    rows = list(csv.DictReader(open(csvs[-1])))
    laps = list(csv.DictReader(open(csvs[-1].replace('.csv', '_laps.csv'))))
    lap_time = float(laps[0]['lap_time']) if laps else None
    if lap_time is not None:
        rows = [r for r in rows if float(r['time']) <= lap_time]
    if not rows:
        return None
    err = [abs(float(r['error'])) for r in rows]
    spd = [float(r['linear_x']) for r in rows]
    use_gt = all(r.get('gt_x') not in (None, '') for r in rows)
    if use_gt:
        pts = [(float(r['gt_x']), float(r['gt_y'])) for r in rows]
    else:
        pts = [tg.odom_to_world(float(r['x']), float(r['y'])) for r in rows]
    dev = [tg.distance_to_line(x, y) for x, y in pts]
    # Signed area (shoelace) of the driven path: > 0 means counter-clockwise
    area = 0.5 * sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]))
    direction = 'ccw' if area > 0.5 else 'cw' if area < -0.5 else '?'
    curve = [float(r['error']) for r, (x, _) in zip(rows, pts) if abs(x) > tg.STRAIGHT / 2 + 0.2]
    w = [float(r['angular_z']) for r in rows]
    flips = sum(1 for a, b in zip(w, w[1:]) if a * b < 0 and abs(a - b) > 0.3)
    lost = len(re.findall(r'Line lost', open(stdout_log).read()))
    completed = lap_time is not None and direction == 'ccw'
    return {
        'completed': int(completed),
        'lap_time_s': f'{lap_time:.2f}' if completed else '',
        'mean_abs_error': f'{sum(err) / len(err):.4f}',
        'max_abs_error': f'{max(err):.4f}',
        'line_lost_count': lost,
        'mean_linear_speed': f'{sum(spd) / len(spd):.4f}',
        'max_deviation_m': f'{max(dev):.4f}',
        'direction': direction,
        'lap_distance_m': f"{float(laps[0]['distance']):.2f}" if laps else '',
        'pose_source': 'ground_truth' if use_gt else 'odom',
        'mean_error_in_curves': f'{sum(curve) / len(curve):.4f}' if curve else '',
        'steering_sign_flips': flips,
        'csv_file': os.path.relpath(csvs[-1], WS_DIR),
    }


def run_one(base_speed, kp, ki, kd, timeout):
    name = f'v{base_speed:.2f}_kp{kp:g}_ki{ki:g}_kd{kd:g}'
    run_dir = os.path.join(EXP_DIR, name)
    os.makedirs(run_dir, exist_ok=True)
    for old in glob.glob(os.path.join(run_dir, '*')):
        os.remove(old)
    params = os.path.join(run_dir, 'params.yaml')
    stdout_log = os.path.join(run_dir, 'launch.log')
    write_params(params, base_speed, kp, ki, kd, run_dir)

    leftovers = kill_leftovers()
    print(f'\n=== {name}  (timeout {timeout:.0f} s, killed {leftovers} leftover processes)', flush=True)
    t0 = time.time()
    with open(stdout_log, 'w') as out:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'line_follower', 'sim.launch.py', 'headless:=true', 'follow:=true',
             f'params_file:={params}'],
            stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
        while time.time() - t0 < timeout:
            time.sleep(2)
            if 'LAP 1 finished' in open(stdout_log).read():
                break
            if proc.poll() is not None:
                print('  launch exited early!', flush=True)
                break
        wall = time.time() - t0
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            pass
    kill_leftovers()

    res = analyse(run_dir, stdout_log) or {'completed': 0, 'direction': '?', 'line_lost_count': ''}
    res.update({'run': name, 'base_speed': base_speed, 'kp': kp, 'ki': ki, 'kd': kd,
                'wall_time_s': f'{wall:.0f}'})
    print('  ' + ', '.join(f'{k}={res.get(k, "")}' for k in FIELDS[5:15]), flush=True)
    return res


def save(results):
    """Merge into docs/results.csv (a run with the same name is replaced)."""
    os.makedirs(os.path.dirname(RESULTS), exist_ok=True)
    old = list(csv.DictReader(open(RESULTS))) if os.path.exists(RESULTS) else []
    by_name = {r['run']: r for r in old}
    for r in results:
        by_name[r['run']] = r
    with open(RESULTS, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        for r in by_name.values():
            w.writerow({k: r.get(k, '') for k in FIELDS})


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--speeds', type=float, nargs='*', default=[], help='base_speed sweep (uses --kp/--kd)')
    ap.add_argument('--kp', type=float, default=1.0)
    ap.add_argument('--ki', type=float, default=0.0)
    ap.add_argument('--kd', type=float, default=0.0)
    ap.add_argument('--runs', nargs='*', default=[], help='extra runs as speed,kp,kd')
    ap.add_argument('--reanalyse', action='store_true',
                    help='recompute metrics of the runs already in docs/results.csv from their logs (no simulation)')
    ap.add_argument('--timeout-factor', type=float, default=1.8,
                    help='time limit = factor * perimeter / base_speed + 40 s startup')
    args = ap.parse_args()

    if args.reanalyse:
        old = list(csv.DictReader(open(RESULTS)))
        for r in old:
            run_dir = os.path.join(EXP_DIR, r['run'])
            r.update(analyse(run_dir, os.path.join(run_dir, 'launch.log')) or {})
        save(old)
        print(f'Re-analysed {len(old)} runs in {RESULTS}')
        return

    jobs = [(v, args.kp, args.ki, args.kd) for v in args.speeds]
    for spec in args.runs:
        v, kp, kd = (float(s) for s in spec.split(','))
        jobs.append((v, kp, args.ki, kd))
    if not jobs:
        ap.error('nothing to run: give --speeds and/or --runs')

    results = []
    for v, kp, ki, kd in jobs:
        timeout = args.timeout_factor * tg.PERIMETER / v + 40
        results.append(run_one(v, kp, ki, kd, timeout))
        save(results)   # save after every run so nothing is lost if interrupted
    print(f'\nSaved {len(results)} runs to {RESULTS}')


if __name__ == '__main__':
    main()
