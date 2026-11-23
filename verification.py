"""Run the straight, bend and line-loss checks using live ROS topics."""

import csv
import json
import math
from pathlib import Path
import time

from typing import NamedTuple

from .track import LOSS_ZONE_M, TURN_END_M, TURN_START_M, centerline, distance_to_track


class OdometrySample(NamedTuple):
    stamp_s: float
    x: float
    y: float
    linear: float
    angular: float


class Verification:
    def __init__(self, directory, log_info):
        self.directory = Path(directory)
        self.log_info = log_info
        self.points = centerline()
        self.start = time.monotonic()
        self.odom = self.command = self.offset = None
        self.odom_received = self.command_received = self.offset_received = -math.inf
        self.samples = {'straight': [], 'turns': []}
        self.furthest_x = 0.0
        self.stop_since = None
        self.result = None
        self.saw_line = False
        self.csv_file = (self.directory / 'tracking.csv').open('w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            'wall_s', 'odom_sim_s', 'x_m', 'y_m', 'tracking_error_m',
            'line_offset', 'cmd_linear_mps', 'cmd_angular_rad_s',
            'odom_linear_mps', 'odom_angular_rad_s',
        ])

    def on_odom(self, msg):
        self.odom = msg
        self.odom_received = time.monotonic()

    def on_command(self, msg):
        self.command = msg
        self.command_received = time.monotonic()

    def on_offset(self, msg):
        self.offset = msg
        self.offset_received = time.monotonic()
        self.saw_line |= math.isfinite(msg)

    def sample(self):
        now = time.monotonic()
        elapsed = now - self.start
        fresh = all(now - received < 0.65 for received in (
            self.odom_received, self.command_received, self.offset_received,
        ))
        if elapsed > 180:
            self.finish(False, 'Timed out before verifying a stop beyond the final bends.')
            return
        if elapsed > 50 and not self.saw_line:
            self.finish(False, 'No track detection within 50 seconds. Check the camera, bridge and rendering logs.')
            return
        if not fresh:
            self.stop_since = None
            return

        stamp_s, x, y, odom_linear, odom_angular = self.odom
        cmd_linear, cmd_angular = self.command
        self.furthest_x = max(self.furthest_x, x)
        error = distance_to_track(x, y, self.points)
        line_found = math.isfinite(self.offset)
        self.writer.writerow([
            round(elapsed, 3), stamp_s,
            x, y, error, self.offset,
            cmd_linear, cmd_angular,
            odom_linear, odom_angular,
        ])
        self.csv_file.flush()

        if line_found and 0.15 < x < TURN_START_M:
            self.samples['straight'].append(error)
        if line_found and TURN_START_M <= x <= TURN_END_M:
            self.samples['turns'].append(error)

        stopped = (not line_found and self.saw_line and
                   abs(cmd_linear) < 1e-9 and abs(cmd_angular) < 1e-9 and
                   abs(odom_linear) <= 0.015 and abs(odom_angular) <= 0.08)
        if not stopped:
            self.stop_since = None
            return
        if self.stop_since is None:
            self.stop_since = now
        if now - self.stop_since >= 1.0:
            reached_end = x >= LOSS_ZONE_M
            reason = ('Robot stopped in the finish zone after losing the line.' if reached_end else
                      'Robot lost the line and stopped before the finish zone.')
            self.finish(reached_end, reason)

    def finish(self, line_loss_passed, reason):
        checks = {}
        for name, errors in self.samples.items():
            rms = math.sqrt(sum(error ** 2 for error in errors) / len(errors)) if errors else None
            progressed = self.furthest_x >= (TURN_START_M if name == 'straight' else TURN_END_M)
            checks[name] = {
                'passed': progressed and rms is not None and rms <= 0.12,
                'samples': len(errors),
                'rms_tracking_error_m': rms,
                'max_tracking_error_m': max(errors, default=None),
            }
        checks['line_loss'] = {'passed': line_loss_passed}
        passed = all(check['passed'] for check in checks.values())
        self.result = {
            'status': 'passed' if passed else 'failed',
            'reason': reason,
            'furthest_x_m': self.furthest_x,
            'finish_zone_starts_at_m': LOSS_ZONE_M,
            'tracking_reference': 'wheel odometry versus the configured centerline',
            'checks': checks,
        }
        pending = self.directory / 'result.json.tmp'
        pending.write_text(json.dumps(self.result, indent=2) + '\n', encoding='utf-8')
        pending.replace(self.directory / 'result.json')
        self.log_info(f'Verification {self.result["status"]}. {reason}')
