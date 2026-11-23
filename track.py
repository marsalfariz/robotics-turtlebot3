import argparse
import math
from pathlib import Path


TURN_START_M = 1.10
TURN_LENGTH_M = 3.80
TURN_END_M = TURN_START_M + TURN_LENGTH_M
FINISH_M = 6.10
LOSS_ZONE_M = FINISH_M - 0.62
TRACK_WIDTH_M = 0.060


def center_y(x):
    if TURN_START_M < x < TURN_END_M:
        phase = (x - TURN_START_M) / TURN_LENGTH_M
        return 0.46 * (1.0 - math.cos(2.0 * math.pi * phase))
    return 0.0


def centerline():
    intervals = 130
    start = -0.35
    return [
        (x, center_y(x))
        for x in (start + (FINISH_M - start) * i / intervals for i in range(intervals + 1))
    ]


def distance_to_track(x, y, points):
    nearest = math.inf
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        fraction = ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)
        fraction = max(0.0, min(1.0, fraction))
        nearest = min(nearest, math.hypot(x - ax - fraction * dx, y - ay - fraction * dy))
    return nearest


def render_line():
    points = centerline()
    visuals = []
    for i, ((ax, ay), (bx, by)) in enumerate(zip(points, points[1:])):
        length = math.hypot(bx - ax, by - ay) + 0.006
        yaw = math.atan2(by - ay, bx - ax)
        visuals.append(f"""
          <visual name="line_{i}">
            <pose>{(ax + bx) / 2:.6f} {(ay + by) / 2:.6f} 0.0005 0 0 {yaw:.6f}</pose>
            <geometry><box><size>{length:.6f} {TRACK_WIDTH_M} 0.001</size></box></geometry>
            <cast_shadows>false</cast_shadows>
            <material><ambient>0.012 0.012 0.012 1</ambient><diffuse>0.012 0.012 0.012 1</diffuse></material>
          </visual>""")
    return '\n'.join(visuals)


def render_world(template):
    placeholder = '<!-- TRACK_VISUALS -->'
    if template.count(placeholder) != 1:
        raise ValueError('Expected one track placeholder in the world template')
    return template.replace(placeholder, render_line())


def main():
    from ament_index_python.packages import get_package_share_directory

    parser = argparse.ArgumentParser(description='Generate the line-follower Gazebo world.')
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    template = Path(get_package_share_directory('line_follower')) / 'worlds' / 'track.sdf.in'
    args.output.write_text(render_world(template.read_text(encoding='utf-8')), encoding='utf-8')
