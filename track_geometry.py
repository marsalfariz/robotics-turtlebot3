"""Shared track geometry for the analysis scripts (must match generate_track.py defaults)."""
import math

STRAIGHT = 3.0      # length of each straight [m]
RADIUS = 1.2        # curve radius [m]
START_X, START_Y, START_YAW = 0.0, -1.2, 0.0   # robot spawn pose in the world (sim.launch.py)
PERIMETER = 2 * STRAIGHT + 2 * math.pi * RADIUS


def odom_to_world(x, y):
    """/odom starts at (0, 0) at the spawn pose; convert to world coordinates."""
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    return START_X + c * x - s * y, START_Y + s * x + c * y


def distance_to_line(wx, wy):
    """Shortest distance from a world point to the centre of the black line [m]."""
    half = STRAIGHT / 2
    if abs(wx) <= half:
        return abs(abs(wy) - RADIUS)
    cx = half if wx > 0 else -half
    return abs(math.hypot(wx - cx, wy) - RADIUS)


def track_outline(n=400):
    """(xs, ys) of the line centre, for plotting."""
    xs, ys = [], []
    half = STRAIGHT / 2
    seg = [(STRAIGHT, 'line'), (math.pi * RADIUS, 'arc')] * 2
    total = sum(length for length, _ in seg)
    for i in range(n + 1):
        d = total * i / n
        if d <= STRAIGHT:                                   # bottom straight, left -> right
            xs.append(-half + d); ys.append(-RADIUS)
        elif d <= STRAIGHT + math.pi * RADIUS:              # right curve
            a = -math.pi / 2 + (d - STRAIGHT) / RADIUS
            xs.append(half + RADIUS * math.cos(a)); ys.append(RADIUS * math.sin(a))
        elif d <= 2 * STRAIGHT + math.pi * RADIUS:          # top straight, right -> left
            xs.append(half - (d - STRAIGHT - math.pi * RADIUS)); ys.append(RADIUS)
        else:                                               # left curve
            a = math.pi / 2 + (d - 2 * STRAIGHT - math.pi * RADIUS) / RADIUS
            xs.append(-half + RADIUS * math.cos(a)); ys.append(RADIUS * math.sin(a))
    return xs, ys
