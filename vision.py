"""Camera centroid and velocity law, independent of ROS message classes."""

import math


ENCODING_CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
FRAME_TIMEOUT_S = 0.45


def line_offset(data, width, height, step, encoding):
    """Return the line position in [-1, 1], positive on the image right."""
    channels = ENCODING_CHANNELS.get(encoding)
    if (channels is None or width < 24 or height < 24 or
            step < width * channels or len(data) < step * height):
        return None

    def centroid(start, end):
        column_sum = 0
        count = 0
        row_start, row_end = int(height * start), int(height * end)
        for row in range(row_start, row_end, 2):
            for column in range(0, width, 2):
                pixel = row * step + column * channels
                if channels == 1:
                    dark = data[pixel] < 75
                else:
                    dark = max(data[pixel], data[pixel + 1], data[pixel + 2]) < 75
                if dark:
                    column_sum += column
                    count += 1
        if count < 8:
            return None
        return 2.0 * (column_sum / count) / (width - 1) - 1.0

    near = centroid(0.63, 0.84)
    if near is None:
        return None
    far = centroid(0.41, 0.58)
    return near if far is None else 0.30 * near + 0.70 * far


def velocity(offset, frame_age_s, *, speed, gain, max_yaw):
    if offset is None or not math.isfinite(offset) or frame_age_s > FRAME_TIMEOUT_S:
        return 0.0, 0.0
    yaw = max(-max_yaw, min(max_yaw, -gain * offset))
    return speed * (1.0 - 0.55 * min(1.0, abs(offset))), yaw
