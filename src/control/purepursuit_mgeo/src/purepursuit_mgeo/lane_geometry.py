"""Move a camera observation between ego poses without moving the road."""
import math


def pose_at(history, timestamp, tolerance=0.15):
    samples = list(history)
    if not samples or not math.isfinite(timestamp):
        return None
    if timestamp < samples[0][0]:
        return samples[0][1] if samples[0][0]-timestamp <= tolerance else None
    if timestamp > samples[-1][0]:
        return samples[-1][1] if timestamp-samples[-1][0] <= tolerance else None
    for (ta, a), (tb, b) in zip(samples, samples[1:]):
        if ta <= timestamp <= tb:
            if timestamp == ta:
                return a
            if timestamp == tb:
                return b
            if tb-ta > 2*tolerance:
                return None
            u = (timestamp-ta)/max(tb-ta, 1e-9)
            angle = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
            return (a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1]), a[2]+u*angle)
    return None


def reproject(points, observed_pose, current_pose):
    ox, oy, oa = observed_pose[:3]
    cx, cy, ca = current_pose[:3]
    co, so, cc, sc = math.cos(oa), math.sin(oa), math.cos(ca), math.sin(ca)
    out = []
    for x, y in points:
        mx, my = ox+co*x-so*y-cx, oy+so*x+co*y-cy
        out.append((cc*mx+sc*my, -sc*mx+cc*my))
    return out
