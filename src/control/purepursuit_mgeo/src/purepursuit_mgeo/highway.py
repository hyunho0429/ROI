"""Single-target Frenet highway supervisor. Pure Python, metres/seconds/radians.

Road geometry is world-fixed, never aligned to the turning vehicle's yaw.
One committed polynomial ends at exactly one adjacent lane with zero lateral
slope and acceleration. Camera association cannot move that target mid-change.
"""
import math
from dataclasses import dataclass, field
from typing import Optional


def clamp(x, low, high):
    return max(low, min(high, x))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Config:
    cruise_speed: float = 80.0 / 3.6
    lane_hold_s: float = 5.0
    change_time_s: float = 4.0
    lateral_accel: float = 1.8
    lateral_jerk: float = 2.5
    lane_timeout: float = 1.0
    lidar_timeout: float = 0.5
    odom_timeout: float = 0.3
    center_tolerance: float = 0.22
    heading_tolerance: float = math.radians(1.5)
    width_min: float = 2.7
    width_max: float = 4.3
    vehicle_width: float = 1.892
    vehicle_length: float = 4.635
    center_offset: float = 1.50
    headway: float = 1.5
    standstill_gap: float = 5.0
    normal_decel: float = 3.0
    max_accel: float = 1.5

    def __post_init__(self):
        if not math.isfinite(self.cruise_speed) or self.cruise_speed < 0:
            raise ValueError('cruise_speed must be finite and nonnegative')
        if not math.isfinite(self.lane_hold_s) or self.lane_hold_s < 5.:
            raise ValueError('lane_hold_s must be at least five seconds')


@dataclass
class Ego:
    x: float
    y: float
    yaw: float
    speed: float


@dataclass
class Obstacle:
    id: int
    x: float
    y: float
    yaw: float
    length: float
    width: float
    vx: float
    vy: float


@dataclass(frozen=True)
class Road:
    """Local constant-curvature centre reference, with exact arc projection."""
    x: float
    y: float
    yaw: float
    curvature: float = 0.0

    def heading(self, s):
        return self.yaw + self.curvature * s

    def point(self, s, d=0.0):
        a = self.heading(s)
        k = self.curvature
        if abs(k) < 1e-7:
            x, y = self.x + s*math.cos(self.yaw), self.y + s*math.sin(self.yaw)
        else:
            x = self.x + (math.sin(a)-math.sin(self.yaw))/k
            y = self.y - (math.cos(a)-math.cos(self.yaw))/k
        return x-d*math.sin(a), y+d*math.cos(a)

    def project(self, x, y):
        dx, dy = x-self.x, y-self.y
        u = math.cos(self.yaw)*dx + math.sin(self.yaw)*dy
        v = -math.sin(self.yaw)*dx + math.cos(self.yaw)*dy
        k = self.curvature
        if abs(k) < 1e-7:
            return u, v
        a = math.atan2(k*u, 1-k*v)
        return a/k, (1-math.hypot(k*u, 1-k*v))/k

    def shifted(self, s, d=0.0):
        x, y = self.point(s, d)
        return Road(x, y, self.heading(s), self.curvature/(1-self.curvature*d))


@dataclass
class Lane:
    road: Road
    width: float
    left: str
    right: str
    stamp: float
    target_width: Optional[float] = None
    left_composite: bool = False

    @property
    def final(self):
        return (self.left in ('white_solid', 'yellow') or self.left_composite) and self.right == 'white_dashed'

    @property
    def may_change(self):
        if self.final:
            return False
        return self.left == 'white_dashed' or (self.left_composite and self.right == 'white_solid')


def _quadratic(points):
    # Least squares in scaled coordinates; no SciPy/numpy runtime dependency.
    if len(points) < 6:
        raise ValueError('too_few_boundary_points')
    rows = [(1., x/20., (x/20.)**2) for x, _ in points]
    a = [[sum(r[i]*r[j] for r in rows) for j in range(3)] +
         [sum(r[i]*p[1] for r, p in zip(rows, points))] for i in range(3)]
    for i in range(3):
        pivot = max(range(i, 3), key=lambda j: abs(a[j][i]))
        a[i], a[pivot] = a[pivot], a[i]
        if abs(a[i][i]) < 1e-9:
            raise ValueError('degenerate_boundary_fit')
        q = a[i][i]
        a[i] = [v/q for v in a[i]]
        for j in range(3):
            if j != i:
                q = a[j][i]
                a[j] = [v-q*w for v, w in zip(a[j], a[i])]
    c = a[0][3], a[1][3]/20., a[2][3]/400.
    rms = math.sqrt(sum((y-c[0]-c[1]*x-c[2]*x*x)**2 for x,y in points)/len(points))
    if rms > .20:
        raise ValueError('boundary_fit_residual')
    return c


def lane_from_json(info, observed_pose, cfg=None):
    """Use the physical boundary midpoint; reject held/straddling/two-lane pairs.

    The adapter supplies the ego pose at the *original camera observation*.
    No single-boundary nominal-width fallback can authorize a lane change.
    """
    cfg = cfg or Config()
    if not info.get('lane_valid') or info.get('output_status') != 'FRESH':
        raise ValueError('lane_not_fresh')
    if info.get('frame_id', 'base_link') != 'base_link' or float(info.get('confidence', 1.)) < .45:
        raise ValueError('lane_frame_or_confidence')
    if info.get('straddling_lane', {}).get('detected', False):
        raise ValueError('straddling_boundary')
    for name in ('left_lane', 'right_lane'):
        m = info.get(name, {})
        if not m.get('detected') or m.get('coasted') or m.get('from_guide'):
            raise ValueError('unmeasured_boundary')
    pairs = []
    for key in ('left_boundary_points', 'right_boundary_points'):
        pts = [(float(p[0]), float(p[1])) for p in info.get(key, [])
               if len(p) >= 2 and 0 <= float(p[0]) <= 30]
        if any(not math.isfinite(x+y) for x,y in pts):
            raise ValueError('nonfinite_boundary')
        if len(pts) < 6 or max(x for x,_ in pts)-min(x for x,_ in pts) < 8:
            raise ValueError('short_boundary')
        pairs.append((pts, _quadratic(pts)))
    lo = max(min(x for x,_ in p[0]) for p in pairs)
    hi = min(max(x for x,_ in p[0]) for p in pairs)
    if hi-lo < 8 or lo > 5:
        raise ValueError('insufficient_common_boundary')
    left, right = pairs[0][1], pairs[1][1]
    if sum(left) <= 0 or sum(right) >= 0:
        raise ValueError('boundaries_do_not_bracket_vehicle')
    c = tuple((a+b)/2 for a,b in zip(left, right))
    widths = []
    for x in (lo, (lo+hi)/2, hi):
        width = sum((a-b)*x**i for i,(a,b) in enumerate(zip(left,right)))
        width /= math.sqrt(1+(c[1]+2*c[2]*x)**2)
        if not cfg.width_min <= width <= cfg.width_max:
            raise ValueError('invalid_lane_width')
        widths.append(width)
    if max(widths)-min(widths) > .45:
        raise ValueError('diverging_boundaries')
    ox, oy, yaw = observed_pose
    heading = math.atan(c[1])
    curvature = 2*c[2]/(1+c[1]*c[1])**1.5
    if abs(heading) > math.radians(25) or abs(curvature) > .008:
        raise ValueError('implausible_road_geometry')
    road = Road(ox-math.sin(yaw)*c[0], oy+math.cos(yaw)*c[0], yaw+heading, curvature)
    width = sum(widths)/len(widths)
    # If the outer boundary is measured, use its separation for the target width.
    target_width = None
    composite = False
    outer = info.get('left_outer_lane', {})
    if outer.get('detected') and not outer.get('coasted') and not outer.get('from_guide'):
        coef = outer.get('coef') or []
        # Camera coefficients are np.polyfit order [x^2, x, constant].
        if len(coef) == 3:
            x = min(10., hi)
            separation = (coef[0]*x*x+coef[1]*x+coef[2]) - sum(v*x**i for i,v in enumerate(left))
            if cfg.width_min <= separation <= cfg.width_max:
                target_width = separation
            # Entrance markings share the same physical divider, not two
            # different lanes somewhere on the left. Final solid/dashed lane
            # always wins over this entrance-only permission.
            span = outer.get('x_range_m') or []
            kinds = {info['left_lane'].get('type'), outer.get('type')}
            if len(span) == 2 and kinds == {'white_dashed', 'white_solid'}:
                start, end = max(lo,span[0]), min(hi,span[1])
                if end-start >= 8.:
                    composite = all(abs(coef[0]*x*x+coef[1]*x+coef[2]
                                        -sum(v*x**i for i,v in enumerate(left))) <= .20
                                    for x in (start,(start+end)/2,end))
    return Lane(road, width, info['left_lane'].get('type', ''),
                info['right_lane'].get('type', ''), float(info['timestamp']), target_width, composite)


@dataclass
class Change:
    road: Road
    length: float
    target: float
    coefficients: tuple
    speed: float

    @classmethod
    def create(cls, road, ego, width, cfg):
        s, d = road.project(ego.x, ego.y)
        road = road.shifted(s)
        slope = math.tan(wrap(ego.yaw-road.yaw))*(1-road.curvature*d)
        v = max(5., ego.speed)
        # Quintic bounds: max |S''|=5.774, max |S'''|=60. Allow feedback reserve.
        displacement = width-d
        length = max(25., v*cfg.change_time_s,
                     v*math.sqrt(5.774*abs(displacement)/cfg.lateral_accel),
                     v*(60*abs(displacement)/cfg.lateral_jerk)**(1/3))
        b = slope*length
        delta = width-d
        coefs = (d, b, 0., 10*delta-6*b, -15*delta+8*b, 6*delta-3*b)
        return cls(road, length, width, coefs, v)

    def lateral(self, s):
        if s >= self.length:
            return self.target
        u = clamp(s/self.length, 0., 1.)
        return sum(c*u**i for i,c in enumerate(self.coefficients))

    def point(self, s):
        return self.road.point(s, self.lateral(s))

    def admissible(self, speed, cfg):
        last = self.coefficients[0]
        for i in range(101):
            u = i/100.
            d = self.lateral(u*self.length)
            dd = sum(j*(j-1)*c*u**(j-2) for j,c in enumerate(self.coefficients) if j>=2)/self.length**2
            if d < last-.015 or d > self.target+.01:
                return False
            if speed*speed*(abs(dd)+abs(self.road.curvature)) > 2.5:
                return False
            last = d
        return True


@dataclass
class Result:
    state: str
    reason: str
    path: list = field(default_factory=list)
    target_speed: float = 0.
    stop: bool = False
    diagnostics: dict = field(default_factory=dict)


class Highway:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.state = 'OFF'
        self.road = None
        self.lane = None
        self.change = None
        self.locked = False
        self.changes = 0
        self.last_stamp = -math.inf
        self.last_good = -math.inf
        self.good_count = 0
        self.final_count = 0
        self.blocker = None
        self.centered_since = None
        self.gap_since = None
        self.gap_frames = 0
        self.last_lidar_stamp = None
        self.speed_command = None
        self.acceleration = 0.
        self.last_tick = None
        self.lane_reason = 'waiting_camera'

    def observe(self, lane, now, ego):
        if lane.stamp <= self.last_stamp or not -.05 <= now-lane.stamp <= self.cfg.lane_timeout:
            self.lane_reason = 'camera_timestamp_old_or_repeated'
            return False
        self.last_stamp = lane.stamp
        if self.state == 'OFF':
            # Before highway activation the car can follow other roads/turns.
            # Do not bind the highway corridor or latch final-lane markings yet.
            self.road, self.lane = lane.road, lane
            self.last_good = lane.stamp
            self.good_count += 1
            self.lane_reason = 'pre_activation_observation'
            return True
        if self.road is None:
            self.road = lane.road
        if self.change is not None:
            # The target is immutable even when the camera picks an adjacent pair.
            s, _ = self.change.road.project(ego.x, ego.y)
            expected = self.change.road.shifted(s, self.change.target)
        else:
            s, _ = self.road.project(ego.x, ego.y)
            expected = self.road.shifted(s)
        ms, md = expected.project(lane.road.x, lane.road.y)
        angle = wrap(lane.road.yaw-expected.heading(ms))
        # During the first half of a change the source pair remains visible:
        # it verifies road freshness but is NOT a new centre or final marking.
        if self.change is not None and abs(md+self.change.target) < .55 and abs(angle) < math.radians(3):
            self.last_good = lane.stamp
            self.lane_reason = 'source_lane_during_change'
            return False
        if abs(md) > .60 or abs(angle) > math.radians(3):
            self.good_count = 0
            self.final_count = 0
            self.lane_reason = 'camera_pair_does_not_match_target'
            return False
        self.last_good = lane.stamp
        self.good_count += 1
        self.lane = lane
        self.lane_reason = 'measured_target_lane'
        # A solid nearest left boundary is a veto immediately; lock final lane
        # permanently only when its matched pair is observed repeatedly.
        self.final_count = self.final_count+1 if lane.final else 0
        if self.final_count >= 3:
            self.locked = True
        if self.change is None:
            # Filter in world/road coordinates, not the changing ego frame.
            # Reanchor at the current vehicle station to avoid extrapolation drift.
            ls, _ = lane.road.project(ego.x, ego.y)
            measured = lane.road.shifted(ls)
            _, correction = expected.project(measured.x, measured.y)
            x, y = expected.point(0., .20*correction)
            self.road = Road(x, y, expected.yaw+.15*wrap(measured.yaw-expected.yaw),
                             .95*expected.curvature+.05*measured.curvature)
        return True

    def _objects(self, road, obstacles):
        for obj in obstacles:
            s, d = road.project(obj.x, obj.y)
            a = road.heading(s)
            angle = wrap(obj.yaw-a)
            half_s = (abs(math.cos(angle))*obj.length+abs(math.sin(angle))*obj.width)/2
            half_d = (abs(math.sin(angle))*obj.length+abs(math.cos(angle))*obj.width)/2
            vs = obj.vx*math.cos(a)+obj.vy*math.sin(a)
            vd = -obj.vx*math.sin(a)+obj.vy*math.cos(a)
            yield obj, s, d, half_s, half_d, vs, vd

    def _gap(self, change, ego, obstacles):
        c = self.cfg
        speed = max(ego.speed, 5.)
        records = list(self._objects(change.road, obstacles))
        slow_speed = max(2., ego.speed)
        for obj, s, d, hs, hd, vs, vd in records:
            if (0 < s < change.length+40.
                    and -hd-c.vehicle_width/2 < d < change.target+hd+c.vehicle_width/2):
                slow_speed = min(slow_speed, max(2., vs))
        # A slower lead may prolong the change. Rear clearance must also hold
        # if ACC slows to that lead; constant initial ego speed is insufficient.
        duration = change.length/slow_speed
        fast_duration = change.length/speed
        for obj, s, d, hs, hd, vs, vd in records:
            # Check future occupancy too: a moving vehicle can enter the target.
            if min(d, d+vd*duration)-hd > change.target+c.vehicle_width/2+.3 or max(d,d+vd*duration)+hd < -c.vehicle_width/2-.3:
                continue
            if abs(d-change.target) < hd+c.vehicle_width/2+.25:
                ds = s-c.center_offset
                gap = abs(ds)-hs-c.vehicle_length/2
                if ds >= 0:
                    required = c.standstill_gap + max(0., speed-vs)*fast_duration + .8*speed
                    if gap < required:
                        return False, 'target_front_gap', obj.id
                else:
                    required = c.standstill_gap + max(0., vs-slow_speed)*duration + .8*max(0.,vs)
                    if gap < required:
                        return False, 'target_rear_gap', obj.id
            # Sweep one path against constant-velocity boxes, including source
            # lane traffic and lateral cut-ins. No alternative lanes are sampled.
            for i in range(int(duration/.10)+2):
                t = min(duration, i*.10)
                low_s, high_s = slow_speed*t, speed*t
                low_d, high_d = sorted((change.lateral(low_s), change.lateral(high_s)))
                margin_s, margin_d = hs+c.vehicle_length/2+1., hd+c.vehicle_width/2+.30
                os, od = s+vs*t-c.center_offset, d+vd*t
                if (low_s-margin_s < os < high_s+margin_s
                        and low_d-margin_d < od < high_d+margin_d):
                    return False, 'swept_path_occupied', obj.id
        return True, 'gap_clear', None

    def _follow(self, road, ego, obstacles):
        c = self.cfg
        s, d = road.project(ego.x, ego.y)
        best, lead = math.inf, None
        for obj, os, od, hs, hd, vs, vd in self._objects(road, obstacles):
            ds = os-s-c.center_offset
            # Rear objects NEVER become a lead because ego yaw turns left.
            if ds <= 0 or ds > 160:
                continue
            # Actual corridor now and near-term swept corridor during a change.
            lateral = d
            if self.change is not None:
                t = clamp(ds/max(ego.speed, 5.), 0., 1.5)
                lateral = self.change.lateral(s+ego.speed*t)
            overlap = abs(od-d) < hd+c.vehicle_width/2+.12
            future_overlap = abs(od+vd*.8-lateral) < hd+c.vehicle_width/2+.12
            if not overlap and not future_overlap:
                continue
            gap = ds-hs-c.vehicle_length/2
            if gap < best:
                best, lead = gap, (obj.id, max(0.,vs))
        if lead is None:
            return c.cruise_speed, False, {}
        ident, v = lead
        desired = c.standstill_gap+c.headway*ego.speed
        target = clamp(v+.40*(best-desired), 0., c.cruise_speed)
        closing = max(0., ego.speed-v)
        ttc = best/closing if closing > .1 else math.inf
        emergency = best < 1.0 or (closing > .5 and (ttc < .8 or closing*closing/(2*6.) > best-1.))
        return target, emergency, dict(lead=ident, gap=round(best,2), lead_speed=round(v,2),
                                       desired_gap=round(desired,2), ttc=None if math.isinf(ttc) else round(ttc,2))

    def step(self, now, ego, obstacles, enabled, odom_age=0., lidar_age=0., lidar_stamp=None):
        c = self.cfg
        dt = .05 if self.last_tick is None else clamp(now-self.last_tick, .001, .2)
        self.last_tick = now
        if not enabled and self.state == 'OFF':
            return Result('OFF', 'base_path', target_speed=c.cruise_speed)
        if self.state == 'OFF':
            self.state = 'ACQUIRE'
        if self.road is None:
            return Result(self.state, 'waiting_measured_lane', stop=True)
        road = self.change.road if self.change is not None else self.road
        s, d = road.project(ego.x, ego.y)
        goal_d = self.change.target if self.change is not None else 0.
        yaw_error = wrap(ego.yaw-road.heading(s))
        centered = abs(d-goal_d) <= c.center_tolerance and abs(yaw_error) <= c.heading_tolerance
        fresh = now-self.last_good <= c.lane_timeout
        fresh_target = self.lane is not None and now-self.lane.stamp <= c.lane_timeout
        reason = 'lane_hold'
        if self.change is not None:
            reason = 'following_fixed_one_lane_path'
            if s >= self.change.length:
                self.state = 'SETTLE'
                reason = 'aligning_target_lane'
            if s >= self.change.length and centered and fresh_target and self.good_count >= 3:
                self.road = self.change.road.shifted(s, self.change.target)
                self.change = None
                self.changes += 1
                self.state = 'LOCKED' if self.locked else 'HOLD'
                self.centered_since = now
                self.good_count = 0
                road = self.road
                s,d = road.project(ego.x, ego.y)
                goal_d = 0.
                reason = 'change_complete_parallel'
        else:
            self.state = 'LOCKED' if self.locked else 'HOLD'
            if centered and fresh_target:
                if self.centered_since is None:
                    self.centered_since = now
            else:
                self.centered_since = None
            hold = .8 if self.changes == 0 else c.lane_hold_s
            ready = self.centered_since is not None and now-self.centered_since >= hold
            reason = 'final_lane_locked' if self.locked else 'centering_or_hold_timer'
            dashed = self.lane is not None and self.lane.may_change
            if not self.locked and not dashed:
                reason = 'nearest_left_not_dashed'
            can_change = (not self.locked and ready and fresh_target and dashed
                          and fresh and lidar_age <= c.lidar_timeout and odom_age <= c.odom_timeout
                          and self.good_count >= 3 and ego.speed >= 2.)
            if can_change:
                # Average source/target widths determines adjacent centre spacing.
                width = (self.lane.width+(self.lane.target_width or self.lane.width))/2
                candidate = Change.create(self.road, ego, width, c)
                ok, reason, blocker = self._gap(candidate, ego, obstacles)
                self.blocker = blocker
                if not candidate.admissible(ego.speed, c):
                    ok, reason = False, 'curvature_or_initial_heading_limit'
                self.state = 'WAIT_GAP'
                if ok:
                    if self.gap_since is None:
                        self.gap_since, self.gap_frames = now, 0
                    if lidar_stamp is not None and lidar_stamp != self.last_lidar_stamp:
                        self.gap_frames += 1
                    if now-self.gap_since >= .25 and self.gap_frames >= 3:
                        self.change = candidate
                        self.state = 'CHANGE'
                        self.good_count = 0
                        self.lane = None
                        self.centered_since = None
                        road = candidate.road
                        s,d = road.project(ego.x, ego.y)
                        goal_d = candidate.target
                        reason = 'lane_change_committed'
                else:
                    self.gap_since, self.gap_frames = None, 0
            else:
                self.gap_since, self.gap_frames = None, 0
        self.last_lidar_stamp = lidar_stamp
        target, emergency, follow = self._follow(road, ego, obstacles)
        if self.change is not None:
            target = min(target, self.change.speed)
        # Preserve lateral authority on bends; 80 km/h is the straight-road
        # cruise target, not permission to exceed the tyre/steering budget.
        target = min(target, math.sqrt(2.0/max(abs(road.curvature), 1e-6)))
        fault = None
        if odom_age > c.odom_timeout:
            fault = 'odometry_stale'
        elif lidar_age > c.lidar_timeout:
            fault = 'lidar_stale'
        elif not fresh:
            fault = 'lane_geometry_stale'
        if fault:
            target = 0.
            reason = fault
        if self.speed_command is None:
            self.speed_command = min(ego.speed, c.cruise_speed)
        # Smooth cruise/ACC transitions. No derivative kick when lead IDs change.
        desired_a = clamp((target-self.speed_command)/.5, -c.normal_decel, c.max_accel)
        self.acceleration += clamp(desired_a-self.acceleration, -3.*dt, 3.*dt)
        self.speed_command = clamp(self.speed_command+self.acceleration*dt, 0., c.cruise_speed)
        # Urgent front closure cannot wait behind normal target smoothing.
        if follow and follow['gap'] < c.standstill_gap+.8*ego.speed:
            self.speed_command = min(self.speed_command, max(target, ego.speed-c.normal_decel*.5))
        horizon = max(65., ego.speed*3.)
        path = []
        for i in range(int((horizon+5.)/.75)+1):
            station = s-5.+i*.75
            path.append(self.change.point(station) if self.change is not None else road.point(station))
        stop = emergency or (fault is not None and ego.speed < .5)
        if odom_age > c.odom_timeout or lidar_age > c.lidar_timeout:
            stop = True
        if emergency:
            reason = 'front_collision_emergency'
        return Result(self.state, reason, path, self.speed_command, stop,
                      dict(changes=self.changes, locked=self.locked, lateral_error=round(d-goal_d,3),
                           heading_error_deg=round(math.degrees(yaw_error),2),
                           lane_age=round(now-self.last_good,3), lane_association=self.lane_reason,
                           blocker=self.blocker,
                           left_type=self.lane.left if self.lane else None,
                           right_type=self.lane.right if self.lane else None,
                           gap_confirm_frames=self.gap_frames,
                           hold_elapsed=round(now-self.centered_since,2) if self.centered_since is not None else 0.,
                           follow=follow, target_d=round(goal_d,3),
                           progress=round(s/self.change.length,3) if self.change else None))


def track_path(points, ego, wheelbase=3.):
    """Curvature feedforward + critically damped road-frame error feedback.

    Rear-axle reference matches localization/base_link. Curvature is measured
    symmetrically around the nearest path point, not at a moving lookahead.
    """
    i = min(range(len(points)), key=lambda j: (points[j][0]-ego.x)**2+(points[j][1]-ego.y)**2)
    lo, hi = max(0,i-2), min(len(points)-1,i+2)
    a,b = points[lo],points[hi]
    angle = math.atan2(b[1]-a[1], b[0]-a[0])
    error = -math.sin(angle)*(points[i][0]-ego.x)+math.cos(angle)*(points[i][1]-ego.y)
    curvature = 0.
    if 0 < i < len(points)-1:
        p,q,r = points[lo],points[i],points[hi]
        u,v = (q[0]-p[0],q[1]-p[1]),(r[0]-q[0],r[1]-q[1])
        den = math.hypot(*u)*math.hypot(*v)*math.hypot(r[0]-p[0],r[1]-p[1])
        if den > 1e-6:
            curvature = 2*(u[0]*v[1]-u[1]*v[0])/den
    length = max(6., .65*ego.speed)
    k = curvature+2*wrap(angle-ego.yaw)/length+error/(length*length)
    steering = math.atan(wheelbase*k)
    limit = math.atan(2.5*wheelbase/max(ego.speed**2, 1.))
    return clamp(steering,-limit,limit), i, length
