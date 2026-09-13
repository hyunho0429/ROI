"""Small deterministic RRT* planner for live, vehicle-local path planning.

The planner is transport agnostic. Callers provide the latest local obstacle
rectangles and a road-corridor predicate, then publish the returned points
directly as a ROS Path. No CSV or pre-generated route is involved.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Callable, Iterable, List, Optional, Sequence, Tuple


Point = Tuple[float, float]


@dataclass(frozen=True)
class RectObstacle:
    x_m: float
    y_m: float
    half_length_m: float
    half_width_m: float


class _Node:
    __slots__ = ("x", "y", "cost", "parent")

    def __init__(self, x: float, y: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.cost = 0.0
        self.parent: Optional[_Node] = None

    @property
    def point(self) -> Point:
        return self.x, self.y


def _distance(a: _Node, b: _Node) -> float:
    return math.hypot(a.x-b.x, a.y-b.y)


class RRTStarPlanner:
    """Forward-only RRT* suitable for a short lane-change corridor."""

    def __init__(
        self,
        start_xy: Point,
        goal_xy: Point,
        obstacles: Iterable[RectObstacle],
        x_bounds: Tuple[float, float],
        y_bounds: Tuple[float, float],
        state_is_valid: Optional[Callable[[float, float], bool]] = None,
        step_size_m: float = 2.0,
        max_iterations: int = 350,
        goal_sample_rate: float = 0.25,
        search_radius_m: float = 6.0,
        goal_tolerance_m: float = 2.5,
        edge_sample_m: float = 0.25,
        min_forward_m: float = 0.05,
        max_edge_heading_rad: Optional[float] = None,
        random_seed: int = 20,
    ) -> None:
        self.start = _Node(*start_xy)
        self.goal = _Node(*goal_xy)
        self.obstacles = list(obstacles)
        self.x_bounds = (float(x_bounds[0]), float(x_bounds[1]))
        self.y_bounds = (float(y_bounds[0]), float(y_bounds[1]))
        self.state_is_valid = state_is_valid
        self.step_size_m = max(0.1, float(step_size_m))
        self.max_iterations = max(1, int(max_iterations))
        self.goal_sample_rate = min(1.0, max(0.0, float(goal_sample_rate)))
        self.search_radius_m = max(self.step_size_m, float(search_radius_m))
        self.goal_tolerance_m = max(self.step_size_m, float(goal_tolerance_m))
        self.edge_sample_m = max(0.05, float(edge_sample_m))
        self.min_forward_m = max(0.0, float(min_forward_m))
        self.max_edge_heading_rad = (
            None if max_edge_heading_rad is None
            else max(0.01, float(max_edge_heading_rad))
        )
        self.random = random.Random(int(random_seed))
        self.nodes = [self.start]

    def plan(self) -> List[Point]:
        if not self._state_is_safe(self.start.x, self.start.y):
            raise RuntimeError("rrt_start_blocked")
        if not self._state_is_safe(self.goal.x, self.goal.y):
            raise RuntimeError("rrt_goal_blocked")

        # The direct edge is the optimum in an empty corridor. RRT* is used as
        # soon as a live obstacle blocks that edge.
        if self._edge_is_safe(self.start, self.goal):
            return [self.start.point, self.goal.point]

        # Seed the tree with visibility-style detours around expanded vehicle
        # boxes. This keeps the online solve deterministic and gives RRT* a
        # feasible first solution to shorten and rewire within its time budget.
        best_goal = self._seeded_goal()
        for _ in range(self.max_iterations):
            sampled = self._sample()
            forward_nodes = [
                node for node in self.nodes
                if node.x+self.min_forward_m <= sampled.x
            ]
            if not forward_nodes:
                continue
            nearest = min(forward_nodes, key=lambda node: _distance(node, sampled))
            new = self._steer(nearest, sampled)
            if not self._edge_is_safe(nearest, new):
                continue

            near = self._near(new)
            self._choose_parent(new, near)
            self.nodes.append(new)
            self._rewire(new, near)

            if _distance(new, self.goal) <= self.goal_tolerance_m:
                candidate = _Node(self.goal.x, self.goal.y)
                candidate.parent = new
                candidate.cost = new.cost + _distance(new, candidate)
                if self._edge_is_safe(new, candidate):
                    if best_goal is None or candidate.cost < best_goal.cost:
                        best_goal = candidate

        if best_goal is None:
            raise RuntimeError("rrt_no_path")
        return self._trace(best_goal)

    def _seeded_goal(self) -> Optional[_Node]:
        if not self.obstacles:
            return None
        best: Optional[_Node] = None
        clearance = max(0.35, 2.0*self.edge_sample_m)
        ordered = sorted(self.obstacles, key=lambda obstacle: obstacle.x_m)
        for side in (1.0, -1.0):
            points = [self.start.point]
            for obstacle in ordered:
                y = obstacle.y_m + side*(obstacle.half_width_m+clearance)
                left = obstacle.x_m-obstacle.half_length_m-clearance
                right = obstacle.x_m+obstacle.half_length_m+clearance
                points.extend([(left, y), (right, y)])
            points.append(self.goal.point)

            parent = self.start
            chain: List[_Node] = []
            valid = True
            for point in points[1:]:
                node = _Node(*point)
                if not self._edge_is_safe(parent, node):
                    valid = False
                    break
                node.parent = parent
                node.cost = parent.cost + _distance(parent, node)
                chain.append(node)
                parent = node
            if not valid:
                continue
            self.nodes.extend(chain[:-1])
            candidate = chain[-1]
            if best is None or candidate.cost < best.cost:
                best = candidate
        return best

    def path_is_safe(self, points: Sequence[Point]) -> bool:
        if not points:
            return False
        if any(not self._state_is_safe(x, y) for x, y in points):
            return False
        return all(
            self._edge_is_safe(_Node(*points[i-1]), _Node(*points[i]))
            for i in range(1, len(points))
        )

    def _sample(self) -> _Node:
        if self.random.random() < self.goal_sample_rate:
            return _Node(self.goal.x, self.goal.y)
        return _Node(
            self.random.uniform(*self.x_bounds),
            self.random.uniform(*self.y_bounds),
        )

    def _steer(self, source: _Node, target: _Node) -> _Node:
        dx, dy = target.x-source.x, target.y-source.y
        if self.max_edge_heading_rad is not None and dx > 0.0:
            max_dy = math.tan(self.max_edge_heading_rad)*dx
            dy = max(-max_dy, min(max_dy, dy))
        distance = math.hypot(dx, dy)
        scale = min(1.0, self.step_size_m/max(distance, 1e-9))
        node = _Node(
            source.x + scale*dx,
            source.y + scale*dy,
        )
        node.parent = source
        node.cost = source.cost + _distance(source, node)
        return node

    def _near(self, node: _Node) -> List[_Node]:
        count = max(2, len(self.nodes)+1)
        radius = min(
            self.search_radius_m,
            self.search_radius_m*math.sqrt(math.log(count)/count)+self.step_size_m,
        )
        return [candidate for candidate in self.nodes if _distance(candidate, node) <= radius]

    def _choose_parent(self, node: _Node, near: Sequence[_Node]) -> None:
        for candidate in near:
            cost = candidate.cost + _distance(candidate, node)
            if cost < node.cost and self._edge_is_safe(candidate, node):
                node.parent = candidate
                node.cost = cost

    def _rewire(self, node: _Node, near: Sequence[_Node]) -> None:
        for candidate in near:
            cost = node.cost + _distance(node, candidate)
            if cost < candidate.cost and self._edge_is_safe(node, candidate):
                candidate.parent = node
                candidate.cost = cost

    def _state_is_safe(self, x: float, y: float) -> bool:
        if not (self.x_bounds[0]-1e-6 <= x <= self.x_bounds[1]+1e-6):
            return False
        if not (self.y_bounds[0]-1e-6 <= y <= self.y_bounds[1]+1e-6):
            return False
        if self.state_is_valid is not None and not self.state_is_valid(x, y):
            return False
        for obstacle in self.obstacles:
            if (
                abs(x-obstacle.x_m) <= obstacle.half_length_m
                and abs(y-obstacle.y_m) <= obstacle.half_width_m
            ):
                return False
        return True

    def _edge_is_safe(self, source: _Node, target: _Node) -> bool:
        dx, dy = target.x-source.x, target.y-source.y
        if dx < self.min_forward_m:
            return False
        if (
            self.max_edge_heading_rad is not None
            and abs(math.atan2(dy, dx)) > self.max_edge_heading_rad
        ):
            return False
        distance = math.hypot(dx, dy)
        steps = max(1, int(math.ceil(distance/self.edge_sample_m)))
        for i in range(steps+1):
            u = float(i)/steps
            if not self._state_is_safe(source.x+u*dx, source.y+u*dy):
                return False
        return True

    @staticmethod
    def _trace(goal: _Node) -> List[Point]:
        path: List[Point] = []
        node: Optional[_Node] = goal
        while node is not None:
            path.append(node.point)
            node = node.parent
        path.reverse()
        return path


def chaikin_smooth(points: Sequence[Point], iterations: int = 3) -> List[Point]:
    """Corner-cut a polyline while preserving its exact endpoints."""
    result = [(float(x), float(y)) for x, y in points]
    for _ in range(max(0, int(iterations))):
        if len(result) < 3:
            break
        refined = [result[0]]
        for a, b in zip(result[:-1], result[1:]):
            refined.append((0.75*a[0]+0.25*b[0], 0.75*a[1]+0.25*b[1]))
            refined.append((0.25*a[0]+0.75*b[0], 0.25*a[1]+0.75*b[1]))
        refined.append(result[-1])
        result = refined
    return result


def elastic_smooth(
    points: Sequence[Point],
    iterations: int = 250,
    weight_data: float = 0.05,
    weight_smooth: float = 0.35,
) -> List[Point]:
    """Spread steering changes over several metres without external libraries."""
    original = [[float(x), float(y)] for x, y in points]
    result = [point[:] for point in original]
    if len(result) < 3:
        return [(point[0], point[1]) for point in result]
    for _ in range(max(0, int(iterations))):
        previous = [point[:] for point in result]
        for index in range(1, len(result)-1):
            for axis in (0, 1):
                result[index][axis] += weight_data*(
                    original[index][axis]-result[index][axis]
                )
                result[index][axis] += weight_smooth*(
                    previous[index-1][axis]+previous[index+1][axis]
                    - 2.0*previous[index][axis]
                )
    return [(point[0], point[1]) for point in result]


def resample_path(points: Sequence[Point], spacing_m: float = 0.5) -> List[Point]:
    if len(points) < 2:
        return list(points)
    spacing = max(0.05, float(spacing_m))
    arc = [0.0]
    for a, b in zip(points[:-1], points[1:]):
        arc.append(arc[-1]+math.hypot(b[0]-a[0], b[1]-a[1]))
    if arc[-1] <= 1e-9:
        return [points[0]]
    out: List[Point] = []
    segment = 1
    count = int(math.floor(arc[-1]/spacing))
    for index in range(count+1):
        distance = min(index*spacing, arc[-1])
        while segment < len(arc)-1 and arc[segment] < distance:
            segment += 1
        span = max(arc[segment]-arc[segment-1], 1e-9)
        u = (distance-arc[segment-1])/span
        a, b = points[segment-1], points[segment]
        out.append((a[0]+u*(b[0]-a[0]), a[1]+u*(b[1]-a[1])))
    if math.hypot(out[-1][0]-points[-1][0], out[-1][1]-points[-1][1]) > 0.05:
        out.append(points[-1])
    return out
