#!/usr/bin/env python3
"""RRT* path planner used as an optional path generator.

This is a competition-friendly adaptation of the RRT* idea used in
``devcourse``.  The planner is deliberately pure Python: it does not subscribe
to ROS Object_topic and it does not require camera/LiDAR packets by itself.
Callers provide start/goal, search bounds, and an obstacle list.
"""

import math
import random
from dataclasses import dataclass


@dataclass
class CircularObstacle:
    x_m: float
    y_m: float
    radius_m: float


class RRTStarNode:
    __slots__ = ("x_m", "y_m", "cost_m", "parent")

    def __init__(self, x_m, y_m):
        self.x_m = float(x_m)
        self.y_m = float(y_m)
        self.cost_m = 0.0
        self.parent = None

    def point(self):
        return self.x_m, self.y_m


def _distance(first, second):
    return math.hypot(first.x_m - second.x_m, first.y_m - second.y_m)


def _segment_distance_to_point(ax, ay, bx, by, px, py):
    dx = bx - ax
    dy = by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    fraction = ((px - ax) * dx + (py - ay) * dy) / length_sq
    fraction = max(0.0, min(1.0, fraction))
    projected_x = ax + fraction * dx
    projected_y = ay + fraction * dy
    return math.hypot(px - projected_x, py - projected_y)


class RRTStarPlanner:
    """Plan a 2-D collision-free path with circular obstacles."""

    def __init__(
        self,
        start_xy,
        goal_xy,
        obstacles=None,
        x_bounds=(-50.0, 50.0),
        y_bounds=(-50.0, 50.0),
        step_size_m=1.0,
        max_iterations=1500,
        goal_sample_rate=0.15,
        search_radius_m=8.0,
        goal_tolerance_m=2.0,
        collision_margin_m=0.5,
        random_seed=20,
    ):
        self.start = RRTStarNode(start_xy[0], start_xy[1])
        self.goal = RRTStarNode(goal_xy[0], goal_xy[1])
        self.obstacles = list(obstacles or [])
        self.x_bounds = tuple(float(value) for value in x_bounds)
        self.y_bounds = tuple(float(value) for value in y_bounds)
        self.step_size_m = float(step_size_m)
        self.max_iterations = int(max_iterations)
        self.goal_sample_rate = float(goal_sample_rate)
        self.search_radius_m = float(search_radius_m)
        self.goal_tolerance_m = float(goal_tolerance_m)
        self.collision_margin_m = float(collision_margin_m)
        self.random = random.Random(random_seed)
        self.nodes = [self.start]

    def plan(self):
        if not self._edge_is_safe(self.start, self.start):
            raise ValueError("RRT* start point is inside an obstacle")
        if not self._edge_is_safe(self.goal, self.goal):
            raise ValueError("RRT* goal point is inside an obstacle")

        best_goal = None
        for _iteration in range(self.max_iterations):
            sampled = self._sample_node()
            nearest = self._nearest_node(sampled)
            new_node = self._steer(nearest, sampled)
            if not self._edge_is_safe(nearest, new_node):
                continue

            near_nodes = self._near_nodes(new_node)
            self._choose_parent(new_node, near_nodes)
            self.nodes.append(new_node)
            self._rewire(new_node, near_nodes)

            if _distance(new_node, self.goal) <= self.goal_tolerance_m:
                candidate = RRTStarNode(self.goal.x_m, self.goal.y_m)
                candidate.parent = new_node
                candidate.cost_m = new_node.cost_m + _distance(new_node, candidate)
                if self._edge_is_safe(new_node, candidate):
                    if best_goal is None or candidate.cost_m < best_goal.cost_m:
                        best_goal = candidate

        if best_goal is None:
            best_goal = self._best_effort_goal()
        return self._trace_path(best_goal)

    def _sample_node(self):
        if self.random.random() < self.goal_sample_rate:
            return RRTStarNode(self.goal.x_m, self.goal.y_m)
        return RRTStarNode(
            self.random.uniform(self.x_bounds[0], self.x_bounds[1]),
            self.random.uniform(self.y_bounds[0], self.y_bounds[1]),
        )

    def _nearest_node(self, sampled):
        return min(self.nodes, key=lambda node: _distance(node, sampled))

    def _near_nodes(self, node):
        node_count = max(2, len(self.nodes) + 1)
        radius = min(
            self.search_radius_m,
            self.search_radius_m * math.sqrt(math.log(node_count) / node_count) + self.step_size_m,
        )
        return [candidate for candidate in self.nodes if _distance(candidate, node) <= radius]

    def _steer(self, from_node, to_node):
        distance = _distance(from_node, to_node)
        if distance <= self.step_size_m:
            new_x, new_y = to_node.x_m, to_node.y_m
        else:
            yaw = math.atan2(to_node.y_m - from_node.y_m, to_node.x_m - from_node.x_m)
            new_x = from_node.x_m + self.step_size_m * math.cos(yaw)
            new_y = from_node.y_m + self.step_size_m * math.sin(yaw)
        new_node = RRTStarNode(new_x, new_y)
        new_node.parent = from_node
        new_node.cost_m = from_node.cost_m + _distance(from_node, new_node)
        return new_node

    def _choose_parent(self, new_node, near_nodes):
        best_parent = new_node.parent
        best_cost = new_node.cost_m
        for candidate in near_nodes:
            if not self._edge_is_safe(candidate, new_node):
                continue
            candidate_cost = candidate.cost_m + _distance(candidate, new_node)
            if candidate_cost < best_cost:
                best_parent = candidate
                best_cost = candidate_cost
        new_node.parent = best_parent
        new_node.cost_m = best_cost

    def _rewire(self, new_node, near_nodes):
        for candidate in near_nodes:
            new_cost = new_node.cost_m + _distance(new_node, candidate)
            if new_cost >= candidate.cost_m:
                continue
            if not self._edge_is_safe(new_node, candidate):
                continue
            candidate.parent = new_node
            candidate.cost_m = new_cost

    def _edge_is_safe(self, from_node, to_node):
        for obstacle in self.obstacles:
            distance = _segment_distance_to_point(
                from_node.x_m,
                from_node.y_m,
                to_node.x_m,
                to_node.y_m,
                obstacle.x_m,
                obstacle.y_m,
            )
            if distance <= obstacle.radius_m + self.collision_margin_m:
                return False
        return True

    def _best_effort_goal(self):
        nearest = min(self.nodes, key=lambda node: _distance(node, self.goal))
        if nearest is self.start:
            raise RuntimeError("RRT* could not grow a usable tree")
        return nearest

    @staticmethod
    def _trace_path(goal_node):
        path = []
        node = goal_node
        while node is not None:
            path.append((node.x_m, node.y_m))
            node = node.parent
        path.reverse()
        return path


def smooth_path(path, iterations=60, weight_data=0.35, weight_smooth=0.20):
    """Apply a light gradient-descent smoother while preserving endpoints."""
    if len(path) <= 2:
        return list(path)
    smoothed = [[float(x), float(y)] for x, y in path]
    original = [[float(x), float(y)] for x, y in path]
    for _ in range(max(0, int(iterations))):
        for index in range(1, len(smoothed) - 1):
            for axis in (0, 1):
                smoothed[index][axis] += weight_data * (
                    original[index][axis] - smoothed[index][axis]
                )
                smoothed[index][axis] += weight_smooth * (
                    smoothed[index - 1][axis]
                    + smoothed[index + 1][axis]
                    - 2.0 * smoothed[index][axis]
                )
    return [(x, y) for x, y in smoothed]

