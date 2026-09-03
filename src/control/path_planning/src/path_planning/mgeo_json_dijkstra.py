#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MGeo JSON 기반 도로 그래프를 만들고 Dijkstra 최단경로를 계산하는 공통 모듈.

이 파일은 `node_set.json`과 `link_set.json`을 읽어 방향 그래프를 구성한다.
ROS 노드들이 공통으로 사용하는 `MGeoJsonGraph` 클래스와 단독 경로 계산 테스트용 CLI를 제공한다.
"""

import argparse
import heapq
import json
import math
import os


DEFAULT_MAP_DIR = os.path.normpath(
    os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        "mgeo",
        "R_KR_PR_K-city_2025",
    )
)


def _load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _point_distance(a, b):
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def _polyline_length(points):
    if len(points) < 2:
        return 0.0
    return sum(_point_distance(points[i - 1], points[i]) for i in range(1, len(points)))


class MGeoJsonGraph:
    """MGeo node/link 정보를 그래프로 보관하고 최단경로와 근접 노드 검색을 수행한다."""

    def __init__(self, mgeo_dir):
        self.mgeo_dir = os.path.abspath(mgeo_dir)
        self.nodes = {
            node["idx"]: node
            for node in _load_json(os.path.join(self.mgeo_dir, "node_set.json"))
        }
        self.links = {
            link["idx"]: link
            for link in _load_json(os.path.join(self.mgeo_dir, "link_set.json"))
        }
        self.adjacency = self._build_adjacency()

    def _build_adjacency(self):
        adjacency = {node_id: [] for node_id in self.nodes}
        for link in self.links.values():
            from_node = link.get("from_node_idx")
            to_node = link.get("to_node_idx")
            points = link.get("points") or []
            if from_node not in self.nodes or to_node not in self.nodes or len(points) < 2:
                continue

            adjacency[from_node].append(
                {
                    "to_node": to_node,
                    "link_id": link["idx"],
                    "cost": _polyline_length(points),
                }
            )
        return adjacency

    def nearest_node(self, x, y):
        nearest = self.nearest_nodes(x, y, limit=1)
        if not nearest:
            return None, float("inf")
        return nearest[0]

    def nearest_nodes(self, x, y, limit=None):
        ranked = []
        for node_id, node in self.nodes.items():
            point = node["point"]
            cost = (point[0] - x) * (point[0] - x) + (point[1] - y) * (point[1] - y)
            ranked.append((math.sqrt(cost), node_id))
        ranked.sort()
        if limit is not None:
            ranked = ranked[:limit]
        return [(node_id, distance) for distance, node_id in ranked]

    def shortest_path(self, start_node, goal_node):
        if start_node not in self.nodes:
            raise ValueError("Unknown start node: {}".format(start_node))
        if goal_node not in self.nodes:
            raise ValueError("Unknown goal node: {}".format(goal_node))

        distances = {start_node: 0.0}
        previous = {}
        queue = [(0.0, start_node)]
        visited = set()

        while queue:
            current_cost, current_node = heapq.heappop(queue)
            if current_node in visited:
                continue
            visited.add(current_node)

            if current_node == goal_node:
                break

            for edge in self.adjacency.get(current_node, []):
                next_node = edge["to_node"]
                next_cost = current_cost + edge["cost"]
                if next_cost < distances.get(next_node, float("inf")):
                    distances[next_node] = next_cost
                    previous[next_node] = (current_node, edge["link_id"])
                    heapq.heappush(queue, (next_cost, next_node))

        if goal_node not in distances:
            return {
                "success": False,
                "distance": float("inf"),
                "node_path": [],
                "link_path": [],
                "point_path": [],
            }

        node_path = [goal_node]
        link_path = []
        cursor = goal_node
        while cursor != start_node:
            prev_node, link_id = previous[cursor]
            node_path.append(prev_node)
            link_path.append(link_id)
            cursor = prev_node

        node_path.reverse()
        link_path.reverse()

        return {
            "success": True,
            "distance": distances[goal_node],
            "node_path": node_path,
            "link_path": link_path,
            "point_path": self.points_for_links(link_path),
        }

    def route_through_nodes(self, node_list):
        if len(node_list) < 2:
            raise ValueError("node_list must contain at least two nodes")

        merged_nodes = []
        merged_links = []
        merged_points = []
        total_distance = 0.0

        for start_node, goal_node in zip(node_list[:-1], node_list[1:]):
            segment = self.shortest_path(start_node, goal_node)
            if not segment["success"]:
                return segment

            total_distance += segment["distance"]
            if merged_nodes:
                merged_nodes.extend(segment["node_path"][1:])
            else:
                merged_nodes.extend(segment["node_path"])
            merged_links.extend(segment["link_path"])
            _append_points(merged_points, segment["point_path"])

        return {
            "success": True,
            "distance": total_distance,
            "node_path": merged_nodes,
            "link_path": merged_links,
            "point_path": merged_points,
        }

    def points_for_links(self, link_path):
        points = []
        for link_id in link_path:
            _append_points(points, self.links[link_id].get("points") or [])
        return points


def _append_points(dst, src):
    for point in src:
        if dst and _same_point(dst[-1], point):
            continue
        dst.append(point)


def _same_point(a, b, eps=1.0e-6):
    return (
        abs(a[0] - b[0]) <= eps
        and abs(a[1] - b[1]) <= eps
        and abs(a[2] - b[2]) <= eps
    )


def _parse_xy(value):
    if value is None:
        return None
    if isinstance(value, str):
        parts = [part.strip() for part in value.split(",")]
    else:
        parts = value
    if len(parts) < 2:
        raise ValueError("xy must contain x and y")
    return float(parts[0]), float(parts[1])


def plan_path(mgeo_dir, start_node=None, goal_node=None, start_xy=None, goal_xy=None, node_list=None):
    graph = MGeoJsonGraph(mgeo_dir)

    if node_list:
        return graph, graph.route_through_nodes(node_list)

    if start_xy is not None:
        start_node, _ = graph.nearest_node(*_parse_xy(start_xy))
    if goal_xy is not None:
        goal_node, _ = graph.nearest_node(*_parse_xy(goal_xy))

    if not start_node or not goal_node:
        raise ValueError("Set start/goal nodes or start/goal xy coordinates")

    return graph, graph.shortest_path(start_node, goal_node)


def main():
    parser = argparse.ArgumentParser(description="Create a shortest global path from MGeo JSON files.")
    parser.add_argument("--mgeo-dir", default=DEFAULT_MAP_DIR)
    parser.add_argument("--start-node")
    parser.add_argument("--goal-node")
    parser.add_argument("--start-xy", help="x,y coordinate. Nearest node is used.")
    parser.add_argument("--goal-xy", help="x,y coordinate. Nearest node is used.")
    parser.add_argument("--node-list", nargs="+", help="Route through node ids in order.")
    parser.add_argument("--output-json")
    args = parser.parse_args()

    _, path = plan_path(
        args.mgeo_dir,
        start_node=args.start_node,
        goal_node=args.goal_node,
        start_xy=args.start_xy,
        goal_xy=args.goal_xy,
        node_list=args.node_list,
    )

    if not path["success"]:
        raise SystemExit("No route found")

    print(
        "distance={:.3f}m nodes={} links={} points={}".format(
            path["distance"],
            len(path["node_path"]),
            len(path["link_path"]),
            len(path["point_path"]),
        )
    )

    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as f:
            json.dump(path, f, ensure_ascii=False, indent=2)


if __name__ == "__main__":
    main()
