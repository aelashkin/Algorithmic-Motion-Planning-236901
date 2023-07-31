import argparse
import os
from itertools import combinations
from typing import List, Tuple

import heapq

import numpy as np

from Plotter import Plotter
from shapely.geometry.polygon import Polygon, LineString


def dijkstra(graph, source, dest):
    visited = {v: False for v in graph}
    parents = {v: -1 for v in graph}
    dists = {v: float('inf') for v in graph}
    dists[source] = 0
    q = []
    heapq.heappush(q, (dists[source], source))

    while len(q):
        v = heapq.heappop(q)[1]
        if v == dest:
            reverse_path = [v]
            current = parents[v]
            while current != source:
                reverse_path.append(current)
                current = parents[current]
                assert (current != -1)
            reverse_path.append(source)
            reverse_path.reverse()
            return reverse_path, dists[v]
        for u, w in graph[v].items():
            if visited[u]:
                continue
            new_dist = dists[v] + w
            if new_dist < dists[u]:
                dists[u] = new_dist
                parents[u] = v
                heapq.heappush(q, (new_dist, u))

        visited[v] = True

    return [], -1


def calc_dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def make_graph(visibility_graph: List[LineString]):
    graph = {}
    for ls in visibility_graph:
        vertices = ls.coords
        for i in range(len(vertices) - 1):
            v = vertices[i]
            u = vertices[i + 1]
            if not graph.get(v):
                graph[v] = {}
            if not graph.get(u):
                graph[u] = {}
            dist_v_u = calc_dist(v, u)
            graph[v][u] = dist_v_u
            graph[u][v] = dist_v_u

    return graph


def get_shortest_path(visibility_graph: List[LineString], source, dest):
    simple_graph = make_graph(visibility_graph)
    path, path_cost = dijkstra(simple_graph, source, dest)

    return path, path_cost


# TODO
def get_minkowsky_sum(original_shape: Polygon, r: float) -> Polygon:
    """
    Get the polygon representing the Minkowsky sum
    :param original_shape: The original obstacle
    :param r: The radius of the rhombus
    :return: The polygon composed from the Minkowsky sums
    """

    points = []

    for p in original_shape.exterior.coords:
        points.append((p[0] + r, p[1]))
        points.append((p[0] - r, p[1]))
        points.append((p[0], p[1] + r))
        points.append((p[0], p[1] - r))

    return Polygon(points).convex_hull


def intersects_set(shapely_line, obstacles):
    for polygon in obstacles:
        # polygon and its edge are intersecting
        if shapely_line.intersects(polygon) and not shapely_line.touches(polygon):
            return True
    return False

# TODO
def get_visibility_graph(obstacles: List[Polygon], source=None, dest=None) -> List[LineString]:
    """
    Get The visibility graph of a given map
    :param obstacles: A list of the obstacles in the map
    :param source: The starting position of the robot. None for part 1.
    :param dest: The destination of the query. None for part 1.
    :return: A list of LineStrings holding the edges of the visibility graph
    """
    visibility_edges = []
    vertex_list = []

    #create the list of all vertices
    for obs in obstacles:
        for point in obs.exterior.coords:
            vertex_list.append(point)
    if source is not None:
        vertex_list.append(source)
    if dest is not None:
        vertex_list.append(dest)

    #iterate over all pairs of vertexes to create non-intersecting LineStrings
    for point1, point2 in combinations(vertex_list, 2):
        line = [point1, point2]
        shapely_line = LineString(line)
        #if line does not intersect any of the obstacles
        if not intersects_set(shapely_line, obstacles):
            visibility_edges.append(shapely_line)

    return visibility_edges


def is_valid_file(parser, arg):
    if not os.path.exists(arg):
        parser.error("The file %s does not exist!" % arg)


def get_points_and_dist(line):
    source, dist = line.split(' ')
    dist = float(dist)
    source = tuple(map(float, source.split(',')))
    return source, dist


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("Robot",
                        help="A file that holds the starting position of the robot, and the distance from the center of the robot to any of its vertices")
    parser.add_argument("Obstacles", help="A file that contains the obstacles in the map")
    parser.add_argument("Query", help="A file that contains the ending position for the robot.")
    args = parser.parse_args()
    obstacles = args.Obstacles
    robot = args.Robot
    query = args.Query
    is_valid_file(parser, obstacles)
    is_valid_file(parser, robot)
    is_valid_file(parser, query)
    workspace_obstacles = []
    with open(obstacles, 'r') as f:
        for line in f.readlines():
            ob_vertices = line.split(' ')
            if ',' not in ob_vertices:
                ob_vertices = ob_vertices[:-1]
            points = [tuple(map(float, t.split(','))) for t in ob_vertices]
            workspace_obstacles.append(Polygon(points))
    with open(robot, 'r') as f:
        source, dist = get_points_and_dist(f.readline())

    # step 1:
    c_space_obstacles = [get_minkowsky_sum(p, dist) for p in workspace_obstacles]
    plotter1 = Plotter()

    plotter1.add_obstacles(workspace_obstacles)
    plotter1.add_c_space_obstacles(c_space_obstacles)
    plotter1.add_robot(source, dist)

    plotter1.show_graph()

    # step 2:

    lines = get_visibility_graph(c_space_obstacles)
    plotter2 = Plotter()

    plotter2.add_obstacles(workspace_obstacles)
    plotter2.add_c_space_obstacles(c_space_obstacles)
    plotter2.add_visibility_graph(lines)
    plotter2.add_robot(source, dist)

    plotter2.show_graph()

    # step 3:
    with open(query, 'r') as f:
        dest = tuple(map(float, f.readline().split(',')))

    lines = get_visibility_graph(c_space_obstacles, source, dest)
    # TODO: fill in the next line
    shortest_path, cost = get_shortest_path(lines, source, dest)

    plotter3 = Plotter()
    plotter3.add_robot(source, dist)
    plotter3.add_obstacles(workspace_obstacles)
    plotter3.add_robot(dest, dist)
    plotter3.add_visibility_graph(lines)
    plotter3.add_shorterst_path(list(shortest_path))
    plotter3.show_graph()
