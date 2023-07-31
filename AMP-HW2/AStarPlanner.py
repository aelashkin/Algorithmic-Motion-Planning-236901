import itertools

import numpy as np
import heapq

import MapEnvironment


def expand(pos):
    steps = [1, 0, -1]
    deltas = list(itertools.product(steps, steps))
    deltas.remove((0, 0))

    new_poses = []
    for delta in deltas:
        new_poses.append((pos[0] + delta[0], pos[1] + delta[1]))

    # print(pos)
    # print(new_poses)
    return new_poses


class AStarPlanner(object):
    def __init__(self, planning_env: MapEnvironment):
        self.planning_env = planning_env
        self.epsilon = 1

        # used for visualizing the expanded nodes
        # make sure that this structure will contain a list of positions (states, numpy arrays) without duplicates
        self.expanded_nodes = []

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''

        # initialize an empty plan.
        plan = []
        open_list = []
        open_poses = {}
        closed_list = {}
        parent = {}
        expanded = set()

        start = tuple(self.planning_env.start)
        h_val = self.epsilon * self.planning_env.compute_heuristic(start)
        initial_node = (h_val + 0, 0, self.planning_env.compute_heuristic(start), start)
        # initial_node = (h_val + 0, self.planning_env.compute_heuristic(start), 0, start)
        open_list.append(initial_node)
        open_poses[start] = 0
        parent[start] = None
        heapq.heapify(open_list)

        while len(open_list) > 0:
            (f_val, g_val, h_val, pos) = heapq.heappop(open_list)
            # (f_val, h_val, g_val, pos) = heapq.heappop(open_list)
            # print("position: " + str(pos) + ", f: " + str(f_val) + ", g: " + str(g_val) + ", h: " + str(h_val))
            # if pos not in self.expanded_nodes:
            #     self.expanded_nodes.append(pos)
            expanded.add(pos)
            if pos[0] == self.planning_env.goal[0] and pos[1] == self.planning_env.goal[1]:
                prev = pos
                current = pos
                cost = 0
                while current is not None:
                    plan.append(current)
                    cost += self.planning_env.compute_distance(prev, current)
                    prev = current
                    current = parent[current]
                plan.reverse()
                print("plan cost is: " + str(cost))
                break
            for new_pos in expand(pos):
                new_g_val = g_val + self.planning_env.compute_distance(pos, new_pos)
                if open_poses.get(new_pos) is None and closed_list.get(new_pos) is None:
                    if not self.planning_env.state_validity_checker(new_pos) \
                            or not self.planning_env.edge_validity_checker(pos, new_pos):
                        continue
                elif open_poses.get(new_pos) is not None:
                    current_g_val = open_poses[new_pos]
                    if new_g_val >= current_g_val:
                        continue
                #if closed_list.get(new_pos) is not None:
                else:
                    current_g_val = closed_list[new_pos]
                    if new_g_val >= current_g_val:
                        continue
                    closed_list.pop(new_pos)
                # Insert new node to open
                open_poses[new_pos] = new_g_val
                new_h_val = self.epsilon * self.planning_env.compute_heuristic(new_pos)
                new_f_val = new_g_val + new_h_val
                node = (new_f_val, new_g_val, new_h_val, new_pos)
                parent[new_pos] = pos
                heapq.heappush(open_list, node)

            closed_list[pos] = g_val

        self.expanded_nodes = list(expanded)
        # print(sorted(self.expanded_nodes))
        return np.array(plan)

    def get_expanded_nodes(self):
        '''
        Return list of expanded nodes without duplicates.
        '''

        # used for visualizing the expanded nodes
        print(str(len(self.expanded_nodes)) + " nodes were expanded")
        return self.expanded_nodes
