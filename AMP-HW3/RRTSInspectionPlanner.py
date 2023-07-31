import numpy as np
from RRTTree import RRTTree
import time

class RRTSInspectionPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob, coverage):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env, task="ip")

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.coverage = coverage

        self.ext_step = 1
        self.number_of_updates = 0
        self.k = 12
        self.k_log = False

        self.neg_angle = -np.pi
        self.pos_angle = np.pi

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        #add starting node as a source of the tree
        inspected_points = self.planning_env.get_inspected_points(self.planning_env.start)
        self.tree.add_vertex(self.planning_env.start, inspected_points=inspected_points, inspected_now=inspected_points)
        goal_id = self.expand_tree()

        #print("type of vertices.state is:",type(self.tree.vertices[goal_id].state))
        #add the goal state as a node to the plan
        plan.append(self.tree.vertices[goal_id].config)
        next_id = goal_id
        #while we haven't reached the start we traverse the tree backwards, adding the
        while next_id != self.tree.get_root_id():
            next_id = self.tree.edges[next_id]
            next_state = self.tree.vertices[next_id].config
            # print("next_state is:", next_state, type(next_state))
            next_cost = self.tree.vertices[next_id].cost
            # print("next_state cost is:", next_cost, type(next_cost))
            plan.append(next_state)
        plan.reverse()
        plan = np.array(plan)

        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return plan

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        plan_cost = 0

        for i in range(len(plan) - 1):
            edge_price = self.planning_env.robot.compute_distance(plan[i], plan[i+1])
            plan_cost += edge_price

        return plan_cost

    def expand_tree(self):

        while True:
        # for i in range(2000):
            # sample returns random state in form of [x,y]
            new_node = self.sample_new()
            # print("Total vetrices in the tree now:", len(self.tree.vertices))

            # inspected_now = self.planning_env.get_inspected_points(new_node)
            # closest_node_id, closest_node = self.tree.get_nearest_config(new_node)


            # knn_ids = self.tree.get_k_nearest_neighbors(new_node, self.k, self.k_log)
            # print("knn_ids is:", knn_ids)



            knn_ids = self.tree.get_k_nearest_neighbors(new_node, self.k, self.k_log)
            # print("knn_ids is:", knn_ids)

            # if the new node is valid we want to add it to our tree
            for closest_node_id in knn_ids:
                closest_node = self.tree.vertices[closest_node_id].config
                if self.planning_env.config_validity_checker(new_node) and self.planning_env.edge_validity_checker(
                        closest_node, new_node):
                    if self.ext_mode == 'E2':
                        new_node = self.extend(closest_node, new_node, self.ext_step)
                    # calculate the set of points inspected on the path up to new_node + in the node itself
                    inspected_before = self.tree.vertices[closest_node_id].inspected_points
                    inspected_now = self.planning_env.get_inspected_points(new_node)
                    total_inspected = self.planning_env.compute_union_of_points(inspected_before, inspected_now)
                    # add new_node to the tree
                    new_node_id = self.tree.add_vertex(new_node, total_inspected, inspected_now)
                    edge_cost = self.planning_env.robot.compute_distance(closest_node, new_node)
                    self.tree.add_edge(closest_node_id, new_node_id, edge_cost)

                    # print("new node is:", new_node, type(new_node), "goal is:", self.planning_env.goal, type(self.planning_env.goal))
                    # if np.array_equal(new_node, self.planning_env.goal):
                    if self.planning_env.compute_coverage(total_inspected) >= self.coverage:
                        # print("got to i = {}", format(i))
                        print("Final coverage is: {}, id is {}".format(self.tree.max_coverage, self.tree.max_coverage_id))
                        return new_node_id
                    # if i > 1900:
                    #     print("got to i = {}", format(i))
                    #     return new_node_id
                    break
            if self.number_of_updates % 3e3 == 0:
                max_vertex = self.tree.vertices[self.tree.max_coverage_id]
                print("Current max coverage is: {}, id is {}".format(self.tree.max_coverage, self.tree.max_coverage_id))
                print("Number of opened nodes is: {}".format(len(self.tree.vertices)))
            self.number_of_updates += 1

    def extend(self, near_state, rand_state, step):
        '''
        Compute and return a new configuration for the sampled one.
        @param near_config The nearest configuration to the sampled configuration.
        @param rand_config The sampled configuration.
        '''
        if self.planning_env.robot.compute_distance(near_state, rand_state) < step or step == -1:
            return rand_state
        else:
            # TODO: Check if this works for k dim
            diff = np.subtract(rand_state, near_state)
            # Normalize the difference vector
            diff = diff / np.linalg.norm(diff)
            # Multiply the normalized difference by the step size
            diff = np.multiply(diff, step)
            # Add the difference to the near_state to get the new position
            new_pos = np.add(near_state, diff)
            # Round the new position to the nearest integer coordinates
            # new_pos = np.round(new_pos).astype(int)

            return new_pos

    def get_random_config(self, neg_range, pos_range, k):

        # x = np.random.randint(x_range[0], x_range[1])
        # y = np.random.randint(y_range[0], y_range[1])
        new_config = np.random.uniform(neg_range, pos_range, k)
        # print("new config is {}".format(new_config))
        return new_config

    def sample_new(self):
        '''
        Samples a state limited by the map boudaries with goal_prob chance to sample the goal state
        '''

        p = np.random.random()
        # if p < self.goal_prob:
        #     return self.planning_env.goal
        # else:
            # print(self.planning_env.xlimit, self.planning_env.ylimit)
        x = self.neg_angle
        y = self.pos_angle
        freedom_deg = self.planning_env.robot.dim
        # print (x,y)
        sample = self.get_random_config(x, y, freedom_deg)
        return sample

    