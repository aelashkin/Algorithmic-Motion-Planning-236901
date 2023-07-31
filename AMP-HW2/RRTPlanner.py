import numpy as np
from RRTTree import RRTTree
import time

class RRTPlanner(object):

    def __init__(self, planning_env, ext_mode, goal_prob):

        # set environment and search tree
        self.planning_env = planning_env
        self.tree = RRTTree(self.planning_env)

        # set search params
        self.ext_mode = ext_mode
        self.goal_prob = goal_prob
        self.ext_step = 5

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states (positions) of the robot.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan = []

        # TODO: Task 4.4
        #add starting node as a source of the tree
        self.tree.add_vertex(self.planning_env.start)
        goal_id = self.expand_tree()

        #print("type of vertices.state is:",type(self.tree.vertices[goal_id].state))
        #add the goal state as a node to the plan
        plan.append(self.tree.vertices[goal_id].state)
        next_id = goal_id
        #while we haven't reached the start we traverse the tree backwards, adding the
        while next_id != self.tree.get_root_id():
            next_id = self.tree.edges[next_id]
            next_state = self.tree.vertices[next_id].state
            #print("next_state is:", next_state, type(next_state))
            next_cost = self.tree.vertices[next_id].cost
            #print("next_state cost is:", next_cost, type(next_cost))
            plan.append(next_state)
        plan.reverse()
        plan = np.array(plan)

        print('Total cost of path: {:.2f}'.format(self.compute_cost(plan)))
        print('Total time: {:.2f}'.format(time.time()-start_time))

        return plan

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps.
        @param plan A given plan for the robot.
        '''
        # TODO: Task 4.4
        plan_cost = 0

        for i in range(len(plan) - 1):
            edge_price = self.planning_env.compute_distance(plan[i], plan[i+1])
            plan_cost += edge_price

        return plan_cost


    def expand_tree(self):

        while True:
            # sample returns random state in form of [x,y]
            new_node = self.sample_new()
            closest_node_id, closest_node = self.tree.get_nearest_state(new_node)
            if self.ext_mode == 'E1':
                new_node = self.extend(closest_node, new_node, -1)
            else:
                new_node = self.extend(closest_node, new_node, self.ext_step)

            # if the new node is valid we want to add it to our tree
            if self.planning_env.state_validity_checker(new_node) and self.planning_env.edge_validity_checker(
                    closest_node, new_node):
                new_node_id = self.tree.add_vertex(new_node)
                edge_cost = self.planning_env.compute_distance(closest_node, new_node)
                self.tree.add_edge(closest_node_id, new_node_id, edge_cost)

                #print("new node is:", new_node, type(new_node), "goal is:", self.planning_env.goal, type(self.planning_env.goal))
                if np.array_equal(new_node, self.planning_env.goal):
                    return new_node_id



    def extend(self, near_state, rand_state, step):
        '''
        Compute and return a new position for the sampled one.
        @param near_state The nearest position to the sampled position.
        @param rand_state The sampled position.
        '''
        # TODO: Task 4.4
        if self.planning_env.compute_distance(near_state, rand_state) < step or step == -1:
            return rand_state
        else:
            diff = np.subtract(rand_state, near_state)
            # Normalize the difference vector
            diff = diff / np.linalg.norm(diff)
            # Multiply the normalized difference by the step size
            diff = np.multiply(diff, step)
            # Add the difference to the near_state to get the new position
            new_pos = np.add(near_state, diff)
            # Round the new position to the nearest integer coordinates
            new_pos = np.round(new_pos).astype(int)

            return new_pos

    def get_random_coordinates(self, x_range, y_range):
        x = np.random.randint(x_range[0], x_range[1])
        y = np.random.randint(y_range[0], y_range[1])
        return [x, y]

    def sample_new(self):
        '''
        Samples a state limited by the map boudaries with goal_prob chance to sample the goal state
        '''
        p = np.random.random()
        if p < self.goal_prob:
            return self.planning_env.goal
        else:
            #print(self.planning_env.xlimit, self.planning_env.ylimit)
            x = self.planning_env.xlimit
            y = self.planning_env.ylimit
            #print (x,y)
            sample = self.get_random_coordinates(x,y)
            return np.array(sample)