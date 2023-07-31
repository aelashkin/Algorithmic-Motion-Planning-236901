import numpy as np
import time


class NaiveTaskInspectionPlanner(object):

    def __init__(self, planning_env, coverage):

        # set environment
        self.planning_env = planning_env

        # set search params
        self.coverage = coverage

    def plan(self):
        '''
        Compute and return the plan. The function should return a numpy array containing the states in the configuration space.
        '''
        start_time = time.time()

        # initialize an empty plan.
        plan_configs, plan_timestamps = [], []

        # TODO: implement you planner here
        # Your stopping condition should look like this:
        # while coverage < self.coverage:

        # Initialize:
        coverage = 0
        total_inspection_points = len(self.planning_env.gripper_plan)
        # A mapping from timestamp to inspector robot configurations.
        # Initialize with origin configuration
        plan_map = [None] * total_inspection_points
        inspection_origin = self.planning_env.inspector_start
        plan_map[0] = inspection_origin
        # inspected_timestamps will organize the timestamps inspected in the current plan
        inspected_timestamps = [False] * total_inspection_points
        inspected_timestamps[0] = self.planning_env.is_gripper_inspected_from_vertex(inspection_origin, 0)

        while coverage < self.coverage:
            # Sample a configuration and check its validity
            config = self.get_random_config()
            if not self.planning_env.config_validity_checker(config, 'inspector'):
                continue

            # Iterate over unobserved points and check if sampled config observes them
            for t in range(1, len(inspected_timestamps)):
                if inspected_timestamps[t]:
                    continue
                # A configuration is added to plan_map ONLY if it observes the gripper at that timestamp.
                # Therefor, inspected_timestamps[t] is False => plan_map[t] is None
                assert (plan_map[t] is None)
                # If the sampled configuration doesn't observe the current timestamp, continue to next timestamp
                if not self.planning_env.is_gripper_inspected_from_vertex(config, t):
                    continue

                # Get the configurations that would be before and after the sampled configuration if added.
                # If one of the edges is invalid, continue to next timestamp
                i = 0
                prev_config = None
                while prev_config is None and t - i - 1 >= 0:
                    i += 1
                    prev_config = plan_map[t - i]

                if not self.planning_env.edge_validity_checker(prev_config, config, 'inspector'):
                    continue

                j = 0
                next_config = None
                while next_config is None and t + j + 1 < len(plan_map):
                    j += 1
                    next_config = plan_map[t + j]

                # If the plan is empty from this timestamp onward, We gain at least one
                # new inspected point so always add to trajectory
                if next_config is None:
                    edge_inspected_timestamps = self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
                                                                                                        config, t - i,
                                                                                                        t)
                    # Update inspected_timestamps with new observed timestamps
                    for it in range(len(edge_inspected_timestamps)):
                        if edge_inspected_timestamps[it]:
                            inspected_timestamps[it] = True
                    inspected_timestamps[t] = True
                    plan_map[t] = config

                # Else we have a previous and next configuration in the plan.
                # Compare the observed timestamps in the current edge (from previous to next)
                # to the observed timestamps in the two new edges created by adding the sampled
                # configuration.  If we gain observed timestamps, add new configuration
                else:
                    if not self.planning_env.edge_validity_checker(config, next_config, 'inspector'):
                        continue
                    current_edge_inspected_timestamps = self.planning_env.compute_inspected_timestamps_for_edge(
                        prev_config, next_config, t - i, t + j)
                    current_edge_coverage = sum(current_edge_inspected_timestamps)

                    backedge_inspected_timestamps = self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
                                                                                                            config,
                                                                                                            t - i, t)
                    forward_inspected_timestamps = self.planning_env.compute_inspected_timestamps_for_edge(config,
                                                                                                           next_config,
                                                                                                           t, t + j)
                    new_edges_coverage = sum(backedge_inspected_timestamps) + sum(forward_inspected_timestamps) - 1

                    # If we gain by adding the vertex, add new configuration and update observed timestamps
                    observed_diff = new_edges_coverage - current_edge_coverage
                    if observed_diff <= 0:
                        continue
                    for it in range(len(plan_map)):
                        if current_edge_inspected_timestamps[it]:
                            inspected_timestamps[it] = False
                        if backedge_inspected_timestamps[it] or forward_inspected_timestamps[it]:
                            inspected_timestamps[it] = True
                    plan_map[t] = config
                    inspected_timestamps[t] = True

                coverage = sum(inspected_timestamps) / len(inspected_timestamps)


        # transfer plan from plan_map to plan_configs
        for i in range(len(plan_map)):
            if plan_map[i] is not None:
                plan_configs.append(plan_map[i])
                plan_timestamps.append(i)


        # store total path cost and time
        path_cost = self.compute_cost(plan_configs)
        computation_time = time.time() - start_time

        return np.array(plan_configs), np.array(plan_timestamps), coverage, path_cost, computation_time

    def compute_cost(self, plan):
        '''
        Compute and return the plan cost, which is the sum of the distances between steps in the configuration space.
        @param plan A given plan for the robot.
        '''
        # compute cost of a given path
        plan_cost = 0.0
        for i in range(len(plan) - 1):
            plan_cost += self.planning_env.insp_robot.compute_distance(plan[i], plan[i + 1])
        return plan_cost

    def get_random_config(self):
        new_config = np.random.uniform(-np.pi, np.pi, self.planning_env.insp_robot.dim)
        return new_config
