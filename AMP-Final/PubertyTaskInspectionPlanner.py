import random

import numpy as np
import time
from enum import Enum


class Mode(Enum):
    # Don't try to replace an existing vertex - minimizes runtime
    DONT_REPLACE_VERTEX = 0
    # With time, decrease the amount of vertexes we try to replace
    DECREASING_REPLACEMENT = 1
    # Always try to replace existing vertexes - maximum runtime
    ALWAYS_REPLACE = 2


class PubertyTaskInspectionPlanner(object):

    def __init__(self, planning_env, coverage):

        # set environment
        self.planning_env = planning_env

        # set search params
        self.coverage = coverage

        self.reached_coverage = 0.0

        """
        Parameter that controls the strength of the bias in biased_sampling. 
        A higher value leads to a stronger bias towards the average of the two configurations.
        Range [0, 1]
        """
        self.bias_factor = 0.7

        self.mode = Mode.DONT_REPLACE_VERTEX

        # A mapping from timestamp to inspector robot configurations.
        # Initialize with origin configuration
        self.plan_map = [None] * len(self.planning_env.gripper_plan)
        inspection_origin = self.planning_env.inspector_start
        self.plan_map[0] = inspection_origin

        # Organize the timestamps inspected in the current plan
        self.inspected_timestamps = [False] * len(self.planning_env.gripper_plan)
        self.inspected_timestamps[0] = self.planning_env.is_gripper_inspected_from_vertex(inspection_origin, 0)

        # List of edge costs originating from each timestamp
        self.edge_cost = [0] * len(self.planning_env.gripper_plan)

        # List s.t in each timestamp is the number of inspected timestamps seen on the edge originating from
        # a vertex in this timestamp
        self.edge_coverage = [0] * len(self.planning_env.gripper_plan)
        self.edge_coverage[0] = 1

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

        self.reach_coverage(start_time)

        self.improve_path()

        # transfer plan from plan_map to plan_configs
        plan_configs, plan_timestamps = self.update_final_plan()

        # store total path cost and time
        path_cost = self.compute_cost(plan_configs)
        computation_time = time.time() - start_time

        return np.array(plan_configs), np.array(plan_timestamps), self.reached_coverage, path_cost, computation_time


    def reach_coverage(self, start_time):

        coverage = 0
        reach_cov = self.coverage*0.8

        while coverage < reach_cov:
            # Epsilon for decreasing replacement mode
            epsilon = min(max(coverage / self.coverage, 0.05), 0.95)
            # Sample a configuration and check its validity
            config = self.get_random_config()
            if not self.planning_env.config_validity_checker(config, 'inspector'):
                continue

            # Iterate over all inspection points and check if sampled config inspects them
            for t in range(1, len(self.inspected_timestamps)):
                if self.mode == Mode.DONT_REPLACE_VERTEX and self.inspected_timestamps[t]:
                    continue
                if self.mode == Mode.DECREASING_REPLACEMENT:
                    rand = random.uniform(0, 1)
                    if rand < 1 - epsilon:
                        continue
                # If the sampled configuration doesn't observe the current timestamp, continue to next timestamp
                if not self.planning_env.is_gripper_inspected_from_vertex(config, t):
                    continue

                is_current_timestamp_inspected = self.inspected_timestamps[t]

                # Get the configurations that would be before and after the sampled configuration if added.
                # If one of the edges is invalid, continue to next timestamp
                prev_config, prev_timestamp = self.get_previous_configuration(t)

                if not self.planning_env.edge_validity_checker(prev_config, config, 'inspector'):
                    continue

                next_config, next_timestamp = self.get_next_configuration(t)

                if next_config is not None and not self.planning_env.edge_validity_checker(config, next_config,
                                                                                           'inspector'):
                    continue

                # Get the current coverage of the edges\edge through this timestamp
                if self.plan_map[t] is None:
                    current_coverage = self.edge_coverage[prev_timestamp]
                else:
                    current_coverage = self.edge_coverage[prev_timestamp] + self.edge_coverage[t] - 1

                # # ASSERT TODO REMOVE!
                # if self.plan_map[t] is None:
                #     if next_config is not None:
                #         check_coverage = sum(self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
                #                                                                                      next_config,
                #                                                                                      prev_timestamp,
                #                                                                                      next_timestamp))
                #     else:
                #         check_coverage = 1
                # else:
                #     check_back = sum(self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
                #                                                                              self.plan_map[t],
                #                                                                              prev_timestamp,
                #                                                                              t))
                #     if next_config is not None:
                #         check_forward = sum(self.planning_env.compute_inspected_timestamps_for_edge(self.plan_map[t],
                #                                                                                     next_config,
                #                                                                                     t,
                #                                                                                     next_timestamp))
                #     else:
                #         check_forward = 1
                #     check_coverage = check_back + check_forward - 1
                #
                # assert (check_coverage == current_coverage)
                # # ASSERT

                # Compute coverage of forward and backward edges achieved adding new configuration
                backedge_inspected_timestamps = self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
                                                                                                        config,
                                                                                                        prev_timestamp,
                                                                                                        t)
                forward_inspected_timestamps = [
                    False] if next_config is None else self.planning_env.compute_inspected_timestamps_for_edge(config,
                                                                                                               next_config,
                                                                                                               t,
                                                                                                               next_timestamp)
                backedge_coverage_points = sum(backedge_inspected_timestamps)
                forward_coverage_points = sum(forward_inspected_timestamps)
                new_coverage = backedge_coverage_points + forward_coverage_points

                # If there are a previous and next configuration the timestamp of the new configuration is counted twice
                # towards the new coverage so we need to correct for that
                if next_config is not None:
                    new_coverage -= 1

                # If we don't improve the coverage or cost (without harming coverage) continue
                if new_coverage < current_coverage:
                    continue

                new_backedge_cost, new_forward_cost = self.get_edges_cost(prev_config, config, next_config)
                new_edge_cost = new_backedge_cost + new_forward_cost

                current_edge_cost = self.edge_cost[prev_timestamp] + self.edge_cost[t]

                if new_coverage == current_coverage and new_edge_cost >= current_edge_cost:
                    continue

                # # Replace existing vertex only if improving coverage by more then one point TODO REMOVE!
                # if is_current_timestamp_inspected and new_coverage == current_coverage + 1:
                #     continue

                # else coverage improves or coverage is equal and cost improves - add configuration and update
                self.edge_coverage[prev_timestamp] = backedge_coverage_points
                if next_config is not None:
                    self.edge_coverage[t] = forward_coverage_points
                else:
                    self.edge_coverage[t] = 1

                self.edge_cost[prev_timestamp] = new_backedge_cost
                if next_config is not None:
                    self.edge_cost[t] = new_forward_cost

                for it in range(prev_timestamp, t):
                    self.inspected_timestamps[it] = backedge_inspected_timestamps[it]

                self.inspected_timestamps[t] = True

                if next_config is not None:
                    for it in range(t + 1, next_timestamp):
                        self.inspected_timestamps[it] = forward_inspected_timestamps[it]

                self.plan_map[t] = config

                coverage = sum(self.inspected_timestamps) / len(self.inspected_timestamps)
                if coverage >= reach_cov:
                    self.reached_coverage = coverage
                    break


    def improve_path(self):

        coverage = self.reached_coverage
        while coverage <= self.coverage:
            for index, point in enumerate(self.plan_map):
                if point is None:
                    continue
                next_point, next_index = self.get_next_configuration(index)
                if next_point is None:
                    break
                if np.array_equal(point, next_point, equal_nan=True):
                    continue

                if (next_index - index) > 1:
                    sample_idx = (next_index + index) // 2
                    # print ("Point and next points are: ", point, next_point)
                    sample = self.biased_sampling(point, next_point)
                    # print("Sample is ", sample)

                    if not self.planning_env.config_validity_checker(sample, 'inspector'):
                        continue
                    if not self.planning_env.is_gripper_inspected_from_vertex(sample, sample_idx):
                        continue

                    if (self.planning_env.edge_validity_checker(point, sample, 'inspector') and
                        self.planning_env.edge_validity_checker(sample, next_point, 'inspector')):

                        insp_before = self.planning_env.compute_inspected_timestamps_for_edge(point, sample, index, sample_idx)
                        insp_after = self.planning_env.compute_inspected_timestamps_for_edge(sample, next_point, sample_idx, next_index)

                        backedge_coverage_points = sum(insp_before)
                        forward_coverage_points = sum(insp_after)
                        #sample is counted twice so we need to substract 1
                        new_coverage = backedge_coverage_points + forward_coverage_points - 1

                        current_coverage = self.edge_coverage[index]

                        if new_coverage < current_coverage:
                            continue


                        #Update all the costs and coverages in the tables
                        new_backedge_cost, new_forward_cost = self.get_edges_cost(point, sample, next_point)
                        new_edge_cost = new_backedge_cost + new_forward_cost

                        current_edge_cost = self.edge_cost[index] + self.edge_cost[sample_idx]

                        self.edge_coverage[index] = backedge_coverage_points
                        self.edge_coverage[sample_idx] = forward_coverage_points

                        self.edge_cost[index] = new_backedge_cost
                        self.edge_cost[sample_idx] = new_forward_cost

                        for it in range(index, sample_idx):
                            self.inspected_timestamps[it] = insp_before[it]
                        self.inspected_timestamps[sample_idx] = True

                        for it in range(sample_idx + 1, next_index):
                            self.inspected_timestamps[it] = insp_after[it]

                        self.plan_map[sample_idx] = sample

                        coverage = sum(self.inspected_timestamps) / len(self.inspected_timestamps)
                        print("Coverage is: ", coverage)

                        if coverage >= self.coverage:
                            self.reached_coverage = coverage
                            break

        return None


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

    def get_edges_cost(self, prev, config, next):
        '''
        compute and return the edge costs from @prev to @config and from @config to @next (if exists)
        '''

        back_edge_cost = self.planning_env.insp_robot.compute_distance(prev, config)
        forward_edge_cost = 0 if next is None else self.planning_env.insp_robot.compute_distance(config, next)
        return back_edge_cost, forward_edge_cost

    def get_random_config(self):
        new_config = np.random.uniform(-np.pi, np.pi, self.planning_env.insp_robot.dim)
        return new_config

    def get_previous_configuration(self, timestamp):
        '''
        Get the previous configuration of the inspector robot in the current plan relative to a timestamp
        @param timestamp: The time of the configuration to start from
        returns the previous configuration and its timestamp
        '''

        i = 0
        prev_config = None
        while prev_config is None and timestamp - i - 1 >= 0:
            i += 1
            prev_config = self.plan_map[timestamp - i]

        return prev_config, timestamp - i

    def get_next_configuration(self, timestamp):
        '''
        Get the next configuration of the inspector robot in the current plan relative to a timestamp
        @param timestamp: The time of the configuration to start from
        returns the next configuration and its timestamp (or NONE if there isn't a next configuration)
        '''

        i = 0
        next_config = None
        while next_config is None and timestamp + i + 1 < len(self.plan_map):
            i += 1
            next_config = self.plan_map[timestamp + i]

        return next_config, timestamp + i

    def update_final_plan(self):
        '''
        Transfer final plan data from plan_map to @plan_configs and @plan_timestamps
        '''
        plan_configs = []
        plan_timestamps = []
        for i in range(len(self.plan_map)):
            if self.plan_map[i] is not None:
                plan_configs.append(self.plan_map[i])
                plan_timestamps.append(i)

        # Count damaging configurations TODO ROMOVE!
        # for i in range(len(plan_timestamps) - 2):
        #     prev_t = plan_timestamps[i]
        #     curr_t = plan_timestamps[i + 1]
        #     next_t = plan_timestamps[i + 2]
        #     prev_config = plan_configs[i]
        #     curr_config = plan_configs[i+1]
        #     next_config = plan_configs[i+2]
        #
        #     current_coverage = sum(self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
        #                                                                                curr_config,
        #                                                                                prev_t,
        #                                                                                curr_t)) + \
        #                        sum(self.planning_env.compute_inspected_timestamps_for_edge(curr_config,
        #                                                                                next_config,
        #                                                                                curr_t,
        #                                                                                next_t))
        #     coverage_by_removal = sum(self.planning_env.compute_inspected_timestamps_for_edge(prev_config,
        #                                                                                   next_config,
        #                                                                                   prev_t,
        #                                                                                   next_t))
        #
        #     if coverage_by_removal >= current_coverage:
        #         print("oh boy! rogue point")

        return plan_configs, plan_timestamps

    def biased_sampling(self, config1, config2):
        """
        Generate a biased sample point between two k-dimensional configurations.

        Args:
        - config1, config2: array-like of shape (k,)
            Two k-dimensional configurations.

        - self.bias_factor: float
            A parameter that controls the strength of the bias. A higher value leads
            to a stronger bias towards the average of the two configurations.

        Returns:
        - sample: array of shape (k,)
            A new k-dimensional sample point biased towards the average of the two configurations. Each sample dimention is limited by [-pi, pi]
        """
        if len(config1) != len(config2):
            raise print("config1 and config2 must have the same length")

        config1 = np.array(config1)
        config2 = np.array(config2)

        # Calculate the average configuration
        avg_config = 0.5 * (config1 + config2)

        # # Calculate the biased distribution parameters
        # mean = self.bias_factor * avg_config + (1 - self.bias_factor) * config1
        # cov = np.cov(np.vstack((config1, config2, avg_config)))
        #
        # # Generate a sample from the biased distribution
        # sample = np.random.multivariate_normal(mean, cov)
        #
        # for i in range(len(sample)):
        #     if sample[i] < -np.pi:
        #         sample[i] = -np.pi
        #     elif sample[i] > np.pi:
        #         sample[i] = np.pi


        # # Compute the average of the two configurations
        # average = (config1 + config2) / 2
        #
        # # Compute the difference between the two configurations
        # difference = config2 - config1
        #
        # # Compute the biased random weights
        # weights = np.random.rand(len(config1))
        #
        # # Normalize the weights with the bias_factor
        # normalized_weights = weights * (1 - self.bias_factor) + self.bias_factor
        #
        # # Compute the biased sample point
        # sample = config1 + difference * normalized_weights
        #
        # # Clip the sample point dimensions to the range [-pi, pi]
        # sample = np.clip(sample, -np.pi, np.pi)

        k = len(config1)

        # Calculate the average of the two configurations
        avg = (config1 + config2) / 2

        # Generate a random sample with dimensions limited by [-pi, pi]
        random_sample = np.random.uniform(-np.pi, np.pi, k)

        # Calculate the weighted average between the average configuration and the random sample
        sample = (1 - self.bias_factor) * random_sample + self.bias_factor * avg

        # Clip the sample dimensions to the range [-pi, pi]
        sample = np.clip(sample, -np.pi, np.pi)

        return sample
