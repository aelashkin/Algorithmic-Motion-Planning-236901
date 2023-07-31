import argparse
import numpy as np
from MapEnvironment import MapEnvironment
from TaskInspectionPlanner import TaskInspectionPlanner
from NaiveTaskInspectionPlanner import NaiveTaskInspectionPlanner
from PubertyTaskInspectionPlanner import PubertyTaskInspectionPlanner
import matplotlib.pyplot as plt

NUM_RUNS = 10
FILENAME_SUFFIX = "map2_0.75_decr_replacement_improve_by_two"
MIN_COVERAGE = 0.1
MAX_COVERAGE = 0.6


def run_and_avg(plan_env, goal_coverage, suffix=''):
    total_coverage = 0
    total_cost = 0
    total_compute_time = 0
    for i in range(NUM_RUNS):
        print("run " + str(i))
        planner = PubertyTaskInspectionPlanner(planning_env=plan_env, coverage=goal_coverage)
        plan, plan_timestamps, path_coverage, path_cost, computation_time = planner.plan()

        total_coverage += path_coverage
        total_cost += path_cost
        total_compute_time += computation_time

        # # visualize the final path
        # planner.planning_env.visualize_plan(plan=plan, plan_timestamps=plan_timestamps)

    avg_coverage = total_coverage / NUM_RUNS
    avg_cost = total_cost / NUM_RUNS
    avg_compute_time = total_compute_time / NUM_RUNS
    # print to file
    write_stats(path_coverage=avg_coverage, path_cost=avg_cost, computation_time=avg_compute_time,
                name_suffix=suffix)
    return avg_coverage, avg_cost, avg_compute_time


def create_graph(X, Y, x_label, y_label):
    fig, ax = plt.subplots()
    ax.plot(X, Y)
    #fig.suptitle('')
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.legend()
    plt.show()


def run_and_graph(min_coverage, max_coverage, plan_env):
    coverage_range = np.linspace(min_coverage, max_coverage, 6)
    coverages = []
    costs = []
    computes = []
    for c in coverage_range:
        print("Calculating cost and time for ", c, " coverage")
        c_str = f"{c:.2f}"
        suffix_c = "cov_" + c_str
        avg_coverage, avg_cost, avg_compute_time = run_and_avg(plan_env, c, suffix=suffix_c)
        coverages.append(avg_coverage)
        costs.append(avg_cost)
        computes.append(avg_compute_time)

    create_graph(coverages, costs, "Coverage", "Path Cost")
    create_graph(coverages, computes, "Coverage", "Computation Time")


def write_stats(path_coverage, path_cost, computation_time, name_suffix=''):
    '''
    Write plan stats to a file.
    @param path_coverage The coverage of the plan.
    @param path_cost The cost of the plan (in C-space).
    @param computation_time The time it took to compute the plan.
    '''
    # interpolate plan
    file_lines = [f"Coverage: {path_coverage}\n",
                  f"Cost: {path_cost}\n",
                  f"Computation Time: {computation_time}\n"]

    # write plan to file
    filename = "stats_" + name_suffix + ".txt"
    f = open(filename, "w")
    f.writelines(file_lines)
    f.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map_plan_p1.json',
                        help='Json file name containing all map information')
    parser.add_argument('-coverage', '--coverage', type=float, default=0.5,
                        help='percentage of points to inspect (inspection planning)')
    args = parser.parse_args()

    # prepare the map
    print("Running the algorithm for map", args.map)
    planning_env = MapEnvironment(json_file=args.map)

    # setup and execute the planner
    # run_and_graph(MIN_COVERAGE, MAX_COVERAGE, planning_env)
    run_and_avg(planning_env, args.coverage)

