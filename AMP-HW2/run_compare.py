import argparse
from MapEnvironment import MapEnvironment
from RRTPlanner import RRTPlanner
from RRTStarPlanner import RRTStarPlanner
from AStarPlanner import AStarPlanner
import time

def run_and_average(args, num_runs):
  total_time = 0
  total_value = 0
  for i in range(num_runs):
    # Record the start time
    start_time = time.perf_counter()
    # Run the algorithm
    if args.planner == 'rrt':
        planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    elif args.planner == 'rrtstar':
        planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
    plan = planner.plan()
    plan_cost = planner.compute_cost(plan)
    # Record the end time and calculate the elapsed time
    end_time = time.perf_counter()
    elapsed_time = end_time - start_time
    # Add the elapsed time and return value to the totals
    total_time += elapsed_time
    total_value += plan_cost
  # Calculate and return the averages
  avg_time = total_time / num_runs
  avg_value = total_value / num_runs
  return avg_time, avg_value

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map2.json',
                        help='Json file name containing all map information')
    parser.add_argument('-planner', '--planner', type=str, default='rrt',
                        help='The planner to run. Choose from [astar, rrt, rrtstar]')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1',
                        help='edge extension mode for RRT and RRTStar')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.2,
                        help='probability to draw goal vertex for RRT and RRTStar')
    parser.add_argument('-k', '--k', type=int, default=10, help='number of nearest neighbours for RRTStar')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map)

    # setup the planner
    if args.planner == 'astar':
        planner = AStarPlanner(planning_env=planning_env)
    elif args.planner == 'rrt':
        planner = RRTPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    elif args.planner == 'rrtstar':
        planner = RRTStarPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, k=args.k)
    else:
        raise ValueError('Unknown planner option: %s' % args.planner);

    # execute plan
    num_runs = 20
    avg_time, avg_cost = run_and_average(args, num_runs)
    print('Average cost of path: {:.2f}'.format(avg_cost))
    print('Average time: {:.2f} seconds'.format(avg_time))
