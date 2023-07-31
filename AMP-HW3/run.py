import argparse
from MapEnvironment import MapEnvironment
from RRTMotionPlanner import RRTMotionPlanner
from RRTInspectionPlanner import RRTInspectionPlanner
from RRTSInspectionPlanner import RRTSInspectionPlanner

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('-map', '--map', type=str, default='map_ip.json', help='Json file name containing all map information')
    parser.add_argument('-task', '--task', type=str, default='ip', help='choose from mp (motion planning) and ip (inspection planning)')
    parser.add_argument('-ext_mode', '--ext_mode', type=str, default='E1', help='edge extension mode')
    parser.add_argument('-goal_prob', '--goal_prob', type=float, default=0.05, help='probability to draw goal vertex')
    parser.add_argument('-coverage', '--coverage', type=float, default=0.75, help='percentage of points to inspect (inspection planning)')
    args = parser.parse_args()

    # prepare the map
    planning_env = MapEnvironment(json_file=args.map, task=args.task)

    # setup the planner
    if args.task == 'mp':
        planner = RRTMotionPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob)
    elif args.task == 'ip':
        planner = RRTInspectionPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob, coverage=args.coverage)
    elif args.task == 'ipk':
        planner = RRTSInspectionPlanner(planning_env=planning_env, ext_mode=args.ext_mode, goal_prob=args.goal_prob,
                                        coverage=args.coverage)
    else:
        raise ValueError('Unknown task option: %s' % args.task);

    # execute plan
    print("PLANNING")
    plan = planner.plan()
    print(plan, plan.shape)

    # Visualize the final path.
    planner.planning_env.visualize_plan(plan)