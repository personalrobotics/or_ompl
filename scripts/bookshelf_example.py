#!/usr/bin/env python
import argparse, herbpy, prpy.rave, openravepy, numpy, os, signal, subprocess, time

OBJECTS = [
    ('bookcase', 'objects/bookcase.kinbody.xml', numpy.array([
        [  0.,  0., -1.,  0.90 ],
        [ -1.,  0.,  0.,  0.00 ],
        [  0.,  1.,  0.,  0.75 ],
        [  0.,  0.,  0.,  1.00 ],
    ])),
    ('dracula', 'objects/dracula.kinbody.xml', numpy.array([
        [  1.,  0.,  0.,  0.94641644],
        [  0.,  1.,  0., -0.09652227],
        [  0.,  0.,  1.,  1.21749949],
        [  0.,  0.,  0.,  1.        ],
    ])),
    ('quantum', 'objects/quantum.kinbody.xml', numpy.array([
        [  1.,  0.,  0.,  0.95076942],
        [  0.,  1.,  0.,  0.199839  ],
        [  0.,  0.,  1.,  1.23035896],
        [  0.,  0.,  0.,  1.        ],
    ])),
    ('gnomes', 'objects/gnomes.kinbody.xml', numpy.array([
        [  1.,  0.,  0.,  0.92677158],
        [  0.,  1.,  0.,  0.28213334],
        [  0.,  0.,  1.,  1.26208866],
        [  0.,  0.,  0.,  1.        ],
    ])),
]

GRASP_POSE = numpy.array([
    [ 0.0,  0.0,  1.0,  0.69494835],
    [ 1.0,  0.0,  0.0, -0.09941771],
    [ 0.0,  1.0,  0.0,  1.2089925 ],
    [ 0.0,  0.0,  0.0,  1.        ],
])

PLACE_POSE = numpy.array([
    [ 0.0, 0.0, 1.0, 0.694948522],
    [ 1.0, 0.0, 0.0, 0.142812863],
    [ 0.0, 1.0, 0.0, 0.859838868],
    [ 0.0, 0.0, 0.0, 1.         ],
])


PLANNER_NAMES =  [
    'BKPIECE1',
    'EST',
    'KPIECE1',
    'LazyRRT',
    'LBKPIECE1',
    'PDST',
    'PRM',
    'LazyPRM',
    'PRMstar',
    'pRRT',
    'pSBL',
    'RRT',
    'RRTConnect',
    'RRTstar',
    'SBL',
    'TRRT',
]

parser = argparse.ArgumentParser()
parser.add_argument('--timeout', type=float, default=60.0)
parser.add_argument('--window-id', type=str, default=0x4800011)
parser.add_argument('--output-dir', type=str, default='')
parser.add_argument('planner_name', type=str)
args = parser.parse_args()

env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
with env:
    dof_indices, dof_values = robot.configurations.get_configuration('relaxed_home')
    robot.SetDOFValues(dof_values, dof_indices)
    full_dof_values = robot.GetDOFValues()

    manipulator = robot.right_arm
    manipulator.SetActive()
    home_dof_values = robot.GetActiveDOFValues()

    for i, (object_name, object_path, object_pose) in enumerate(OBJECTS):
        body = prpy.rave.add_object(env, object_name, object_path, object_pose)
        prpy.rave.disable_padding(body)

    book = env.GetKinBody('dracula')
    book_pose = book.GetTransform()

planner = openravepy.RaveCreatePlanner(env, 'OMPL')
simplifier = openravepy.RaveCreatePlanner(env, 'OMPLSimplifier')
params = openravepy.Planner.PlannerParameters()
params.SetExtraParameters(
    '<planner_type>{planner:s}</planner_type>'\
    '<time_limit>{timeout:f}</time_limit>'.format(
        planner=args.planner_name, timeout=args.timeout)
)

def plan_to_configuration(config):
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(config)

    traj = openravepy.RaveCreateTrajectory(env, '')
    
    with env:
        with robot:
            planner.InitPlan(robot, params)
            result = planner.PlanPath(traj)

    if result == openravepy.PlannerStatus.HasSolution:
        return traj
    else:
        return None

def plan_to_ik(pose, try_all=False):
    ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
    ik_solutions = robot.right_arm.FindIKSolutions(pose, ik_options)

    for i, ik_solution in enumerate(ik_solutions):
        traj = plan_to_configuration(ik_solution)

        if traj is not None:
            print('Solution {:d} succeeded!'.format(i))
            return traj
        else:
            print('Solution {:d} failed'.format(i))

        break # FIXME

    return None

def snap_to_end(robot, traj):
    cspec = traj.GetConfigurationSpecification()
    waypoint = traj.GetWaypoint(traj.GetNumWaypoints() - 1)
    dof_values = cspec.ExtractJointValues(waypoint, robot, robot.GetActiveDOFIndices())
    robot.SetActiveDOFValues(dof_values)

def start_recording(window_id, output_path):
    args = [ 'recordmydesktop', '--full-shots', '--no-sound', '--fps', '30',
             '--windowid', window_id, '-o', output_path ]
    return subprocess.Popen(args)

def stop_recording(proc):
    proc.send_signal(signal.SIGINT)
    proc.wait()

def execute_trajectory(robot, traj):
    result = openravepy.planningutils.RetimeTrajectory(traj)
    assert result == openravepy.PlannerStatus.HasSolution
    robot.GetController().SetPath(traj)
    robot.WaitForController(0)

#
# Plan
#
# Grab the book.
grab_traj = plan_to_ik(GRASP_POSE)
snap_to_end(robot, grab_traj)
manipulator.hand.CloseHand()
robot.Grab(book)

# Place the book.
book_traj = plan_to_ik(PLACE_POSE)
snap_to_end(robot, book_traj)
manipulator.hand.MoveHand(0.0, 0.9, 0.9, 0.9)
robot.Release(book)

# Go home.
home_traj = plan_to_configuration(home_dof_values)
snap_to_end(robot, home_traj)

#
# Execute
#
with env:
    robot.SetDOFValues(full_dof_values)
    book.SetTransform(book_pose)
    env.GetViewer().EnvironmentSync()

output_name = os.path.join(args.output_dir, 'or_ompl_{:s}'.format(args.planner_name))
proc = start_recording(args.window_id, output_name)
time.sleep(1.0)

execute_trajectory(robot, grab_traj)
manipulator.hand.CloseHand()
robot.Grab(book)

execute_trajectory(robot, book_traj)
manipulator.hand.MoveHand(0.0, 0.9, 0.9, 0.9)
robot.Release(book)

execute_trajectory(robot, home_traj)

time.sleep(1.0)
stop_recording(proc)
