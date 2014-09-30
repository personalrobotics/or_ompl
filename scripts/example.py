#!/usr/bin/env python
import herbpy, prpy.rave, openravepy, numpy

OBJECTS = [
    ('bookcase', 'objects/bookcase.kinbody.xml', numpy.array([
        [  0.,  0., -1.,  0.90 ],
        [ -1.,  0.,  0.,  0.00 ],
        [  0.,  1.,  0.,  0.75 ],
        [  0.,  0.,  0.,  1.00 ],
    ])),
    ('dracula',
     'ordata/objects/dracula.kinbody.xml', numpy.array([
        [  1.,  0.,  0.,  0.94641644],
        [  0.,  1.,  0., -0.09652227],
        [  0.,  0.,  1.,  1.21749949],
        [  0.,  0.,  0.,  1.        ],
    ])),
    ('quantum',
     'ordata/objects/quantum.kinbody.xml', numpy.array([
        [  1.,  0.,  0.,  0.95076942],
        [  0.,  1.,  0.,  0.199839  ],
        [  0.,  0.,  1.,  1.23035896],
        [  0.,  0.,  0.,  1.        ],
    ])),
    ('gnomes',
     'ordata/objects/gnomes.kinbody.xml', numpy.array([
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

env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
with env:
    dof_indices, dof_values = robot.configurations.get_configuration('relaxed_home')
    robot.SetDOFValues(dof_values, dof_indices)

    manipulator = robot.right_arm
    manipulator.SetActive()
    home_dof_values = robot.GetActiveDOFValues()

    for i, (object_name, object_path, object_pose) in enumerate(OBJECTS):
        body = prpy.rave.add_object(env, object_name, object_path, object_pose)
        prpy.rave.disable_padding(body)

planner = openravepy.RaveCreatePlanner(env, 'ompl')
traj = openravepy.RaveCreateTrajectory(env, '')
params = openravepy.Planner.PlannerParameters()
params.SetExtraParameters(
    '<planner_type>{planner:s}</planner_type>'\
    '<time_limit>{timeout:f}</time_limit>'.format(
        planner='EST', timeout=60.0)
)

def plan_to_configuration(config):
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(config)
    
    with env:
        with robot:
            planner.InitPlan(robot, params)
            result = planner.PlanPath(traj)

    if result == openravepy.PlannerStatus.HasSolution:
        return traj
    else:
        return None

def plan_to_ik(pose):
    ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
    ik_solutions = robot.right_arm.FindIKSolutions(pose, ik_options)

    for i, ik_solution in enumerate(ik_solutions):
        traj = plan_to_configuration(ik_solution)

        if traj is not None:
            print('Solution {:d} succeeded!'.format(i))
            return traj
        else:
            print('Solution {:d} failed'.format(i))

    return None

# Grab the book.
robot.ExecuteTrajectory(plan_to_ik(GRASP_POSE))
manipulator.hand.CloseHand()
robot.Grab(env.GetKinBody('dracula'))

# Place the book.
robot.ExecuteTrajectory(plan_to_ik(PLACE_POSE))
manipulator.hand.MoveHand(0.0, 0.9, 0.9, 0.9)
robot.Release(env.GetKinBody('dracula'))

# Go home.
robot.ExecuteTrajectory(plan_to_configuration(home_dof_values))
