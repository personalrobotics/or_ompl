import herbpy, openravepy, numpy, prpy

def simplify_path(robot, traj):
    with robot:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug);
        params = openravepy.Planner.PlannerParameters()
        simplifier = openravepy.RaveCreatePlanner(env, 'OMPLSimplifier')
        simplifier.InitPlan(robot, params)
        result = simplifier.PlanPath(traj)
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Info);
        assert result == openravepy.PlannerStatus.HasSolution

CheckEnvCollisions = openravepy.IkFilterOptions.CheckEnvCollisions

book_uri = '/opt/pr/librarian_ordata/ordata/objects/dracula.kinbody.xml'
book_pose =  numpy.array(
    [[ 1.0, 0.0, 0.0,  0.89931095 ],
     [ 0.0, 1.0, 0.0, -0.32686412 ],
     [ 0.0, 0.0, 1.0,  1.30060484 ],
     [ 0.0, 0.0, 0.0,  1.         ]]
)

bookshelf_uri = '/opt/pr/librarian_ordata/ordata/objects/bookcase.kinbody.xml'
bookshelf_pose = numpy.array(
    [[ 0.0, 0.0, 1.0,  0.884],
     [ 1.0, 0.0, 0.0, -0.244],
     [ 0.0, 1.0, 0.0,  0.814],
     [ 0.0, 0.0, 0.0,  1.000]]
)

grasp_pose = numpy.array(
    [[ 0.0, 0.0, 1.0,  0.642 ],
     [ 1.0, 0.0, 0.0, -0.329 ],
     [ 0.0, 1.0, 0.0,  1.324 ],
     [ 0.0, 0.0, 0.0,  1.000 ]]
)

env, robot = herbpy.initialize(sim=True, attach_viewer='interactivemarker')
manipulator = robot.right_arm
robot.planner = prpy.planning.Sequence(robot.named_planner, robot.ompl_planner)
dof_indices, dof_values = robot.configurations.get_configuration('relaxed_home')

with env:
    manipulator.SetActive()
    robot.SetDOFValues(dof_values, dof_indices)

    book = prpy.rave.add_object(env, 'book', book_uri)
    book.SetTransform(book_pose)
    prpy.rave.disable_padding(book)

    bookshelf = prpy.rave.add_object(env, 'bookshelf', bookshelf_uri)
    bookshelf.SetTransform(bookshelf_pose)
    prpy.rave.disable_padding(bookshelf)

# Transit
ik = manipulator.FindIKSolution(grasp_pose, CheckEnvCollisions)
transfer_traj = manipulator.PlanToConfiguration(ik, execute=False)
simplify_path(robot, transfer_traj)
robot.ExecuteTrajectory(transfer_traj)

# Grasp
manipulator.hand.CloseHand()
robot.Grab(book)

# Transfer
transit_traj = manipulator.PlanToNamedConfiguration('relaxed_home', execute=False)
simplify_path(robot, transit_traj)
robot.ExecuteTrajectory(transit_traj)
