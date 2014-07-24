import herbpy, openravepy, numpy, prpy

CheckEnvCollisions = openravepy.IkFilterOptions.CheckEnvCollisions

book_uri = '/opt/pr/librarian_ordata/ordata/objects/dracula.kinbody.xml'
book_pose =  numpy.array(
    [[ 1.0, 0.0, 0.0,  0.89931095 ],
     [ 0.0, 1.0, 0.0, -0.32686412 ],
     [ 0.0, 0.0, 1.0,  1.28060484 ],
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
robot.planner = robot.ompl_planner
dof_indices, dof_values = robot.configurations.get_configuration('relaxed_home')

with env:
    book = prpy.rave.add_object(env, 'book', book_uri)
    book.SetTransform(book_pose)
    book.Enable(False)

    bookshelf = prpy.rave.add_object(env, 'bookshelf', bookshelf_uri)
    bookshelf.SetTransform(bookshelf_pose)

# Plan the nominal trajectory.
ik = manipulator.FindIKSolution(grasp_pose, CheckEnvCollisions)

with prpy.rave.AllDisabled(env, [ book, bookshelf ], padding_only=True):
    robot.SetDOFValues(dof_values, dof_indices)
    traj = manipulator.PlanToConfiguration(ik, execute=False)

    x = raw_input('Simplify? <Y/N> ')
    if x == 'Y':
        with robot:
            params = openravepy.Planner.PlannerParameters()
            simplifier = openravepy.RaveCreatePlanner(env, 'OMPLSimplifier')
            simplifier.InitPlan(robot, params)
            result = simplifier.PlanPath(traj)
            assert result == openravepy.PlannerStatus.HasSolution

    book.Enable(True)

    raw_input('Press <ENTER> to execute raw trajectory.')
    robot.ExecuteTrajectory(traj)
    with prpy.rave.Disabled(book, padding_only=True):
        manipulator.hand.CloseHand()

    raw_input('Press <ENTER> to quit.')
