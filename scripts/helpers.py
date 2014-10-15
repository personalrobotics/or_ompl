import numpy, openravepy

def plan_to_configuration(robot, config):
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

def plan_to_ik(robot, pose, try_all=False):
    ik_options = openravepy.IkFilterOptions.CheckEnvCollisions
    ik_solutions = robot.right_arm.FindIKSolutions(pose, ik_options)

    for i, ik_solution in enumerate(ik_solutions):
        traj = plan_to_configuration(robot, ik_solution)

        if traj is not None:
            print('Solution {:d} succeeded!'.format(i))
            return traj
        else:
            print('Solution {:d} failed'.format(i))

        if not try_all:
            break

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
