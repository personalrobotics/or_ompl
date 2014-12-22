#!/usr/bin/env python
from openravepy import *

start_config = [  0.80487864,  0.42326865, -0.54016693,  2.28895761,
                 -0.34930645, -1.19702164,  1.95971213 ]
goal_config  = [  2.41349473, -1.43062044, -2.69016693,  2.12681216,
                 -0.75643783, -1.52392537,  1.01239878 ]

# Setup the environment.
env = Environment()
env.SetViewer('qtcoin')
env.Load('wamtest1.env.xml')
robot = env.GetRobot('BarrettWAM')
manipulator = robot.GetManipulator('arm')

planner = RaveCreatePlanner(env, 'OMPL_RRTConnect')
simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')

with env:
    robot.SetActiveDOFs(manipulator.GetArmIndices())
    robot.SetActiveDOFValues(start_config)
    robot.SetActiveManipulator(manipulator)

# Setup the planning instance.
params = Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig(goal_config)

# Set the timeout and planner-specific parameters. You can view a list of
# supported parameters by calling: planner.SendCommand('GetParameters')
print 'Parameters:'
print planner.SendCommand('GetParameters')

params.SetExtraParameters('<range>0.02</range>')

with env:
    with robot:
        # Invoke the planner.
        print 'Calling the OMPL_RRTConnect planner.'
        traj = RaveCreateTrajectory(env, '')
        planner.InitPlan(robot, params)
        result = planner.PlanPath(traj)
        assert result == PlannerStatus.HasSolution

        # Shortcut the path.
        print 'Calling the OMPL_Simplifier for shortcutting.'
        simplifier.InitPlan(robot, Planner.PlannerParameters())
        result = simplifier.PlanPath(traj)
        assert result == PlannerStatus.HasSolution

        # Time the trajectory.
        print 'Timing trajectory'
        result = planningutils.RetimeTrajectory(traj)
        assert result == PlannerStatus.HasSolution

# Execute the trajectory.
raw_input('Press <ENTER> to execute trajectory.')
robot.GetController().SetPath(traj)
robot.WaitForController(0)
