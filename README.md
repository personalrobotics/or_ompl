or_ompl
=======

[OpenRAVE](http://openrave.org/) bindings for the [OMPL](http://ompl.kavrakilab.org/) suite of motion planning algorithms. This
package provides an "OMPL" plugin that implements the `OpenRAVE::PlannerBase`
interface and delegates planning requests to an OMPL planner. It also includes
the "OMPLSimplifier" plugin that exposes OMPL's `PathSimplifier` to OpenRAVE
through the same interface.

This is implemented by initializing OMPL with a state space that matches the
joint limits and resolutions of the robot's active DOFs. Collision checking
is implemented by providing OMPL with a custom "state validity checker"
that uses OpenRAVE's `CheckCollision` and `CheckSelfCollision` calls to check
for collision with the environment.

This plugin supports all of the geometric planners that are currently
implemented in OMPL. The desired planner and time limit for each query is
specified by passing the `planner_type` and `time_limit` parameters,
respectively, to `InitPlan`. Any planner-specific parameters that are exposed
through the planner's `PlannerSet` can be similarly set by name as custom
`PlannerParameters` in OpenRAVE.

The wrapper classes necessary to call a geometric
planner are automatically generated from the `planners.yaml` configuration file
by the Python script `scripts/wrap_planners.py`. If you find that a planner is
missing, please [open an
issue](https://github.com/personalrobotics/or_ompl/issues/new) or send a [send
us a pull request](https://github.com/personalrobotics/or_ompl/compare/) with
an updated 'planners.yaml' file. Please make a best-effort attempt to bound the
versions of OMPL that support the planner as closely as possible.

Dependencies
------------

See the `package.xml` file for a full list of dependencies. These are the major
dependencies:

 - [ROS](http://ros.org/) fuerte or above (both rosbuild and Catkin are supported)
 - [OpenRAVE](http://openrave.org/) 0.8 or above (primarily developed in 0.9)
 - [OMPL](http://ompl.kavrakilab.org/) 0.10 or above (primarily developed in 0.10 and 0.13)
 - [semantic_version](https://pypi.python.org/pypi/semantic_version/), available via `pip install semantic-version`

Limitations
-----------

Wherever possible we aim to fully support the full breadth of features in both
OMPL and OpenRAVE. However, you should be aware of a few limitations:

 - per-joint weights are not supported (unit weights are assumed)
 - per-joint resolutions are not supported (a conservative resolution is selected)
 - kineodynamic planning is not supported
 - planning with affine DOFs is not implemented
 - continuous rotation joints are not supported
 - simplifier does not work with the `SmoothTrajectory` helper function
 - no support for multi-query planning (e.g. PRM)
 - no support for continuing to plan with anytime planners (e.g. RRT\*)

None of these are fundamental issues and we hope, perhaps with your help, to
shrink this list over time.  We would welcome [pull
requests](https://github.com/personalrobotics/or_ompl/compare/) for any of
these features.

Usage
------

The following Python code will plan using OMPL's implementation of RRT-Connect,
then shortcut the trajectory using OMPL's path simplifier.  We assume that the
variable `robot` is an OpenRAVE robot that is configured with an appropriate
set of active DOFs:

    from openravepy import *

    env = ... # your environment
    robot = ... # your robot
    planner = RaveCreatePlanner(env, 'OMPL')
    simplifier = RaveCreatePlanner(env, 'OMPLSimplifier')

    # Setup the planning instance.
    params = Planner.PlannerParameters()
    params.SetRobotActiveJoints(robot)
    params.SetGoalConfig(goal)
    params.SetExtraParameters('<planner_type>RRTConnect</planner_type>')
    planner.InitPlan(robot, params)

    # Invoke the planner.
    traj = RaveCreateTrajectory(env, '')
    result = planner.PlanPath(traj)
    assert result == PlannerStatus.HasSolution
    
    # Shortcut the path.
    simplifier.InitPlan(robot, Planner.PlannerParameters())
    result = simplifier.PlanPath(traj)
    assert result == PlannerStatus.HasSolution

    # Retime and cxecute the trajectory.
    result = planningutils.RetimeTrajectory(traj)
    assert result == PlannerStatus.HasSolution

 See the [documentation on the OpenRAVE
website](http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/#directly-launching-planners)
for more information about how to invoke an OpenRAVE planner.

Contributors
------------

or_ompl was developed by the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the [Robotics
Institute](http://ri.cmu.edu) at [Carnegie Mellon University](http://www.cmu.edu). This plugin was written by [Michael
Koval](http://mkoval.org) and grew out of earlier plugin written by [Christopher Dellin](http://www.ri.cmu.edu/person.html?person_id=2267) and [Matthew Klingensmith](http://www.ri.cmu.edu/person.html?person_id=2744).
