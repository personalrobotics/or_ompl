# or_ompl

[![Build Status](https://travis-ci.org/personalrobotics/or_ompl.svg?branch=master)](https://travis-ci.org/personalrobotics/or_ompl)

[OpenRAVE](http://openrave.org/) bindings for the
[OMPL](http://ompl.kavrakilab.org/) suite of motion planning algorithms. This
package provides an OMPL plugin that implements the `OpenRAVE::PlannerBase`
interface and delegates planning requests to an OMPL planner. It also includes
the "OMPLSimplifier" plugin that exposes OMPL's `PathSimplifier` to OpenRAVE
through the same interface. See [this demo video](http://youtu.be/6qRRbvNzHG8)
for a brief overview of or_ompl's features.

This is implemented by initializing OMPL with a state space that matches the
joint limits and resolutions of the robot's active DOFs. Collision checking
is implemented by providing OMPL with a custom "state validity checker"
that uses OpenRAVE's `CheckCollision` and `CheckSelfCollision` calls to check
for collision with the environment.

or_ompl wraps each OMPL geometric planner as an OpenRAVE planner that
implements the `PlannerBase` interface. E.g. OMPL's RRTConnect algorithm is
exposed through a `OMPL_RRTConnect` planner in OpenRAVE. The planning time
limit and any planner parameters that are exposed through OMPL's `ParamSet`
class can be set by name in the `PlannerParameters` struct in OpenRAVE.

The wrapper classes necessary to call a planner are automatically generated
from the `planners.yaml` configuration file by the Python script
`scripts/wrap_planners.py`. If you find that a planner is missing, please [open
an issue](https://github.com/personalrobotics/or_ompl/issues/new) or send a
[send us a pull request](https://github.com/personalrobotics/or_ompl/compare/)
with an updated 'planners.yaml' file. The presence or absence of each planner
is determined by testing whether the corresponding header file exists in the
OMPL include directory.

## Dependencies

See the `package.xml` file for a full list of dependencies. These are the major
dependencies:

 - [OpenRAVE](http://openrave.org/) 0.8 or above (primarily developed in 0.9)
 - [OMPL](http://ompl.kavrakilab.org/) 0.10 or above (primarily developed in 0.10 and 0.13)
 - [ROS](http://ros.org/) optional, see below

## Installation Instructions

The `CMakeLists.txt` file in the root of this repository supports Catkin and
standalone CMake builds. See the appropriate section below for installation
instructions specific to your environment.

### Catkin Instructions

This preferred way of building or_ompl. In this case, you should have OpenRAVE
and OMPL installed as system dependencies. We use a helper package called
[openrave_catkin](https://github.com/personalrobotics/openrave_catkin) to
manage the build process.

Once the dependencies are satisfied, you can simply clone this repository into
your Catkin workspace and run `catkin_make`:

```shell
$ . /my/workspace/devel/setup.bash
$ cd /my/workspace/src
$ git clone https://github.com/personalrobotics/openrave_catkin.git
$ git clone https://github.com/personalrobotics/or_ompl.git
$ cd ..
$ catkin_make
```

This will build the OpenRAVE plugins into the `share/openrave-0.9/plugins`
directory in your devel space. If you run `catkin_make install` the plugin will
be installed to the same directory in your install space. In both cases, the
corresponding directory will be automatically added to your `OPENRAVE_PLUGINS`
path using a [Catkin environment
hook](http://docs.ros.org/fuerte/api/catkin/html/macros.html#catkin_add_env_hooks).
See the [documentation for
openrave_catkin](https://github.com/personalrobotics/openrave_catkin/blob/master/README.md)
for more information.

### Standalone CMake Build Instructions

You can build or_ompl entirely ROS-agnostic by setting the `NO_ROS` variable:

```shell
$ git clone https://github.com/personalrobotics/or_ompl.git
$ mkdir build
$ cd build
$ cmake -DNO_ROS:bool=1 ..
$ make
```

Just as in the rosbuild case, this will build the plugin in the `lib/`
directory. You will need to add this directory to your `OPENRAVE_PLUGINS` path
so that OpenRAVE can find it.

## Limitations

Wherever possible we aim to fully support the full breadth of features in both
OMPL and OpenRAVE. However, you should be aware of a few limitations:

 - per-joint weights are not supported (unit weights are assumed)
 - per-joint resolutions are not supported (a conservative resolution is selected)
 - kineodynamic planning is not supported
 - planning with affine DOFs is not implemented
 - simplifier does not work with the `SmoothTrajectory` helper function
 - no support for multi-query planning (e.g. PRM)

None of these are fundamental issues and we hope, perhaps with your help, to
shrink this list over time.  We would welcome [pull
requests](https://github.com/personalrobotics/or_ompl/compare/) for any of
these features.

## Usage

You can find planners provided by this plugin by looking for planners that
start with the `OMPL_` prefix in the output of `openrave --listplugins`.

The following Python code will plan using OMPL's implementation of RRT-Connect,
then shortcut the trajectory using OMPL's path simplifier.  We assume that the
variable `robot` is an OpenRAVE robot that is configured with an appropriate
set of active DOFs:

```python
from openravepy import *

env = ... # your environment
robot = ... # your robot
planner = RaveCreatePlanner(env, 'OMPL_RRTConnect')
simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')

# Setup the planning instance.
params = Planner.PlannerParameters()
params.SetRobotActiveJoints(robot)
params.SetGoalConfig(goal)

# Set the timeout and planner-specific parameters. You can view a list of
# supported parameters by calling: planner.SendCommand('GetParameters')
params.SetExtraParameters('<range>0.02</range>')

planner.InitPlan(robot, params)

# Invoke the planner.
traj = RaveCreateTrajectory(env, '')
result = planner.PlanPath(traj)
assert result == PlannerStatus.HasSolution

# Shortcut the path.
simplifier.InitPlan(robot, Planner.PlannerParameters())
result = simplifier.PlanPath(traj)
assert result == PlannerStatus.HasSolution

# Time the trajectory.
result = planningutils.RetimeTrajectory(traj)
assert result == PlannerStatus.HasSolution

# Execute the trajectory.
robot.GetController().SetPath(traj)
```

A working version of this script is included in `scripts/example.py`. See the
[documentation on the OpenRAVE website](http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/#directly-launching-planners)
for more information about how to invoke an OpenRAVE planner.

## Tuning Planner Performance

Collision checking is often the bottleneck for motion planning for manipulators
with the sample-based motion planners included in OMPL. You should consider
using a fast collision checker plugin, like
[or_fcl](https://github.com/personalrobotics/or_fcl), to achieve best
performance with `or_ompl`.

Additionally, you should consider setting the `ActiveDOFs` option during
planning:

>  Allows planners to greatly reduce redundant collision checks. If set and the
>  target object is a robot, then only the links controlled by the currently
>  set active DOFs and their attached bodies will be checked for collisions.
>
> The things that **will not be** checked for collision are: links that do not
> move with respect to each other as a result of moving the active dofs.

You can use a `CollisionOptionsStateSaver` to set the flag and automatically
restore the collision detector to its original state after planning is done:

```python
with CollisionOptionsStateSaver(env.GetCollisionChecker(), CollisionOptions.ActiveDOFs):
    result = planner.PlanPath(traj)
```

## Available Planners

The following table shows which OMPL planners are available via `or_ompl`.   

(1 Oct 2015) Note that if you are using the ROS package of OMPL `ros-indigo-ompl` then BIT* and FMT* will not be available. To use those planners&#9827; you will need to install the latest OMPL from source. There is a catkinized package here [OMPL_catkin_pkg](https://github.com/DavidB-CMU/OMPL_catkin_pkg)

| Planner Name | <sub>OMPL Library</sub> | <sub>OpenRAVE<br>Plugin Name</sub> | <sub>`or_ompl`<br>Unit Test</sub> |
|--------------|-------------------------|------------------------------------|-----------------------------------|
| <sub>BIT* (Batch Informed Trees) &#9827;</sub> | <sub>BITstar</sub> | <sub>OMPL_BITstar</sub> | &#x2717; |
| <sub>BKPIECE1 (Bi-directional KPIECE)</sub> | <sub>BKPIECE1</sub> | <sub>OMPL_BKPIECE1</sub> | &#10004; |
| <sub>EST (Expansive Space Trees)</sub> | <sub>EST</sub> | <sub>OMPL_EST</sub> | &#10004; |
| <sub>FMT* (Fast Marching Tree) &#9827;</sub> | <sub>FMT</sub> | <sub>OMPL_FMT</sub> |&#x2717;|
| <sub>KPIECE1 (Kinematic Planning by Interior-Exterior Cell Exploration)</sub> | <sub>KPIECE1</sub> | <sub>OMPL_KPIECE1</sub> | &#10004; |
| <sub>LBKPIECE1 (Lazy Bi-directional KPIECE)</sub> | <sub>LBKPIECE1</sub> | <sub>OMPL_LBKPIECE1</sub> | &#10004; |
| <sub>LazyPRM</sub> | <sub>LazyPRM</sub> | <sub>OMPL_LazyPRM</sub> | &#10004; |
| <sub>LazyRRT</sub> | <sub>LazyRRT</sub> | <sub>OMPL_LazyRRT</sub> | &#10004; |
| <sub>PDST (Path-Directed Subdivision Trees)</sub> | <sub>PDST</sub> | <sub>OMPL_PDST</sub> |&#x2717;|
| <sub>PRM (Probabilistic Road Map)</sub> | <sub>PRM</sub> | <sub>OMPL_PRM</sub> | &#10004; |
| <sub>PRM*</sub> | <sub>PRMstar</sub> | <sub>OMPL_PRMstar</sub> | &#10004; |
| <sub>RRT (Rapidly Exploring Random Trees)</sub> | <sub>RRT</sub> | <sub>OMPL_RRT</sub> | &#10004; |
| <sub>RRTConnect (Bi-directional RRT)</sub> | <sub>RRTConnect</sub> | <sub>OMPL_RRTConnect</sub> | &#10004; |
| <sub>RRT*</sub> | <sub>RRTstar</sub> | <sub>OMPL_RRTstar</sub> |&#x2717;|
| <sub>SBL (Single-query Bi-directional Lazy collision checking planner)</sub> | <sub>SBL</sub> | <sub>OMPL_SBL</sub> | &#10004; |
| <sub>SPARS (SPArse Roadmap Spanner)</sub> | <sub>SPARS</sub> | <sub>OMPL_SPARS</sub> |&#x2717;|
| <sub>SPARS2</sub> | <sub>SPARStwo</sub> | <sub>OMPL_SPARStwo</sub> |&#x2717;|
| <sub>T-RRT (Transition-based RRT)</sub> | <sub>TRRT</sub> | <sub>OMPL_TRRT</sub> |&#x2717;|
| <sub>pRRT (Parallel RRT)</sub> | <sub>pRRT</sub> | <sub>OMPL_pRRT</sub> |&#x2717;|
| <sub>pSBL (Parallel SBL)</sub> | <sub>pSBL</sub> | <sub>OMPL_pSBL</sub> |&#x2717;|
| <sub>Cforest (Coupled Forest of Random Engrafting Search Trees - parallelization framework)</sub> | <sub>CForest</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>Thunder</sub> | <sub>Thunder</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>Lightning</sub> | <sub>Lightning</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>LazyPRM*</sub> | <sub>LazyPRMstar</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>BiTRRT (Bidirectional T-RRT)</sub> | <sub>BiTRRT</sub> |   <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>LazyLBTRRT</sub> | <sub>LazyLBTRRT</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>LBTRRT (Lower Bound Tree RRT)</sub> | <sub>LBTRRT</sub> | <sub>N/A</sub> | <sub>N/A</sub> |
| <sub>STRIDE (Search Tree with Resolution Independent Density Estimation)</sub> | <sub>STRIDE</sub> | <sub>N/A</sub> | <sub>N/A</sub> |

## License
or_ompl is licensed under a BSD license. See `LICENSE` for more information.

## Contributors
or_ompl was developed by the [Personal Robotics Lab](https://personalrobotics.ri.cmu.edu) in the [Robotics
Institute](http://ri.cmu.edu) at [Carnegie Mellon University](http://www.cmu.edu). This plugin was written by [Michael
Koval](http://mkoval.org) and grew out of earlier plugin written by [Christopher Dellin](http://www.ri.cmu.edu/person.html?person_id=2267) and [Matthew Klingensmith](http://www.ri.cmu.edu/person.html?person_id=2744).
