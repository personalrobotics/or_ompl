or_ompl
=======

[OpenRAVE](http://openrave.org/) bindings for the [OMPL](http://ompl.kavrakilab.org/) suite of motion planning algorithms. This package provides an "ompl" plugin that implements the `OpenRAVE::PlannerBase` interface and delegates planning requests to an OMPL planner. This is implemented by providing OMPL with a custom "state validity checker" that uses OpenRAVE to check collision with the environment.

Each instance of the OpenRAVE plugin wraps a single OMPL planning algorithm that specified during construction. The following keyword arguments are supported:

- RRT
- RRTConnect
- pRRT
- LazyRRT
- RRTstar
- BallTreeRRTstar
- PRM
- EST
- KPIECE
- BKPIECE
- LBKPIECE

It is possible to load multiple instances of the plugin with different planners into the same environment, e.g. to plan with both RRT-Connect and RRT*. We aim to support all planners that are implemented in OMPL. Please [send us a pull request](https://github.com/personalrobotics/or_ompl/compare/) or [open an issue](https://github.com/personalrobotics/or_ompl/issues/new) if a planner you are interested in using is not wrapped.

Example
-------

The following Python code will plan using OMPL's implementation of RRT-Connect. We assume that the variables `env`, `robot`, and `params` have been properly set for your particular scenario. See the [documentation on the OpenRAVE website](http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/#directly-launching-planners) for more information about how to directly invoke a planner.

    planner = openravepy.RaveCreatePlanner(env, 'ompl RRTConnect')
    planner.InitPlan(robot, params)
        
    traj = openravepy.RaveCreateTrajectory(env, '')
    planner.PlanPath(traj)
    
