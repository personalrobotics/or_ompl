^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_ompl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.0 (2015-10-12)
------------------
* Added FMT* and BIT* (`#28 <https://github.com/personalrobotics/or_ompl/issues/28>`_)
* Added integration tests (`#23 <https://github.com/personalrobotics/or_ompl/issues/23>`_)
* Added a print of which planners were found at build time (`#30 <https://github.com/personalrobotics/or_ompl/issues/30>`_)
* Fixed codegen to output planner registery code into a package-specific location to avoid conflicts
* Improved style of the auto-generated code
* Upgraded package.xml to format 2
* Contributors: Chris Dellin, David Butterworth, Michael Koval

0.6.0 (2015-05-01)
------------------

0.5.0 (2015-04-08)
------------------
* Added `GetTimes` command to measure timing performance
* Added support for multiple initial/goal configs to using standard OpenRAVE
  vinitialconfig/vgoalconfig sequencing
* Return trajectories with "linear" interpolation.
* Adding logic to set one element of the state to NaN and logic to the state
  validity checker to test for this.
* Support for goal TSRs.
* Throwing exception if construct is called more than once.
* Removed unused `CD_` macros.
* Call planner callbacks from OMPL_Simplifier.
* Switched from `simplify` to `shortcutPath`.
* Made TSRRobot logging more consistent.
* Removed planner type from `PlannerParameters`.
* Switched to `PlannerFactory` architecture.
* Switched to using a custom robot state space.
* Create a separate OpenRAVE Planner for each OMPL planner (e.g.
  `OMPL_RRTConnect` wraps OMPL's RRTConnect planner).
* `OMPLSimplifier` now calls `shortcutPath` instead of `simplify`.
* Renamed the `OMPLSimplifier` planner to `OMPL_Simplifier`.
* Added a `GetParameters` command for querying planner parameters.
* Added the `example.py` example script.
* Contributors: Chris Dellin, Jennifer King, Michael Koval, Pras Velagapudi

0.4.0 (2014-12-22)
------------------
* Added support for building a standalone OpenRAVE plugin.
* Fixed several CMake issues.
* Added licensing information.
* Contributors: Christopher Dellin, Michael Koval

0.3.0 (2014-10-09)
------------------
* Fixed a bug that could cause OMPLSimplifier to use an incorrect (and very
  coarse) resolution for collision checking.
* Only return if an exact solution is available.
* Only check collisions with active DOFs.
* Switched to using openrave_catkin in `catkin.cmake`.
* Added missing dependencies, including TinyXML
* Use headers, instead of version number, to query planner support.
* Contributors: Michael Koval, Pras Velagapudi

0.2.0 (2014-06-22 13:12)
------------------------
* Wrapped OMPL's PathSimplifier in an OpenRAVE
* Contributors: Michael Koval, Jen King

0.1.0 (2014-06-22 02:32)
------------------------
* Added missing Python dependencies.
* Forward OMPL log messages to OpenRAVE.
* Generate planner wrappers from a YAML config file.
* Check for non-unit DOF weights.
* Set the collision checking resolution.
* Contributors: Christopher Dellin, Matt Klingensmith, Michael Koval, Michael
  Vandeweghe, Pras Velagapudi, Jen King,
