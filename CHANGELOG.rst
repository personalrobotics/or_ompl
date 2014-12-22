^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package or_ompl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
