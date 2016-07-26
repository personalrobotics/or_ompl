/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <boost/make_shared.hpp>
#include <boost/scope_exit.hpp>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Time.h>

#include <or_ompl/config.h>
#include <or_ompl/OMPLConversions.h>
#include <or_ompl/OMPLSimplifer.h>

using OpenRAVE::PA_None;
using OpenRAVE::PA_Interrupt;
using OpenRAVE::PA_ReturnWithAnySolution;

using OpenRAVE::PS_HasSolution;
using OpenRAVE::PS_InterruptedWithSolution;

namespace or_ompl {

OMPLSimplifier::OMPLSimplifier(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::PlannerBase(env) {
}

OMPLSimplifier::~OMPLSimplifier() {
}

bool OMPLSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot,
                              PlannerParametersConstPtr params_raw) {
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return false;
    } else if (!params_raw) {
        RAVELOG_ERROR("Parameters must not be NULL.\n");
        return false;
    }

    m_robot = robot;
    m_cspec = m_robot->GetActiveConfigurationSpecification();
    std::vector<int> dof_indices = robot->GetActiveDOFIndices();

    m_parameters = boost::make_shared<OMPLPlannerParameters>();
    m_parameters->copy(params_raw);

    try {
        using ompl::base::SpaceInformation;
        using ompl::geometric::PathSimplifier;

        m_state_space = CreateStateSpace(robot, *m_parameters);
        m_space_info.reset(new SpaceInformation(m_state_space));
        if (m_state_space->isCompound()) {
            m_or_validity_checker.reset(new OrStateValidityChecker(
                m_space_info, m_robot, dof_indices, m_parameters->m_doBaked));
        } else {
            m_or_validity_checker.reset(new RealVectorOrStateValidityChecker(
                m_space_info, m_robot, dof_indices, m_parameters->m_doBaked));
        }
#ifdef OR_OMPL_HAS_BOOSTSMARTPTRS
        m_space_info->setStateValidityChecker(
            boost::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#else
        m_space_info->setStateValidityChecker(
            std::static_pointer_cast<ompl::base::StateValidityChecker>(m_or_validity_checker));
#endif
        
        m_space_info->setup();
        m_simplifier.reset(new PathSimplifier(m_space_info));
        return true;
    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("IntPlan failed: %s\n", e.what());
        return false;
    }
}

bool OMPLSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input) {
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

OpenRAVE::PlannerStatus OMPLSimplifier::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    typedef ompl::base::ScopedState<ompl::base::StateSpace> ScopedState;

    if (!m_simplifier) {
        RAVELOG_ERROR("Not initialized. Did you call InitPlan?\n");
        return OpenRAVE::PS_Failed;
    } else if (!ptraj) {
        RAVELOG_ERROR("Input trajectory is NULL.\n");
        return OpenRAVE::PS_Failed;
    } else if (m_parameters->m_timeLimit <= 0) {
        RAVELOG_ERROR("Time limit must be positive; got %f.",
                      m_parameters->m_timeLimit);
        return OpenRAVE::PS_Failed;
    } else if (ptraj->GetNumWaypoints() == 0) {
        RAVELOG_WARN("Input trajectory is empty; there is nothing to do.\n");
        return OpenRAVE::PS_HasSolution;
    }

    // Convert the OpenRAVE trajectory into an OMPL path.
    BOOST_ASSERT(m_space_info);
    ompl::geometric::PathGeometric path(m_space_info);
    size_t const num_dof = m_cspec.GetDOF();

    RAVELOG_DEBUG("Create OMPL path with %d DOF and %d waypoints.\n",
                  num_dof, ptraj->GetNumWaypoints());

    for (size_t iwaypoint = 0; iwaypoint < ptraj->GetNumWaypoints(); ++iwaypoint) {
        // Extract the OpenRAVE waypoint. Default to the current configuration
        // for any missing DOFs.
        std::vector<OpenRAVE::dReal> waypoint_openrave;
        m_robot->GetActiveDOFValues(waypoint_openrave);
        ptraj->GetWaypoint(iwaypoint, waypoint_openrave, m_cspec);

        // Insert the waypoint into the OMPL path.
        ScopedState waypoint_ompl(m_space_info);
        for (size_t idof = 0; idof < num_dof; ++idof) {
            waypoint_ompl[idof] = waypoint_openrave[idof];
        }
        path.append(waypoint_ompl.get());
    }

    // Run path simplification.
    OpenRAVE::PlannerBase::PlannerProgress progress;
    OpenRAVE::PlannerAction planner_action = PA_None;
    double const length_before = path.length();
    int num_changes = 0;

    ompl::time::duration const time_limit
        = ompl::time::seconds(m_parameters->m_timeLimit);
    ompl::time::point const time_before = ompl::time::now();
    ompl::time::point time_current;

    RAVELOG_DEBUG("Running path simplification for %f seconds.\n",
                  m_parameters->m_timeLimit);
    
    // start validity checker
    m_or_validity_checker->start();
    BOOST_SCOPE_EXIT((m_or_validity_checker)) {
        m_or_validity_checker->stop();
    } BOOST_SCOPE_EXIT_END

    do {
        // Run one iteration of shortcutting. This gives us fine control over
        // the termination condition and allows us to invoke the planner
        // callbacks between iterations.
        //
        // The numeric arguments are the following:
        // - maxSteps: maximum number of iterations
        // - maxEmptySteps: maximum number of iterations without improvement
        // - rangeRatio: maximum connection distance attempted, specified as a
        //               ratio of the total number of states
        // - snapToVertex: ratio of total path length used to snap samples to
        //                 vertices
        bool const changed = m_simplifier->shortcutPath(path, 1, 1, 1.0, 0.005);
        num_changes += !!changed;
        progress._iteration += 1;

        // Call any user-registered callbacks. These functions can terminate
        // planning early.
        planner_action = _CallCallbacks(progress);

        time_current = ompl::time::now();
    } while (time_current - time_before <= time_limit
          && planner_action == PA_None);

    double const length_after = path.length();

    RAVELOG_DEBUG(
        "Ran %d iterations of smoothing over %.3f seconds. %d of %d iterations"
        " (%.2f%%) were effective. Reduced path length from %.3f to %.3f.\n",
        progress._iteration,
        ompl::time::seconds(time_current - time_before),
        num_changes,
        progress._iteration,
        100 * static_cast<double>(num_changes) / progress._iteration,
        length_before, length_after
    );

    // Store the result in the OpenRAVE trajectory.
    RAVELOG_DEBUG("Reconstructing OpenRAVE trajectory with %d waypoints.\n",
                  path.getStateCount());
    BOOST_ASSERT(ptraj);
    ptraj->Remove(0, ptraj->GetNumWaypoints());

    ToORTrajectory(m_robot, path, ptraj);

    if (planner_action == PA_None) {
        return PS_HasSolution;
    } else {
        return PS_InterruptedWithSolution;
    }
}

} // namespace or_ompl
