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
#include <ompl/base/ScopedState.h>
#include "OMPLConversions.h"
#include "OMPLSimplifier.h"

namespace or_ompl
{

OMPLSimplifier::OMPLSimplifier(OpenRAVE::EnvironmentBasePtr env)
    : OpenRAVE::PlannerBase(env)
{
}

OMPLSimplifier::~OMPLSimplifier()
{
}

bool OMPLSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot,
                              PlannerParametersConstPtr params_raw)
{
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return false;
    } else if (!params_raw) {
        RAVELOG_ERROR("Parameters must not be NULL.\n");
        return false;
    }

    m_robot = robot;
    m_cspec = m_robot->GetActiveConfigurationSpecification();

    m_parameters = boost::make_shared<OMPLPlannerParameters>();
    m_parameters->copy(params_raw);

    try {
        using ompl::base::SpaceInformation;
        using ompl::geometric::PathSimplifier;

        m_state_space = CreateStateSpace(robot, *m_parameters);
        m_space_info = boost::make_shared<SpaceInformation>(m_state_space);
        m_space_info->setStateValidityChecker(
                boost::bind(&OMPLSimplifier::IsStateValid, this, _1));
        m_space_info->setup();
        m_simplifier = boost::make_shared<PathSimplifier>(m_space_info);
        return true;
    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("IntPlan failed: %s\n", e.what());
        return false;
    }
}

bool OMPLSimplifier::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input)
{
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

OpenRAVE::PlannerStatus OMPLSimplifier::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
    typedef ompl::base::RealVectorStateSpace::StateType StateType;
    typedef ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ScopedState;

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
    // TODO: Why does this SEGFAULT? Something must be wrong with the OMPL path
    RAVELOG_DEBUG("Running path simplification for %f seconds.\n",
                  m_parameters->m_timeLimit);
    BOOST_ASSERT(m_parameters);
    m_simplifier->simplify(path, m_parameters->m_timeLimit);

    // Store the result in the OpenRAVE trajectory.
    RAVELOG_DEBUG("Reconstructing OpenRAVE trajectory with %d waypoints.\n",
                  path.getStateCount());
    BOOST_ASSERT(ptraj);
    ptraj->Remove(0, ptraj->GetNumWaypoints());

    for (size_t iwaypoint = 0; iwaypoint < path.getStateCount(); ++iwaypoint) {
        ompl::base::State const *waypoint_generic = path.getState(iwaypoint);
        StateType const &waypoint_ompl = *waypoint_generic->as<StateType>();

        std::vector<OpenRAVE::dReal> waypoint_openrave(num_dof);
        for (size_t idof = 0; idof < num_dof; ++idof) {
            waypoint_openrave[idof] = waypoint_ompl[idof];
        }
        ptraj->Insert(iwaypoint, waypoint_openrave, m_cspec);
    }
    return OpenRAVE::PS_HasSolution;
}

bool OMPLSimplifier::IsInOrCollision(std::vector<double> const &values)
{
    m_robot->SetActiveDOFValues(values, OpenRAVE::KinBody::CLA_Nothing);
    return GetEnv()->CheckCollision(m_robot) || m_robot->CheckSelfCollision();
}

bool OMPLSimplifier::IsStateValid(ompl::base::State const *state)
{
    typedef ompl::base::RealVectorStateSpace::StateType StateType;
    StateType const *realVectorState = state->as<StateType>();
    size_t const num_dof = m_robot->GetActiveDOF();

    if (realVectorState) {
        std::vector<OpenRAVE::dReal> values(num_dof);
        for (size_t i = 0; i < num_dof; i++) {
            values[i] = realVectorState->values[i];
        }
        return !IsInOrCollision(values);
    } else {
        RAVELOG_ERROR("Invalid StateType. This should never happen.\n");
        return false;
    }
}

}
