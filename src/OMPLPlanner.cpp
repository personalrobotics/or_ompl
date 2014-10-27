/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

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
#include <time.h>
#include <tinyxml.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <ompl/config.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "OMPLConversions.h"
#include "OMPLPlanner.h"
#include "PlannerRegistry.h"

#define OMPL_VERSION_COMP (  OMPL_MAJOR_VERSION * 1000000 \
                           + OMPL_MINOR_VERSION * 1000 \
                           + OMPL_PATCH_VERSION)

#define CD_OS_TIMESPEC_SET_ZERO(t) do { (t)->tv_sec = 0; (t)->tv_nsec = 0; } while (0)
#define CD_OS_TIMESPEC_ADD(dst, src) do { (dst)->tv_sec += (src)->tv_sec; (dst)->tv_nsec += (src)->tv_nsec; \
   if ((dst)->tv_nsec > 999999999) { (dst)->tv_sec += 1; (dst)->tv_nsec -= 1000000000; } } while (0)
#define CD_OS_TIMESPEC_SUB(dst, src) do { (dst)->tv_sec -= (src)->tv_sec; (dst)->tv_nsec -= (src)->tv_nsec; \
   if ((dst)->tv_nsec < 0) { (dst)->tv_sec -= 1; (dst)->tv_nsec += 1000000000; } } while (0)
#define CD_OS_TIMESPEC_DOUBLE(src) ((src)->tv_sec + ((double)((src)->tv_nsec))/1000000000.0)

#define TIME_COLLISION_CHECKS

namespace or_ompl
{

OMPLPlanner::OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv)
    : OpenRAVE::PlannerBase(penv)
    , m_initialized(false)
{
}

OMPLPlanner::~OMPLPlanner()
{
}

bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
{
    OMPLPlannerParametersPtr params = boost::make_shared<OMPLPlannerParameters>();
    input >> *params;
    return InitPlan(robot, params);
}

bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot,
                           PlannerParametersConstPtr params_raw)
{
    m_initialized = false;

    try {
        typedef ompl::base::ScopedState<ompl::base::RealVectorStateSpace> ScopedState;

        if (!robot) {
            RAVELOG_ERROR("Robot must not be NULL.\n");
            return false;
        } else if (!params_raw) {
            RAVELOG_ERROR("Parameters must not be NULL.\n");
            return false;
        }

        m_robot = robot;
        m_totalCollisionTime = 0.0;
        m_numCollisionChecks = 0;

        size_t const num_dof = robot->GetActiveDOF();
        m_parameters = boost::make_shared<OMPLPlannerParameters>();
        m_parameters->copy(params_raw);

        RAVELOG_DEBUG("Creating state space.\n");
        m_state_space = CreateStateSpace(robot, *m_parameters);
        if (!m_state_space) {
            RAVELOG_ERROR("Failed creating state space.\n");
            return false;
        }

        RAVELOG_DEBUG("Creating OMPL setup.\n");
        m_simple_setup = boost::make_shared<ompl::geometric::SimpleSetup>(m_state_space);

        RAVELOG_DEBUG("Setting initial configuration.\n");
        if (m_parameters->vinitialconfig.size() != num_dof) {
            RAVELOG_ERROR("Start configuration has incorrect DOF;"
                          " expected %d, got %d.\n",
                          num_dof, m_parameters->vinitialconfig.size());
            return false;
        } else if (IsInOrCollision(m_parameters->vinitialconfig)) {
            RAVELOG_ERROR("Initial configuration in collision.\n");
            return false;
        }

        ScopedState q_start(m_state_space);
        for (size_t i = 0; i < num_dof; i++) {
            q_start->values[i] = m_parameters->vinitialconfig[i];
        }
        m_simple_setup->setStartState(q_start);

        RAVELOG_DEBUG("Setting goal configuration.\n");
        if (m_parameters->vgoalconfig.size() != num_dof) {
            RAVELOG_ERROR("End configuration has incorrect DOF;"
                          "  expected %d, got %d.\n",
                          num_dof, m_parameters->vgoalconfig.size());
            return false;
        } else if (IsInOrCollision(m_parameters->vgoalconfig)) {
            RAVELOG_ERROR("Goal configuration is in collision.\n");
            return false;
        }

        ScopedState q_goal(m_state_space);
        for (size_t i = 0; i < num_dof; i++) {
            q_goal->values[i] = m_parameters->vgoalconfig[i];
        }
        m_simple_setup->setGoalState(q_goal);

        RAVELOG_DEBUG("Creating planner.\n");
        m_planner = CreatePlanner(*m_parameters);
        if (!m_planner) {
            RAVELOG_ERROR("Failed creating OMPL planner.\n");
            return false;
        }
        m_simple_setup->setPlanner(m_planner);

        RAVELOG_DEBUG("Setting state validity checker.\n");
        m_simple_setup->setStateValidityChecker(
            boost::bind(&or_ompl::OMPLPlanner::IsStateValid, this, _1));

        m_initialized = true;
        return true;
    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("IntPlan failed: %s\n", e.what());
        return false;
    }
}

ompl::base::PlannerPtr OMPLPlanner::CreatePlanner(
    OMPLPlannerParameters const &params)
{
    // Create the planner.
    std::string const plannerName = m_parameters->m_plannerType;
    ompl::base::SpaceInformationPtr const spaceInformation
            = m_simple_setup->getSpaceInformation();

    ompl::base::PlannerPtr planner(registry::create(
            plannerName, spaceInformation));
    if (!planner) {
        RAVELOG_ERROR("Failed creating planner '%s'.\n", plannerName.c_str());
        return ompl::base::PlannerPtr();
    }

    // Populate planner parameters from the PlannerParameters.
    std::string const params_str = "<ExtraParams>"
                                   + m_parameters->_sExtraParameters
                                   + "</ExtraParams>";
    TiXmlDocument doc_xml;
    doc_xml.Parse(params_str.c_str());
    if (doc_xml.Error()) {
        RAVELOG_ERROR("Failed parsing XML parameters: %s\n",
                      doc_xml.ErrorDesc());
        return ompl::base::PlannerPtr();
    }

    TiXmlElement const *root_xml = doc_xml.RootElement();
    std::map<std::string, std::string> params_map;

    for (TiXmlElement const *it_ele = root_xml->FirstChildElement();
         it_ele;
         it_ele = it_ele->NextSiblingElement()) {
        // Extract the property name.
        std::string const key = it_ele->ValueStr();

        // Extract the property value.
        TiXmlNode const *node = it_ele->FirstChild();
        if (!node) {
            RAVELOG_ERROR("Failed parsing planner parameters:"
                          " Element '%s' does not contain a value.\n",
                          key.c_str());
            return ompl::base::PlannerPtr();
        }
        TiXmlText const *text = node->ToText();
        TiXmlNode const *next_node = node->NextSibling();
        if (!text || next_node) {
            RAVELOG_ERROR("Failed parsing planner parameters:"
                          " Element '%s' contains complex data.\n",
                          key.c_str());
        }
        std::string const value = text->Value();

        params_map.insert(std::make_pair(key, value));
    }

    ompl::base::ParamSet &param_set = planner->params();
    bool const is_success = param_set.setParams(params_map, false);

    // Print out the list of valid parameters.
    if (!is_success) {
        std::vector<std::string> param_names;
        param_set.getParamNames(param_names);

        std::stringstream param_stream; 
        BOOST_FOREACH (std::string const &param_name, param_names) {
            param_stream << " " << param_name;
        }
        std::string const param_str = param_stream.str();

        RAVELOG_ERROR("Invalid planner parameters."
                      " The following parameters are supported:%s\n",
                      param_str.c_str());
        return ompl::base::PlannerPtr();
    }
    return planner;
}

OpenRAVE::PlannerStatus OMPLPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
    try {
        if (!m_initialized) {
            RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n");
            return OpenRAVE::PS_Failed;
        }

        // TODO: Configure anytime algorithms to keep planning.
        //m_simpleSetup->getGoal()->setMaximumPathLength(0.0);

        {
            // Don't check collision with inactive links.
            OpenRAVE::CollisionCheckerBasePtr const collision_checker
                = GetEnv()->GetCollisionChecker();
            OpenRAVE::CollisionOptionsStateSaver const collision_saver(
                collision_checker, OpenRAVE::CO_ActiveDOFs, false);

            // Call the planner.
            m_simple_setup->solve(m_parameters->m_timeLimit);
        }

        if (m_simple_setup->haveExactSolutionPath()) {
            ToORTrajectory(m_simple_setup->getSolutionPath(), ptraj);
            return OpenRAVE::PS_HasSolution;
        } else {
            return OpenRAVE::PS_Failed;
        }

    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("Planning failed: %s\n", e.what());
        return OpenRAVE::PS_Failed;
    }
}

bool OMPLPlanner::IsInOrCollision(std::vector<double> const &values)
{
    m_robot->SetActiveDOFValues(values, OpenRAVE::KinBody::CLA_Nothing);
    bool const collided = GetEnv()->CheckCollision(m_robot)
                       || m_robot->CheckSelfCollision();
    m_numCollisionChecks++;
    return collided;
}

bool OMPLPlanner::IsStateValid(ompl::base::State const *state)
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

OpenRAVE::PlannerStatus OMPLPlanner::ToORTrajectory(
        ompl::geometric::PathGeometric &ompl_traj,
        OpenRAVE::TrajectoryBasePtr or_traj) const
{
#if OMPL_VERSION_COMP >= 000010002
    std::vector<ompl::base::State*> const &states = ompl_traj.getStates();
#else
    std::vector<ompl::base::State*> const &states = ompl_traj.states;
#endif

    size_t const num_dof = m_robot->GetActiveDOF();
    or_traj->Init(m_robot->GetActiveConfigurationSpecification());

    for (size_t i = 0; i < states.size(); ++i){
        ompl::base::RealVectorStateSpace::StateType const *state
            = states[i]->as<ompl::base::RealVectorStateSpace::StateType>();
        if (!state) {
            RAVELOG_ERROR("Unable to convert output trajectory."
                          "State is not a RealVectorStateSpace::StateType.");
            return OpenRAVE::PS_Failed;
        }

        std::vector<OpenRAVE::dReal> sample(num_dof);
        for (size_t j = 0; j < num_dof; ++j) {
            sample[j] = (*state)[j];
        }
        or_traj->Insert(i, sample, true);
    }
    return OpenRAVE::PS_HasSolution;
}

}
