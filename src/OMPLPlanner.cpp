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
#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <ompl/config.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <or_ompl/OMPLConversions.h>
#include <or_ompl/OMPLPlanner.h>
#include <or_ompl/TSRGoal.h>
#include <or_ompl/PlannerRegistry.h>

namespace or_ompl
{

OMPLPlanner::OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv,
                         PlannerFactory const &planner_factory)
    : OpenRAVE::PlannerBase(penv)
    , m_initialized(false)
    , m_planner_factory(planner_factory)

{
    RegisterCommand("GetParameters",
        boost::bind(&OMPLPlanner::GetParametersCommand, this, _1, _2),
        "returns the list of accepted planner parameters"
    );
    RegisterCommand("GetTimes",
        boost::bind(&OMPLPlanner::GetTimes,this,_1,_2),
        "get timing information from last plan");
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
        typedef ompl::base::ScopedState<RobotStateSpace> ScopedState;

        if (!robot) {
            RAVELOG_ERROR("Robot must not be NULL.\n");
            return false;
        } else if (!params_raw) {
            RAVELOG_ERROR("Parameters must not be NULL.\n");
            return false;
        }

        m_robot = robot;
        m_totalCollisionTime = 0.0;
        m_totalPlanningTime = 0.0;
        m_numCollisionChecks = 0;

        std::vector<int> dof_indices = robot->GetActiveDOFIndices();
        const unsigned int num_dof = dof_indices.size();
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
        if (m_parameters->vinitialconfig.size() % num_dof != 0) {
            RAVELOG_ERROR("Start configuration has incorrect DOF;"
                          " expected multiple of %d, got %d.\n",
                          num_dof, m_parameters->vinitialconfig.size());
            return false;
        }
        unsigned int num_starts = m_parameters->vinitialconfig.size() / num_dof;
        if (num_starts == 0) {
            RAVELOG_ERROR("No initial configurations provided.\n");
            return false;
        }
        
        if (num_starts == 1 && IsInOrCollision(m_parameters->vinitialconfig, dof_indices)) {
            RAVELOG_ERROR("Single initial configuration in collision.\n");
            return false;
        }
        
        for (unsigned int istart=0; istart<num_starts; istart++)
        {
            ScopedState q_start(m_state_space);
            for (size_t i = 0; i < num_dof; i++) {
                q_start->value(i) = m_parameters->vinitialconfig[istart*num_dof + i];
            }
            m_simple_setup->addStartState(q_start);
        }

        RAVELOG_DEBUG("Setting goal configuration.\n");
		std::vector<TSRChain::Ptr> goal_chains;
		BOOST_FOREACH(TSRChain::Ptr tsr_chain, m_parameters->m_tsrchains){
			if(tsr_chain->sampleGoal()){
                tsr_chain->setEnv(robot->GetEnv()); // required to enable distance to TSR chains
				goal_chains.push_back(tsr_chain);
			}else{
                RAVELOG_ERROR("Only goal TSR chains are supported by OMPL. Failing.\n");
                return false;
            }
		}

        if(goal_chains.size() > 0 && m_parameters->vgoalconfig.size() > 0){
            RAVELOG_ERROR("A goal TSR chain has been supplied and a goal configuration"
                          " has been specified. The desired behavior is ambiguous."
                          " Please specified one or the other.\n");
            return false;
        }

        if(goal_chains.size() > 0) {
            TSRGoal::Ptr goaltsr = boost::make_shared<TSRGoal>(m_simple_setup->getSpaceInformation(),
															   goal_chains,
															   robot);
            m_simple_setup->setGoal(goaltsr);
        }else{
            if (m_parameters->vgoalconfig.size() % num_dof != 0) {
                RAVELOG_ERROR("End configuration has incorrect DOF;"
                              "  expected multiple of %d, got %d.\n",
                              num_dof, m_parameters->vgoalconfig.size());
                return false;
            }
            unsigned int num_goals = m_parameters->vgoalconfig.size() / num_dof;
            if (num_goals == 0) {
                RAVELOG_ERROR("No goal configurations provided.\n");
                return false;
            }
            
            if (num_goals == 1) {
                if (IsInOrCollision(m_parameters->vgoalconfig, dof_indices)) {
                    RAVELOG_ERROR("Single goal configuration is in collision.\n");
                    return false;
                }
            
                ScopedState q_goal(m_state_space);
                for (size_t i = 0; i < num_dof; i++) {
                    q_goal->value(i) = m_parameters->vgoalconfig[i];
                }
                m_simple_setup->setGoalState(q_goal);
            } else {
                // if multiple possible goals specified,
                // don't check them all (this might be expensive)
                // and instead lead the planner check some
                ompl::base::GoalPtr ompl_goals(new ompl::base::GoalStates(
                    m_simple_setup->getSpaceInformation()));
                for (unsigned int igoal=0; igoal<num_goals; igoal++)
                {
                    ScopedState q_goal(m_state_space);
                    for (size_t i = 0; i < num_dof; i++) {
                        q_goal->value(i) = m_parameters->vgoalconfig[igoal*num_dof + i];
                    }
                    ompl_goals->as<ompl::base::GoalStates>()->addState(q_goal);
                }
                m_simple_setup->setGoal(ompl_goals);
            }
        }

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
        RAVELOG_ERROR("InitPlan failed: %s\n", e.what());
        return false;
    }
}

ompl::base::PlannerPtr OMPLPlanner::CreatePlanner(
    OMPLPlannerParameters const &params)
{
    // Create the planner.
    ompl::base::SpaceInformationPtr const spaceInformation
            = m_simple_setup->getSpaceInformation();

    ompl::base::PlannerPtr planner(m_planner_factory(spaceInformation));
    if (!planner) {
        RAVELOG_ERROR("Failed creating planner.");
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
    if (!m_initialized) {
        RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n");
        return OpenRAVE::PS_Failed;
    }

    boost::chrono::steady_clock::time_point const tic
       = boost::chrono::steady_clock::now();
    
    OpenRAVE::PlannerStatus planner_status;
    try {
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
            ToORTrajectory(m_robot, m_simple_setup->getSolutionPath(), ptraj);
            planner_status = OpenRAVE::PS_HasSolution;
        } else {
            planner_status = OpenRAVE::PS_Failed;
        }

    } catch (std::runtime_error const &e) {
        RAVELOG_ERROR("Planning failed: %s\n", e.what());
        planner_status = OpenRAVE::PS_Failed;
    }
    
    boost::chrono::steady_clock::time_point const toc
        = boost::chrono::steady_clock::now();
    m_totalPlanningTime += boost::chrono::duration_cast<
        boost::chrono::duration<double> >(toc - tic).count();
    
    return planner_status;
}

bool OMPLPlanner::IsInOrCollision(std::vector<double> const &values, std::vector<int> const &indices)
{
    boost::chrono::steady_clock::time_point const tic
       = boost::chrono::steady_clock::now();
    
    m_robot->SetDOFValues(values, OpenRAVE::KinBody::CLA_Nothing, indices);
    bool const collided = GetEnv()->CheckCollision(m_robot)
                       || m_robot->CheckSelfCollision();
    
    boost::chrono::steady_clock::time_point const toc
        = boost::chrono::steady_clock::now();
    m_totalCollisionTime += boost::chrono::duration_cast<
        boost::chrono::duration<double> >(toc - tic).count();
    m_numCollisionChecks++;
    
    return collided;
}

bool OMPLPlanner::IsStateValid(ompl::base::State const *state)
{
    RobotState const *realVectorState = state->as<RobotState>();
    
    if (!realVectorState)
    {
        RAVELOG_ERROR("Invalid StateType. This should never happen.\n");
        return false;
    }
    
    std::vector<double> values = realVectorState->getValues();
    BOOST_FOREACH(double v, values) {
        if(std::isnan(v)) {
            RAVELOG_ERROR("Invalid value in state.\n");
            return false;
        }
    }
    
    return !IsInOrCollision(realVectorState->getValues(), realVectorState->getSpace()->getIndices());
}

bool OMPLPlanner::GetParametersCommand(std::ostream &sout, std::istream &sin) const
{
    typedef std::map<std::string, ompl::base::GenericParamPtr> ParamMap;

    ompl::base::PlannerPtr planner;
    if (m_planner) {
        planner = m_planner;
    }
    // We need an instance of the planner to query its ParamSet. Unfortunately,
    // constructing the planner requires a SpaceInformationPtr, which can only
    // be generated from an existing StateSpace. As a workaround, we construct
    // a simple one-DOF state space and make a temporary planner instance.
    else {
        ompl::base::StateSpacePtr const state_space
            = boost::make_shared<ompl::base::RealVectorStateSpace>(1);
        ompl::base::SpaceInformationPtr const space_information 
            = boost::make_shared<ompl::base::SpaceInformation>(state_space);
        planner.reset(m_planner_factory(space_information));
    }

    // Query the supported parameters. Each planner has a name and a "range
    // suggestion", which is used to generate the GUI in OMPL.app.
    ompl::base::ParamSet const &param_set = planner->params();
    ParamMap const &param_map = param_set.getParams();

    ParamMap::const_iterator it;
    for (it = param_map.begin(); it != param_map.end(); ++it) {
        sout << it->first << " (" << it->second->getRangeSuggestion() << ")\n";
    }

    return true;
}

bool OMPLPlanner::GetTimes(std::ostream & sout, std::istream & sin) const
{
    sout << "checktime " << m_totalCollisionTime;
    sout << " totaltime " << m_totalPlanningTime;
    sout << " n_checks " << m_numCollisionChecks;
    return true;
}

}
