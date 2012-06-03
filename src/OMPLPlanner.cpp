/*
 * OMPLPlanner.cpp
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#include "OMPLPlanner.h"

using namespace OpenRAVE;

namespace or_ompl
{

    OMPLPlanner::OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv) :  OpenRAVE::PlannerBase(penv), m_simpleSetup(NULL)
    {

    }

    OMPLPlanner::~OMPLPlanner()
    {
        if(m_simpleSetup != NULL)
        {
            delete m_simpleSetup;
        }
    }

    bool OMPLPlanner::InitPlan(RobotBasePtr robot, PlannerParametersConstPtr params)
    {
        if(m_simpleSetup != NULL)
        {
            delete m_simpleSetup;
            m_simpleSetup = NULL;
        }

        m_robot = robot;

        m_stateSpace.reset(new ompl::base::RealVectorStateSpace(robot->GetActiveDOF()));

        ompl::base::RealVectorBounds bounds(m_robot->GetActiveDOF());
        std::vector<double> lowerLimits;
        std::vector<double> upperLimits;
        m_robot->GetActiveDOFLimits(lowerLimits, upperLimits);

        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            bounds.setLow(i, lowerLimits[i]);
            bounds.setHigh(i, upperLimits[i]);
        }

        m_stateSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);


        m_simpleSetup = new ompl::geometric::SimpleSetup(GetStateSpace());

        if(!InitializePlanner())
        {
            return false;
        }

        m_simpleSetup->setPlanner(m_planner);


        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> startPose(m_stateSpace);
        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            (*(startPose->as<ompl::base::RealVectorStateSpace::StateType>()))[i]= params->vinitialconfig[i];
        }

        if(IsInCollision(params->vinitialconfig))
        {
            ROS_ERROR("Can't plan. Initial configuration in collision!");
            delete m_simpleSetup;
            return false;
        }

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> endPose(m_stateSpace);
        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            (*(endPose->as<ompl::base::RealVectorStateSpace::StateType>()))[i] = params->vgoalconfig[i];
        }

        if(IsInCollision(params->vgoalconfig))
        {
            ROS_ERROR("Can't plan. Final configuration is in collision!");
            delete m_simpleSetup;
            return false;
        }

        m_simpleSetup->setStateValidityChecker(boost::bind(&or_ompl::OMPLPlanner::IsStateValid, this, _1));
        m_simpleSetup->setStartState(startPose);
        m_simpleSetup->setGoalState(endPose);

        return true;
    }

    bool OMPLPlanner::InitializePlanner()
    {
        std::string plannerName = m_parameters->m_plannerType;

        ompl::base::SpaceInformationPtr spaceInformation(new ompl::base::SpaceInformation(m_stateSpace));
        if(plannerName == "KPIECE")
        {
            m_planner.reset(new ompl::geometric::KPIECE1(spaceInformation));
        }
        else if(plannerName == "BKPIECE")
        {
            m_planner.reset(new ompl::geometric::BKPIECE1(spaceInformation));
        }
        else if(plannerName == "LBKPIECE")
        {
            m_planner.reset(new ompl::geometric::LBKPIECE1(spaceInformation));
        }
        else if(plannerName == "EST")
        {
            m_planner.reset(new ompl::geometric::EST(spaceInformation));
        }
        else if(plannerName == "RRT")
        {
            m_planner.reset(new ompl::geometric::RRT(spaceInformation));
        }
        else if(plannerName == "RRTConnect")
        {
            m_planner.reset(new ompl::geometric::RRTConnect(spaceInformation));
        }
        else if(plannerName == "pRRT")
        {
            m_planner.reset(new ompl::geometric::pRRT(spaceInformation));
        }
        else if(plannerName == "LazyRRT")
        {
            m_planner.reset(new ompl::geometric::LazyRRT(spaceInformation));
        }
        else if(plannerName == "PRM")
        {
            m_planner.reset(new ompl::geometric::BasicPRM(spaceInformation));
        }
        else if(plannerName == "RRTstar")
        {
            m_planner.reset(new ompl::geometric::RRTstar(spaceInformation));
        }
        else if(plannerName == "BallTreeRRTstar")
        {
            m_planner.reset(new ompl::geometric::BallTreeRRTstar(spaceInformation));
        }
        else
        {
            ROS_ERROR("Urecognized planner: %s", plannerName.c_str());
            return false;
        }

        return true;
    }

    OpenRAVE::PlannerStatus OMPLPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
    {
        if(!m_simpleSetup)
        {
            ROS_ERROR("Couldn't plan a path. Simple setup was null!");
            return OpenRAVE::PS_Failed;
        }
        else
        {
            bool success = m_simpleSetup->solve(m_parameters->m_timeLimit);

            ConfigurationSpecification spec =  m_robot->GetActiveConfigurationSpecification();
            ptraj->Init(spec);

            if(success)
            {
                std::vector<ompl::base::State*>& states = m_simpleSetup->getSolutionPath().states;
                for(size_t i = 0; i < states.size(); i++)
                {
                    const ompl::base::RealVectorStateSpace::StateType* state = states[i]->as<ompl::base::RealVectorStateSpace::StateType>();
                    if(!state)
                    {
                        ROS_ERROR("Invalid state type!");
                        return OpenRAVE::PS_Failed;
                    }
                    else
                    {
                        OpenRAVE::TrajectoryBase::Point point;

                        for(int j = 0; j < m_robot->GetActiveDOF(); j++)
                        {
                            point.q.push_back((*state)[j]);
                        }
                        ptraj->Insert(i, point.q, true);
                    }
                }

                return OpenRAVE::PS_HasSolution;
            }
            else
            {
                return OpenRAVE::PS_Failed;
            }
        }

        return OpenRAVE::PS_Failed;
    }

    bool OMPLPlanner::IsInCollision(std::vector<double> values)
    {
        m_robot->SetActiveDOFValues(values, true);
        CollisionReportPtr preport(new CollisionReport());
        return (GetEnv()->CheckCollision(KinBodyConstPtr(m_robot), preport)) || (m_robot->CheckSelfCollision(preport));
    }

    bool OMPLPlanner::IsStateValid(const ompl::base::State* state)
    {
        const ompl::base::RealVectorStateSpace::StateType* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();

        if(!realVectorState)
        {
            ROS_ERROR("State type requested was invalid!");
            return false;
        }
        else
        {
            std::vector<double> values;
            for(int i = 0; i < m_robot->GetActiveDOF(); i++)
            {
                values.push_back(realVectorState->values[i]);
            }

            return !IsInCollision(values);
        }

    }


} /* namespace or_ompl */
