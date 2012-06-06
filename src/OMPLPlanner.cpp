/*
 * OMPLPlanner.cpp
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#include "OMPLPlanner.h"

using namespace OpenRAVE;

#include <time.h>

#define CD_OS_TIMESPEC_SET_ZERO(t) do { (t)->tv_sec = 0; (t)->tv_nsec = 0; } while (0)
#define CD_OS_TIMESPEC_ADD(dst, src) do { (dst)->tv_sec += (src)->tv_sec; (dst)->tv_nsec += (src)->tv_nsec; \
   if ((dst)->tv_nsec > 999999999) { (dst)->tv_sec += 1; (dst)->tv_nsec -= 1000000000; } } while (0)
#define CD_OS_TIMESPEC_SUB(dst, src) do { (dst)->tv_sec -= (src)->tv_sec; (dst)->tv_nsec -= (src)->tv_nsec; \
   if ((dst)->tv_nsec < 0) { (dst)->tv_sec -= 1; (dst)->tv_nsec += 1000000000; } } while (0)
#define CD_OS_TIMESPEC_DOUBLE(src) ((src)->tv_sec + ((double)((src)->tv_nsec))/1000000000.0)

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

    bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
    {
        OMPLPlannerParameters* params = new OMPLPlannerParameters();
        input >> (*params);

        return InitPlan(robot, PlannerParametersConstPtr(params));
    }

    bool OMPLPlanner::InitPlan(RobotBasePtr robot, PlannerParametersConstPtr params)
    {
        RAVELOG_INFO("Initializing plan\n");
        if(m_simpleSetup != NULL)
        {
            delete m_simpleSetup;
            m_simpleSetup = NULL;
        }

        m_parameters.reset(new OMPLPlannerParameters());
        m_parameters->copy(params);

        m_robot = robot;

        RAVELOG_INFO("Setting state space\n");
        m_stateSpace.reset(new ompl::base::RealVectorStateSpace(robot->GetActiveDOF()));

        RAVELOG_INFO("Setting joint limits\n");
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

        RAVELOG_INFO("Setting up simplesetup class.\n");
        m_simpleSetup = new ompl::geometric::SimpleSetup(GetStateSpace());

        RAVELOG_INFO("Initializing the planner.\n");
        if(!InitializePlanner())
        {
            return false;
        }

        RAVELOG_INFO("Setting the planner.\n");
        m_simpleSetup->setPlanner(m_planner);


        RAVELOG_INFO("Creating the start pose.\n");
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> startPose(m_stateSpace);
        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            startPose->values[i] = m_parameters->vinitialconfig[i];
        }

        RAVELOG_INFO("Checking collisions.\n");
        if(IsInCollision(params->vinitialconfig))
        {
            ROS_ERROR("Can't plan. Initial configuration in collision!\n");
            delete m_simpleSetup;
            return false;
        }

        RAVELOG_INFO("Setting the end pose.\n");
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> endPose(m_stateSpace);
        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            endPose->values[i] = params->vgoalconfig[i];
        }

        RAVELOG_INFO("Checking collisions\n");
        if(IsInCollision(params->vgoalconfig))
        {
            ROS_ERROR("Can't plan. Final configuration is in collision!");
            delete m_simpleSetup;
            return false;
        }

        RAVELOG_INFO("Setting state validity checker\n");
        m_simpleSetup->setStateValidityChecker(boost::bind(&or_ompl::OMPLPlanner::IsStateValid, this, _1));
        m_simpleSetup->setStartState(startPose);
        m_simpleSetup->setGoalState(endPose);

        m_collisionReport.reset(new CollisionReport());

        return true;
    }

    bool OMPLPlanner::InitializePlanner()
    {
        RAVELOG_INFO("Getting planner name.\n");
        std::string plannerName = m_parameters->m_plannerType;

        RAVELOG_INFO("Setting up space information");
        ompl::base::SpaceInformationPtr spaceInformation = m_simpleSetup->getSpaceInformation();
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
            ompl::geometric::RRT* rrt = new ompl::geometric::RRT(spaceInformation);
            m_planner.reset(rrt);
            rrt->setGoalBias(m_parameters->m_rrtGoalBias);
            rrt->setRange(m_parameters->m_rrtRange);
        }
        else if(plannerName == "RRTConnect")
        {
            ompl::geometric::RRTConnect* rrtConnect = new ompl::geometric::RRTConnect(spaceInformation);
            m_planner.reset(rrtConnect);
            rrtConnect->setRange(m_parameters->m_rrtRange);
        }
        else if(plannerName == "pRRT")
        {
            ompl::geometric::pRRT* prrt = new ompl::geometric::pRRT(spaceInformation);
            m_planner.reset(prrt);
            prrt->setRange(m_parameters->m_rrtRange);
            prrt->setGoalBias(m_parameters->m_rrtGoalBias);
        }
        else if(plannerName == "LazyRRT")
        {
            ompl::geometric::LazyRRT* lazyRRT = new ompl::geometric::LazyRRT(spaceInformation);
            m_planner.reset(new ompl::geometric::LazyRRT(spaceInformation));
            lazyRRT->setGoalBias(m_parameters->m_rrtGoalBias);
            lazyRRT->setRange(m_parameters->m_rrtRange);
        }
        else if(plannerName == "PRM")
        {
            m_planner.reset(new ompl::geometric::BasicPRM(spaceInformation));
        }
        else if(plannerName == "RRTstar")
        {
            RAVELOG_INFO("RRT Star Planner name\n");

            ompl::geometric::RRTstar* rrtStar = new ompl::geometric::RRTstar(spaceInformation);

            RAVELOG_INFO("Created RRTStar\n");

            m_planner.reset(rrtStar);

            RAVELOG_INFO("Setting parameters\n");
            rrtStar->setGoalBias(m_parameters->m_rrtGoalBias);
            rrtStar->setMaxBallRadius(m_parameters->m_rrtStarMaxBallRadius);
            rrtStar->setRange(m_parameters->m_rrtRange);
        }
        else if(plannerName == "BallTreeRRTstar")
        {
            ompl::geometric::BallTreeRRTstar* ballTree = new ompl::geometric::BallTreeRRTstar(spaceInformation);
            m_planner.reset(ballTree);
            ballTree->setGoalBias(m_parameters->m_rrtGoalBias);
            ballTree->setMaxBallRadius(m_parameters->m_rrtStarMaxBallRadius);
            ballTree->setRange(m_parameters->m_rrtRange);
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
            struct timespec tic;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);

            bool success = m_simpleSetup->solve(m_parameters->m_timeLimit);

            struct timespec toc;
            clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
            CD_OS_TIMESPEC_SUB(&toc, &tic);
            std::ofstream fileStream;
            std::stringstream fileName;
            double seconds = CD_OS_TIMESPEC_DOUBLE(&toc);

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

                fileName << m_parameters->m_dumpFileName << seconds;
                fileStream.open(fileName.str().c_str());
                ptraj->serialize(fileStream);
                fileStream.close();

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
        OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        m_robot->SetActiveDOFValues(values, false);
        return (GetEnv()->CheckCollision(KinBodyConstPtr(m_robot), m_collisionReport)) || (m_robot->CheckSelfCollision(m_collisionReport));
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
