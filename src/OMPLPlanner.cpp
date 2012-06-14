/*
 * OMPLPlanner.cpp
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#include <time.h>

#include <ompl/config.h>
#define OMPL_VERSION_COMP (  OMPL_MAJOR_VERSION * 1000000 \
                           + OMPL_MINOR_VERSION * 1000 \
                           + OMPL_PATCH_VERSION)

#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>

#if OMPL_VERSION_COMP >= 000010002
#  include <ompl/geometric/planners/prm/PRM.h>
#else
#  include <ompl/geometric/planners/prm/BasicPRM.h>
#endif

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/contrib/rrt_star/BallTreeRRTstar.h>

#include <ompl_rrtstar_modified/RRTstarModified.h>

#include "OMPLPlanner.h"

using namespace OpenRAVE;



#define CD_OS_TIMESPEC_SET_ZERO(t) do { (t)->tv_sec = 0; (t)->tv_nsec = 0; } while (0)
#define CD_OS_TIMESPEC_ADD(dst, src) do { (dst)->tv_sec += (src)->tv_sec; (dst)->tv_nsec += (src)->tv_nsec; \
   if ((dst)->tv_nsec > 999999999) { (dst)->tv_sec += 1; (dst)->tv_nsec -= 1000000000; } } while (0)
#define CD_OS_TIMESPEC_SUB(dst, src) do { (dst)->tv_sec -= (src)->tv_sec; (dst)->tv_nsec -= (src)->tv_nsec; \
   if ((dst)->tv_nsec < 0) { (dst)->tv_sec -= 1; (dst)->tv_nsec += 1000000000; } } while (0)
#define CD_OS_TIMESPEC_DOUBLE(src) ((src)->tv_sec + ((double)((src)->tv_nsec))/1000000000.0)

#define TIME_COLLISION_CHECKS

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
        m_totalCollisionTime = 0.0;
        m_numCollisionChecks = 0;

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
        
        if (m_parameters->m_seed)
        {
           RAVELOG_INFO("Setting random seed to %u ...\n", m_parameters->m_seed);
           ompl::RNG::setSeed(m_parameters->m_seed);
           if (ompl::RNG::getSeed() != m_parameters->m_seed)
           {
              RAVELOG_ERROR("Could not set the seed! Was this the first or_ompl plan attempted?\n");
              return false;
           }
        }
        else
           RAVELOG_INFO("Using default (time-based) seed (%u) for OMPL ...\n", ompl::RNG::getSeed());

        m_robot = robot;

        RAVELOG_INFO("Setting state space\n");
        m_stateSpace.reset(new ompl::base::RealVectorStateSpace(robot->GetActiveDOF()));
        
        if (robot->GetActiveDOF() != 7)
        {
           RAVELOG_ERROR("Right now, or_omple has hacks for the WAM arm only!\n");
           return false;
        }
        scale_radii[0] = scale_radii[1] = 1.2;
        scale_radii[2] = scale_radii[3] = 0.7;
        scale_radii[4] = scale_radii[5] = 0.4;
        scale_radii[6] = 0.2;

        RAVELOG_INFO("Setting joint limits\n");
        ompl::base::RealVectorBounds bounds(m_robot->GetActiveDOF());
        std::vector<double> lowerLimits;
        std::vector<double> upperLimits;
        m_robot->GetActiveDOFLimits(lowerLimits, upperLimits);

        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            bounds.setLow(i, scale_radii[i] * lowerLimits[i]);
            bounds.setHigh(i, scale_radii[i] * upperLimits[i]);
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
            startPose->values[i] = scale_radii[i] * m_parameters->vinitialconfig[i];
        }

        RAVELOG_INFO("Checking collisions.\n");
        if(IsInOrCollision(params->vinitialconfig))
        {
            RAVELOG_ERROR("Can't plan. Initial configuration in collision!\n");
            delete m_simpleSetup;
            return false;
        }

        RAVELOG_INFO("Setting the end pose.\n");
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> endPose(m_stateSpace);
        for(int i = 0; i < m_robot->GetActiveDOF(); i++)
        {
            endPose->values[i] = scale_radii[i] * params->vgoalconfig[i];
        }

        RAVELOG_INFO("Checking collisions\n");
        if(IsInOrCollision(params->vgoalconfig))
        {
            RAVELOG_ERROR("Can't plan. Final configuration is in collision!");
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

        RAVELOG_INFO("Setting up space information\n");
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
            printf("setting range to be %f!\n", m_parameters->m_rrtRange);
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
#if OMPL_VERSION_COMP >= 000010002
            m_planner.reset(new ompl::geometric::PRM(spaceInformation));
#else
            m_planner.reset(new ompl::geometric::BasicPRM(spaceInformation));
#endif
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
        else if(plannerName == "RRTstarModified")
        {
            RAVELOG_INFO("RRT Star Modified Planner name\n");

            ompl::geometric::RRTstarModified* rrtStarModified = new ompl::geometric::RRTstarModified(spaceInformation);

            RAVELOG_INFO("Created RRTstarModified\n");

            m_planner.reset(rrtStarModified);

            RAVELOG_INFO("Setting parameters\n");
            rrtStarModified->setGoalBias(m_parameters->m_rrtGoalBias);
            rrtStarModified->setMaxBallRadius(m_parameters->m_rrtStarMaxBallRadius);
            rrtStarModified->setRange(m_parameters->m_rrtRange);
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
            RAVELOG_ERROR("Urecognized planner: %s", plannerName.c_str());
            return false;
        }

        return true;
    }

    OpenRAVE::PlannerStatus OMPLPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
    {
        OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        if(!m_simpleSetup)
        {
            RAVELOG_ERROR("Couldn't plan a path. Simple setup was null!");
            return OpenRAVE::PS_Failed;
        }
        else
        {
            struct timespec tic;
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);

            bool success = m_simpleSetup->solve(m_parameters->m_timeLimit);

            struct timespec toc;
            clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc);
            CD_OS_TIMESPEC_SUB(&toc, &tic);
            printf("cputime seconds: %f\n", CD_OS_TIMESPEC_DOUBLE(&toc));
            

            ConfigurationSpecification spec =  m_robot->GetActiveConfigurationSpecification();
            ptraj->Init(spec);

            if(success)
            {
               /* write success dat file */
               if (m_parameters->m_dat_filename.c_str()[0])
               {
                  FILE * fp;
                  fp = fopen(m_parameters->m_dat_filename.c_str(), "w");
                  fprintf(fp, "0 %f 1 %d %f\n", CD_OS_TIMESPEC_DOUBLE(&toc), m_numCollisionChecks, m_totalCollisionTime);
                  fclose(fp);
               }
               
               /* write trajectory file */
               if (m_parameters->m_trajs_fileformat.c_str()[0])
               {
                  char trajs_filename[1024];
                  sprintf(trajs_filename, m_parameters->m_trajs_fileformat.c_str(), 0); /* iter = 0 */
                  OpenRAVE::TrajectoryBasePtr t = OpenRAVE::RaveCreateTrajectory(GetEnv());
                  t->Init(m_robot->GetActiveConfigurationSpecification());
                  
#if OMPL_VERSION_COMP >= 000010002
                  std::vector<ompl::base::State*>& states = m_simpleSetup->getSolutionPath().getStates();
#else
                  std::vector<ompl::base::State*>& states = m_simpleSetup->getSolutionPath().states;
#endif
                  for(size_t i = 0; i < states.size(); i++)
                  {
                     const ompl::base::RealVectorStateSpace::StateType* state = states[i]->as<ompl::base::RealVectorStateSpace::StateType>();
                     if(!state) { RAVELOG_ERROR("Invalid state type!"); return OpenRAVE::PS_Failed; }
                     OpenRAVE::TrajectoryBase::Point point;
                     for(int j = 0; j < m_robot->GetActiveDOF(); j++)
                        point.q.push_back((*state)[j] / scale_radii[j]);
                     t->Insert(i, point.q, true);
                  }
                  std::ofstream f(trajs_filename);
                  f << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1); /// have to do this or otherwise precision gets lost
                  t->serialize(f);
                  f.close();
               }
               
#if OMPL_VERSION_COMP >= 000010002
               std::vector<ompl::base::State*>& states = m_simpleSetup->getSolutionPath().getStates();
#else
               std::vector<ompl::base::State*>& states = m_simpleSetup->getSolutionPath().states;
#endif
               /* write into the actual trajectory to be passed back to our caller */
               ptraj->Init(m_robot->GetActiveConfigurationSpecification());
               for(size_t i = 0; i < states.size(); i++)
               {
                  const ompl::base::RealVectorStateSpace::StateType* state = states[i]->as<ompl::base::RealVectorStateSpace::StateType>();
                  if(!state) { RAVELOG_ERROR("Invalid state type!"); return OpenRAVE::PS_Failed; }
                  OpenRAVE::TrajectoryBase::Point point;
                  for(int j = 0; j < m_robot->GetActiveDOF(); j++)
                     point.q.push_back((*state)[j] / scale_radii[j]);
                  ptraj->Insert(i, point.q, true);
               }
               
               return OpenRAVE::PS_HasSolution;
            }
            else
            {
               /* write failure dat file */
               if (m_parameters->m_dat_filename.c_str()[0])
               {
                  FILE * fp;
                  fp = fopen(m_parameters->m_dat_filename.c_str(), "w");
                  fprintf(fp, "0 %f 0 %d %f\n", CD_OS_TIMESPEC_DOUBLE(&toc), m_numCollisionChecks, m_totalCollisionTime);
                  fclose(fp);
               }
               return OpenRAVE::PS_Failed;
            }
        }

        return OpenRAVE::PS_Failed;
    }

    bool OMPLPlanner::IsInOrCollision(std::vector<double> values)
    {
#ifdef TIME_COLLISION_CHECKS
        struct timespec tic;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);
#endif
        OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
        m_robot->SetActiveDOFValues(values, false);
        bool collided = (GetEnv()->CheckCollision(KinBodyConstPtr(m_robot), m_collisionReport)) || (m_robot->CheckSelfCollision(m_collisionReport));
        m_numCollisionChecks++;

#ifdef TIME_COLLISION_CHECKS
        struct timespec toc;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc);
        CD_OS_TIMESPEC_SUB(&toc, &tic);
        m_totalCollisionTime += CD_OS_TIMESPEC_DOUBLE(&toc);
#endif
        return collided;
    }

    bool OMPLPlanner::IsStateValid(const ompl::base::State* state)
    {
        const ompl::base::RealVectorStateSpace::StateType* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();

        if(!realVectorState)
        {
            RAVELOG_ERROR("State type requested was invalid!");
            return false;
        }
        else
        {
            std::vector<double> values;
            for(int i = 0; i < m_robot->GetActiveDOF(); i++)
            {
                values.push_back(realVectorState->values[i] / scale_radii[i]);
            }

            return !IsInOrCollision(values);
        }

    }


} /* namespace or_ompl */
