#include <time.h>

#include <boost/make_shared.hpp>
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

#include "OMPLPlanner.h"

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
{
}

OMPLPlanner::~OMPLPlanner()
{
}

bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input)
{
    m_totalCollisionTime = 0.0;
    m_numCollisionChecks = 0;

    OMPLPlannerParameters* params = new OMPLPlannerParameters();
    input >> (*params);

    return InitPlan(robot, PlannerParametersConstPtr(params));
}

bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot,
                           PlannerParametersConstPtr params)
{
    RAVELOG_DEBUG("Initializing plan\n");
    if (m_simpleSetup) {
        m_simpleSetup.reset();
    }

    m_parameters = boost::make_shared<OMPLPlannerParameters>();
    m_parameters->copy(params);
    
    if (m_parameters->m_seed)
    {
       RAVELOG_DEBUG("Setting random seed to %u ...\n", m_parameters->m_seed);
       ompl::RNG::setSeed(m_parameters->m_seed);
       if (ompl::RNG::getSeed() != m_parameters->m_seed) {
          RAVELOG_ERROR("Could not set the seed! Was this the first or_ompl plan attempted?\n");
          return false;
       }
    }
    else {
       RAVELOG_DEBUG("Using default (time-based) seed (%u) for OMPL ...\n", ompl::RNG::getSeed());
    }

    m_robot = robot;

    RAVELOG_DEBUG("Setting state space\n");
    m_stateSpace = boost::make_shared<ompl::base::RealVectorStateSpace>(robot->GetActiveDOF());

    RAVELOG_DEBUG("Setting joint limits\n");
    ompl::base::RealVectorBounds bounds(m_robot->GetActiveDOF());
    std::vector<double> lowerLimits, upperLimits;
    m_robot->GetActiveDOFLimits(lowerLimits, upperLimits);

    for (int i = 0; i < m_robot->GetActiveDOF(); i++) {
        bounds.setLow(i, lowerLimits[i]);
        bounds.setHigh(i, upperLimits[i]);
    }

    m_stateSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    RAVELOG_DEBUG("Setting up simplesetup class.\n");
    m_simpleSetup = boost::make_shared<ompl::geometric::SimpleSetup>(GetStateSpace());

    RAVELOG_DEBUG("Initializing the planner.\n");
    if (!InitializePlanner()) {
        return false;
    }

    RAVELOG_DEBUG("Setting the planner.\n");
    m_simpleSetup->setPlanner(m_planner);

    RAVELOG_DEBUG("Creating the start pose.\n");
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> startPose(m_stateSpace);
    for (int i = 0; i < m_robot->GetActiveDOF(); i++) {
        startPose->values[i] = m_parameters->vinitialconfig[i];
    }

    RAVELOG_DEBUG("Checking collisions.\n");
    if (IsInOrCollision(params->vinitialconfig)) {
        RAVELOG_ERROR("Can't plan. Initial configuration in collision!\n");
        return false;
    }

    RAVELOG_DEBUG("Setting the end pose.\n");
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> endPose(m_stateSpace);
    for (int i = 0; i < m_robot->GetActiveDOF(); i++) {
        endPose->values[i] = params->vgoalconfig[i];
    }

    RAVELOG_DEBUG("Checking collisions\n");
    if (IsInOrCollision(params->vgoalconfig)) {
        RAVELOG_ERROR("Can't plan. Final configuration is in collision!");
        return false;
    }

    RAVELOG_DEBUG("Setting state validity checker\n");
    m_simpleSetup->setStateValidityChecker(boost::bind(&or_ompl::OMPLPlanner::IsStateValid, this, _1));
    m_simpleSetup->setStartState(startPose);
    m_simpleSetup->setGoalState(endPose);

    m_collisionReport = boost::make_shared<OpenRAVE::CollisionReport>();

    return true;
}

bool OMPLPlanner::InitializePlanner()
{
    RAVELOG_DEBUG("Getting planner name.\n");
    std::string plannerName = m_parameters->m_plannerType;

    RAVELOG_DEBUG("Setting up space information\n");
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
        RAVELOG_DEBUG("RRT Star Planner name\n");

        ompl::geometric::RRTstar* rrtStar = new ompl::geometric::RRTstar(spaceInformation);

        RAVELOG_DEBUG("Created RRTStar\n");

        m_planner.reset(rrtStar);

        RAVELOG_DEBUG("Setting parameters\n");
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
        RAVELOG_ERROR("Urecognized planner: %s", plannerName.c_str());
        return false;
    }
    return true;
}

OpenRAVE::PlannerStatus OMPLPlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj)
{
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    if (!m_simpleSetup) {
        RAVELOG_ERROR("Unable to plan. Did you call InitPlan?");
        return OpenRAVE::PS_Failed;
    }
    /* also, set something max path length so it continues after finding a solution! */
    //m_simpleSetup->getGoal()->setMaximumPathLength(0.0);
   
    // TODO: What is all of this? Should this really be in the planner?
    struct timespec tic;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);

    bool success;
    success = m_simpleSetup->solve(m_parameters->m_timeLimit);

    struct timespec toc;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc);
    CD_OS_TIMESPEC_SUB(&toc, &tic);
    printf("cputime seconds: %f\n", CD_OS_TIMESPEC_DOUBLE(&toc));

    if (success) { 
        return ToORTrajectory(m_simpleSetup->getSolutionPath(), ptraj);
    } else {
        return OpenRAVE::PS_Failed;
    }
}

bool OMPLPlanner::IsInOrCollision(std::vector<double> values)
{
#ifdef TIME_COLLISION_CHECKS
    struct timespec tic;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);
#endif
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(GetEnv()->GetMutex());
    m_robot->SetActiveDOFValues(values, false);
    bool collided = (GetEnv()->CheckCollision(m_robot, m_collisionReport))
                 || (m_robot->CheckSelfCollision(m_collisionReport));
    m_numCollisionChecks++;

#ifdef TIME_COLLISION_CHECKS
    struct timespec toc;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc);
    CD_OS_TIMESPEC_SUB(&toc, &tic);
    m_totalCollisionTime += CD_OS_TIMESPEC_DOUBLE(&toc);
#endif
    return collided;
}

bool OMPLPlanner::IsStateValid(ompl::base::State const *state)
{
    ompl::base::RealVectorStateSpace::StateType const* realVectorState = state->as<ompl::base::RealVectorStateSpace::StateType>();

    if (!realVectorState) {
        RAVELOG_ERROR("State type requested was invalid!");
        return false;
    } else {
        std::vector<double> values;
        for (int i = 0; i < m_robot->GetActiveDOF(); i++) {
            values.push_back(realVectorState->values[i]);
        }
        return !IsInOrCollision(values);
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

} /* namespace or_ompl */
