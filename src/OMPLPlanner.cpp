/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

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

    std::cout << ">>>before" << std::endl;
    bool foo = InitPlan(robot, PlannerParametersConstPtr(params));
    std::cout << "<<<after" << std::endl;
    return foo;
}

bool OMPLPlanner::InitPlan(OpenRAVE::RobotBasePtr robot,
                           PlannerParametersConstPtr params)
{
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return false;
    } else if (!params) {
        RAVELOG_ERROR("Params must not be NULL.\n");
        return false;
    }

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

    for (int i = 0; i < m_robot->GetActiveDOF(); ++i) {
        bounds.setLow(i, lowerLimits[i]);
        bounds.setHigh(i, upperLimits[i]);
    }
    m_stateSpace->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Set the resolution at which OMPL should discretize edges for collision
    // checking. OpenRAVE supports per-joint resolutions, so we compute one
    // conservative value for all joints. We then convert this to a fraction
    // of the workspace extents to call setLongestValidSegmentFraction.
    // TODO: We could emulate per-joint resolutions by scaling the joint
    // values. Is that a good idea?
    RAVELOG_DEBUG("Setting collision checking resolution.\n");
    std::vector<OpenRAVE::dReal> dof_resolutions;
    m_robot->GetActiveDOFResolutions(dof_resolutions);

    double conservative_resolution = std::numeric_limits<double>::max();
    for (int i = 0; i < m_robot->GetActiveDOF(); ++i) {
        conservative_resolution = std::min(dof_resolutions[i], conservative_resolution);
    }

    double const max_extent = m_stateSpace->getMaximumExtent();
    double const segment_fraction = conservative_resolution / max_extent;
    m_stateSpace->setLongestValidSegmentFraction(segment_fraction);

    // Per-DOF weights are not supported by OMPL. We could emulate this by
    // scaling the joint values, but I'm not sure if that's a good idea.
    RAVELOG_DEBUG("Checking joint weights.\n");
    std::vector<OpenRAVE::dReal> dof_weights;
    m_robot->GetActiveDOFWeights(dof_weights);
    bool has_weights = false;

    for (int i = 0; !has_weights && i < m_robot->GetActiveDOF(); ++i) {
        has_weights = dof_weights[i] != 1.;
    }

    if (has_weights) {
        RAVELOG_WARN("Robot specifies DOF weights. Only unit weights are"
                     " supported by OMPL; planning will commence as if"
                     " there are no weights.\n");
    }
    
    RAVELOG_DEBUG("Setting up simplesetup class.\n");
    m_simpleSetup = boost::make_shared<ompl::geometric::SimpleSetup>(GetStateSpace());

    RAVELOG_DEBUG("Initializing the planner.\n");
    if (!InitializePlanner()) {
        return false;
    }

    RAVELOG_DEBUG("Setting the planner.\n");
    m_simpleSetup->setPlanner(m_planner);

    RAVELOG_DEBUG("Creating the start pose.\n");
    if (m_parameters->vinitialconfig.size() != m_robot->GetActiveDOF()) {
        RAVELOG_ERROR("Start point has incorrect DOF; expected %d, got %d.\n",
                      m_robot->GetActiveDOF(),
                      m_parameters->vinitialconfig.size());
        return false;
    }

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
    if (m_parameters->vgoalconfig.size() != m_robot->GetActiveDOF()) {
        RAVELOG_ERROR("End point has incorrect DOF; expected %d, got %d.\n",
                      m_robot->GetActiveDOF(),
                      m_parameters->vgoalconfig.size());
        return false;
    }
    
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
    // Create the planner.
    std::string const plannerName = m_parameters->m_plannerType;
    ompl::base::SpaceInformationPtr const spaceInformation
            = m_simpleSetup->getSpaceInformation();

    m_planner.reset(registry::create(plannerName, spaceInformation));
    if (!m_planner) {
        RAVELOG_ERROR("Failed creating planner '%s'.\n", plannerName.c_str());
        return false;
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
        return false;
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
            return false;
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

    ompl::base::ParamSet &param_set = m_planner->params();
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
