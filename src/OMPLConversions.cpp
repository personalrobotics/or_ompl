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

#include <boost/make_shared.hpp>
#include <ompl/config.h>
#include "OMPLConversions.h"

#define OMPL_VERSION_COMP (  OMPL_MAJOR_VERSION * 1000000 \
                           + OMPL_MINOR_VERSION * 1000 \
                           + OMPL_PATCH_VERSION)

namespace or_ompl {

void OpenRAVEHandler::log(std::string const &text, ompl::msg::LogLevel level,
                          char const *filename, int line)
{
    int const openrave_level = (OpenRAVE::RaveGetDebugLevel()
                              & OpenRAVE::Level_OutputMask);

    switch (level) {
    case ompl::msg::LOG_DEBUG:
        if (openrave_level >= (int) OpenRAVE::Level_Debug) {
            OpenRAVE::RavePrintfA_DEBUGLEVEL("[%s:%d] %s\n",
                OpenRAVE::RaveGetSourceFilename(filename), line,
                text.c_str());
        }
        break;

    case ompl::msg::LOG_INFO:
        if (openrave_level >= (int) OpenRAVE::Level_Info) {
            OpenRAVE::RavePrintfA_INFOLEVEL("[%s:%d] %s\n",
                OpenRAVE::RaveGetSourceFilename(filename), line,
                text.c_str());
        }
        break;

    case ompl::msg::LOG_WARN:
        if (openrave_level >= (int) OpenRAVE::Level_Warn) {
            OpenRAVE::RavePrintfA_WARNLEVEL("[%s:%d] %s\n",
                OpenRAVE::RaveGetSourceFilename(filename), line,
                text.c_str());
        }
        break;

    case ompl::msg::LOG_ERROR:
        if (openrave_level >= (int) OpenRAVE::Level_Error) {
            OpenRAVE::RavePrintfA_ERRORLEVEL("[%s:%d] %s\n",
                OpenRAVE::RaveGetSourceFilename(filename), line,
                text.c_str());
        }
        break;

    case ompl::msg::LOG_NONE:
    default:
        RAVELOG_ERROR("Unknown OMPL log level %d.\n", level);
    }
}

std::vector<bool> GetContinuousJoints(const OpenRAVE::RobotBasePtr robot, const std::vector<int> idx)
{
    const std::vector<OpenRAVE::RobotBase::JointPtr>& joints = robot->GetJoints();
    std::vector<bool> isContinuous;
    for (size_t j = 0; j < idx.size(); j++)
    {
        isContinuous.push_back(joints[idx[j]]->IsCircular(0));
    }
    return isContinuous;
}

RobotStateSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                    OMPLPlannerParameters const &params)
{
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return RobotStateSpacePtr();
    } else if (robot->GetActiveDOF() == 0) {
        RAVELOG_ERROR("Zero DOFs are active.\n");
        return RobotStateSpacePtr();
    }

    if (params.m_seed) {
        RAVELOG_DEBUG("Setting OMPL seed to %u.\n", params.m_seed);
        ompl::RNG::setSeed(params.m_seed);
        if (ompl::RNG::getSeed() != params.m_seed) {
            RAVELOG_ERROR("Could not set OMPL seed. Was this the first or_ompl"
                          "  plan attempted?\n");
            return RobotStateSpacePtr();
        }
    } else {
        RAVELOG_DEBUG("Using default seed of %u for OMPL.\n",
                      ompl::RNG::getSeed());
    }

    std::vector<int> dof_indices = robot->GetActiveDOFIndices();
    const unsigned int num_dof = dof_indices.size();
    std::vector<bool> is_continuous = GetContinuousJoints(robot, dof_indices);
    RobotStateSpacePtr state_space = boost::make_shared<RobotStateSpace>(dof_indices, is_continuous);

    RAVELOG_DEBUG("Setting joint limits.\n");
    std::vector<OpenRAVE::dReal> lowerLimits, upperLimits;
    robot->GetActiveDOFLimits(lowerLimits, upperLimits);
    BOOST_ASSERT(lowerLimits.size() == num_dof);
    BOOST_ASSERT(upperLimits.size() == num_dof);

    ompl::base::RealVectorBounds bounds(num_dof);
    for (size_t i = 0; i < num_dof; ++i) {
        BOOST_ASSERT(lowerLimits[i] <= upperLimits[i]);
        bounds.setLow(i, lowerLimits[i]);
        bounds.setHigh(i, upperLimits[i]);
    }
    state_space->setBounds(bounds);

    // Set the resolution at which OMPL should discretize edges for collision
    // checking. OpenRAVE supports per-joint resolutions, so we compute one
    // conservative value for all joints. We then convert this to a fraction
    // of the workspace extents to call setLongestValidSegmentFraction.
    RAVELOG_DEBUG("Setting resolution.\n");
    std::vector<OpenRAVE::dReal> dof_resolutions;
    robot->GetActiveDOFResolutions(dof_resolutions);
    BOOST_ASSERT(dof_resolutions.size() == num_dof);

    double conservative_fraction = std::numeric_limits<double>::max();
    double longest_extent = 0;
    for (size_t i = 0; i < num_dof; ++i) {
        if (upperLimits[i] > lowerLimits[i]) {
            double const joint_extents = upperLimits[i] - lowerLimits[i];
            double const joint_fraction = dof_resolutions[i] / joint_extents;
            conservative_fraction = std::min(conservative_fraction, joint_fraction);
            longest_extent = std::max(longest_extent, joint_extents);
        }
    }

    if (std::isinf(conservative_fraction)) {
        RAVELOG_ERROR("All joints have equal lower and upper limits.\n");
        return RobotStateSpacePtr();
    }
    state_space->setLongestValidSegmentFraction(conservative_fraction);
    RAVELOG_DEBUG("Computed resolution of %f (%f fraction of extents).\n",
                  conservative_fraction * longest_extent, conservative_fraction);

    // Per-DOF weights are not supported by OMPL.
    // TODO: Emulate this by scaling joint values.
    RAVELOG_DEBUG("Setting joint weights.\n");
    std::vector<OpenRAVE::dReal> dof_weights;
    robot->GetActiveDOFWeights(dof_weights);
    BOOST_ASSERT(dof_weights.size() == num_dof);

    bool has_weights = false;
    for (size_t i = 0; !has_weights && i < num_dof; ++i) {
        has_weights = dof_weights[i] != 1.0;
    }

    if (has_weights) {
        RAVELOG_WARN("Robot specifies DOF weights. Only unit weights are"
                     " supported by OMPL; planning will commence as if"
                     " there are no weights.\n");
    }

    state_space->setup();

    return state_space;
}

OpenRAVE::PlannerStatus ToORTrajectory(
        OpenRAVE::RobotBasePtr const &robot,
        ompl::geometric::PathGeometric& ompl_traj,
        OpenRAVE::TrajectoryBasePtr or_traj)
{
    using ompl::geometric::PathGeometric;

    size_t const num_dof = robot->GetActiveDOF();
    or_traj->Init(robot->GetActiveConfigurationSpecification("linear"));

    for (size_t i = 0; i < ompl_traj.getStateCount(); ++i){
        RobotState  *state = ompl_traj.getState(i)->as<RobotState>();
        if (!state) {
            RAVELOG_ERROR("Unable to convert output trajectory."
                          "State is not a RealVectorStateSpace::StateType.");
            return OpenRAVE::PS_Failed;
        }
        or_traj->Insert(i, state->getValues(), true);
    }
    return OpenRAVE::PS_HasSolution;
}

}
