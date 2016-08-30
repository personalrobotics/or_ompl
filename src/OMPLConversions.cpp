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
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <or_ompl/SemiToroidalStateSpace.h>
#include <or_ompl/OMPLConversions.h>

namespace or_ompl {

void OpenRAVEHandler::log(std::string const &text, ompl::msg::LogLevel level,
                          char const *filename, int line) {

    int const openrave_threshold_level = (
        OpenRAVE::RaveGetDebugLevel() & OpenRAVE::Level_OutputMask);
    int openrave_message_level = OpenRAVE::Level_Debug;

    switch (level) {
    case ompl::msg::LOG_DEBUG:
        openrave_message_level = OpenRAVE::Level_Debug;
        break;

    case ompl::msg::LOG_INFO:
        openrave_message_level = OpenRAVE::Level_Info;
        break;

    case ompl::msg::LOG_WARN:
        openrave_message_level = OpenRAVE::Level_Warn;
        break;

    case ompl::msg::LOG_ERROR:
        openrave_message_level = OpenRAVE::Level_Error;
        break;

    case ompl::msg::LOG_NONE:
    default:
        RAVELOG_ERROR("Unknown OMPL log level '%d'.\n", level);
    }

    if (openrave_message_level >= openrave_threshold_level)
    {
#ifdef OPENRAVE_LOG4CXX
        log4cxx::spi::LocationInfo const location_info(
            OpenRAVE::RaveGetSourceFilename(filename), "", line);
        OpenRAVE::RavePrintfA_DEBUGLEVEL(
            OpenRAVE::RaveGetLogger(), location_info, text);
#else
        OpenRAVE::RavePrintfA_DEBUGLEVEL("[%s:%d] %s\n",
            OpenRAVE::RaveGetSourceFilename(filename), line,
            text.c_str());
#endif
    }
}

std::vector<bool> GetContinuousJoints(const OpenRAVE::RobotBasePtr robot, const std::vector<int> idx) {
    const std::vector<OpenRAVE::RobotBase::JointPtr>& joints = robot->GetJoints();
    std::vector<bool> isContinuous;
    for (size_t j = 0; j < idx.size(); j++)
    {
        isContinuous.push_back(joints[idx[j]]->IsCircular(0));
    }
    return isContinuous;
}

ompl::base::StateSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                           OMPLPlannerParameters const &params) {
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return ompl::base::StateSpacePtr();
    } else if (robot->GetActiveDOF() == 0) {
        RAVELOG_ERROR("Zero DOFs are active.\n");
        return ompl::base::StateSpacePtr();
    }

    if (params.m_seed) {
        RAVELOG_DEBUG("Setting OMPL seed to %u.\n", params.m_seed);
        ompl::RNG::setSeed(params.m_seed);
        if (ompl::RNG::getSeed() != params.m_seed) {
            RAVELOG_ERROR("Could not set OMPL seed. Was this the first or_ompl"
                          "  plan attempted?\n");
            return ompl::base::StateSpacePtr();
        }
    } else {
        RAVELOG_DEBUG("Using default seed of %u for OMPL.\n",
                      ompl::RNG::getSeed());
    }

    std::vector<int> dof_indices = robot->GetActiveDOFIndices();
    const unsigned int num_dof = dof_indices.size();
    std::vector<bool> is_continuous = GetContinuousJoints(robot, dof_indices);
    BOOST_ASSERT(is_continuous.size() == num_dof);
    bool any_continuous = false;
    for (size_t i = 0; i < num_dof; ++i) {
        if (is_continuous[i]) {
            any_continuous = true;
        }
    }
    
    std::vector<OpenRAVE::dReal> lowerLimits, upperLimits;
    robot->GetActiveDOFLimits(lowerLimits, upperLimits);
    BOOST_ASSERT(lowerLimits.size() == num_dof);
    BOOST_ASSERT(upperLimits.size() == num_dof);

    ompl::base::RealVectorBounds bounds(num_dof);
    for (size_t i = 0; i < num_dof; ++i) {
        BOOST_ASSERT(lowerLimits[i] <= upperLimits[i]);
        if (is_continuous[i])
        {
            double diam = upperLimits[i] - lowerLimits[i];
            if (fabs(diam - 2.0*M_PI) > std::numeric_limits<double>::epsilon() * 2.0)
            {
                RAVELOG_WARN("Robot DOF [%lu] is circular, but has limits [%f,%f]!\n",
                    i, lowerLimits[i], upperLimits[i]);
                RAVELOG_WARN("Ignoring limits and using [-PI,PI] instead ...\n");
                lowerLimits[i] = -M_PI;
                upperLimits[i] = M_PI;
            }
        }
        bounds.setLow(i, lowerLimits[i]);
        bounds.setHigh(i, upperLimits[i]);
    }
    
    // construct state space
    ompl::base::StateSpacePtr state_space;
    if (any_continuous) {
        state_space.reset(new SemiToroidalStateSpace(num_dof));
        state_space->as<SemiToroidalStateSpace>()->setIsWrapping(is_continuous);
        RAVELOG_DEBUG("Setting joint limits.\n");
        state_space->as<SemiToroidalStateSpace>()->setBounds(bounds);
    } else {
        state_space.reset(new ompl::base::RealVectorStateSpace(num_dof));
        RAVELOG_DEBUG("Setting joint limits.\n");
        state_space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    }

    // Set the resolution at which OMPL should discretize edges for collision
    // checking. OpenRAVE supports per-joint resolutions, so we compute
    // a conservative minimum resolution required across all joints.
    // We then convert this to a fraction
    // of the workspace extents to call setLongestValidSegmentFraction.
    RAVELOG_DEBUG("Setting resolution.\n");
    std::vector<OpenRAVE::dReal> dof_resolutions;
    robot->GetActiveDOFResolutions(dof_resolutions);
    BOOST_ASSERT(dof_resolutions.size() == num_dof);

    double conservative_resolution = std::numeric_limits<double>::max();
    for (size_t i = 0; i < num_dof; ++i) {
        conservative_resolution = std::min(conservative_resolution, dof_resolutions[i]);
    }

    double conservative_fraction = conservative_resolution / state_space->getMaximumExtent();
    state_space->setLongestValidSegmentFraction(conservative_fraction);
    RAVELOG_DEBUG("Computed resolution of %f (%f fraction of extents).\n",
                  conservative_resolution, conservative_fraction);

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
        OpenRAVE::TrajectoryBasePtr or_traj) {
    using ompl::geometric::PathGeometric;

    size_t const num_dof = robot->GetActiveDOF();
    or_traj->Init(robot->GetActiveConfigurationSpecification("linear"));

    ompl::base::StateSpacePtr space = ompl_traj.getSpaceInformation()->getStateSpace();

    for (size_t i = 0; i < ompl_traj.getStateCount(); ++i){
        std::vector<double> values;
        space->copyToReals(values, ompl_traj.getState(i));
        or_traj->Insert(i, values, true);
    }
    return OpenRAVE::PS_HasSolution;
}

} // namespace or_ompl
