#include <boost/make_shared.hpp>
#include "OMPLConversions.h"

namespace or_ompl {

RealVectorSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                    OMPLPlannerParameters const &params)
{
    if (!robot) {
        RAVELOG_ERROR("Robot must not be NULL.\n");
        return RealVectorSpacePtr();
    } else if (robot->GetActiveDOF() == 0) {
        RAVELOG_ERROR("Zero DOFs are active.\n");
        return RealVectorSpacePtr();
    }

    if (params.m_seed) {
        RAVELOG_DEBUG("Setting OMPL seed to %u.\n", params.m_seed);
        ompl::RNG::setSeed(params.m_seed);
        if (ompl::RNG::getSeed() != params.m_seed) {
            RAVELOG_ERROR("Could not set OMPL seed. Was this the first or_ompl"
                          "  plan attempted?\n");
            return RealVectorSpacePtr();
        }
    } else {
        RAVELOG_DEBUG("Using default seed of %u for OMPL.\n",
                      ompl::RNG::getSeed());
    }

    size_t const num_dof = robot->GetActiveDOF();
    boost::shared_ptr<ompl::base::RealVectorStateSpace> state_space
            = boost::make_shared<ompl::base::RealVectorStateSpace>(num_dof);

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
    for (size_t i = 0; i < num_dof; ++i) {
        if (upperLimits[i] > lowerLimits[i]) {
            double const joint_extents = upperLimits[i] - upperLimits[i];
            double const joint_fraction = dof_resolutions[i] / joint_extents;
            conservative_fraction = std::min(conservative_fraction, joint_fraction);
        }
    }

    if (std::isinf(conservative_fraction)) {
        RAVELOG_ERROR("All joints have equal lower and upper limits.\n");
        return RealVectorSpacePtr();
    }
    state_space->setLongestValidSegmentFraction(conservative_fraction);

    // Per-DOF weights are not supported by OMPL.
    // TODO: Emulate this by scaling the joint values.
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
    return state_space;
}

#if 0
bool CreateSimpleSetup(OpenRAVE::RobotBasePtr robot,
                       OMPLPlannerParameters const &params)
{
    ompl::geometric::SimpleSetupPtr simple_setup
        = boost::make_shared<ompl::geometric::SimpleSetup>(state_space);

    RAVELOG_DEBUG("Setting start configuration.\n");
    if (params.vinitialconfig.size() != num_dof) {
        RAVELOG_ERROR("Start configuration has incorrect DOF;"
                      " expected %d, got %d.\n",
                      num_dof, params.vinitialconfig.size());
        return false;
    }

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> q_start(state_space);
    for (size_t i = 0; i < num_dof; i++) {
        q_start->values[i] = params.vinitialconfig[i];
    }

#if 0
    RAVELOG_DEBUG("Checking start configuration for collision.\n");
    if (IsInOrCollision(ompl_params->vinitialconfig)) {
        RAVELOG_ERROR("Can't plan. Initial configuration in collision!\n");
        return false;
    }
#endif

    RAVELOG_DEBUG("Setting end configuration.\n");
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> q_end(state_space);
    if (params.vgoalconfig.size() != num_dof) {
        RAVELOG_ERROR("End configuration has incorrect DOF;"
                      "  expected %d, got %d.\n",
                      num_dof, params.vgoalconfig.size());
        return false;
    }
    
    for (size_t i = 0; i < num_dof; i++) {
        q_end->values[i] = params.vgoalconfig[i];
    }

#if 0
    RAVELOG_DEBUG("Checking collisions\n");
    if (IsInOrCollision(params->vgoalconfig)) {
        RAVELOG_ERROR("Can't plan. Final configuration is in collision!");
        return false;
    }

    RAVELOG_DEBUG("Setting state validity checker.\n");
    m_simpleSetup->setStateValidityChecker(
            boost::bind(&or_ompl::OMPLPlanner::IsStateValid, this, _1));
    m_simpleSetup->setStartState(startPose);
    m_simpleSetup->setGoalState(endPose);
#endif
}
#endif

}
