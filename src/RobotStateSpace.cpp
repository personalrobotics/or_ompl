#include "RobotStateSpace.h"
#include <openrave/openrave.h>

using namespace or_ompl;
namespace ob = ompl::base;

RobotState::RobotState(RobotStateSpace* stateSpace) :
        _stateSpace(stateSpace) {

}

RobotState::~RobotState() {
}

void RobotState::set(const std::vector<double> &dof_values) {

    for(unsigned int idx=0; idx < _stateSpace->getDimension(); idx++){
        value(idx) = dof_values[idx];
    }
    enforceBounds();
}

double& RobotState::value(const size_t& idx) {
    return *(_stateSpace->getValueAddressAtIndex(this, idx));
}

const double& RobotState::value(const size_t& idx) const {
    return *(_stateSpace->getValueAddressAtLocation(this, _stateSpace->getValueLocations()[idx]));
}

std::vector<double> RobotState::getValues() const {
    std::vector<double> retval(_stateSpace->getDimension());

    for(unsigned int idx=0; idx < retval.size(); idx++){
        retval[idx] = value(idx);
    }

    return retval;
}

void RobotState::enforceBounds() {
    _stateSpace->enforceBounds(this);
}

void RobotStateSpace::registerProjections() {
    registerProjection("default", _projectionEvaluator);
    registerDefaultProjection(_projectionEvaluator);
    StateSpace::registerProjections();
}

RobotStateSpace::RobotStateSpace(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous) :
        ompl::base::CompoundStateSpace(), _indices(dof_indices), _isContinuous(is_continuous) {
    BOOST_ASSERT(dof_indices.size() == is_continuous.size());

    // Keep track of how many real-valued DOFs we've encountered in a row.
    size_t realDOFCount = 0;
    // For each dof...
    for (size_t dofIdx = 0; dofIdx < dof_indices.size(); dofIdx++) {
        // If the DOF is continuous...
        if (is_continuous[dofIdx]) {
            // If we've already encountered some real-valued DOFs, add a new subspace with them.
            if (realDOFCount > 0) {
                addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
                // Reset the counter.
                realDOFCount = 0;
            }
            // Add a continuous subspace.
            addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 1.0);
        }
        else {
            // Otherwise, the joint is a real DOF, and should be added to a subspace.
            realDOFCount++;
        }
    }

    // If at the end of the process there are some trailing real-valued DOFs, add a subspace for them.
    if (realDOFCount > 0) {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
    }
    BOOST_ASSERT(getSubspaceCount() > 0);

    _projectionEvaluator.reset(new RobotProjectionEvaluator(this));
}

ompl::base::State* RobotStateSpace::allocState() const {
    RobotState* state = new RobotState((RobotStateSpace*)this);
    allocStateComponents(state);
    return state;

}

void RobotStateSpace::setBounds(const ompl::base::RealVectorBounds& bounds) {
    BOOST_ASSERT(bounds.high.size() == bounds.low.size());
    // The index of the current subspace
    size_t subspaceIdx = 0;
    // For each joint...
    for (size_t dofIdx = 0; dofIdx < bounds.high.size(); dofIdx++) {
        // If the DOF is continuous, it has no limits. So skip to the next subspace.
        if (_isContinuous[dofIdx]) {
            subspaceIdx++;
            continue;
        }
        else {
            // Otherwise get the current real-vector subspace (not the current joint)
            ompl::base::RealVectorStateSpace* space = components_[subspaceIdx]->as<ompl::base::RealVectorStateSpace>();
            ompl::base::RealVectorBounds subBounds(space->getDimension());

            // Set the bounds of each of the sub-components
            for (size_t k = 0; k < space->getDimension(); k++) {
                subBounds.high[k] = bounds.high[dofIdx + k];
                subBounds.low[k] = bounds.low[dofIdx + k];
            }

            space->setBounds(subBounds);
            // The DOF index goes up by the number of DOFs in the subspace
            dofIdx += space->getDimension();

            subspaceIdx++;
        }
    }
    // Sanity check to make sure that we covered every subspace.
    BOOST_ASSERT(subspaceIdx == components_.size());
}

RobotProjectionEvaluator::RobotProjectionEvaluator(ompl::base::StateSpace* stateSpace) :
    ompl::base::ProjectionEvaluator(stateSpace) {
    RobotStateSpace* robotStateSpace = dynamic_cast<RobotStateSpace*>(stateSpace);

    if (!robotStateSpace) {
        RAVELOG_ERROR("Can only use RobotStateSpace with RobotProjectionEvaluator!");
        return;
    }

    _robotStateSpace = robotStateSpace;
}

RobotProjectionEvaluator::RobotProjectionEvaluator(ompl::base::StateSpacePtr stateSpace) :
    ProjectionEvaluator(stateSpace) {
    RobotStateSpace* robotStateSpace = dynamic_cast<RobotStateSpace*>(stateSpace.get());

    if (!robotStateSpace) {
        RAVELOG_ERROR("Can only use RobotStateSpace with RobotProjectionEvaluator!");
        return;
    }

    _robotStateSpace = robotStateSpace;
}

RobotProjectionEvaluator::~RobotProjectionEvaluator() {

}

void RobotProjectionEvaluator::setup() {
    _projectionMatrix.mat = _projectionMatrix.ComputeRandom(_robotStateSpace->getDimension(), getDimension());
    defaultCellSizes();
    ProjectionEvaluator::setup();
}

void RobotProjectionEvaluator::defaultCellSizes() {
    cellSizes_.resize(getDimension());

    for (size_t i = 0; i < getDimension(); i++) {
        cellSizes_[i] = 0.5f;
    }
}

/** \brief Return the dimension of the projection defined by this evaluator */
unsigned int RobotProjectionEvaluator::getDimension() const {
    int dim = _robotStateSpace->getDimension();
    if (dim <= 2) {
        return dim;
    }
    else {
        return (int)(log(_robotStateSpace->getDimension())) + 1;
    }
}

/** \brief Compute the projection as an array of double values */
void RobotProjectionEvaluator::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const {
    const RobotState* robotState = dynamic_cast<const RobotState*>(state);

    if (!robotState) {
        RAVELOG_ERROR("Can only project robot states!");
        return;
    }

    std::vector<double> values = robotState->getValues();
    projection.resize(getDimension());
    _projectionMatrix.project(values.data(), projection);
}
