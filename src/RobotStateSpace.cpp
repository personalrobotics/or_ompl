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



void RobotStateSpace::registerProjections() {
    registerProjection("default", _projectionEvaluator);
    registerDefaultProjection(_projectionEvaluator);
    StateSpace::registerProjections();
}

RobotStateSpace::RobotStateSpace(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous) :
        ompl::base::CompoundStateSpace(), _indices(dof_indices), _isContinuous(is_continuous) {
    BOOST_ASSERT(dof_indices.size() == is_continuous.size());
    // TODO: THIS AINT RIGHT
    size_t realDOFCount = 0;
    for (size_t i = 0; i < dof_indices.size(); i++) {
        if (is_continuous[i]) {
            if (realDOFCount > 0)
            {
                addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
                realDOFCount = 0;
            }
            addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 1.0);
        }
        else {
            realDOFCount++;
        }
    }
    if (realDOFCount > 0)
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
    }
    _projectionEvaluator.reset(new RobotProjectionEvaluator(this));
}

ompl::base::State* RobotStateSpace::allocState() const {

    RobotState* state = new RobotState((RobotStateSpace*)this);
    allocStateComponents(state);
    return state;

}

void RobotStateSpace::setBounds(const ompl::base::RealVectorBounds& bounds) {
    BOOST_ASSERT(bounds.high.size() == bounds.low.size());

    for (size_t i = 0; i < bounds.high.size(); i++) {
        if (_isContinuous[i]) {
            continue;
        }
        else {
            // TODO: THIS AINT RIGHT
            ompl::base::RealVectorStateSpace* space = components_[i]->as<ompl::base::RealVectorStateSpace>();
            ompl::base::RealVectorBounds subBounds(space->getDimension());

            for (size_t k = 0; k < space->getDimension(); k++) {
                subBounds.high[k] = bounds.high[i + k];
                subBounds.low[k] = bounds.low[i + k];
            }

            space->setBounds(subBounds);
            i += space->getDimension();
        }
    }
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
