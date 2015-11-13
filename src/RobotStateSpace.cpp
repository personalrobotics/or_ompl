#include "RobotStateSpace.h"
#include <openrave/openrave.h>

using namespace or_ompl;
namespace ob = ompl::base;

RobotState::RobotState(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous) :
        _indices(dof_indices), _isContinuous(is_continuous) {

}

RobotState::~RobotState() {
    _indices.clear();
}

void RobotState::set(const std::vector<double> &dof_values) {

    for(unsigned int idx=0; idx < _indices.size(); idx++){
        value(idx) = dof_values[idx];
    }
    enforceBounds();
}

double& RobotState::value(const size_t& idx) {
    if (_isContinuous[idx]) {
        return as<ompl::base::SO2StateSpace::StateType>(idx)->value;
    }
    else {
        return as<ompl::base::RealVectorStateSpace::StateType>(idx)->values[0];
    }
}

const double& RobotState::value(const size_t& idx) const {
    if (_isContinuous[idx]) {
        return as<ompl::base::SO2StateSpace::StateType>(idx)->value;
    }
    else {
        return as<ompl::base::RealVectorStateSpace::StateType>(idx)->values[0];
    }
}

std::vector<double> RobotState::getValues() const {
    std::vector<double> retval(_indices.size());

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

void  RobotState::enforceBounds() {
    for (size_t i = 0; i < _indices.size(); i++) {
        if (_isContinuous[i]) {
            ompl::base::SO2StateSpace::StateType* state = this->as<ompl::base::SO2StateSpace::StateType>(i);
            if (!state) continue;
            else {
                double v = fmod(state->value, 2.0 * M_PI);
                if (v <= -M_PI)
                 v += 2.0 * M_PI;
                else
                 if (v > M_PI)
                     v -= 2.0 * M_PI;
                state->value = v;
            }
        }
    }
}

RobotStateSpace::RobotStateSpace(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous) :
        ompl::base::CompoundStateSpace(), _indices(dof_indices), _isContinuous(is_continuous) {
    BOOST_ASSERT(dof_indices.size() == is_continuous.size());
    for (size_t i = 0; i < dof_indices.size(); i++) {
        if (is_continuous[i]) {
            addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 1.0);
        }
        else {
            addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(1)), 1.0);
        }
    }
    _projectionEvaluator.reset(new RobotProjectionEvaluator(this));
}

ompl::base::State* RobotStateSpace::allocState() const {

    RobotState* state = new RobotState(_indices, _isContinuous);
    allocStateComponents(state);
    return state;

}

void RobotStateSpace::setBounds(const ompl::base::RealVectorBounds& bounds) {
    BOOST_ASSERT(bounds.high.size() == bounds.low.size());
    BOOST_ASSERT(bounds.high.size() == components_.size());
    for (size_t i = 0; i < bounds.high.size(); i++) {
        if (_isContinuous[i]) {
            continue;
        }
        else {
            components_[i]->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds.low[i], bounds.high[i]);
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
