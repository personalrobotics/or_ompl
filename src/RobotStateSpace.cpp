#include "RobotStateSpace.h"

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
