#include <or_ompl/RobotStateSpace.h>

using namespace or_ompl;
namespace ob = ompl::base;

RobotState::RobotState(const std::vector<int> &dof_indices) : _indices(dof_indices) {

    values = new double[_indices.size()];
    for(unsigned int idx=0; idx < _indices.size(); idx++){
        values[idx] = 0.0;
    }

}

RobotState::~RobotState() {
    _indices.clear();
}

void RobotState::set(const std::vector<double> &dof_values) {

    for(unsigned int idx=0; idx < _indices.size(); idx++){
        values[idx] = dof_values[idx];
    }

}

std::vector<double> RobotState::getValues() const {
    std::vector<double> retval(_indices.size());

    for(unsigned int idx=0; idx < retval.size(); idx++){
        retval[idx] = values[idx];
    }

    return retval;
}

RobotStateSpace::RobotStateSpace(const std::vector<int> &dof_indices) : ompl::base::RealVectorStateSpace(dof_indices.size()), _indices(dof_indices) {

}

ompl::base::State* RobotStateSpace::allocState() const {

    RobotState* state = new RobotState(_indices);
    return state;

}
