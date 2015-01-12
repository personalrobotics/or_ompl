#include "RobotStateSpace.h"
#include <boost/foreach.hpp>
#include <ompl/util/Exception.h>

using namespace or_ompl;
namespace ob = ompl::base;

RobotState::RobotState(const std::vector<int> &dof_indices)
    : _dof_indices(dof_indices){

    values = new double[_dof_indices.size()];
    for(unsigned int idx=0; idx < _dof_indices.size(); idx++){
        values[idx] = 0.0;
    }

}

RobotState::~RobotState() {
    _dof_indices.clear();
}

bool RobotState::containsIndices(const std::vector<int> &indices) const{

    bool contains_indices = true;
    BOOST_FOREACH(int idx, indices){
        if(std::find(_dof_indices.begin(), _dof_indices.end(), idx) == _dof_indices.end()){
            contains_indices = false;
            break;
        }
    }

    return contains_indices;
}

void RobotState::set(const std::vector<double> &dof_values) {

    if(dof_values.size() != _dof_indices.size()){
        throw ompl::Exception(
            "[RobotState] set called with invalid values.");
    }

    for(unsigned int idx=0; idx < _dof_indices.size(); idx++){
        values[idx] = dof_values[idx];
    }

}

void RobotState::setPartial(const std::vector<int> &dof_indices, 
                            const std::vector<double> &dof_values){

    std::cout << "Begin set partial: " << dof_indices.size() << std::endl;
    for(unsigned int idx=0; idx < dof_indices.size(); idx++){
        int dof = dof_indices[idx];
        std::cout << "dof: " << dof << std::endl;
        std::vector<int>::iterator it = std::find(_dof_indices.begin(),
                                                  _dof_indices.end(),
                                                  dof);
        if(it != _dof_indices.end()){
            unsigned int sidx = it - _dof_indices.begin();
            std::cout << "\tsidx: " << sidx << std::endl;
            values[sidx] = dof_values[idx];
        }else{
            throw ompl::Exception(
                "[RobotState] setPartial called with dof_indices not in the state.");
        }
    }
    std::cout << "Done set partial" << std::endl;
}

std::vector<double> RobotState::getValues() const {
    std::vector<double> retval(_dof_indices.size());

    for(unsigned int idx=0; idx < retval.size(); idx++){
        retval[idx] = values[idx];
    }

    return retval;
}

RobotStateSpace::RobotStateSpace(const std::vector<int> &dof_indices)
    : ompl::base::RealVectorStateSpace(dof_indices.size()), _indices(dof_indices) {

}

ompl::base::State* RobotStateSpace::allocState() const {

    RobotState* state = new RobotState(_indices);
    return state;

}
