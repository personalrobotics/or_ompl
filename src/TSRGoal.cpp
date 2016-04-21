/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Jennifer King <jeking04@gmail.com>

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

#include <boost/foreach.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>

#include <or_ompl/StateSpaces.h>
#include <or_ompl/TSRGoal.h>

using namespace or_ompl;
namespace ob = ompl::base;

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
                 const TSR::Ptr &tsr, 
                 OpenRAVE::RobotBasePtr robot,
                 OrStateValidityCheckerPtr or_validity_checker)
    : ob::GoalSampleableRegion(si), _robot(robot)
    , _state_space(si->getStateSpace().get())
    , _or_validity_checker(or_validity_checker) {
    
    std::vector<TSR::Ptr> tsrs(1);
    tsrs.push_back(tsr);
    TSRChain::Ptr tsrchain = boost::make_shared<TSRChain>(true, false, false, tsrs);
    _tsr_chains.push_back(tsrchain);
}

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
                 const TSRChain::Ptr &tsrchain, 
                 OpenRAVE::RobotBasePtr robot,
                 OrStateValidityCheckerPtr or_validity_checker)
    : ob::GoalSampleableRegion(si), _robot(robot)
    , _state_space(si->getStateSpace().get())
    , _or_validity_checker(or_validity_checker) {

    _tsr_chains.push_back(tsrchain);
}

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
                 const std::vector<TSRChain::Ptr> &tsrchains, 
                 OpenRAVE::RobotBasePtr robot,
                 OrStateValidityCheckerPtr or_validity_checker)
    : ob::GoalSampleableRegion(si), _tsr_chains(tsrchains), _robot(robot)
    , _state_space(si->getStateSpace().get())
    , _or_validity_checker(or_validity_checker) {
}

TSRGoal::~TSRGoal() {
    
}

bool TSRGoal::isSatisfied(const ompl::base::State *state) const {

    bool satisfied = (distanceGoal(state) == 0.0);
    return satisfied;
}
            
double TSRGoal::distanceGoal(const ompl::base::State *state) const {

    // Save the state of the robot
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_robot->GetEnv()->GetMutex());
    OpenRAVE::KinBody::KinBodyStateSaver rsaver(_robot);

    OpenRAVE::RobotBase::ManipulatorPtr active_manip = _robot->GetActiveManipulator();

    // Put the robot in the pose that is represented in the state
    unsigned int check_limits = 0; // The planner does this
    _or_validity_checker->computeFk(state, check_limits);

    // Get the end effector transform
    OpenRAVE::Transform or_tf = active_manip->GetEndEffectorTransform();
    OpenRAVE::TransformMatrix or_matrix(or_tf);
    
    // Convert to Eigen
    Eigen::Affine3d ee_pose = Eigen::Affine3d::Identity();
    ee_pose.linear() << or_matrix.m[0], or_matrix.m[1], or_matrix.m[2],
        or_matrix.m[4], or_matrix.m[5], or_matrix.m[6],
        or_matrix.m[8], or_matrix.m[9], or_matrix.m[10];
    ee_pose.translation() << or_matrix.trans.x, or_matrix.trans.y, or_matrix.trans.z;

    // Get distance to TSR
    double distance = std::numeric_limits<double>::infinity();
    BOOST_FOREACH(TSRChain::Ptr tsrchain, _tsr_chains){
        Eigen::Matrix<double, 6, 1> ee_distance = tsrchain->distance(ee_pose);
        double tdistance = ee_distance.norm();
        if(tdistance < distance){
            distance = tdistance;
        }
    }

    // Reset the state of the robot
    rsaver.Restore();

    return distance;
}
            
void TSRGoal::sampleGoal(ompl::base::State *state) const {

    bool success = false;

    // TODO: Figure out how to bail correctly if an IK isn't found
    for(unsigned int count=0; count < 20 && !success; count++){
        // Pick a TSR to sample
        int idx = 0;
        if(_tsr_chains.size() > 1){
            ompl::RNG rng;
            idx = rng.uniformInt(0, _tsr_chains.size()-1);
        }

        // Sample the TSR
        Eigen::Affine3d ee_pose = _tsr_chains[idx]->sample();

        // Find an associated IK
        OpenRAVE::TransformMatrix or_matrix;
        or_matrix.rotfrommat(
            ee_pose.matrix()(0, 0), ee_pose.matrix()(0, 1), ee_pose.matrix()(0, 2),
            ee_pose.matrix()(1, 0), ee_pose.matrix()(1, 1), ee_pose.matrix()(1, 2),
            ee_pose.matrix()(2, 0), ee_pose.matrix()(2, 1), ee_pose.matrix()(2, 2)
            );
        or_matrix.trans.x = ee_pose(0, 3);
        or_matrix.trans.y = ee_pose(1, 3);
        or_matrix.trans.z = ee_pose(2, 3);

        OpenRAVE::IkParameterization ik_param(or_matrix, OpenRAVE::IKP_Transform6D);
        std::vector<OpenRAVE::dReal> ik_solution;
        success = _robot->GetActiveManipulator()->FindIKSolution(ik_param, ik_solution, OpenRAVE::IKFO_CheckEnvCollisions);

        // Set the state
        if(success){

            std::vector<int> arm_indices = _robot->GetActiveManipulator()->GetArmIndices();
            const std::vector<int> & state_indices = _or_validity_checker->getIndices();
            std::vector<double> values(state_indices.size());
            for(unsigned int idx=0; idx < ik_solution.size(); idx++){

                unsigned int sidx = std::find(state_indices.begin(),
                                              state_indices.end(),
                                              arm_indices[idx]) - state_indices.begin();

                values[sidx] = ik_solution[idx];
            }
            _state_space->copyFromReals(state, values);
        }
    }

    if(!success){
        RAVELOG_ERROR("[TSRGoal] Failed to sample valid goal.\n");
        const std::vector<int> & state_indices = _or_validity_checker->getIndices();
        std::vector<double> values(state_indices.size(), std::numeric_limits<double>::quiet_NaN());
        _state_space->copyFromReals(state, values);
    }
}

unsigned int TSRGoal::maxSampleCount() const {

    return std::numeric_limits<unsigned int>::max();
}
