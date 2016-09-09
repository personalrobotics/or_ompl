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

#include <limits>
#include <vector>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <or_ompl/TSRChain.h>

using namespace or_ompl;
using OpenRAVE::openrave_exception;
using OpenRAVE::ORE_Failed;


TSRChain::TSRChain()
    : _initialized(false)
    , _sample_start(false)
    , _sample_goal(false)
    , _constrain(false)
    , _tsr_robot()
{
}

TSRChain::TSRChain(const bool &sample_start,
                   const bool &sample_goal,
                   const bool &constrain,
                   const std::vector<TSR::Ptr> &tsrs)
    : _initialized(true)
    , _sample_start(sample_start)
    , _sample_goal(sample_goal)
    , _constrain(constrain)
    , _tsrs(tsrs)
    , _tsr_robot()
{
}

bool TSRChain::deserialize(std::stringstream &ss)
{
    return deserialize(static_cast<std::istream &>(ss));
}

bool TSRChain::deserialize(std::istream &ss)
{
    int num_tsrs;

    ss >> _sample_start
       >> _sample_goal
       >> _constrain
       >> num_tsrs;

    _tsrs.clear();

    for(int idx = 0; idx < num_tsrs; idx++)
    {
        TSR::Ptr new_tsr = boost::make_shared<TSR>();

        if (!new_tsr->deserialize(ss)){
            RAVELOG_ERROR("Failed deserializing TSR %d of %d in TSRChain.\n",
                idx + 1, num_tsrs);
            return false;
        }

        _tsrs.push_back(new_tsr);
    }

    // TODO: Ignored are mmicbody name and mimicbodyjoints

    if (!ss)
    {
        RAVELOG_ERROR("Failed deserializing TSRChain.\n");
        return false;
    }

    return true;
}

void TSRChain::serialize(std::ostream& ss)
{
    ss << _sample_start
       << ' ' << _sample_goal
       << ' ' << _constrain
       << ' ' << _tsrs.size();

    BOOST_FOREACH(const boost::shared_ptr<TSR> &tsr_chain, _tsrs){
        ss << ' ';
        tsr_chain->serialize(ss);
    }
}

Eigen::Affine3d TSRChain::sample() const {

    Eigen::Affine3d T0_w;
    if(_tsrs.size() == 0){
        throw OpenRAVE::openrave_exception(
            "There are no TSRs in this TSR Chain.",
            OpenRAVE::ORE_InvalidState
        );
    }

    T0_w = _tsrs.front()->getOriginTransform();
    BOOST_FOREACH(TSR::Ptr tsr, _tsrs){
        T0_w  = T0_w * tsr->sampleDisplacementTransform() * tsr->getEndEffectorOffsetTransform();
    }

    return T0_w;
}

Eigen::Matrix<double, 6, 1> TSRChain::distance(const Eigen::Affine3d &ee_pose) const {

    if(_tsrs.size() == 1){
        TSR::Ptr tsr = _tsrs.front();
        return tsr->distance(ee_pose);
    }

    if(!_tsr_robot){
        throw OpenRAVE::openrave_exception(
            "Failed to compute distance to TSRChain. Did you set the"
            " environment by calling the setEnv function?",
            OpenRAVE::ORE_InvalidState
        );
    }

    if(!_tsr_robot->construct()){
        throw OpenRAVE::openrave_exception(
            "Failed to robotize TSR.",
            OpenRAVE::ORE_Failed
        );
    }

    RAVELOG_DEBUG("[TSRChain] Solving IK to compute distance");
    // Compute the ideal pose of the end-effector
    Eigen::Affine3d Ttarget = ee_pose * _tsrs.back()->getEndEffectorOffsetTransform().inverse();

    // Ask the robot to solve ik to find the closest possible end-effector transform
    Eigen::Affine3d Tnear = _tsr_robot->findNearestFeasibleTransform(Ttarget);

    // Compute the distance between the two
    Eigen::Affine3d offset = Tnear.inverse() * Ttarget;
    Eigen::Matrix<double, 6, 1> dist = Eigen::Matrix<double, 6, 1>::Zero();
    dist[0] = offset.translation()[0];
    dist[1] = offset.translation()[1];
    dist[2] = offset.translation()[2];
    dist[3] = atan2(offset.rotation()(2,1), offset.rotation()(2,2));
    dist[4] = -asin(offset.rotation()(2,0));
    dist[5] = atan2(offset.rotation()(1,0), offset.rotation()(0,0));

    return dist;
}

void TSRChain::setEnv(const OpenRAVE::EnvironmentBasePtr &penv)
{
    _tsr_robot = boost::make_shared<TSRRobot>(_tsrs, penv);

}
