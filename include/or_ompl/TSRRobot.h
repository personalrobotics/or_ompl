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

#ifndef OR_OMPL_TSRROBOT_H_
#define OR_OMPL_TSRROBOT_H_

#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>

#include <or_ompl/TSR.h>

namespace or_ompl {

/**
 * Decorator for an OpenRAVE Robot constructed to represent
 * a TSR chain
 */
class TSRRobot {
    
public:

    /**
     * Expose a shared ptr for the robot
     */
    typedef boost::shared_ptr<TSRRobot> Ptr;
    
    /**
     * Constructor
     */
    TSRRobot(const std::vector<TSR::Ptr> &tsrs, const OpenRAVE::EnvironmentBasePtr &penv);

    /**
     * @return True if the construction was successful, false otherwise
     */
    bool construct();

    /**
     * Finds the nearest reachable end-effector transform to the given transform
     * @param Ttarget - The target end-effector transform
     */
    Eigen::Affine3d findNearestFeasibleTransform(const Eigen::Affine3d &Ttarget);

    /**
     * @return True if this is a point TSR chain - meaning no TSRs have any freedom in the Bw matrix
     */
    bool isPointRobot() const { return _point_tsr; }

    /**
     * @return True if robot has been properly initialized, false otherwise
     */
    bool isInitialized() const { return _initialized; }

private:

    std::vector<TSR::Ptr> _tsrs;
    OpenRAVE::EnvironmentBasePtr  _penv;
    OpenRAVE::RobotBasePtr _probot;
    std::string _solver;
    OpenRAVE::IkSolverBasePtr _ik_solver;
    std::vector<OpenRAVE::dReal> _upperlimits;
    std::vector<OpenRAVE::dReal> _lowerlimits;
    bool _initialized;
    bool _point_tsr;
    unsigned int _num_dof;
};

} // namespace or_ompl

#endif // OR_OMPL_TSRROBOT_H_
