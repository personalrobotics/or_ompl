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

#ifndef OR_OMPL_TSRCHAIN_H_
#define OR_OMPL_TSRCHAIN_H_

#include <vector>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>

#include <or_ompl/TSR.h>
#include <or_ompl/TSRRobot.h>

namespace or_ompl {

class TSRChain {

public:
    typedef boost::shared_ptr<TSRChain> Ptr;

    /**
     * Constructor
     */
    TSRChain();

    /**
     * Constructor
     *
     * @param sample_start True if the chain should be used to sample the start configuration
     * @param sample_goal True if the chain should be used to sample the start configuration
     * @param constrain True if the chain should be applied trajectory wide
     * @param tsrs The list of TSRs in the chain
     */
    TSRChain(const bool &sample_start,
             const bool &sample_goal,
             const bool &constrain,
             const std::vector<TSR::Ptr> &tsrs);

    /**
     * Deserialize a serialized TSR Chain.
     *
     * @param ss The stream to read the serialized TSR from
     */
    bool deserialize(std::istream &ss);
    bool deserialize(std::stringstream &ss);

    /**
     * Serialize a TSR Chain.
     *
     * @param ss The stream to read the serialized TSR from
     */
    void serialize(std::ostream& ss);

    /**
     * @return True if this chain should be used to sample a start
     */
    bool sampleStart() const { return _sample_start; }

    /**
     * @return True if this chain should be used to sample a goal
     */
    bool sampleGoal() const { return _sample_goal; }

    /**
     * @return True if this chain should be applied as a trajectory wide constraint
     */
    bool isTrajectoryConstraint() const { return _constrain; }


    /**
     * @return The list of TSRs that make up this chain
     */
    std::vector<TSR::Ptr> getTSRs() const { return _tsrs; }

    /**
     * @return A sample from the TSR chain
     */
    Eigen::Affine3d sample(void) const;

    /**
     * Compute the distance to the TSR
     *
     * @param ee_pose The pose of the end-effector in world frame
     */
    Eigen::Matrix<double, 6, 1> distance(const Eigen::Affine3d &ee_pose) const;

    /**
     * Set the planning environment. This is required to enable computing
     * distance to a TSR.
     * @param penv The OpenRAVE environment this TSRChain will be used in
     */
    void setEnv(const OpenRAVE::EnvironmentBasePtr &penv);

private:
    bool _initialized;
    bool _sample_start;
    bool _sample_goal;
    bool _constrain;
    std::vector<TSR::Ptr> _tsrs;
    TSRRobot::Ptr _tsr_robot;
};

} // namespace or_ompl

#endif // OR_OMPL_TSRCHAIN_H_
