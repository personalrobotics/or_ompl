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

#ifndef OR_OMPL_TSRGOAL_H_
#define OR_OMPL_TSRGOAL_H_

#include <boost/make_shared.hpp>
#include <openrave-core.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

#include <or_ompl/config.h>
#include <or_ompl/TSR.h>
#include <or_ompl/TSRChain.h>

namespace or_ompl {

/**
 * Implements a goal region defined by a center goal position and a radius
 */
class TSRGoal : public ompl::base::GoalSampleableRegion {

public:
#ifdef OR_OMPL_HAS_BOOSTSMARTPTRS
    typedef boost::shared_ptr<TSRGoal> Ptr;
#else
    typedef std::shared_ptr<TSRGoal> Ptr;
#endif

    /**
     * Constructor
     *
     * @param si The space information object describing the space where the algorithm will be used
     * @param tsr The TSR chain describe the goal region
     * @param robot The robot to sample a pose for
     */
    TSRGoal(const ompl::base::SpaceInformationPtr &si,
            const TSR::Ptr &tsr,
            OpenRAVE::RobotBasePtr robot,
            OrStateValidityCheckerPtr or_validity_checker);

    /**
     * Constructor
     *
     * @param si The space information object describing the space where the algorithm will be used
     * @param tsrchain The TSR chain describe the goal region
     * @param robot The robot to sample a pose for
     */
    TSRGoal(const ompl::base::SpaceInformationPtr &si,
            const TSRChain::Ptr &tsrchain,
            OpenRAVE::RobotBasePtr robot,
            OrStateValidityCheckerPtr or_validity_checker);

    /**
     * Constructor
     *
     * @param si The space information object describing the space where the algorithm will be used
     * @param tsrchains The list of TSR chain describe the goal region
     * @param robot The robot to sample a pose for
     */
    TSRGoal(const ompl::base::SpaceInformationPtr &si,
            const std::vector<TSRChain::Ptr> &tsrchains,
            OpenRAVE::RobotBasePtr robot,
            OrStateValidityCheckerPtr or_validity_checker);

    /**
     * Destructor
     */
    ~TSRGoal();
        
    /**
     * Determines if the given state falls within the goal radius
     *
     * @param state The state to check
     * @return True if the state is within the goal radius
     */
    virtual bool isSatisfied(const ompl::base::State *state) const;
        
    /**
     * Calculates the distance between the state and the edge of the goal region
     *
     * @param state The state
     * @return The distance to the edge of the goal region (0 if the state is within the goal region)
     */
    virtual double distanceGoal(const ompl::base::State *state) const;
        
    /**
     * Samples a state from within the goal region
     *
     * @param state The sampled state
     */
    virtual void sampleGoal(ompl::base::State *state) const;

    /**
     * @return max int
     */
    virtual unsigned int maxSampleCount() const;
        
private:
    std::vector<TSRChain::Ptr> _tsr_chains;
    OpenRAVE::RobotBasePtr _robot;
    ompl::base::StateSpace * _state_space;
    OrStateValidityCheckerPtr _or_validity_checker;
};
      
} // namespace or_ompl

#endif // OR_OMPL_TSRGOAL_H_
