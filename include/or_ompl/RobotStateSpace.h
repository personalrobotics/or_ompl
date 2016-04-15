/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

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

#ifndef OR_OMPL_ROBOT_STATE_SPACE_H_
#define OR_OMPL_ROBOT_STATE_SPACE_H_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace or_ompl {

class RobotStateSpace;
/**
 * Implements a state for an OpenRAVE robot
 */
class RobotState : public ompl::base::CompoundStateSpace::StateType {

public:
    /**
     * Constuctor - all dof values are initialized to 0
     * @param dof_indices The dof indices this state will represent
     */
    RobotState();

    /**
     * Clear the indices vector
     */
    ~RobotState();
};

/**
 * Implements a state space for an OpenRAVE robot
 */
class RobotStateSpace : public ompl::base::CompoundStateSpace {


public:
    typedef RobotState StateType;
    /**
     * Constructor
     * @param dof_indices An ordered list of indices this state corresponds to
     */
    RobotStateSpace(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous);

    /**
     * @return A state of type RobotState
     */
    virtual ompl::base::State* allocState() const;

    /** \brief Register the projections for this state space. Usually, this is at least the default
        projection. These are implicit projections, set by the implementation of the state space. This is called by setup(). */
    virtual void registerProjections();

    /**
     * Set the upper/lower bounds of the state space.
     */
    void setBounds(const ompl::base::RealVectorBounds& bounds);

    /**
     * @return An ordered list of indices this state corresponds to
     */
    const std::vector<int>& getIndices() const { return _indices; }

private:
    std::vector<int> _indices;
    std::vector<bool> _isContinuous;
    ompl::base::ProjectionEvaluatorPtr _projectionEvaluator;

};

typedef boost::shared_ptr<RobotStateSpace> RobotStateSpacePtr;

class RobotProjectionEvaluator : public ompl::base::ProjectionEvaluator {
    public:
        RobotProjectionEvaluator(ompl::base::StateSpace* stateSpace);
        RobotProjectionEvaluator(ompl::base::StateSpacePtr stateSpace);
        virtual ~RobotProjectionEvaluator();

        /** \brief Return the dimension of the projection defined by this evaluator */
        virtual unsigned int getDimension() const;

        /** \brief Compute the projection as an array of double values */
        virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const;

        virtual void defaultCellSizes();

        virtual void setup();
    protected:
        or_ompl::RobotStateSpace* _robotStateSpace;
        ompl::base::ProjectionMatrix _projectionMatrix;
};

typedef boost::shared_ptr<RobotProjectionEvaluator> RobotProjectionEvaluatorPtr;

} // namespace or_ompl

#endif // OR_OMPL_ROBOT_STATE_SPACE_H_
