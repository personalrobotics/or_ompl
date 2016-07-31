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

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <openrave/openrave.h>
#include <ompl/base/SpaceInformation.h>

#include <or_ompl/StateSpaces.h>

using namespace or_ompl;
namespace ob = ompl::base;

void ContinuousJointsStateSpace::registerProjections() {
    registerProjection("default", _projectionEvaluator);
    registerDefaultProjection(_projectionEvaluator);
    StateSpace::registerProjections();
}

ContinuousJointsStateSpace::ContinuousJointsStateSpace(const std::vector<bool>& is_continuous) :
        ompl::base::CompoundStateSpace(), _isContinuous(is_continuous) {
    // TODO: THIS AINT RIGHT
    size_t realDOFCount = 0;
    for (size_t i = 0; i < is_continuous.size(); i++) {
        if (is_continuous[i]) {
            if (realDOFCount > 0)
            {
                addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
                realDOFCount = 0;
            }
            addSubspace(ompl::base::StateSpacePtr(new ompl::base::SO2StateSpace()), 1.0);
        }
        else {
            realDOFCount++;
        }
    }
    if (realDOFCount > 0)
    {
        addSubspace(ompl::base::StateSpacePtr(new ompl::base::RealVectorStateSpace(realDOFCount)), 1.0);
    }
    _projectionEvaluator.reset(new ContinuousJointsProjectionEvaluator(this));
}

void ContinuousJointsStateSpace::setBounds(const ompl::base::RealVectorBounds& bounds) {
    BOOST_ASSERT(bounds.high.size() == bounds.low.size());

    for (size_t i = 0; i < bounds.high.size(); i++) {
        if (_isContinuous[i]) {
            continue;
        }
        else {
            ompl::base::RealVectorStateSpace* space = components_[i]->as<ompl::base::RealVectorStateSpace>();
            ompl::base::RealVectorBounds subBounds(space->getDimension());

            for (size_t k = 0; k < space->getDimension(); k++) {
                subBounds.high[k] = bounds.high[i + k];
                subBounds.low[k] = bounds.low[i + k];
            }

            space->setBounds(subBounds);
            i += space->getDimension();
        }
    }
}

ContinuousJointsProjectionEvaluator::ContinuousJointsProjectionEvaluator(ompl::base::StateSpace* stateSpace) :
    ompl::base::ProjectionEvaluator(stateSpace) {
    ContinuousJointsStateSpace* robotStateSpace = dynamic_cast<ContinuousJointsStateSpace*>(stateSpace);

    if (!robotStateSpace) {
        RAVELOG_ERROR("Can only use ContinuousJointsStateSpace with RobotProjectionEvaluator!");
        return;
    }

    _robotStateSpace = robotStateSpace;
}

ContinuousJointsProjectionEvaluator::ContinuousJointsProjectionEvaluator(ompl::base::StateSpacePtr stateSpace) :
    ProjectionEvaluator(stateSpace) {
    ContinuousJointsStateSpace* robotStateSpace = dynamic_cast<ContinuousJointsStateSpace*>(stateSpace.get());

    if (!robotStateSpace) {
        RAVELOG_ERROR("Can only use ContinuousJointsStateSpace with RobotProjectionEvaluator!");
        return;
    }

    _robotStateSpace = robotStateSpace;
}

ContinuousJointsProjectionEvaluator::~ContinuousJointsProjectionEvaluator() {

}

void ContinuousJointsProjectionEvaluator::setup() {
    _projectionMatrix.mat = _projectionMatrix.ComputeRandom(_robotStateSpace->getDimension(), getDimension());
    defaultCellSizes();
    ProjectionEvaluator::setup();
}

void ContinuousJointsProjectionEvaluator::defaultCellSizes() {
    cellSizes_.resize(getDimension());

    for (size_t i = 0; i < getDimension(); i++) {
        cellSizes_[i] = 0.5f;
    }
}

/** \brief Return the dimension of the projection defined by this evaluator */
unsigned int ContinuousJointsProjectionEvaluator::getDimension() const {
    int dim = _robotStateSpace->getDimension();
    if (dim <= 2) {
        return dim;
    }
    else {
        return (int)(log(_robotStateSpace->getDimension())) + 1;
    }
}

/** \brief Compute the projection as an array of double values */
void ContinuousJointsProjectionEvaluator::project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const {
    std::vector<double> values;
    _robotStateSpace->copyToReals(values, state);
    projection.resize(getDimension());
    _projectionMatrix.project(values.data(), projection);
}


or_ompl::OrStateValidityChecker::OrStateValidityChecker(
        const ompl::base::SpaceInformationPtr &si,
        OpenRAVE::RobotBasePtr robot, std::vector<int> const &indices,
        bool do_baked):
    ompl::base::StateValidityChecker(si),
    m_stateSpace(si->getStateSpace().get()),
    m_env(robot->GetEnv()), m_robot(robot), m_indices(indices),
    m_do_baked(do_baked)
{
    if (m_do_baked)
    {
        m_baked_checker = m_env->GetCollisionChecker();
        std::stringstream sinput("BakeGetType"), soutput;
        try
        {
            if (!m_baked_checker->SendCommand(soutput, sinput))
                throw std::runtime_error("collision checker doesn't support baked checks!");
        }
        catch (const OpenRAVE::openrave_exception & exc)
        {
            throw std::runtime_error("collision checker doesn't support baked checks!");
        }
        m_baked_kinbody_type = soutput.str();
    }
    
    resetStatistics();
}

void or_ompl::OrStateValidityChecker::start() {
    if (m_do_baked)
    {
        // start baking
        std::stringstream sinput("BakeBegin BakeEnd"), soutput;
        m_baked_checker->SendCommand(soutput, sinput); // BakeBegin
        m_baked_kinbody = OpenRAVE::RaveCreateKinBody(m_env, m_baked_kinbody_type);
        m_env->CheckCollision(m_robot);
        m_robot->CheckSelfCollision();
        m_baked_checker->SendCommand(soutput, sinput); // BakeEnd
    }
}

bool or_ompl::OrStateValidityChecker::computeFk(const ompl::base::State *state, uint32_t checklimits) const {
    std::vector<double> values;
    m_stateSpace->copyToReals(values, state);
    
    BOOST_FOREACH(double v, values) {
        if(std::isnan(v)) {
            RAVELOG_ERROR("Invalid value in state.\n");
            return false;
        }
    }
    
    m_robot->SetDOFValues(values, checklimits, m_indices);
    return true;
}

bool or_ompl::OrStateValidityChecker::isValid(const ompl::base::State *state) const {
    boost::chrono::steady_clock::time_point const tic
       = boost::chrono::steady_clock::now();
    
    bool collided = !computeFk(state, OpenRAVE::KinBody::CLA_Nothing);
    
    if (!collided)
    {
        if (m_do_baked)
            collided = collided || m_baked_checker->CheckStandaloneSelfCollision(m_baked_kinbody);
        else
            collided = collided || m_env->CheckCollision(m_robot) || m_robot->CheckSelfCollision();

        boost::chrono::steady_clock::time_point const toc
            = boost::chrono::steady_clock::now();
        m_totalCollisionTime += boost::chrono::duration_cast<
            boost::chrono::duration<double> >(toc - tic).count();
        m_numCollisionChecks++;
    }
    
    return !collided;
}

or_ompl::RealVectorOrStateValidityChecker::RealVectorOrStateValidityChecker(
        const ompl::base::SpaceInformationPtr &si,
        OpenRAVE::RobotBasePtr robot, std::vector<int> const &indices,
        bool do_baked):
    or_ompl::OrStateValidityChecker(si,robot,indices,do_baked),
    m_num_dof(si->getStateDimension()) {
}

bool or_ompl::RealVectorOrStateValidityChecker::computeFk(const ompl::base::State *state, uint32_t checklimits) const {
    ompl::base::RealVectorStateSpace::StateType const * real_state
        = state->as<ompl::base::RealVectorStateSpace::StateType>();
    
    std::vector<double> values(real_state->values, real_state->values+m_num_dof);
    
    BOOST_FOREACH(double v, values) {
        if(std::isnan(v)) {
            RAVELOG_ERROR("Invalid value in state.\n");
            return false;
        }
    }
    
    m_robot->SetDOFValues(values, checklimits, m_indices);
    return true;
}
