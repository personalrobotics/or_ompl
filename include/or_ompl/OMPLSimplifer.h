/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>

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

#ifndef OR_OMPL_OMPLSIMPLIFIER_H_
#define OR_OMPL_OMPLSIMPLIFIER_H_

#include <openrave-core.h>
#include <openrave/planner.h>
#include <ompl/geometric/PathSimplifier.h>

#include <or_ompl/OMPLPlannerParameters.h>

namespace or_ompl {

class OMPLSimplifier : public OpenRAVE::PlannerBase {
public:
    OMPLSimplifier(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~OMPLSimplifier(); 
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

    virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters() const { return m_parameters; }

private:
    OpenRAVE::RobotBasePtr m_robot;
    OMPLPlannerParametersPtr m_parameters;
    ompl::base::StateSpacePtr m_state_space;
    OrStateValidityCheckerPtr m_or_validity_checker;
    ompl::base::SpaceInformationPtr m_space_info;
    ompl::geometric::PathSimplifierPtr m_simplifier;
    OpenRAVE::ConfigurationSpecification m_cspec;
};

typedef boost::shared_ptr<OMPLSimplifier> OMPLSimplifierPtr;

} // namespace or_ompl

#endif // OR_OMPL_OMPLSIMPLIFIER_H_
