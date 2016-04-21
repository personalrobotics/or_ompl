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

#ifndef OR_OMPL_OMPLPLANNER_H_
#define OR_OMPL_OMPLPLANNER_H_

#include <openrave-core.h>
#include <openrave/planner.h>
#include <openrave/planningutils.h>
#include <ompl/geometric/SimpleSetup.h>

#include <or_ompl/StateSpaces.h>
#include <or_ompl/OMPLPlannerParameters.h>

namespace or_ompl {

typedef boost::function<ompl::base::Planner *(ompl::base::SpaceInformationPtr)> PlannerFactory;

class OMPLPlanner: public OpenRAVE::PlannerBase {
public:
    OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv,
                PlannerFactory const &planner_factory);
    virtual ~OMPLPlanner();

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);

    virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters () const {
        return m_parameters;
    }
    
    bool GetTimes(std::ostream & sout, std::istream & sin) const;
    bool GetParameterValCommand(std::ostream &sout, std::istream &sin) const;

protected:
    const ompl::base::PlannerPtr & get_planner() {
        return m_planner;
    }

private:
    bool m_initialized;
    PlannerFactory m_planner_factory;
    OMPLPlannerParametersPtr m_parameters;
    ompl::geometric::SimpleSetupPtr m_simple_setup;
    ompl::base::StateSpacePtr m_state_space;
    OrStateValidityCheckerPtr m_or_validity_checker;
    ompl::base::PlannerPtr m_planner;
    OpenRAVE::RobotBasePtr m_robot;
    OpenRAVE::CollisionReportPtr m_collisionReport;
    double m_totalPlanningTime;

    ompl::base::PlannerPtr CreatePlanner(OMPLPlannerParameters const &params);

    bool GetParametersCommand(std::ostream &sout, std::istream &sin) const;
};

typedef boost::shared_ptr<OMPLPlanner> OMPLPlannerPtr;

} // namespace or_ompl

#endif // OR_OMPL_OMPLPLANNER_H_
