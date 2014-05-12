/*
 * OMPLPlanner.h
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#ifndef OMPLPLANNER_H
#define OMPLPLANNER_H

#include <openrave-core.h>
#include <openrave/planner.h>
#include <openrave/planningutils.h>
#include <ompl/geometric/SimpleSetup.h>

#include "OMPLPlannerParameters.h"

namespace or_ompl
{

class OMPLPlanner: public OpenRAVE::PlannerBase {
public:
    OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~OMPLPlanner();

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);
    virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);
    virtual PlannerParametersConstPtr GetParameters () const { return m_parameters; }
    inline ompl::geometric::SimpleSetupPtr GetSimpleSetup() { return m_simpleSetup; }
    inline ompl::base::StateSpacePtr GetStateSpace() { return m_stateSpace; }

private:
    OMPLPlannerParametersPtr m_parameters;
    ompl::geometric::SimpleSetupPtr m_simpleSetup;
    ompl::base::StateSpacePtr m_stateSpace;
    ompl::base::PlannerPtr m_planner;
    OpenRAVE::RobotBasePtr m_robot;
    OpenRAVE::CollisionReportPtr m_collisionReport;
    int m_numCollisionChecks;
    double m_totalCollisionTime;

    bool InitializePlanner();
    bool IsStateValid(const ompl::base::State* state);
    bool IsInOrCollision(std::vector<double> jointValues);
    OpenRAVE::PlannerStatus ToORTrajectory(ompl::geometric::PathGeometric &ompl_traj,
                                           OpenRAVE::TrajectoryBasePtr or_traj) const;
};

typedef boost::shared_ptr<OMPLPlanner> OMPLPlannerPtr;

} /* namespace or_ompl */

#endif /* OMPLPLANNER_H_ */
