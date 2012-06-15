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

    class OMPLPlanner: public OpenRAVE::PlannerBase
    {
        public:
            OMPLPlanner(OpenRAVE::EnvironmentBasePtr penv);
            virtual ~OMPLPlanner();

            virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params);
            virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input);
            virtual OpenRAVE::PlannerStatus PlanPath (OpenRAVE::TrajectoryBasePtr ptraj);
            virtual PlannerParametersConstPtr GetParameters () const { return m_parameters; }
            inline ompl::geometric::SimpleSetup* GetSimpleSetup() { return m_simpleSetup; }
            inline ompl::base::StateSpacePtr GetStateSpace() { return m_stateSpace; }
            bool IsStateValid(const ompl::base::State* state);
            bool IsInOrCollision(std::vector<double> jointValues);
            bool InitializePlanner();

        private:
            OMPLPlannerParametersPtr m_parameters;
            ompl::geometric::SimpleSetup* m_simpleSetup;
            ompl::base::StateSpacePtr m_stateSpace;
            ompl::base::PlannerPtr m_planner;
            OpenRAVE::RobotBasePtr m_robot;
            OpenRAVE::CollisionReportPtr m_collisionReport;
            int m_numCollisionChecks;
            double m_totalCollisionTime;
            double scale_radii[7];

    };

    typedef boost::shared_ptr<OMPLPlanner> OMPLPlannerPtr;

} /* namespace or_ompl */

#endif /* OMPLPLANNER_H_ */
