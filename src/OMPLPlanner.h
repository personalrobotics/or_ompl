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
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/prm/BasicPRM.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/contrib/rrt_star/RRTstar.h>
#include <ompl/contrib/rrt_star/BallTreeRRTstar.h>
#include <ros/console.h>
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
            bool IsInCollision(std::vector<double> jointValues);
            bool InitializePlanner();

        private:
            OMPLPlannerParametersPtr m_parameters;
            ompl::geometric::SimpleSetup* m_simpleSetup;
            ompl::base::StateSpacePtr m_stateSpace;
            ompl::base::PlannerPtr m_planner;
            OpenRAVE::RobotBasePtr m_robot;
            OpenRAVE::CollisionReportPtr m_collisionReport;


    };

    typedef boost::shared_ptr<OMPLPlanner> OMPLPlannerPtr;

} /* namespace or_ompl */
#endif /* OMPLPLANNER_H_ */
