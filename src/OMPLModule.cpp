/*
 * OMPLModule.cpp
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#include "OMPLModule.h"
namespace or_ompl
{

    OMPLModule::OMPLModule(OpenRAVE::EnvironmentBasePtr penv)
        : OpenRAVE::ModuleBase(penv), m_planner(new OMPLPlanner(penv))
    {
        __description = "Plugin Interface to OMPL";
        RegisterCommand("SetRobot", boost::bind(&OMPLModule::SetRobot, this, _1, _2), "Sets the robot to the one with the given name." );
        RegisterCommand("Plan", boost::bind(&OMPLModule::Plan, this,_1,_2),"Plan with the given parameters.");
    }

    OMPLModule::~OMPLModule()
    {

    }

    bool OMPLModule::Plan(std::ostream& output, std::istream& input)
    {
        OMPLPlannerParameters params;
        input >> params;

        m_params.reset(new OMPLPlannerParameters());
        m_params->copy(OMPLPlannerParametersPtr(&params));

        if(m_robot == NULL)
        {
            ROS_ERROR("Failed to plan. Robot not set.");
            return false;
        }

        if(!m_planner->InitPlan(m_robot, m_params))
        {
            ROS_ERROR("Failed to initialize parameters!");
            return false;
        }

        OpenRAVE::TrajectoryBasePtr trajectory = OpenRAVE::RaveCreateTrajectory(GetEnv(), "OMPL Trajectory");

        if(!m_planner->PlanPath(trajectory))
        {
            ROS_ERROR("Failed to plan!");
            return false;
        }

        output << trajectory;

        return true;
    }

    bool OMPLModule::SetRobot(std::ostream& output, std::istream& input)
    {
        std::string robotName;
        input >> robotName;

        m_robot = GetEnv()->GetRobot(robotName);

        if(m_robot == NULL)
        {
            ROS_ERROR("Robot does not exist!");
            return false;
        }
        else
        {
            return true;
        }
    }

} /* namespace or_ompl */
