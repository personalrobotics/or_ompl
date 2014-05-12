/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

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
