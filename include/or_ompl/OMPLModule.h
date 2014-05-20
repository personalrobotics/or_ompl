/*
 * OMPLModule.h
 *
 *  Created on: Jun 2, 2012
 *      Author: mklingen
 */

#ifndef OMPLMODULE_H_
#define OMPLMODULE_H_

#include "OMPLPlanner.h"
#include <openrave/module.h>

namespace or_ompl
{

    class OMPLModule: public OpenRAVE::ModuleBase
    {
        public:
            OMPLModule(OpenRAVE::EnvironmentBasePtr penv);
            virtual ~OMPLModule();

            bool SetRobot(std::ostream& output, std::istream& input);
            bool Plan(std::ostream& output, std::istream& input);

        private:
            OMPLPlannerPtr m_planner;
            OpenRAVE::RobotBasePtr m_robot;
            OMPLPlannerParametersPtr m_params;
    };

} /* namespace or_ompl */
#endif /* OMPLMODULE_H_ */
