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
