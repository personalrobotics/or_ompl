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

#ifndef OR_OMPL_OMPLCONVERSIONS_H_
#define OR_OMPL_OMPLCONVERSIONS_H_

#include <openrave/openrave.h>
#include <ompl/util/Console.h>
#include <ompl/geometric/PathGeometric.h>

#include <or_ompl/OMPLPlannerParameters.h>
#include <or_ompl/StateSpaces.h>

namespace or_ompl {

struct OpenRAVEHandler : public ompl::msg::OutputHandler {
public:
    virtual void log(std::string const &text, ompl::msg::LogLevel level,
                     char const *filename, int line);
};

std::vector<bool> GetContinuousJoints(const OpenRAVE::RobotBasePtr robot, const std::vector<int> idx);

ompl::base::StateSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                           OMPLPlannerParameters const &params);

OpenRAVE::PlannerStatus ToORTrajectory(OpenRAVE::RobotBasePtr const &robot,
                                       ompl::geometric::PathGeometric &ompl_traj,
                                       OpenRAVE::TrajectoryBasePtr or_traj);

} // namespace or_ompl

#endif // OR_OMPL_OMPLCONVERSIONS_H_
