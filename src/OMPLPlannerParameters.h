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
#ifndef  CPARAMETERS_H
#define  CPARAMETERS_H

#include <openrave-core.h>
#include <openrave/planner.h>

namespace or_ompl
{
    /// class for passing parameters to the ompl planner
    class OMPLPlannerParameters: public OpenRAVE::PlannerBase::PlannerParameters
    {
        public:
            OMPLPlannerParameters() :
                m_seed(0),
                m_timeLimit(10),
                m_plannerType("RRTConnect"),
                m_isProcessing(false),
                m_rrtRange(0.0), /* 0.0 means dont set */
                m_rrtGoalBias(0.0), /* 0.0 means dont set */
                m_rrtStarBallRadiusConstant(0.0), /* 0.0 means dont set */
                m_rrtStarMaxBallRadius(0.0), /* 0.0 means dont set */
                m_dat_filename(""),
                m_trajs_fileformat("")
            {
                _vXMLParameters.push_back("seed");
                _vXMLParameters.push_back("time_limit");
                _vXMLParameters.push_back("planner_type");
                _vXMLParameters.push_back("rrt_range");
                _vXMLParameters.push_back("rrt_goal_bias");
                _vXMLParameters.push_back("rrtstar_ball_radius_constant");
                _vXMLParameters.push_back("rrtstar_max_ball_radius");
                _vXMLParameters.push_back("dat_filename");
                _vXMLParameters.push_back("trajs_fileformat");
            }

            unsigned int m_seed;
            double m_timeLimit;
            std::string m_plannerType;
            bool m_isProcessing;
            double m_rrtRange;
            double m_rrtGoalBias;
            double m_rrtStarBallRadiusConstant;
            double m_rrtStarMaxBallRadius;
            std::string m_dat_filename;
            std::string m_trajs_fileformat;

        protected:
            virtual bool serialize(std::ostream& O) const
            {
                if (!PlannerParameters::serialize(O))
                {
                    return false;
                }

                O << "<seed>" << m_seed << "</seed>" << std::endl;
                O << "<time_limit>" << m_timeLimit << "</time_limit>" << std::endl;
                O << "<planner_type>" << m_plannerType << "</planner_type>" << std::endl;
                O << "<rrt_range>" << m_rrtRange << "</rrt_range>" << std::endl;
                O << "<rrt_goal_bias>" << m_rrtGoalBias << "</rrt_goal_bias>" << std::endl;
                O << "<rrtstar_ball_radius_constant>" << m_rrtStarBallRadiusConstant << "</rrtstar_ball_radius_constant>" << std::endl;
                O << "<rrtstar_max_ball_radius>" << m_rrtStarMaxBallRadius << "</rrtstar_max_ball_radius>" << std::endl;
                O << "<dat_filename>" << m_dat_filename << "</dat_filename>" << std::endl;
                O << "<trajs_fileformat>" << m_trajs_fileformat << "</trajs_fileformat>" << std::endl;

                return !!O;
            }

            ProcessElement startElement(const std::string& name, const std::list<std::pair<std::string, std::string> >& atts)
            {
                if (m_isProcessing)
                {
                    return PE_Ignore;
                }
                switch (OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts))
                {
                    case PE_Pass:
                        break;
                    case PE_Support:
                        return PE_Support;
                    case PE_Ignore:
                        return PE_Ignore;
                }

                m_isProcessing =
                     name == "seed"
                  || name == "time_limit"
                  || name == "planner_type"
                  || name == "rrt_range"
                  || name == "rrt_goal_bias"
                  || name == "rrtstar_ball_radius_constant"
                  || name == "rrtstar_max_ball_radius"
                  || name == "dat_filename"
                  || name == "trajs_fileformat";

                return m_isProcessing ? PE_Support : PE_Pass;
            }

            virtual bool endElement(const std::string& name)
            {
                if (m_isProcessing)
                {
                    if (name == "seed")
                    {
                        _ss >> m_seed;
                    }
                    else if (name == "time_limit")
                    {
                        _ss >> m_timeLimit;
                    }
                    else if(name == "planner_type")
                    {
                        _ss >> m_plannerType;
                    }
                    else if(name == "rrt_range")
                    {
                        _ss >> m_rrtRange;
                    }
                    else if(name == "rrt_goal_bias")
                    {
                        _ss >> m_rrtGoalBias;
                    }
                    else if(name == "rrtstar_ball_radius_constant")
                    {
                        _ss >> m_rrtStarBallRadiusConstant;
                    }
                    else if(name == "rrtstar_max_ball_radius")
                    {
                        _ss >> m_rrtStarMaxBallRadius;
                    }
                    else if(name == "dat_filename")
                    {
                        _ss >> m_dat_filename;
                    }
                    else if(name == "trajs_fileformat")
                    {
                        _ss >> m_trajs_fileformat;
                    }
                    else
                    {
                        RAVELOG_WARN(str(boost::format("unknown tag %s\n") % name));
                    }
                    m_isProcessing = false;
                    return false;
                }

                return PlannerParameters::endElement(name);
            }
    };

    typedef boost::shared_ptr<OMPLPlannerParameters> OMPLPlannerParametersPtr;
}

#endif
