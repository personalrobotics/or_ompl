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
                m_timeLimit(10), m_plannerType("RRTConnect"), m_isProcessing(false), m_rrtGoalBias(0.1), m_rrtStarMaxBallRadius(1.0), m_rrtStarMaxPathLength(5), m_rrtRange(0.1), m_dumpFileName("OMPL ")
            {
                _vXMLParameters.push_back("time_limit");
                _vXMLParameters.push_back("planner_type");
                _vXMLParameters.push_back("rrt_goal_bias");
                _vXMLParameters.push_back("rrtstar_max_ball_radius");
                _vXMLParameters.push_back("rrtstar_max_path_length");
                _vXMLParameters.push_back("rrt_range");
                _vXMLParameters.push_back("dum_file_name");
            }

            double m_timeLimit;
            std::string m_plannerType;
            bool m_isProcessing;
            double m_rrtGoalBias;
            double m_rrtStarMaxBallRadius;
            double m_rrtStarMaxPathLength;
            double m_rrtRange;
            std::string m_dumpFileName;

        protected:
            virtual bool serialize(std::ostream& O) const
            {
                if (!PlannerParameters::serialize(O))
                {
                    return false;
                }

                O << "<time_limit>" << m_timeLimit << "</time_limit>" << std::endl;
                O << "<planner_type>" << m_plannerType << "</planner_type>" << std::endl;
                O << "<rrt_goal_bias>" << m_rrtGoalBias << "</rrt_goal_bias>" << std::endl;
                O << "<rrtstar_max_ball_radius>" << m_rrtStarMaxBallRadius << "</rrtstar_max_ball_radius>" << std::endl;
                O << "<rrtstar_max_path_length>" << m_rrtStarMaxPathLength << "</rrtstar_max_path_length>" << std::endl;
                O << "<rrt_range>" << m_rrtRange << "</rrt_range>" << std::endl;
                O << "<file_name>" << m_dumpFileName << "</file_name>" << std::endl;

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

                m_isProcessing =  name == "time_limit" || name == "planner_type" || name == "rrt_goal_bias"
                        || name == "rrtstar_max_ball_radius" || name == "rrtstar_max_path_length" || name == "rrt_range"
                        || name == "dum_file_name";

                return m_isProcessing ? PE_Support : PE_Pass;
            }

            virtual bool endElement(const std::string& name)
            {
                if (m_isProcessing)
                {

                    if (name == "time_limit")
                    {
                        _ss >> m_timeLimit;
                    }
                    else if(name == "planner_type")
                    {
                        _ss >> m_plannerType;
                    }
                    else if(name == "rrt_goal_bias")
                    {
                        _ss >> m_rrtGoalBias;
                    }
                    else if(name == "rrtstar_max_ball_radius")
                    {
                        _ss >> m_rrtStarMaxBallRadius;
                    }
                    else if(name == "rrtstar_max_path_length")
                    {
                        _ss >> m_rrtStarMaxPathLength;
                    }
                    else if(name == "rrt_range")
                    {
                        _ss >> m_rrtRange;
                    }
                    else if(name == "dum_file_name")
                    {
                        _ss >> m_dumpFileName;
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
