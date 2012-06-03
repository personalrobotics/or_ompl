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
                m_timeLimit(10), m_plannerType("RRTConnect"), m_isProcessing(false)
            {
                _vXMLParameters.push_back("time_limit");
                _vXMLParameters.push_back("planner_type");

            }

            double m_timeLimit;
            std::string m_plannerType;
            bool m_isProcessing;

        protected:
            virtual bool serialize(std::ostream& O) const
            {
                if (!PlannerParameters::serialize(O))
                {
                    return false;
                }

                O << "<time_limit>" << m_timeLimit << "</time_limit>" << std::endl;

                O << "<planner_type>" << m_plannerType << "/<planner_type>" << std::endl;

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

                m_isProcessing =  name == "time_limit" || name == "planner_type";

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
