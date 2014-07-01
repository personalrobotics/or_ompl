#ifndef  CPARAMETERS_H
#define  CPARAMETERS_H

#include <openrave-core.h>
#include <openrave/planner.h>
#include <TSRChain.h>
#include <boost/foreach.hpp>

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
				_vXMLParameters.push_back("tsr_chain");
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
			std::vector<TSRChain::Ptr> m_tsrchains;

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
				BOOST_FOREACH(TSRChain::Ptr chain, m_tsrchains){
					O << "<tsr_chain>" << chain << "</goal_tsr>" << std::endl;
				}

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
						std::cout << "ignore" << std::endl;
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
                  || name == "trajs_fileformat"
				  || name == "tsr_chain";

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
					else if(name == "tsr_chain")
					{
						TSRChain::Ptr chain = boost::make_shared<TSRChain>();
						bool success = chain->deserialize(_ss);
						if(!success){
							RAVELOG_ERROR("failed to deserialize TSRChain");
						}else{
							m_tsrchains.push_back(chain);
						}
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
