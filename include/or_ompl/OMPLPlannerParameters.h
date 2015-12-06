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
#ifndef CPARAMETERS_H
#define CPARAMETERS_H

#include <openrave-core.h>
#include <openrave/planner.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <or_ompl/config.h>
#include <or_ompl/TSRChain.h>

namespace or_ompl
{
class OMPLPlannerParameters : public OpenRAVE::PlannerBase::PlannerParameters
{
public:
    OMPLPlannerParameters()
        : m_seed(0)
        , m_timeLimit(10)
        , m_isProcessingOMPL(false)
        , m_dat_filename("")
        , m_trajs_fileformat("")
    {
        _vXMLParameters.push_back("seed");
        _vXMLParameters.push_back("time_limit");
        _vXMLParameters.push_back("dat_filename");
        _vXMLParameters.push_back("trajs_fileformat");
        _vXMLParameters.push_back("tsr_chain");
    }

    unsigned int m_seed;
    double m_timeLimit;
    bool m_isProcessingOMPL;
    std::string m_dat_filename;
    std::string m_trajs_fileformat;
    std::vector<TSRChain::Ptr> m_tsrchains;

protected:

#ifdef OR_OMPL_HAS_PPSEROPTS
    virtual bool serialize(std::ostream& O, int options=0) const
    {
        if (!PlannerParameters::serialize(O, options)) {
            return false;
        }
#else
    virtual bool serialize(std::ostream& O) const
    {
        if (!PlannerParameters::serialize(O)) {
            return false;
        }
#endif

        O << "<seed>" << m_seed << "</seed>" << std::endl;
        O << "<time_limit>" << m_timeLimit << "</time_limit>" << std::endl;
        O << "<dat_filename>" << m_dat_filename << "</dat_filename>" << std::endl;
        O << "<trajs_fileformat>" << m_trajs_fileformat << "</trajs_fileformat>" << std::endl;
        BOOST_FOREACH(TSRChain::Ptr chain, m_tsrchains) {
            O << "<tsr_chain>" << chain << "</tsr_chain>" << std::endl;
        }

        return !!O;
    }

    ProcessElement startElement(std::string const &name,
                                std::list<std::pair<std::string, std::string> > const &atts)
    {
        if (m_isProcessingOMPL) {
            return PE_Ignore;
        }

        switch (OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts)) {
            case PE_Pass:
                break;
            case PE_Support:
                return PE_Support;
            case PE_Ignore:
                return PE_Ignore;
        }

        m_isProcessingOMPL =
             name == "seed"
          || name == "time_limit"
          || name == "dat_filename"
          || name == "trajs_fileformat"
          || name == "tsr_chain";

        return m_isProcessingOMPL ? PE_Support : PE_Pass;
    }

    virtual bool endElement(std::string const &name)
    {
        if (m_isProcessingOMPL) {
            if (name == "seed") {
                _ss >> m_seed;
            } else if (name == "time_limit") {
                _ss >> m_timeLimit;
            } else if (name == "dat_filename") {
                _ss >> m_dat_filename; 
            } else if (name == "trajs_fileformat") {
                _ss >> m_trajs_fileformat;
            } else if (name == "tsr_chain") {
                TSRChain::Ptr chain = boost::make_shared<TSRChain>();
                bool success = chain->deserialize(_ss);
                if(!success){
                    RAVELOG_ERROR("failed to deserialize TSRChain");
                }else{
                    m_tsrchains.push_back(chain);
                }
            } else {
                RAVELOG_WARN(str(boost::format("unknown tag %s\n") % name));
            }
            m_isProcessingOMPL = false;
            return false;
        }

        return PlannerParameters::endElement(name);
    }
};

typedef boost::shared_ptr<OMPLPlannerParameters> OMPLPlannerParametersPtr;
}

#endif
