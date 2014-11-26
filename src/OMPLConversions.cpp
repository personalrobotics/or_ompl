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

#include <boost/make_shared.hpp>
#include "OMPLConversions.h"

namespace or_ompl {

    void OpenRAVEHandler::log(std::string const &text, ompl::msg::LogLevel level,
      char const *filename, int line)
    {
        int const openrave_level = (OpenRAVE::RaveGetDebugLevel()
          & OpenRAVE::Level_OutputMask);

        switch (level) {
            case ompl::msg::LOG_DEBUG:
            if (openrave_level >= (int) OpenRAVE::Level_Debug) {
                OpenRAVE::RavePrintfA_DEBUGLEVEL("[%s:%d] %s\n",
                    OpenRAVE::RaveGetSourceFilename(filename), line,
                    text.c_str());
            }
            break;

            case ompl::msg::LOG_INFO:
            if (openrave_level >= (int) OpenRAVE::Level_Info) {
                OpenRAVE::RavePrintfA_INFOLEVEL("[%s:%d] %s\n",
                    OpenRAVE::RaveGetSourceFilename(filename), line,
                    text.c_str());
            }
            break;

            case ompl::msg::LOG_WARN:
            if (openrave_level >= (int) OpenRAVE::Level_Warn) {
                OpenRAVE::RavePrintfA_WARNLEVEL("[%s:%d] %s\n",
                    OpenRAVE::RaveGetSourceFilename(filename), line,
                    text.c_str());
            }
            break;

            case ompl::msg::LOG_ERROR:
            if (openrave_level >= (int) OpenRAVE::Level_Error) {
                OpenRAVE::RavePrintfA_ERRORLEVEL("[%s:%d] %s\n",
                    OpenRAVE::RaveGetSourceFilename(filename), line,
                    text.c_str());
            }
            break;

            case ompl::msg::LOG_NONE:
            default:
            RAVELOG_ERROR("Unknown OMPL log level %d.\n", level);
        }
    }

    CompoundSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
        OMPLPlannerParameters const &params)
    {
        if (!robot) {
            RAVELOG_ERROR("Robot must not be NULL.\n");
            return CompoundSpacePtr();
        } else if (robot->GetActiveDOF() == 0) {
            RAVELOG_ERROR("Zero DOFs are active.\n");
            return CompoundSpacePtr();
        }

        if (params.m_seed) {
            RAVELOG_DEBUG("Setting OMPL seed to %u.\n", params.m_seed);
            ompl::RNG::setSeed(params.m_seed);
            if (ompl::RNG::getSeed() != params.m_seed) {
                RAVELOG_ERROR("Could not set OMPL seed. Was this the first or_ompl"
                  "  plan attempted?\n");
                return CompoundSpacePtr();
            }
        } else {
            RAVELOG_DEBUG("Using default seed of %u for OMPL.\n",
              ompl::RNG::getSeed());
        }

        size_t const num_dof = robot->GetActiveDOF();
    //boost::shared_ptr<ompl::base::RealVectorStateSpace> state_space
    //        = boost::make_shared<ompl::base::RealVectorStateSpace>(num_dof);

        size_t num_lim_dof = 0;
        size_t isContDofIndx[num_dof];

        boost::shared_ptr<ompl::base::CompoundStateSpace> 
        state_space = boost::make_shared<ompl::base::CompoundStateSpace>();


        RAVELOG_DEBUG("Setting joint limits.\n");
        std::vector<OpenRAVE::dReal> lowerLimits, upperLimits;
        robot->GetActiveDOFLimits(lowerLimits, upperLimits);
        BOOST_ASSERT(lowerLimits.size() == num_dof);
        BOOST_ASSERT(upperLimits.size() == num_dof);

        RAVELOG_DEBUG("Setting resolution.\n");
        std::vector<OpenRAVE::dReal> dof_resolutions;
        robot->GetActiveDOFResolutions(dof_resolutions);
        BOOST_ASSERT(dof_resolutions.size() == num_dof);

        double conservative_fraction = std::numeric_limits<double>::max();
        double longest_extent = 0;
        for (size_t i = 0; i < num_dof; ++i) {
            if (upperLimits[i] > lowerLimits[i]) {
                double const joint_extents = upperLimits[i] - lowerLimits[i];
                double const joint_fraction = dof_resolutions[i] / joint_extents;
                conservative_fraction = std::min(conservative_fraction, joint_fraction);
                longest_extent = std::max(longest_extent, joint_extents);
            }
        }



        if (std::isinf(conservative_fraction)) {
            RAVELOG_ERROR("All joints have equal lower and upper limits.\n");
            return CompoundSpacePtr();
        }
        RAVELOG_DEBUG("Computed resolution of %f (%f fraction of extents).\n",
          conservative_fraction * longest_extent, conservative_fraction);

        for(size_t dd = 0; dd < num_dof; dd++){
            OpenRAVE::KinBody::JointPtr jointPtr = robot->GetJointFromDOFIndex(dd);
            if(jointPtr->IsCircular(0)){
            //if (0) {
                boost::shared_ptr<ompl::base::SO2StateSpace> so2_state_space = boost::make_shared<ompl::base::SO2StateSpace>();
                //ompl::base::SO2StateSpace so2_state_space = ompl::base::SO2StateSpace();
                state_space->addSubspace(so2_state_space,1.0); 
            }
            else{
                boost::shared_ptr<ompl::base::RealVectorStateSpace> real_vector_state_space
                = boost::make_shared<ompl::base::RealVectorStateSpace>(1);       
                ompl::base::RealVectorBounds bounds(1);
                BOOST_ASSERT(lowerLimits[dd] <= upperLimits[dd]);
                //std::cout<<bounds.high[0];
                //std::cout<<bound.low[0];
                bounds.setLow(0, lowerLimits[dd]);
                bounds.setHigh(0, upperLimits[dd]);
                real_vector_state_space->setBounds(bounds);
                bounds.check();
                real_vector_state_space->setLongestValidSegmentFraction(conservative_fraction);
                state_space->addSubspace(real_vector_state_space,1.0);


                //num_lim_dof = num_lim_dof + 1;
            }
        }
    //std::cout << "*********************** continuousDofs: " << continuousDofs << std::endl;
    //ompl::base::StateSpacePtr realVectorStateSpace(new ompl::base::RealVectorStateSpace(limitDofs));
       // boost::shared_ptr<ompl::base::RealVectorStateSpace> real_vector_state_space
       // = boost::make_shared<ompl::base::RealVectorStateSpace>(num_lim_dof);       



    // Set the resolution at which OMPL should discretize edges for collision
    // checking. OpenRAVE supports per-joint resolutions, so we compute one
    // conservative value for all joints. We then convert this to a fraction
    // of the workspace extents to call setLongestValidSegmentFraction.


    // Per-DOF weights are not supported by OMPL.
    // TODO: Emulate this by scaling joint values.
        RAVELOG_DEBUG("Setting joint weights.\n");
        std::vector<OpenRAVE::dReal> dof_weights;
        robot->GetActiveDOFWeights(dof_weights);
        BOOST_ASSERT(dof_weights.size() == num_dof);

        bool has_weights = false;
        for (size_t i = 0; !has_weights && i < num_dof; ++i) {
            has_weights = dof_weights[i] != 1.0;
        }

        if (has_weights) {
            RAVELOG_WARN("Robot specifies DOF weights. Only unit weights are"
             " supported by OMPL; planning will commence as if"
             " there are no weights.\n");
        }
       // state_space->addSubspace(real_vector_state_space,1.0);
        std::cout << state_space->getDimension();
        return state_space;
    }

}
