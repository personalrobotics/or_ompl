#ifndef OMPL_TSR_ROBOT_H_
#define OMPL_TSR_ROBOT_H_

#include <or_ompl/TSRChain.h>
#include <openrave/openrave.h>
#include <boost/shared_ptr.hpp>

namespace or_ompl {

    /**
     * Decorator for an OpenRAVE Robot constructed to represent
     * a TSR chain
     */
    class TSRRobot {
        
    public:

        /**
         * Expose a shared ptr for the robot
         */
        typedef boost::shared_ptr<TSRRobot> Ptr;
        
        /**
         * Constructor
         */
        TSRRobot(const TSRChain::Ptr &chain, OpenRAVE::EnvironmentBasePtr &penv);

        /**
         * @return True if the construction was successful, false otherwise
         */
        bool construct();

        /**
         * @return True if this is a point TSR chain - meaning no TSRs have any freedom in the Bw matrix
         */
        bool isPointRobot() const { return _point_tsr; }

    private:

        TSRChain::Ptr _chain;
        OpenRAVE::EnvironmentBasePtr  _penv;
        OpenRAVE::RobotBasePtr _probot;
        OpenRAVE::IkSolverBasePtr _ik_solver;
        std::vector<OpenRAVE::dReal> _ikparams;
        std::vector<OpenRAVE::dReal> _upperlimits;
        std::vector<OpenRAVE::dReal> _lowerlimits;
        bool _initialized;
        bool _point_tsr;
        unsigned int _num_dof;
    };
}

#endif
