#ifndef OR_OMPL_ROBOT_STATE_SPACE_H_
#define OR_OMPL_ROBOT_STATE_SPACE_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/shared_ptr.hpp>

namespace or_ompl {

    /**
     * Implements a state for an OpenRAVE robot
     */
    class RobotState : public ompl::base::RealVectorStateSpace::StateType {

    public:
        /**
         * Constuctor - all dof values are initialized to 0
         * @param dof_indices The dof indices this state will represent
         */
        RobotState(const std::vector<int> &dof_indices);

        /**
         * Clear the indices vector
         */
        ~RobotState();

        /**
         * Sets the values in the state. 
         * @param dof_values The values, this ordering must match the dof_indices
         */
        void set(const std::vector<double> &dof_values);

        /**
         * @return The values in the state
         */
        std::vector<double> getValues() const;

        /**
         * @return An ordered list of indices this state corresponds to
         */
        std::vector<int> getIndices() const { return _indices; }

    private:
        std::vector<int> _indices;

    };

    /**
     * Implements a state space for an OpenRAVE robot
     */
    class RobotStateSpace : public ompl::base::RealVectorStateSpace {


    public:
        /**
         * Constructor
         * @param dof_indices An ordered list of indices this state corresponds to
         */
        RobotStateSpace(const std::vector<int> &dof_indices);

        /**
         * @return A state of type RobotState
         */
        virtual ompl::base::State* allocState() const;

    private:
        std::vector<int> _indices;

    };

    typedef boost::shared_ptr<RobotStateSpace> RobotStateSpacePtr;
}

#endif
