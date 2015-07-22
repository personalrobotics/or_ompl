#ifndef OR_OMPL_ROBOT_STATE_SPACE_H_
#define OR_OMPL_ROBOT_STATE_SPACE_H_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/shared_ptr.hpp>

namespace or_ompl {

    class RobotStateSpace;
    /**
     * Implements a state for an OpenRAVE robot
     */
    class RobotState : public ompl::base::CompoundStateSpace::StateType {

    public:
        /**
         * Constuctor - all dof values are initialized to 0
         * @param dof_indices The dof indices this state will represent
         */
        RobotState(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous);

        /**
         * Clear the indices vector
         */
        ~RobotState();

        /**
         * Gets a reference to the value at index i.
         */
        double& value(const size_t& idx);
        const double& value(const size_t& idx) const;

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
        std::vector<bool> _isContinuous;

    };

    /**
     * Implements a state space for an OpenRAVE robot
     */
    class RobotStateSpace : public ompl::base::CompoundStateSpace {


    public:
        typedef RobotState StateType;
        /**
         * Constructor
         * @param dof_indices An ordered list of indices this state corresponds to
         */
        RobotStateSpace(const std::vector<int> &dof_indices, const std::vector<bool>& is_continuous);

        /**
         * @return A state of type RobotState
         */
        virtual ompl::base::State* allocState() const;

        /**
         * Set the upper/lower bounds of the state space.
         */
        void setBounds(const ompl::base::RealVectorBounds& bounds);

    private:
        std::vector<int> _indices;
        std::vector<bool> _isContinuous;

    };

    typedef boost::shared_ptr<RobotStateSpace> RobotStateSpacePtr;
}

#endif
