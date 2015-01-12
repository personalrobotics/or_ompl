#ifndef OR_OMPL_ROBOT_STATE_SPACE_H_
#define OR_OMPL_ROBOT_STATE_SPACE_H_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

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
         * Checks if the given list of indices is in this state.  All values in 
         * the input indices list must be in the state for this to return true.
         * @param indices The dof indices to check
         * @return True if all indices in the input vector are part of this state
         */
        bool containsIndices(const std::vector<int> &indices) const;

        /**
         * Sets the values in the state. 
         * @param dof_values The values, this ordering must match the dof_indices
         */
        void set(const std::vector<double> &dof_values);

        /**
         * Sets the state values for only the given dof indices
         * @param dof_indices The indices
         * @param dof_values The partial state
         */
        void setPartial(const std::vector<int> &dof_indices,
                        const std::vector<double> &dof_values);

        /**
         * @return The values in the state
         */
        std::vector<double> getValues() const;

        /**
         * @return An ordered list of indices this state corresponds to
         */
        std::vector<int> getIndices() const { return _dof_indices; }

        /**
         * Overload the output operator
         */
        friend std::ostream& operator <<(std::ostream &os, const RobotState &st) {
            std::vector<double> st_vals = st.getValues();
            std::vector<int> st_inds = st.getIndices();

            os << " [";
            for(unsigned int idx=0; idx < st_vals.size(); idx++){
                os << " " << st_vals[idx] << " ";
                if(idx != st_vals.size() - 1){
                    os << ",";
                }
            }
            os << "] (indices: [";
            for(unsigned int idx=0; idx < st_inds.size(); idx++){
                os << " " << st_inds[idx] << " ";
                if(idx != st_inds.size() - 1){
                    os << ",";
                }
            }    
            os << "]";
            return os;
        }

    private:
        std::vector<int> _dof_indices;

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
