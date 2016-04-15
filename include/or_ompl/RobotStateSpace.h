#ifndef OR_OMPL_ROBOT_STATE_SPACE_H_
#define OR_OMPL_ROBOT_STATE_SPACE_H_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

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
        RobotState();

        /**
         * Clear the indices vector
         */
        ~RobotState();
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

        /** \brief Register the projections for this state space. Usually, this is at least the default
            projection. These are implicit projections, set by the implementation of the state space. This is called by setup(). */
        virtual void registerProjections();

        /**
         * Set the upper/lower bounds of the state space.
         */
        void setBounds(const ompl::base::RealVectorBounds& bounds);

        /**
         * @return An ordered list of indices this state corresponds to
         */
        const std::vector<int>& getIndices() const { return _indices; }

    private:
        std::vector<int> _indices;
        std::vector<bool> _isContinuous;
        ompl::base::ProjectionEvaluatorPtr _projectionEvaluator;

    };

    typedef boost::shared_ptr<RobotStateSpace> RobotStateSpacePtr;

    class RobotProjectionEvaluator : public ompl::base::ProjectionEvaluator {
        public:
            RobotProjectionEvaluator(ompl::base::StateSpace* stateSpace);
            RobotProjectionEvaluator(ompl::base::StateSpacePtr stateSpace);
            virtual ~RobotProjectionEvaluator();

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension() const;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const;

            virtual void defaultCellSizes();

            virtual void setup();
        protected:
            or_ompl::RobotStateSpace* _robotStateSpace;
            ompl::base::ProjectionMatrix _projectionMatrix;
    };

    typedef boost::shared_ptr<RobotProjectionEvaluator> RobotProjectionEvaluatorPtr;
}

#endif
