#ifndef OR_OMPL_STATE_SPACES_H_
#define OR_OMPL_STATE_SPACES_H_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <openrave/openrave.h>
#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

namespace or_ompl {
    
    /**
     * Implements a state space for an OpenRAVE robot
     */
    class ContinuousJointsStateSpace : public ompl::base::CompoundStateSpace {


    public:
        /**
         * Constructor
         * @param dof_indices An ordered list of indices this state corresponds to
         */
        ContinuousJointsStateSpace(const std::vector<bool>& is_continuous);

        /** \brief Register the projections for this state space. Usually, this is at least the default
            projection. These are implicit projections, set by the implementation of the state space. This is called by setup(). */
        virtual void registerProjections();

        /**
         * Set the upper/lower bounds of the state space.
         */
        void setBounds(const ompl::base::RealVectorBounds& bounds);

    private:
        std::vector<bool> _isContinuous;
        ompl::base::ProjectionEvaluatorPtr _projectionEvaluator;

    };

    typedef boost::shared_ptr<ContinuousJointsStateSpace> ContinuousJointsStateSpacePtr;

    class ContinuousJointsProjectionEvaluator : public ompl::base::ProjectionEvaluator {
        public:
            ContinuousJointsProjectionEvaluator(ompl::base::StateSpace* stateSpace);
            ContinuousJointsProjectionEvaluator(ompl::base::StateSpacePtr stateSpace);
            virtual ~ContinuousJointsProjectionEvaluator();

            /** \brief Return the dimension of the projection defined by this evaluator */
            virtual unsigned int getDimension() const;

            /** \brief Compute the projection as an array of double values */
            virtual void project(const ompl::base::State *state, ompl::base::EuclideanProjection &projection) const;

            virtual void defaultCellSizes();

            virtual void setup();
        protected:
            or_ompl::ContinuousJointsStateSpace* _robotStateSpace;
            ompl::base::ProjectionMatrix _projectionMatrix;
    };

    typedef boost::shared_ptr<ContinuousJointsProjectionEvaluator> ContinuousJointsProjectionEvaluatorPtr;

    /**
     * This is like ompl::base::StateValidityChecker,
     * except it also knows how to compute forward kinematics
     * to match the state
     * 
     * this should work for general state spaces (e.g. CompoundStateSpaces)
     */
    class OrStateValidityChecker: public ompl::base::StateValidityChecker
    {
    public:
        OrStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
            OpenRAVE::RobotBasePtr robot, std::vector<int> const &indices,
            bool do_baked);
        void start();
        void stop();
        virtual bool computeFk(const ompl::base::State *state, uint32_t checklimits) const;
        virtual bool isValid(const ompl::base::State *state) const;
        void resetStatistics() { m_numCollisionChecks = 0; m_totalCollisionTime = 0.0; }
        int getNumCollisionChecks() { return m_numCollisionChecks; }
        double getTotalCollisionTime() { return m_totalCollisionTime; }
        const std::vector<int> & getIndices() { return m_indices; }
    protected:
        ompl::base::StateSpace * m_stateSpace;
        OpenRAVE::EnvironmentBasePtr m_env;
        OpenRAVE::RobotBasePtr m_robot;
        std::vector<int> const m_indices;
        mutable int m_numCollisionChecks;
        mutable double m_totalCollisionTime;
        // optional baked stuff
        const bool m_do_baked;
        boost::function<void ()> m_bake_begin;
        boost::function<OpenRAVE::KinBodyPtr ()> m_bake_end;
        boost::function<bool (OpenRAVE::KinBodyConstPtr, OpenRAVE::CollisionReportPtr)> m_baked_checker;
        OpenRAVE::KinBodyPtr m_baked_kinbody;
    };

    typedef boost::shared_ptr<OrStateValidityChecker> OrStateValidityCheckerPtr;

    /**
     * StateRobotSetter for RealVectorStateSpaces
     */
    class RealVectorOrStateValidityChecker: public OrStateValidityChecker
    {
    public:
        RealVectorOrStateValidityChecker(const ompl::base::SpaceInformationPtr &si,
            OpenRAVE::RobotBasePtr robot, std::vector<int> const &indices,
            bool do_baked);
        virtual bool computeFk(const ompl::base::State *state, uint32_t checklimits) const;
    private:
        const std::size_t m_num_dof;
    };

}

#endif
