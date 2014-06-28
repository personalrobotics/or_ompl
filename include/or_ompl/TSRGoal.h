#ifndef TSR_GOAL_H_
#define TSR_GOAL_H_

#include <or_ompl/TSR.h>
#include <openrave-core.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>

namespace or_ompl {
	/**
	 * Implements a goal region defined by a center goal position and a radius
	 */
	class TSRGoal : public ompl::base::GoalSampleableRegion {

	public:
		typedef boost::shared_ptr<TSRGoal> Ptr;

		/**
		 * Constructor
		 *
		 * @param si The space information object describing the space where the algorithm will be used
		 * @param goal The goal
		 * @param tolerance The radius of the goal region
		 */
		TSRGoal(const ompl::base::SpaceInformationPtr &si,
				const TSR::Ptr &tsr,
				OpenRAVE::RobotBasePtr robot);

		/**
		 * Destructor
		 */
		~TSRGoal();
            
		/**
		 * Determines if the given state falls within the goal radius
		 *
		 * @param state The state to check
		 * @return True if the state is within the goal radius
		 */
		virtual bool isSatisfied(const ompl::base::State *state) const;
            
		/**
		 * Calculates the distance between the state and the edge of the goal region
		 *
		 * @param state The state
		 * @return The distance to the edge of the goal region (0 if the state is within the goal region)
		 */
		virtual double distanceGoal(const ompl::base::State *state) const;
            
		/**
		 * Samples a state from within the goal region
		 *
		 * @param state The sampled state
		 */
		virtual void sampleGoal(ompl::base::State *state) const;

		/**
		 * @return max int
		 */
		virtual unsigned int maxSampleCount() const;
            
	private:
		TSR::Ptr _tsr;
		OpenRAVE::RobotBasePtr _robot;
	};
      
}
#endif
