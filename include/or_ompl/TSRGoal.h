#ifndef TSR_GOAL_H_
#define TSR_GOAL_H_

#include "TSR.h"
#include "TSRChain.h"
#include <openrave-core.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>


#include <boost/make_shared.hpp>

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
		 * @param tsr The TSR chain describe the goal region
		 * @param robot The robot to sample a pose for
		 */
		TSRGoal(const ompl::base::SpaceInformationPtr &si,
				const TSR::Ptr &tsr,
				OpenRAVE::RobotBasePtr robot);

		/**
		 * Constructor
		 *
		 * @param si The space information object describing the space where the algorithm will be used
		 * @param tsrchain The TSR chain describe the goal region
		 * @param robot The robot to sample a pose for
		 */
		TSRGoal(const ompl::base::SpaceInformationPtr &si,
				const TSRChain::Ptr &tsrchain,
				OpenRAVE::RobotBasePtr robot);

		/**
		 * Constructor
		 *
		 * @param si The space information object describing the space where the algorithm will be used
		 * @param tsrchains The list of TSR chain describe the goal region
		 * @param robot The robot to sample a pose for
		 */
		TSRGoal(const ompl::base::SpaceInformationPtr &si,
				const std::vector<TSRChain::Ptr> &tsrchains,
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
		std::vector<TSRChain::Ptr> _tsr_chains;
		OpenRAVE::RobotBasePtr _robot;
	};
      
}
#endif
