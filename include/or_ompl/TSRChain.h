#ifndef OMPL_TSR_CHAIN_H_
#define OMPL_TSR_CHAIN_H_

#include <TSR.h>

#include <vector>
#include <boost/shared_ptr.hpp>

namespace or_ompl {
	class TSRChain {

	public:
		typedef boost::shared_ptr<TSRChain> Ptr;

		/**
		 * Constructor
		 */
		TSRChain();
		
		/**
		 * Constructor
		 *
		 * @param sample_start True if the chain should be used to sample the start configuration
		 * @param sample_goal True if the chain should be used to sample the start configuration
		 * @param constrain True if the chain should be applied trajectory wide
		 * @param tsrs The list of TSRs in the chain
		 */
		TSRChain(const bool &sample_start, 
				 const bool &sample_goal, 
				 const bool &constrain,
				 const std::vector<TSR::Ptr> &tsrs);

		/**
		 * Deserialize a serialized TSR Chain.  
		 *  
		 * @param ss The stream to read the serialized TSR from
		 */
		bool deserialize(std::stringstream &ss);

		/**
		 * @return True if this chain should be used to sample a start
		 */
		bool sampleStart() const { return _sample_start; }

		/**
		 * @return True if this chain should be used to sample a goal
		 */
		bool sampleGoal() const { return _sample_goal; }

		/**
		 * @return True if this chain should be applied as a trajectory wide constraint
		 */
		bool isTrajectoryConstraint() const { return _constrain; }
		

		/**
		 * @return The list of TSRs that make up this chain
		 */
		std::vector<TSR::Ptr> getTSRs() const { return _tsrs; }

		/**
		 * @return A sample from the TSR chain
		 */
		Eigen::Affine3d sample(void) const;

		/**
		 * Compute the distance to the TSR
		 *
		 * @param ee_pose The pose of the end-effector in world frame
		 */
		Eigen::Matrix<double, 6, 1> distance(const Eigen::Affine3d &ee_pose) const;

	private:
		bool _initialized;
		bool _sample_start;
		bool _sample_goal;
		bool _constrain;
		std::vector<TSR::Ptr> _tsrs;
	};
	   
}

#endif
