#ifndef OMPL_TSR_H_
#define OMPL_TSR_H_

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

namespace or_ompl {
	class TSR {

	public: 
		typedef boost::shared_ptr<TSR> Ptr;

		/**
		 * Constructor
		 */
		TSR();

		/**
		 * Constructor
		 *
		 * @param T0_w Transform from world frame to the TSR frame w
		 * @param Tw_e End-effector offset transform in the coordinates of w
		 * @param Bw 6x2 matrix of bounds in the coordinates of w
		 *   bounds are (x, y, z, roll, pitch, yaw - assume RPY Euler angle convention)
		 */
		TSR(const Eigen::Affine3d &T0_w,
			const Eigen::Affine3d &Tw_e,
			const Eigen::Matrix<double, 6, 2> &Bw);

		/**
		 * Deserialize a serialized TSR.  
		 *  
		 * @param ss The stream to read the serialized TSR from
		 */
		bool deserialize(std::stringstream &ss);

		/**
		 * Compute the distance to the TSR
		 *
		 * @param ee_pose The pose of the end-effector in world frame
		 */
		Eigen::Matrix<double, 6, 1> distance(const Eigen::Affine3d &ee_pose) const;

		/**
		 * Compute the displacement to the TSR
		 * 
		 * @param ee_pose The pose of the end-effector in world frame
		 */
		Eigen::Matrix<double, 6, 1> displacement(const Eigen::Affine3d &ee_pose) const;

		/**
		 * Sample a pose from the TSR
		 *
		 * @return The sampled pose
		 */
		Eigen::Affine3d sample(void) const;
			

		/**
		 * Output operator
		 */
		friend std::ostream& operator << (std::ostream &out, const TSR &tsr){
			out << "TSR: " << std::endl;
			out << "\tT0_w: " << tsr._T0_w.matrix() << std::endl;
			out << "\tTw_e: " << tsr._Tw_e.matrix() << std::endl;
			out << "\tBw: " << tsr._Bw << std::endl;

			return out;
		}

	protected:
		Eigen::Affine3d _T0_w;
		Eigen::Affine3d _T0_w_inv;
		Eigen::Affine3d _Tw_e;
		Eigen::Affine3d _Tw_e_inv;
		Eigen::Matrix<double, 6, 2> _Bw;
		bool _initialized;
	};
	   
}

#endif
