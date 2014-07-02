#include <TSR.h>
#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>
#include <vector>

using namespace or_ompl;

TSR::TSR() : _initialized(false) {

}

TSR::TSR(const Eigen::Affine3d &T0_w, const Eigen::Affine3d &Tw_e, const Eigen::Matrix<double, 6, 2> &Bw) :
	_T0_w(T0_w), _Tw_e(Tw_e), _Bw(Bw), _initialized(true) {

	_T0_w_inv = _T0_w.inverse();
	_Tw_e_inv = _Tw_e.inverse();

}

bool TSR::deserialize(std::stringstream &ss) {

	// TODO: Do we need this stuff? 
	int manipind_ignored;
    ss >> manipind_ignored;

	std::string relativebodyname_ignored;
    ss >> relativebodyname_ignored;
   
    if( relativebodyname_ignored != "NULL" )
    {
		std::string relativelinkname_ignored;
        ss >> relativelinkname_ignored;  
    }  
    
	// Read in the T0_w matrix 
	double tmp;
	for(unsigned int c=0; c < 3; c++){
		for(unsigned int r=0; r < 3; r++){
			ss >> tmp;
			_T0_w.matrix()(r,c) = tmp;
		}
	}

	for(unsigned int idx=0; idx < 3; idx++){
		ss >> tmp;
		_T0_w.translation()(idx) = tmp;
	}	

	// Read in the Tw_e matrix 
	for(unsigned int c=0; c < 3; c++){
		for(unsigned int r=0; r < 3; r++){
			ss >> tmp;
			_Tw_e.matrix()(r,c) = tmp;
		}
	}

	for(unsigned int idx=0; idx < 3; idx++){
		ss >> tmp;
		_Tw_e.translation()(idx) = tmp;
	}

	// Read in the Bw matrix 
	for(unsigned int r=0; r < 6; r++){
		for(unsigned int c=0; c < 2; c++){
			ss >> tmp;
			_Bw(r,c) = tmp;
		}
	}

	_T0_w_inv = _T0_w.inverse();
	_Tw_e_inv = _Tw_e.inverse();

	_initialized = true;

    return _initialized;
}


Eigen::Matrix<double, 6, 1> TSR::distance(const Eigen::Affine3d &ee_pose) const {
	Eigen::Matrix<double, 6, 1> dist = Eigen::Matrix<double, 6, 1>::Zero();

	// First compute the pose of the w frame in world coordinates, given the ee_pose
	Eigen::Affine3d w_in_world = ee_pose * _Tw_e_inv;
	
	// Next compute the pose of the w frame relative to its original pose (as specified by T0_w)
	Eigen::Affine3d w_offset = _T0_w_inv * w_in_world;

	// Now compute the elements of the distance matrix
	dist(0,0) = w_offset.translation()(0);
	dist(1,0) = w_offset.translation()(1);
	dist(2,0) = w_offset.translation()(2);
	dist(3,0) = atan2(w_offset.rotation()(2,1), w_offset.rotation()(2,2));
	dist(4,0) = -asin(w_offset.rotation()(2,0));
	dist(5,0) = atan2(w_offset.rotation()(1,0), w_offset.rotation()(0,0));

	return dist;
}

Eigen::Matrix<double, 6, 1> TSR::displacement(const Eigen::Affine3d &ee_pose) const {

	Eigen::Matrix<double, 6, 1> dist = distance(ee_pose);
	Eigen::Matrix<double, 6, 1> disp = Eigen::Matrix<double, 6, 1>::Zero();

	for(unsigned int idx=0; idx < 6; idx++){
		
		if(dist(idx,0) < _Bw(idx,0)){
			disp(idx,0) = dist(idx,0) - _Bw(idx,0);
		}else if(dist(idx,0) > _Bw(idx,1)){
			disp(idx,0) = dist(idx,0) - _Bw(idx,1);
		}
	}

	return disp;
}

Eigen::Affine3d TSR::sampleDisplacementTransform(void) const {

	// First sample uniformly betwee each of the bounds of Bw
	std::vector<double> d_sample(6);
	
	ompl::RNG rng;
	for(unsigned int idx=0; idx < d_sample.size(); idx++){
		if(_Bw(idx,1) > _Bw(idx,0)){
			d_sample[idx] = rng.uniformReal(_Bw(idx,0), _Bw(idx,1)); 		
		}
	}

	Eigen::Affine3d return_tf;
	return_tf.translation() << d_sample[0], d_sample[1], d_sample[2];

	// Convert to a transform matrix
	double roll = d_sample[3];
	double pitch = d_sample[4];
	double yaw = d_sample[5];

	double A = cos(yaw);
	double B = sin(yaw);
	double C = cos(pitch);
	double D = sin(pitch);
	double E = cos(roll);
	double F = sin(roll);
	return_tf.linear() << A*C, A*D*F - B*E, B*F + A*D*E,
		B*C, A*E + B*D*F, B*D*E - A*F,
		-D, C*F, C*E;

	return return_tf;
}

Eigen::Affine3d TSR::sample() const {

	Eigen::Affine3d tf = sampleDisplacementTransform(); 
	
	return _T0_w * tf * _Tw_e;
}
