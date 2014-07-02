#include <TSRChain.h>

#include <vector>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <openrave-core.h>

using namespace or_ompl;

TSRChain::TSRChain() : _initialized(false), _sample_start(false), _sample_goal(false), _constrain(false) {

}

TSRChain::TSRChain(const bool &sample_start, 
				   const bool &sample_goal, 
				   const bool &constrain,
				   const std::vector<TSR::Ptr> &tsrs) :
	_initialized(true), _sample_start(sample_start), _sample_goal(sample_goal),
	_constrain(constrain), _tsrs(tsrs){


}

bool TSRChain::deserialize(std::stringstream &ss) {

	ss >> _sample_start;
	ss >> _sample_goal;
	ss >> _constrain;

	int num_tsrs;
	ss >> num_tsrs;

	_tsrs.resize(num_tsrs);
	for(unsigned int idx = 0; idx < num_tsrs; idx++){
		TSR::Ptr new_tsr = boost::make_shared<TSR>();
		new_tsr->deserialize(ss);
		_tsrs[idx] = new_tsr;
	}

	// TODO: Ignored are mmicbody name and mimicbodyjoints	
}

Eigen::Affine3d TSRChain::sample() const {

	Eigen::Affine3d T0_w;
	if(_tsrs.size() == 0){
		RAVELOG_ERROR("[TSRChain] No TSRs specified for this chain");
		return T0_w;
	}

	T0_w = _tsrs.front()->getOriginTransform();
	BOOST_FOREACH(TSR::Ptr tsr, _tsrs){
		T0_w  = T0_w * tsr->sampleDisplacementTransform() * tsr->getEndEffectorOffsetTransform();
	}

	return T0_w;
}

Eigen::Matrix<double, 6, 1> TSRChain::distance(const Eigen::Affine3d &ee_pose) const {

	if(_tsrs.size() == 1){
		TSR::Ptr tsr = _tsrs.front();
		return tsr->distance(ee_pose);
	}else{
		RAVELOG_DEBUG("[TSRChain] Solving IK to compute distance");
		// Things get complicated here.
	}

}
