#include <TSRChain.h>

#include <vector>
#include <boost/make_shared.hpp>

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
