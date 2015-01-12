#include "TSRGoal.h"
#include "RobotStateSpace.h"

#include <boost/foreach.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>

using namespace or_ompl;
namespace ob = ompl::base;

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
				 const TSR::Ptr &tsr, 
				 OpenRAVE::RobotBasePtr robot)
    : ob::GoalSampleableRegion(si),  _robot(robot){
    
	std::vector<TSR::Ptr> tsrs(1);
	tsrs.push_back(tsr);
	TSRChain::Ptr tsrchain = boost::make_shared<TSRChain>(true, false, false, tsrs);
	_tsr_chains.push_back(tsrchain);
}

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
				 const TSRChain::Ptr &tsrchain, 
				 OpenRAVE::RobotBasePtr robot)
    : ob::GoalSampleableRegion(si), _robot(robot){

	_tsr_chains.push_back(tsrchain);
}

TSRGoal::TSRGoal(const ob::SpaceInformationPtr &si,
				 const std::vector<TSRChain::Ptr> &tsrchains, 
				 OpenRAVE::RobotBasePtr robot)
    : ob::GoalSampleableRegion(si), _tsr_chains(tsrchains), _robot(robot){
    
}

TSRGoal::~TSRGoal() {
    
}

bool TSRGoal::isSatisfied(const ompl::base::State *state) const {

    bool satisfied = (distanceGoal(state) == 0.0);
	return satisfied;
}
            
double TSRGoal::distanceGoal(const ompl::base::State *state) const {

	// Save the state of the robot
    OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_robot->GetEnv()->GetMutex());
	OpenRAVE::KinBody::KinBodyStateSaver rsaver(_robot);

	// Put the robot in the pose that is represented in the state
	const RobotState* mstate = state->as<RobotState>();
    unsigned int check_limits = 0; // The planner does this
    _robot->SetDOFValues(mstate->getValues(), check_limits, mstate->getIndices());

	// Get distance to TSR
	double distance = std::numeric_limits<double>::infinity();
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manipulators = _robot->GetManipulators();
	BOOST_FOREACH(TSRChain::Ptr tsrchain, _tsr_chains){    
        
        TSR::Ptr final_tsr = tsrchain->getTSRs().back();

        // Get the manipulator associated with this tsr
        int midx = final_tsr->getManipulatorIndex();
        if(midx < 0 || midx >= manipulators.size()){
            throw OpenRAVE::openrave_exception(
                "Invalid manipulator index in TSR.",
                OpenRAVE::ORE_Failed
                );
        }

        OpenRAVE::RobotBase::ManipulatorPtr manip = manipulators[midx];

        // Get the end effector transform
        OpenRAVE::Transform or_tf = manip->GetEndEffectorTransform();
        OpenRAVE::TransformMatrix or_matrix(or_tf);
	
        // Convert to Eigen
        Eigen::Affine3d ee_pose = Eigen::Affine3d::Identity();
        ee_pose.linear() << or_matrix.m[0], or_matrix.m[1], or_matrix.m[2],
            or_matrix.m[4], or_matrix.m[5], or_matrix.m[6],
            or_matrix.m[8], or_matrix.m[9], or_matrix.m[10];
        ee_pose.translation() << or_matrix.trans.x, or_matrix.trans.y, or_matrix.trans.z;


		Eigen::Matrix<double, 6, 1> ee_distance = tsrchain->distance(ee_pose);
		double tdistance = ee_distance.norm();
		if(tdistance < distance){
			distance = tdistance;
		}
	}

	// Reset the state of the robot
	rsaver.Restore();

	return distance;
}
            
void TSRGoal::sampleGoal(ompl::base::State *state) const {

	bool success = false;

    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manipulators = _robot->GetManipulators();

    // Grab the state so we can get some info from it
    RobotState* mstate = state->as<RobotState>();

	// TODO: Figure out how to bail correctly if an IK isn't found
	//for(unsigned int count=0; count < 20 && !success; count++){
    for(unsigned int count=0; count < 2 && !success; count++){
        // We need a set of flags to tell us when we have already set the manipulator
        //  value
        std::vector<bool> ik_found(manipulators.size(), false); 

		// Pick a TSR to sample
		int tsr_idx = 0;
		if(_tsr_chains.size() > 1){
			ompl::RNG rng;
			tsr_idx = rng.uniformInt(0, _tsr_chains.size()-1);
		}

        // Grab the randomly selected chain
        TSRChain::Ptr tsr_chain = _tsr_chains[tsr_idx];

		// Sample the TSR
		Eigen::Affine3d ee_pose = tsr_chain->sample();
                
        // Start at the end and work backwards, sampling for each manipulator that is part of the chain
        std::vector<TSR::Ptr> tsrs = tsr_chain->getTSRs();
        success = true;
        for(int idx = tsrs.size() - 1; idx >= 0 && success; idx--){

            TSR::Ptr tsr = tsrs[idx];

            // Get the manipulator associated with this tsr
            int midx = tsr->getManipulatorIndex();
            if(midx < 0 || midx >= manipulators.size()){
                throw OpenRAVE::openrave_exception(
                    "Invalid manipulator index in TSR.",
                    OpenRAVE::ORE_Failed
                    );
            }
            OpenRAVE::RobotBase::ManipulatorPtr manip = manipulators[midx];

            if(!ik_found[midx] && mstate->containsIndices(manip->GetArmIndices())){

                // Find an associated IK
                OpenRAVE::TransformMatrix or_matrix;
                or_matrix.rotfrommat(
                    ee_pose.matrix()(0, 0), ee_pose.matrix()(0, 1), ee_pose.matrix()(0, 2),
                    ee_pose.matrix()(1, 0), ee_pose.matrix()(1, 1), ee_pose.matrix()(1, 2),
                    ee_pose.matrix()(2, 0), ee_pose.matrix()(2, 1), ee_pose.matrix()(2, 2)
                    );
                or_matrix.trans.x = ee_pose(0, 3);
                or_matrix.trans.y = ee_pose(1, 3);
                or_matrix.trans.z = ee_pose(2, 3);


                OpenRAVE::IkParameterization ik_param(or_matrix, OpenRAVE::IKP_Transform6D);
                std::vector<OpenRAVE::dReal> ik_solution;
                success = manip->FindIKSolution(ik_param, ik_solution, OpenRAVE::IKFO_CheckEnvCollisions);
                if(success){                    
                    mstate->setPartial(manip->GetArmIndices(), ik_solution);

                    // Mark this manip as already having DOF value set
                    ik_found[midx] = true;
                }
            }

            ee_pose = ee_pose * tsr->getEndEffectorOffsetTransform().inverse();
        }
	}

	if(!success){
		RAVELOG_ERROR("[TSRGoal] Failed to sample valid goal.\n");
	}
}

unsigned int TSRGoal::maxSampleCount() const {

    return std::numeric_limits<unsigned int>::max();
}
