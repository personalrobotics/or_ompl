/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Jennifer King <jeking04@gmail.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#include <boost/make_shared.hpp>

#include <or_ompl/TSRRobot.h>
#include <or_ompl/or_conversions.h>

using namespace or_ompl;

TSRRobot::TSRRobot(const std::vector<TSR::Ptr> &tsrs, const OpenRAVE::EnvironmentBasePtr &penv)
    : _tsrs(tsrs), _penv(penv), _initialized(false), _solver("GeneralIK") {

}

bool TSRRobot::construct() {

    if(_initialized){
        RAVELOG_ERROR("[TSRRobot] Already initialized. TSRRobot::construct cannot be called twice.");
        throw OpenRAVE::openrave_exception(
            "TSRRobot::construct cannot be called twice",
            OpenRAVE::ORE_Failed
            );
    }

    _initialized = false;

    // Create an emtpy robot of the correct type
    _probot = RaveCreateRobot(_penv, "GenericRobot");
    if( _probot.get() == NULL ){
        RAVELOG_ERROR("[TSRRobot] Failed to create robot of type GenericRobot");
        return _initialized;
    }

    // TODO: mimic body

    // Build the robot
    std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos;
    std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos;
    std::vector<OpenRAVE::RobotBase::ManipulatorInfoConstPtr> manip_infos;
    std::vector<OpenRAVE::RobotBase::AttachedSensorInfoConstPtr> sensor_infos;

    const std::string bodyprefix = "Body";
    int bodynumber = 1;
    Eigen::Affine3d Tw0_e = Eigen::Affine3d::Identity();
       
    for(unsigned int i=0; i < _tsrs.size(); i++){

        TSR::Ptr tsr = _tsrs[i];
        Eigen::Matrix<double, 6, 2> Bw = tsr->getBounds();

        if (tsr->relative_body_name() != "NULL")
        {
            RAVELOG_ERROR("[TSRRobot] ERROR: TSRs relative to a body is not supported.\n");
            return _initialized;
        }
        
        for(int j=0; j < 6; j++){
            
            // Don't add a body if there is no freedom in this dimension
            if(Bw(j,0) == 0.0 && Bw(j,1) == 0.0){
                continue;
            }

            // If the bounds are equal and non-zero, we should do something reasonable
            //  For now, this isn't supported
            if(Bw(j,0) == Bw(j,1)){
                RAVELOG_ERROR("[TSRRobot] ERROR: TSR Chains are currently unable to deal with cases where two bounds are equal but non-zero, cannot robotize.\n");
                return _initialized;
            }

            // Check for axis flip, marked by Bw values being backwards
            bool bFlipAxis = false;
            if(Bw(j,0) > Bw(j,1)){
                Bw(j,0) = -Bw(j,0);
                Bw(j,1) = -Bw(j,1);

                bFlipAxis = true;
            }
            
            // TODO: Handle mimic joints

            // Store joint limits
            _lowerlimits.push_back(Bw(j,0));
            _upperlimits.push_back(Bw(j,1));
            
            // Create a Link
            std::string prev_bodyname = (boost::format("%s%d") % bodyprefix % (bodynumber-1)).str();
            std::string bodyname = (boost::format("%s%d") % bodyprefix % bodynumber).str();
            OpenRAVE::KinBody::LinkInfoPtr link_info
                = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();
            link_info->_name = bodyname; 
            link_info->_t = toOR<double>(Tw0_e); // transform
            

            OpenRAVE::KinBody::GeometryInfoPtr geom_info
                = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            if(j < 3){
                geom_info->_type = OpenRAVE::GT_Box;
            }else{
                geom_info->_type = OpenRAVE::GT_Cylinder;
                geom_info->_vGeomData = OpenRAVE::Vector(0.03, 0.07, 0.0); //cylinder radius, height, ignored
            }

            if(j == 0){
                geom_info->_vGeomData = OpenRAVE::Vector(0.04, 0.02, 0.02); // box extents
            }else if(j == 1){
                geom_info->_vGeomData = OpenRAVE::Vector(0.02, 0.04, 0.02); // box extents
            }else if(j == 2){
                geom_info->_vGeomData = OpenRAVE::Vector(0.02, 0.02, 0.04); // box extents
            }else if(j == 3){
                OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> t 
                    = OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(0, 0, 1), 90.);
                geom_info->_t = t;
            }else if(j == 4){
                OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> t 
                    = OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(0, 1, 0), 90.);
                geom_info->_t = t;
            }else if(j == 5){
                OpenRAVE::RaveTransformMatrix<OpenRAVE::dReal> t 
                    = OpenRAVE::geometry::matrixFromAxisAngle(OpenRAVE::Vector(1, 0, 0), 90.);
                geom_info->_t = t;
            }

            geom_info->_vDiffuseColor = OpenRAVE::Vector(0.3, 0.7, 0.3);
            link_info->_vgeometryinfos.push_back(geom_info);
            link_infos.push_back(link_info);

            // Now create a joint
            OpenRAVE::KinBody::JointInfoPtr joint_info 
                = boost::make_shared<OpenRAVE::KinBody::JointInfo>();
            std::string joint_name = (boost::format("J%d") % bodynumber).str();
            joint_info->_name = joint_name;
            if(j < 3){
                joint_info->_type = OpenRAVE::KinBody::JointSlider;
            }else{
                joint_info->_type = OpenRAVE::KinBody::JointHinge;
            }
            joint_info->_linkname0 = prev_bodyname;
            joint_info->_linkname1 = bodyname;
            joint_info->_vweights[0] = 1.;
            joint_info->_vmaxvel[0] = 1.;
            joint_info->_vresolution[0] = 1.;
            joint_info->_vlowerlimit[0] = Bw(j,0);
            joint_info->_vupperlimit[0] = Bw(j,1);

            joint_info->_vaxes[0] = OpenRAVE::Vector(0., 0., 0.);
            unsigned int aidx = (j % 3);
            if(j > 3 && bFlipAxis){
                joint_info->_vaxes[0][aidx] = -1.;
            }else{
                joint_info->_vaxes[0][aidx] = 1.;
            }
            joint_infos.push_back(joint_info);

            bodynumber++;
        }

        Tw0_e = Tw0_e * tsr->getEndEffectorOffsetTransform();
    }
    _num_dof = bodynumber - 1;

    // now add a geometry to the last body with the offset of the last TSR, this will be the target for the manipulator 
    TSR::Ptr last_tsr = _tsrs.back();
    Tw0_e = last_tsr->getEndEffectorOffsetTransform();

    OpenRAVE::KinBody::LinkInfoPtr link_info
        = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();
    std::string bodyname = (boost::format("%s%d") % bodyprefix % (bodynumber-1)).str();
    link_info->_name = bodyname;
    link_info->_bStatic = false;
    
    OpenRAVE::KinBody::GeometryInfoPtr geom_info
        = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
    geom_info->_t = toOR<double>(Tw0_e);
    geom_info->_type = OpenRAVE::GT_Sphere;
    geom_info->_vGeomData = OpenRAVE::Vector(0.03, 0., 0.); //radius, ignored, ignored
    geom_info->_vDiffuseColor = OpenRAVE::Vector(0.3, 0.7, 0.3);
    link_info->_vgeometryinfos.push_back(geom_info);
    link_infos.push_back(link_info);

    if(bodynumber > 1){
        _point_tsr = false;
        
        OpenRAVE::RobotBase::ManipulatorInfoPtr manip_info
            = boost::make_shared<OpenRAVE::RobotBase::ManipulatorInfo>();
        manip_info->_name = "dummy";
        manip_info->_sBaseLinkName = (boost::format("%s0") % bodyprefix).str();
        manip_info->_sEffectorLinkName = bodyname;
        manip_infos.push_back(manip_info);
    }else{
        _point_tsr = true;
        RAVELOG_DEBUG("[TSRRobot] This is a point TSR, no robotized TSR needed.");
        _initialized = true;
        return _initialized;
    }

    if(_point_tsr && _tsrs.size() != 1){
        RAVELOG_ERROR("[TSRRobot] Can't yet handle case where the TSRChain has no freedom but multiple TSRs, try making it a chain of length 1.\n");
        _initialized = false;
        return _initialized;
    }

    // If we made it this far, then we can build the robot.
    _probot->Init(link_infos, joint_infos, manip_infos, sensor_infos);

    // Set the name properly
    std::string robotname = (boost::format("TSRChain%lu") % (unsigned long int) this).str();
    _probot->SetName(robotname);

    // Add it to the environment
    _penv->Add(_probot, true);

    // Set the pose
    // TODO: mimic joint stuff
    _probot->SetTransform(toOR<double>(_tsrs[0]->getOriginTransform()));

    // Create an IK Solver
    _ik_solver = OpenRAVE::RaveCreateIkSolver(_penv, _solver);
    if(_ik_solver.get() == NULL){
        RAVELOG_ERROR("[TSRRobot] Cannot create IK solver, make sure you have the GeneralIK plugin loadable by OpenRAVE\n");
        _initialized = false;
        return _initialized;
    }

    // Grab the active manipulator on our newly created robot
    OpenRAVE::RobotBase::ManipulatorPtr pmanip = _probot->GetActiveManipulator();
    _ik_solver->Init(pmanip);

    // Finally, disable the robot so we don't do collision checking against it
    _probot->Enable(false);
    _initialized = true;
    
    return _initialized;
}

Eigen::Affine3d TSRRobot::findNearestFeasibleTransform(const Eigen::Affine3d &Ttarget) {

    OpenRAVE::Transform or_target = toOR<double>(Ttarget);

    if(_solver.compare("GeneralIK") != 0){
        RAVELOG_ERROR("[TSRRobot] Only GeneralIK solver supported.");
        throw OpenRAVE::openrave_exception(
            "Only GeneralIK solver supported.",
            OpenRAVE::ORE_Failed
            );
    }

    // Setup the free parameters - the format and meaning of these is defined directly by 
    //  the IK solver - in our case GeneralIK
    std::vector<OpenRAVE::dReal> ikfreeparams; 

    ikfreeparams.resize(12);
    ikfreeparams[0] = 1; // The number of targets - in this case always 1
    ikfreeparams[1] = 0; // The manipulator associated the target - only one manipulator to always 0
    
    // Pose of target
    ikfreeparams[2] = or_target.rot.x; 
    ikfreeparams[3] = or_target.rot.y;
    ikfreeparams[4] = or_target.rot.z;
    ikfreeparams[5] = or_target.rot.w;
    ikfreeparams[6] = or_target.trans.x;
    ikfreeparams[7] = or_target.trans.y;
    ikfreeparams[8] = or_target.trans.z;

    
    ikfreeparams[9] = 0;  // no balancing
    ikfreeparams[10] = 0; // junk parameters - mode in previous versions of GeneralIK
    ikfreeparams[11] = 0; // not translation only - aka do rotation

    std::vector<OpenRAVE::dReal> q0;
    boost::shared_ptr<std::vector<OpenRAVE::dReal> > solution
        = boost::make_shared<std::vector<OpenRAVE::dReal> >();

    // solve ik
    _ik_solver->Solve(OpenRAVE::IkParameterization(), q0, ikfreeparams, 
                      OpenRAVE::IKFO_IgnoreSelfCollisions, solution);

    // Set the dof values to the solution and grab the end-effector transform in world coordinates
    _probot->SetDOFValues(*solution);
    Eigen::Affine3d ee_pose = toEigen(_probot->GetActiveManipulator()->GetEndEffectorTransform());

    // Convert to proper frame
    Eigen::Affine3d closest = ee_pose * _tsrs.back()->getEndEffectorOffsetTransform();
    return closest;
}
