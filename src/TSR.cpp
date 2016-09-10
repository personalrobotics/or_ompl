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

#include <vector>
#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>
#include <or_ompl/TSR.h>

using namespace or_ompl;

TSR::TSR()
    : _initialized(false)
{
}

TSR::TSR(
        const Eigen::Affine3d &T0_w,
        const Eigen::Affine3d &Tw_e,
        const Eigen::Matrix<double, 6, 2> &Bw)
    : _T0_w(T0_w)
    , _Tw_e(Tw_e)
    , _Bw(Bw)
    , _manipulator_index(-1)
    , _relative_body_name("NULL")
    , _relative_link_name("")
    , _initialized(true)
{
    _T0_w_inv = _T0_w.inverse();
    _Tw_e_inv = _Tw_e.inverse();
}

bool TSR::deserialize(std::stringstream &ss)
{
    return deserialize(static_cast<std::istream &>(ss));
}

bool TSR::deserialize(std::istream &ss)
{
    // Set _initialized to false in case an error occurs.
    _initialized = false;

    ss >> _manipulator_index
       >> _relative_body_name;

    if(_relative_body_name != "NULL")
        ss >> _relative_link_name;

    // Read in the T0_w matrix
    for(unsigned int c=0; c < 3; c++)
    for(unsigned int r=0; r < 3; r++)
        ss >> _T0_w.matrix()(r,c);

    for(unsigned int idx=0; idx < 3; idx++)
        ss >> _T0_w.translation()(idx);

    // Read in the Tw_e matrix
    for(unsigned int c=0; c < 3; c++)
    for(unsigned int r=0; r < 3; r++)
        ss >> _Tw_e.matrix()(r,c);

    for(unsigned int idx=0; idx < 3; idx++)
        ss >> _Tw_e.translation()(idx);

    // Read in the Bw matrix
    for(unsigned int r=0; r < 6; r++)
    for(unsigned int c=0; c < 2; c++)
        ss >> _Bw(r,c);

    // Check for an error.
    if (!ss)
        return false;

    _T0_w_inv = _T0_w.inverse();
    _Tw_e_inv = _Tw_e.inverse();
    _initialized = true;

    return true;
}

void TSR::serialize(std::ostream& ss)
{
    if (!_initialized)
        throw std::runtime_error("TSR is not initialized.");

    ss << _manipulator_index
       << ' ' << _relative_body_name;

    if(_relative_body_name != "NULL")
        ss << ' ' << _relative_link_name;

    // T0_w matrix
    for(unsigned int c=0; c < 3; c++)
    for(unsigned int r=0; r < 3; r++)
        ss << ' ' << _T0_w.matrix()(r, c);

    for(unsigned int idx=0; idx < 3; idx++)
        ss << ' ' << _T0_w.translation()(idx);

    // Tw_e matrix
    for(unsigned int c=0; c < 3; c++)
    for(unsigned int r=0; r < 3; r++)
        ss << ' ' << _Tw_e.matrix()(r, c);

    for(unsigned int idx=0; idx < 3; idx++)
        ss << ' ' << _Tw_e.translation()(idx);

    // Read in the Bw matrix
    for(unsigned int r=0; r < 6; r++)
    for(unsigned int c=0; c < 2; c++)
        ss << ' ' << _Bw(r, c);
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
