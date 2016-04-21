/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

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

#ifndef OR_OMPL_OR_CONVERSIONS_H_
#define OR_OMPL_OR_CONVERSIONS_H_

namespace or_ompl {

inline Eigen::Vector3d toEigen3(OpenRAVE::Vector const &or_v) {
    Eigen::Vector3d eigen_v;
    eigen_v << or_v.x, or_v.y, or_v.z;
    return eigen_v;
}

inline Eigen::Affine3d toEigen(OpenRAVE::Transform const &or_tf) {
    OpenRAVE::TransformMatrix or_matrix(or_tf);
    Eigen::Affine3d eigen_tf = Eigen::Affine3d::Identity();
    eigen_tf.linear() << or_matrix.m[0], or_matrix.m[1], or_matrix.m[2],
        or_matrix.m[4], or_matrix.m[5], or_matrix.m[6],
        or_matrix.m[8], or_matrix.m[9], or_matrix.m[10];
    eigen_tf.translation() << or_matrix.trans.x, or_matrix.trans.y, or_matrix.trans.z;
    return eigen_tf;
}

template <typename Derived>
inline OpenRAVE::Vector toOR(Derived const &eigen_v) {
    BOOST_STATIC_ASSERT(Derived::IsVectorAtCompileTime);
    if (eigen_v.size() == 3) {
        return OpenRAVE::Vector(eigen_v[0], eigen_v[1], eigen_v[2]);
    } else if (eigen_v.size() == 4) {
        return OpenRAVE::Vector(eigen_v[0], eigen_v[1], eigen_v[2], eigen_v[3]);
    } else {
        throw std::invalid_argument(boost::str(
                                        boost::format("Expected a three or four element vector; got a %d elements.")
                                        % eigen_v.size()));
    }
}

template <typename Derived>
inline OpenRAVE::Transform toOR(Eigen::Transform<Derived, 3, Eigen::Affine> const &tf) {
    OpenRAVE::TransformMatrix or_matrix;
    or_matrix.rotfrommat(
        tf.matrix()(0, 0), tf.matrix()(0, 1), tf.matrix()(0, 2),
        tf.matrix()(1, 0), tf.matrix()(1, 1), tf.matrix()(1, 2),
        tf.matrix()(2, 0), tf.matrix()(2, 1), tf.matrix()(2, 2)
        );
    or_matrix.trans.x = tf(0, 3);
    or_matrix.trans.y = tf(1, 3);
    or_matrix.trans.z = tf(2, 3);
    return or_matrix;
}

} // namespace or_ompl

#endif // OR_OMPL_OR_CONVERSIONS_H_
