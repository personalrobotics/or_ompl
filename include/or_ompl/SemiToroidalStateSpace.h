/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Chris Dellin <cdellin@gmail.com>

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

namespace or_ompl {

/*! \brief Semi-torus OMPL state space for wrapping dimensions.
 * 
 * This is a generalization of ompl::base::RealVectorStateSpace
 * which supports wrapping on each dimension individually
 * (that is, the dimension's lower bound and upper bound correspond).
 * This is especially useful for handling robots with circular joints.
 * It should be funcionally equivalent to building a coupound state
 * space with many SO(2) components, except it should be faster because
 * all states are stored contiguously.
 */
class SemiToroidalStateSpace: public ompl::base::RealVectorStateSpace
{
public:
   SemiToroidalStateSpace(unsigned int dim=0);
   
   virtual void setIsWrapping(const std::vector<bool> &isWrapping);
   virtual const std::vector<bool> & getIsWrapping() const { return isWrapping_; }

   virtual void addDimension(double minBound=0.0, double maxBound=0.0, bool isWrapping=false);
   virtual void addDimension(const std::string &name, double minBound=0.0, double maxBound=0.0, bool isWrapping=false);
   
   virtual double getMaximumExtent() const;
   virtual double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
   virtual bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const;
   virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const;
   
private:
   std::vector<bool> isWrapping_;
};

} // namespace or_ompl
