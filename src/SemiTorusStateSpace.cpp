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

#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <or_ompl/SemiTorusStateSpace.h>

or_ompl::SemiTorusStateSpace::SemiTorusStateSpace(unsigned int dim):
   ompl::base::RealVectorStateSpace(dim),
   isWrapping_(dim, false)
{
}

void or_ompl::SemiTorusStateSpace::setIsWrapping(const std::vector<bool> &isWrapping)
{
   if (isWrapping_.size() != dimension_)
      throw ompl::Exception("IsWrapping does not match dimension of state space: expected dimension " +
         boost::lexical_cast<std::string>(dimension_) + " but got dimension " +
         boost::lexical_cast<std::string>(isWrapping_.size()));
   isWrapping_ = isWrapping;
}

void or_ompl::SemiTorusStateSpace::addDimension(double minBound, double maxBound, bool isWrapping)
{
   ompl::base::RealVectorStateSpace::addDimension(minBound,maxBound);
   isWrapping_.push_back(isWrapping);
}

void or_ompl::SemiTorusStateSpace::addDimension(const std::string &name, double minBound, double maxBound, bool isWrapping)
{
   addDimension(minBound, maxBound, isWrapping);
   setDimensionName(dimension_ - 1, name);
}

double or_ompl::SemiTorusStateSpace::getMaximumExtent() const
{
   double e = 0.0;
   for (unsigned int i=0 ; i<dimension_ ; ++i)
   {
      double d = bounds_.high[i] - bounds_.low[i];
      if (isWrapping_[i])
         d *= 0.5;
      e += d*d;
   }
   return sqrt(e);
}

double or_ompl::SemiTorusStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
   double dist = 0.0;
   const double *s1 = static_cast<const StateType*>(state1)->values;
   const double *s2 = static_cast<const StateType*>(state2)->values;

   for (unsigned int i=0; i<dimension_; ++i)
   {
      double diff;
      diff = (*s1++) - (*s2++);
      if (isWrapping_[i])
      {
         double diam = bounds_.high[i] - bounds_.low[i];
         if (diff > 0.5*diam)
            diff = diam - diff;
         else if (-diff > 0.5*diam)
            diff = diam + diff;
      }
      dist += diff * diff;
   }
   return sqrt(dist);
}

bool or_ompl::SemiTorusStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const
{
   const double *s1 = static_cast<const StateType*>(state1)->values;
   const double *s2 = static_cast<const StateType*>(state2)->values;
   for (unsigned int i=0; i<dimension_; ++i)
   {
      double diff = (*s1++) - (*s2++);
      if (fabs(diff) > std::numeric_limits<double>::epsilon() * 2.0)
      {
         if (!isWrapping_[i])
            return false;
         double diam = bounds_.high[i] - bounds_.low[i];
         if (fabs(diff) - diam > std::numeric_limits<double>::epsilon() * 2.0)
            return false;
      }
   }
   return true;
}

void or_ompl::SemiTorusStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const
{
   const StateType *rfrom = static_cast<const StateType*>(from);
   const StateType *rto = static_cast<const StateType*>(to);
   const StateType *rstate = static_cast<StateType*>(state);
   for (unsigned int i=0; i < dimension_ ; ++i)
   {
      if (!isWrapping_[i])
      {
         rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
      }
      else
      {
         
         double diam = bounds_.high[i] - bounds_.low[i];
         double from = rfrom->values[i];
         double to = rto->values[i];
         double diff = to - from;
         if (diff > 0.5*diam)
         {
            rstate->values[i] = rfrom->values[i] + ((rto->values[i]-diam) - rfrom->values[i]) * t;
            if (rstate->values[i] < bounds_.low[i])
               rstate->values[i] += diam;
         }
         else if (-diff > 0.5*diam)
         {
            rstate->values[i] = rfrom->values[i] + ((rto->values[i]+diam) - rfrom->values[i]) * t;
            if (rstate->values[i] > bounds_.high[i])
               rstate->values[i] -= diam;
         }
         else
         {
            rstate->values[i] = rfrom->values[i] + (rto->values[i] - rfrom->values[i]) * t;
         }
      }
   }
}
