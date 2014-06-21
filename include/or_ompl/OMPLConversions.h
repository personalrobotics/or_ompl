#ifndef OMPLCONVERSIONS_H_
#define OMPLCONVERSIONS_H_
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <openrave/openrave.h>
#include "OMPLPlannerParameters.h"

typedef boost::shared_ptr<ompl::base::RealVectorStateSpace> RealVectorSpacePtr;

namespace or_ompl {

RealVectorSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                    OMPLPlannerParameters const &params);

}

#endif
