#ifndef OMPLCONVERSIONS_H_
#define OMPLCONVERSIONS_H_
#include <ompl/util/Console.h>
#include <or_ompl/RobotStateSpace.h>
#include <openrave/openrave.h>
#include "OMPLPlannerParameters.h"

typedef boost::shared_ptr<ompl::base::RealVectorStateSpace> RealVectorSpacePtr;

namespace or_ompl {

struct OpenRAVEHandler : public ompl::msg::OutputHandler {
public:
    virtual void log(std::string const &text, ompl::msg::LogLevel level,
                     char const *filename, int line);
};

RobotStateSpacePtr CreateStateSpace(OpenRAVE::RobotBasePtr const robot,
                                    OMPLPlannerParameters const &params);

}

#endif
