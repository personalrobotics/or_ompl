#ifndef PLANNERREGISTRY_H_
#define PLANNERREGISTRY_H_
#include <string>
#include <ompl/base/Planner.h>
#include <ompl/base/SpaceInformation.h>

namespace or_ompl {
namespace registry {

ompl::base::Planner *create(std::string const &name,
                            ompl::base::SpaceInformationPtr space);

}
}
#endif
