#!/usr/bin/env python
from __future__ import print_function
import json, sys

factory_frontmatter = """\
#include <map>
#include <string>
#include <boost/assign/list_of.hpp>
{includes:s}
#include "PlannerRegistry.h"

namespace or_ompl {{
namespace registry {{

struct BasePlannerFactory {{
    virtual ~BasePlannerFactory();
    virtual ompl::base::Planner *create(ompl::base::SpaceInformationPtr space) = 0;
}};

/*
 * Planner Factories
 */\
"""
factory_template = """\
struct {name:s}Factory : public virtual BasePlannerFactory {{
    virtual ompl::base::Planner *create(ompl::base::SpaceInformationPtr space)
    {{
        return new {qualified_name:s}(space);
    }}
}};
"""

registry_frontmatter = """\
/*
 * Planner Registry
 */
typedef std::map<std::string, BasePlannerFactory *> PlannerRegistry;

// The dynamic_cast is necessary to work around a type inference bug when
// using map_list_of on a polymorphic type.
static PlannerRegistry registry = boost::assign::map_list_of\
"""
registry_entry = '    ("{name:s}", dynamic_cast<BasePlannerFactory *>(new {name:s}Factory))'
registry_backmatter = """\
    ;

ompl::base::Planner *create(std::string const &name,
                            ompl::base::SpaceInformationPtr space)
{
    PlannerRegistry::const_iterator const it = registry.find(name);
    if (it != registry.end()) {
        return it->second->create(space);
    } else {
        throw std::runtime_error("Unknown planner '" + name + "'.");
    }
}

}
}\
"""

def main():
    planners = json.load(sys.stdin)

    headers = [ planner['header'] for planner in planners ]
    includes = [ '#include <{:s}>'.format(path) for path in headers ]
    print(factory_frontmatter.format(includes='\n'.join(includes)))

    # Generate the factory class implementations.
    names = [ planner['name'] for planner in planners ]
    registry_entries = []

    for qualified_name in names:
        _, _, name = qualified_name.rpartition('::')
        args = { 'name': name,
                 'qualified_name': qualified_name }
        print(factory_template.format(**args))
        registry_entries.append(registry_entry.format(**args))

    # Generate the registry of factory classes.
    print(registry_frontmatter)
    print('\n'.join(registry_entries))
    print(registry_backmatter)


if __name__ == '__main__':
    main()
