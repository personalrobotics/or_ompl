#include <openrave/plugin.h>
#include "OMPLPlanner.h"
#if 0
#include "OMPLModule.h"
#endif

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "ompl")
    {
        return InterfaceBasePtr(new or_ompl::OMPLPlanner(penv));
    }
#if 0
    else if(type == PT_Module && interfacename == "ompl")
    {
        return InterfaceBasePtr(new or_ompl::OMPLModule(penv));
    }
#endif
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Planner].push_back("ompl");
#if 0
    info.interfacenames[PT_Module].push_back("OMPL");
#endif
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

