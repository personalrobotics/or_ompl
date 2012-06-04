#include <openrave/plugin.h>
#include "OMPLModule.h"

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "ompl")
    {
        return InterfaceBasePtr(new or_ompl::OMPLPlanner(penv));
    }
    else if(type == PT_Module && interfacename == "ompl")
    {
        return InterfaceBasePtr(new or_ompl::OMPLModule(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Planner].push_back("OMPL");
    info.interfacenames[PT_Module].push_back("OMPL");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

