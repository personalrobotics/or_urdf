#include "urdf_loader.h"
#include <openrave/plugin.h>

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(
        OpenRAVE::InterfaceType type, const std::string& interfacename,
        std::istream& sinput, OpenRAVE::EnvironmentBasePtr env)
{
    if (type == OpenRAVE::PT_Module && interfacename == "urdf") {
        return OpenRAVE::InterfaceBasePtr(new or_urdf::URDFLoader(env));
    } else {
        return OpenRAVE::InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{
    info.interfacenames[OpenRAVE::PT_Planner].push_back("urdf");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
