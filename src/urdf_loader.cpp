/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include "urdf_loader.h"

/** Boilerplate plugin definition for OpenRAVE */
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr env)
{
  if( type == OpenRAVE::PT_Module && interfacename == "urdfloader" ) {
    return OpenRAVE::InterfaceBasePtr(new urdf_loader::URDFLoader(env));
  }
}

/** Boilerplate plugin definition for OpenRAVE */
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
  info.interfacenames[OpenRAVE::PT_Module].push_back("URDFLoader");
}

namespace urdf_loader
{

  /** Opens a URDF file and returns a robot in OpenRAVE */
  bool URDFLoader::load(std::ostream& sout, std::istream& sin)
  {
    // Get filename from input arguments

    // Parse file via XML

    // Recursively construct robot from XML

    // Return reference to created object

    return true;
  }

} /* namespace orcdchomp */
