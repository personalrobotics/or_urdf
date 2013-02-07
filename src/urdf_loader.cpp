/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include "urdf_loader.h"
#include <urdf/model.h>

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
  bool URDFLoader::load(std::ostream& soutput, std::istream& sinput)
  {
    // Get filename from input arguments
    std::string urdf_filename;
    sinput >> urdf_filename;
    
    // Parse file via URDF reader
    urdf::Model model;
    if (!model.initFile(urdf_filename)) {
      RAVELOG_ERROR("URDFLoader : Unable to open URDF file [%s].\n", urdf_filename.c_str());
      throw OpenRAVE::openrave_exception("Failed to open URDF file!");
    }

    // Recursively construct robot from URDF
    

    // Return reference to created object
    // TODO: return object somehow
    return true;
  }

} /* namespace orcdchomp */
