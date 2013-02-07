/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include "urdf_loader.h"

#include <tinyxml.h>
#include <urdf/model.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

/** Boilerplate plugin definition for OpenRAVE */
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr env)
{
  if( type == OpenRAVE::PT_Module && interfacename == "urdfloader" ) {
    return OpenRAVE::InterfaceBasePtr(new urdf_loader::URDFLoader(env));
  } else {
    return OpenRAVE::InterfaceBasePtr();
  }
}

/** Boilerplate plugin definition for OpenRAVE */
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
  info.interfacenames[OpenRAVE::PT_Module].push_back("URDFLoader");
}

OpenRAVE::Transform URDFPoseToRaveTransform(const urdf::Pose &pose)
{
  return OpenRAVE::Transform( OpenRAVE::Vector(pose.rotation.x, 
					       pose.rotation.y, 
					       pose.rotation.z, 
					       pose.rotation.w),
			      OpenRAVE::Vector(pose.position.x, 
					       pose.position.y, 
					       pose.position.z) );
}

OpenRAVE::KinBody::Joint::JointType URDFJointTypeToRaveJointType(int type)
{
  switch(type) {
  case urdf::Joint::REVOLUTE:
    return OpenRAVE::KinBody::Joint::JointRevolute;
  case urdf::Joint::PRISMATIC:
    return OpenRAVE::KinBody::Joint::JointPrismatic;
  case urdf::Joint::FIXED:
  case urdf::Joint::PLANAR:
  case urdf::Joint::FLOATING:
  case urdf::Joint::CONTINUOUS:
  case urdf::Joint::UNKNOWN:
  default:
    // TODO: Fill the rest of these joint types in!
    RAVELOG_ERROR("URDFLoader : Unable to determine joint type [%d].\n", type);
    throw OpenRAVE::openrave_exception("Failed to convert URDF joint!");
  }
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

    // Create new XML document internally
    //
    // NOTE: Some of you may ask, "Why are you making an XML string in here? You
    // have already parsed the URDF."  Well children, OpenRAVE 0.8 and earlier
    // seem to have no other mechanism for creating kinbodies with joints and links.
    //
    // So, I am forced to do this.  In OpenRAVE 0.9 and above, this code should
    // probably be replaced with programmatic construction.
    TiXmlDocument xml;
    xml.LinkEndChild(new TiXmlDeclaration("1.0", "", ""));
    
    // Create a root robot node for this robot
    TiXmlElement *robot = new TiXmlElement("Robot");
    robot->SetAttribute("name", model.getName());
    xml.LinkEndChild(robot);

    // Create a kinbody to contain all the links and joints
    TiXmlElement *kinBody = new TiXmlElement("KinBody");
    robot->LinkEndChild(kinBody);

    // Populate vector of links
    std::string link_name; 
    boost::shared_ptr<urdf::Link> link_ptr;
    BOOST_FOREACH(boost::tie(link_name, link_ptr), model.links_) {

      // Create a new link from the URDF model
      TiXmlElement *link = new TiXmlElement("Body");
      link->SetAttribute("name", link_name);
      
      // TODO: fill in links

      // Add link to XML
      kinBody->LinkEndChild(link);
    }

    // Populate vector of joints
    std::string joint_name; 
    boost::shared_ptr<urdf::Joint> joint_ptr;
    BOOST_FOREACH(boost::tie(joint_name, joint_ptr), model.joints_) {

      // Create a new joint from the URDF model
      TiXmlElement *joint = new TiXmlElement("Joint");
      joint->SetAttribute("name", joint_name);

      // TODO: fill in joints

      // Add joint to XML
      kinBody->LinkEndChild(joint);
    }

    // Write out the XML document to a string interface
    // (e.g. http://www.grinninglizard.com/tinyxmldocs/classTiXmlPrinter.html)
    TiXmlPrinter robotPrinter;
    robotPrinter.SetStreamPrinting();
    xml.Accept(&robotPrinter);
    
    // Load the robot in OpenRAVE from the in-memory XML string
    _env->LoadData(robotPrinter.CStr());

    // Return reference to created object
    // TODO: return object somehow?
    return true;
  }

} 

