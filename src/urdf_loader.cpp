/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include "urdf_loader.h"

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

    // Populate vector of links
    std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos;

    std::string link_name; 
    boost::shared_ptr<urdf::Link> link_ptr;
    BOOST_FOREACH(boost::tie(link_name, link_ptr), model.links_) {

      // Create a new link and set basic properties
      OpenRAVE::KinBody::LinkInfoPtr link_info = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();
      link_info->_name = link_name;

      // Mass parameters
      urdf::InertialPtr inertial = link_ptr->inertial;
      link_info->_tMassFrame = URDFPoseToRaveTransform(inertial->origin);
      link_info->_mass = inertial->mass;
      link_info->_vinertiamoments = OpenRAVE::Vector(inertial->xx, inertial->yy, inertial->zz);

      // Add geometries for model
      // TODO: fill me in!
      
      link_infos.push_back(link_info);
    }

    // Populate vector of joints
    std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos;

    std::string joint_name; 
    boost::shared_ptr<urdf::Joint> joint_ptr;
    BOOST_FOREACH(boost::tie(joint_name, joint_ptr), model.joints_) {

      // Create a new joint and set basic properties
      OpenRAVE::KinBody::JointInfoPtr joint_info = boost::make_shared<OpenRAVE::KinBody::JointInfo>();
      joint_info->_name = joint_name;
      joint_info->_type = URDFJointTypeToRaveJointType(joint_ptr->type);
      
      // Set parent and child links
      joint_info->_linkname0 = joint_ptr->parent_link_name;
      joint_info->_linkname1 = joint_ptr->child_link_name;

      // TODO: Fill in everything else!

      joint_infos.push_back(joint_info);
    }

    // Construct robot from URDF link and joint information
    OpenRAVE::RobotBasePtr robot = OpenRAVE::RaveCreateRobot(_env, model.getName());
    robot->Init(link_infos, joint_infos);
    _env->Add(robot);

    // Return reference to created object
    // TODO: return object somehow
    return true;
  }

} 

