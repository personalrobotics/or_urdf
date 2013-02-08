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
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>

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

namespace urdf_loader
{
  
  /** Resolves URIs for file:// and package:// paths */
  const std::string resolveURI(const std::string &path)
  {
    std::string uri = path;

    if (uri.find("file://") == 0) {

      // Strip off the file://
      uri.erase(0, strlen("file://"));

      // Resolve the mesh path as a file URI
      boost::filesystem::path file_path(uri);
      return file_path.string();

    } else if (uri.find("package://") == 0) {

      // Strip off the package://
      uri.erase(0, strlen("package://"));
	
      // Resolve the mesh path as a ROS package URI
      size_t package_end = uri.find("/");
      std::string package = uri.substr(0, package_end);
      std::string package_path = ros::package::getPath(package);
      
      // Show a warning if the package was not resolved
      if (package_path.empty())	{
	RAVELOG_WARN("Unable to find package [%s].\n", package.c_str());
	return "";
      }
      
      // Append the remaining relative path
      boost::filesystem::path file_path(package_path);
      uri.erase(0, package_end);
      file_path /= uri;
      
      // Return the canonical path
      return file_path.string();

    } else {
      RAVELOG_WARN("Cannot handle mesh URI type [%s].\n");
      return "";
    }
  }

  const std::pair<std::string, bool> URDFJointTypeToRaveJointType(int type)
  {
    switch(type) {
    case urdf::Joint::REVOLUTE:
      return std::pair<std::string, bool>("hinge",  true);
    case urdf::Joint::PRISMATIC:
      return std::pair<std::string, bool>("slider", true);
    case urdf::Joint::FIXED:
      return std::pair<std::string, bool>("hinge", false);
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
  
  void makeTextElement(TiXmlElement *element, const std::string &name, const std::string &value)
  {
    TiXmlElement *node = new TiXmlElement(name);  
    node->LinkEndChild(new TiXmlText(value));  
    element->LinkEndChild(node);
  }
  
  class Geometry {
  public:
    enum Type { COLLISION, RENDER };
  };

  TiXmlElement *makeGeomElement(const urdf::Geometry &geometry, Geometry::Type type)
  {
    TiXmlElement *node = new TiXmlElement("Geom");
    // TODO: set a "render" attribute depending on collision or render

    // Convert depending on geometry type
    switch(geometry.type) {
    case urdf::Geometry::SPHERE:
      {
	node->SetAttribute("type", "sphere");

	const urdf::Sphere &sphere = dynamic_cast<const urdf::Sphere&>(geometry);
	makeTextElement(node, "radius", boost::lexical_cast<std::string>(sphere.radius));
      }
      break;
    case urdf::Geometry::BOX:
      {
	node->SetAttribute("type", "box");

	const urdf::Box &box = dynamic_cast<const urdf::Box&>(geometry);
	makeTextElement(node, "extents", boost::str(boost::format("%f %f %f")
						    % box.dim.x % box.dim.y % box.dim.z ));
      }
      break;
    case urdf::Geometry::CYLINDER:
      {
	node->SetAttribute("type", "cylinder");

	const urdf::Cylinder &cylinder = dynamic_cast<const urdf::Cylinder&>(geometry);
	makeTextElement(node, "height", boost::lexical_cast<std::string>(cylinder.length));
	makeTextElement(node, "radius", boost::lexical_cast<std::string>(cylinder.radius));
      }
      break;
    case urdf::Geometry::MESH:
      {
	node->SetAttribute("type", "trimesh");

	const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh&>(geometry);
	std::string mesh_filename = resolveURI(mesh.filename);

	// Either create a collision or render geometry
	switch(type) {
	case Geometry::COLLISION:
	  makeTextElement(node, "data", mesh_filename); // TODO: scale?
	  break;
	case Geometry::RENDER:
	  makeTextElement(node, "render", mesh_filename); // TODO: scale?
	  break;
	default:
	  RAVELOG_ERROR("URDFLoader : Unable to determine trimesh type [%d].\n", type);
	  throw OpenRAVE::openrave_exception("Failed to convert URDF trimesh!");
	}
      }
      break;
    default:
      RAVELOG_ERROR("URDFLoader : Unable to determine geometry type [%d].\n", geometry.type);
      throw OpenRAVE::openrave_exception("Failed to convert URDF geometry!");
    }
    
    return node;
  }

  /** Opens a URDF file and returns a robot in OpenRAVE */
  bool URDFLoader::load(std::ostream &soutput, std::istream &sinput)
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
      link->SetAttribute("type", "dynamic");
      
      // Set inertial parameters
      boost::shared_ptr<urdf::Inertial> inertial = link_ptr->inertial;
      if (inertial) {
	TiXmlElement *mass = new TiXmlElement("Mass");
	makeTextElement(mass, "total", boost::lexical_cast<std::string>(inertial->mass));
	makeTextElement(mass, "inertia", boost::str(boost::format("%f %f %f %f %f %f %f %f %f")
						    % inertial->ixx % inertial->ixy % inertial->ixz
						    % inertial->ixy % inertial->iyy % inertial->iyz
						    % inertial->ixz % inertial->iyz % inertial->izz));
	makeTextElement(mass, "com", boost::str(boost::format("%f %f %f") 
						% inertial->origin.position.x
						% inertial->origin.position.y
						% inertial->origin.position.z));
	link->LinkEndChild(mass);
      }

      // Set information for collision geometry
      boost::shared_ptr<urdf::Collision> collision = link_ptr->collision;
      if (collision) {
	TiXmlElement *collision_geom = makeGeomElement(*(collision->geometry), Geometry::COLLISION);
	makeTextElement(collision_geom, "translation", boost::str(boost::format("%f %f %f")
								  % collision->origin.position.x
								  % collision->origin.position.y
								  % collision->origin.position.z));
	makeTextElement(collision_geom, "quat", boost::str(boost::format("%f %f %f %f")
							   % collision->origin.rotation.x
							   % collision->origin.rotation.y
							   % collision->origin.rotation.z
							   % collision->origin.rotation.w));
	link->LinkEndChild(collision_geom);
      }

      // Set information for rendered geometry
      boost::shared_ptr<urdf::Visual> visual = link_ptr->visual;
      if (visual) {
	TiXmlElement *render_geom = makeGeomElement(*(visual->geometry), Geometry::RENDER);
	makeTextElement(render_geom, "translation", boost::str(boost::format("%f %f %f")
							       % visual->origin.position.x
							       % visual->origin.position.y
							       % visual->origin.position.z));
	makeTextElement(render_geom, "quat", boost::str(boost::format("%f %f %f %f")
							% visual->origin.rotation.x
							% visual->origin.rotation.y
							% visual->origin.rotation.z
							% visual->origin.rotation.w));
	link->LinkEndChild(render_geom);
      }
      // TODO: set material information for rendered geometry

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

      // Set the type of joint
      std::pair<std::string, bool> joint_params = URDFJointTypeToRaveJointType(joint_ptr->type);
      std::string joint_type = joint_params.first;
      bool joint_enabled = joint_params.second;
      joint->SetAttribute("type", joint_type);
      joint->SetAttribute("enable", (joint_enabled ? "true" : "false"));

      // Connect joint to appropriate parent and child links
      makeTextElement(joint, "Body", joint_ptr->parent_link_name);
      makeTextElement(joint, "Body", joint_ptr->child_link_name);

      // Configure joint axis (or make one up if the joint isn't enabled)
      urdf::Vector3 axis = (joint_enabled ? joint_ptr->axis : urdf::Vector3(1.0, 0.0, 0.0));
      makeTextElement(joint, "axis", boost::str(boost::format("%f %f %f") 
						% axis.x % axis.y % axis.z));
      
      // Configure joint limits
      boost::shared_ptr<urdf::JointLimits> limits = joint_ptr->limits;
      if (limits) {
	makeTextElement(joint, "maxvel", boost::lexical_cast<std::string>(limits->velocity));
	makeTextElement(joint, "limitsrad", boost::str(boost::format("%f %f") 
						       % limits->lower % limits->upper));
      }

      // TODO: fill in joints

      // Add joint to XML
      kinBody->LinkEndChild(joint);
    }

    // Write out the XML document to a string interface
    // (e.g. http://www.grinninglizard.com/tinyxmldocs/classTiXmlPrinter.html)
    TiXmlPrinter robotPrinter;
    //    robotPrinter.SetStreamPrinting();
    xml.Accept(&robotPrinter);

    // TODO: remove this debug print
    RAVELOG_INFO("Generated: %s%\n", robotPrinter.CStr());
    
    // Load the robot in OpenRAVE from the in-memory XML string
    _env->LoadData(robotPrinter.CStr());

    // Return reference to created object
    // TODO: return object somehow?
    return true;
  }

} 

