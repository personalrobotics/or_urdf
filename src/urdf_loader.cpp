/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi, Michael Koval
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include "urdf_loader.h"
#include "boostfs_helpers.h"
#include "urdf_yaml_helpers.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

/** Boilerplate plugin definition for OpenRAVE */
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr env)
{
  if( type == OpenRAVE::PT_Module && interfacename == "urdf" ) {
    return OpenRAVE::InterfaceBasePtr(new or_urdf::URDFLoader(env));
  } else {
    return OpenRAVE::InterfaceBasePtr();
  }
}

/** Boilerplate plugin definition for OpenRAVE */
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
  info.interfacenames[OpenRAVE::PT_Module].push_back("URDF");
}

/** Boilerplate plugin definition for OpenRAVE */
void DestroyPlugin()
{
  // Nothing to clean up!
}

namespace or_urdf
{
  template <class T>
  std::vector<boost::shared_ptr<T const> > MakeConst(std::vector<boost::shared_ptr<T> > const &vconst)
  {
      std::vector<boost::shared_ptr<T const> > v;
      v.reserve(vconst.size());

      BOOST_FOREACH (boost::shared_ptr<T> const &x, vconst) {
          v.push_back(x);
      }

      return v;
  }

  /** Converts from URDF 3D vector to OpenRAVE 3D vector. */
  OpenRAVE::Vector URDFVectorToRaveVector(const urdf::Vector3 &vector)
  {
    return OpenRAVE::Vector(vector.x, vector.y, vector.z);
  }

  /** Converts from URDF 3D rotation to OpenRAVE 3D vector. */
  OpenRAVE::Vector URDFRotationToRaveVector(const urdf::Rotation &rotation)
  {
    return OpenRAVE::Vector(rotation.w, rotation.x, rotation.y, rotation.z);
  }

  OpenRAVE::Vector URDFColorToRaveVector(const urdf::Color &color)
  {
    return OpenRAVE::Vector(color.r, color.g, color.b, color.a);
  }

  OpenRAVE::Transform URDFPoseToRaveTransform(const urdf::Pose &pose)
  {
    return OpenRAVE::Transform(URDFRotationToRaveVector(pose.rotation),
                               URDFVectorToRaveVector(pose.position));
  }
  
  /** Resolves URIs for file:// and package:// paths */
  const std::string resolveURI(const std::string &path)
  {
    static std::map<std::string, std::string> package_cache;
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
      std::string package_path;

      // Use the package cache if we have resolved this package before
      std::map<std::string, std::string>::iterator it = package_cache.find(package);
      if (it != package_cache.end()) {
	package_path = it->second;
      } else {
	package_path = ros::package::getPath(package);
	package_cache[package] = package_path;
      }
      
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

  /** Converts URDF joint to an OpenRAVE joint string and a boolean
      representing whether the joint is moving or fixed */
  const std::pair<OpenRAVE::KinBody::JointType, bool> URDFJointTypeToRaveJointType(int type)
  {
    switch(type) {
    case urdf::Joint::REVOLUTE:
      return std::make_pair(OpenRAVE::KinBody::JointRevolute, true);
    case urdf::Joint::PRISMATIC:
      return std::make_pair(OpenRAVE::KinBody::JointSlider, true);
    case urdf::Joint::FIXED:
      return std::make_pair(OpenRAVE::KinBody::JointHinge, false);
    case urdf::Joint::CONTINUOUS:
      return std::make_pair(OpenRAVE::KinBody::JointHinge, true);
    case urdf::Joint::PLANAR:
    case urdf::Joint::FLOATING:
    case urdf::Joint::UNKNOWN:
    default:
      // TODO: Fill the rest of these joint types in!
      RAVELOG_ERROR("URDFLoader : Unable to determine joint type [%d].\n", type);
      throw OpenRAVE::openrave_exception("Failed to convert URDF joint!");
    }
  }

  void URDFLoader::ParseURDF(urdf::Model &model, std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                                                 std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos)
  {
    // Populate list of links from URDF model. We'll force the root link to be first.
    std::vector< boost::shared_ptr<urdf::Link> > link_vector;
    model.getLinks(link_vector);

    std::list<boost::shared_ptr<urdf::Link const> > link_list;
    std::set<std::string> finished_links;

    link_list.insert(link_list.begin(), model.getRoot());
    BOOST_FOREACH (boost::shared_ptr<urdf::Link> link, link_vector) {
        if (link != model.getRoot()) {
            link_list.insert(link_list.end(), link);
        }
    }

    // TODO: prevent infinite loops here
    // Iterate through all links, allowing deferred evaluation (putting links
    // back on the list) if their parents do not exist yet
    boost::shared_ptr<urdf::Link const> link_ptr;

    while (!link_list.empty()) {
      // Get next element in list
      link_ptr = link_list.front();
      link_list.pop_front();

      OpenRAVE::KinBody::LinkInfoPtr link_info = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();

      link_info->_name = link_ptr->name;
      // TODO: Set "type" to "dynamic".
      
      // Set inertial parameters
      boost::shared_ptr<urdf::Inertial> inertial = link_ptr->inertial;
      if (inertial) {
        // XXX: We should also specify the off-diagonal terms (ixy, iyz, ixz)
        // of the inertia tensor. We can do this in KinBody XML files, but I
        // cannot figure out how to do so through this API.
        link_info->_mass = inertial->mass;
        link_info->_tMassFrame = URDFPoseToRaveTransform(inertial->origin);
        link_info->_vinertiamoments = OpenRAVE::Vector(inertial->ixx, inertial->iyy, inertial->izz);
      }

      // Set local transformation to be same as parent joint
      boost::shared_ptr<urdf::Joint> parent_joint = link_ptr->parent_joint;
      while (parent_joint) {
        link_info->_t = URDFPoseToRaveTransform(parent_joint->parent_to_joint_origin_transform) * link_info->_t;
        boost::shared_ptr<urdf::Link const> parent_link = model.getLink(parent_joint->parent_link_name);
        parent_joint = parent_link->parent_joint;
      }
      
      // Set information for collision geometry
      boost::shared_ptr<urdf::Collision> collision = link_ptr->collision;
      if (collision) {
        OpenRAVE::KinBody::GeometryInfoPtr geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();

        //geom_info->_t = URDFPoseToRaveTransform(collision->origin);
        geom_info->_bVisible = false;
        geom_info->_bModifiable = false;

        switch (collision->geometry->type) {
        case urdf::Geometry::MESH: {
            const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh &>(*collision->geometry);
            geom_info->_filenamecollision = resolveURI(mesh.filename);
            geom_info->_type = OpenRAVE::GT_TriMesh;

            boost::shared_ptr<OpenRAVE::TriMesh> trimesh = boost::make_shared<OpenRAVE::TriMesh>();
            trimesh = GetEnv()->ReadTrimeshURI(trimesh, geom_info->_filenamecollision);
            if (trimesh) {
                geom_info->_meshcollision = *trimesh;
            } else {
                RAVELOG_WARN("Link[%s]: Failed loading collision mesh %s\n",
                             link_ptr->name.c_str(), geom_info->_filenamecollision.c_str());
            }
            break;
        }

        case urdf::Geometry::SPHERE: {
            const urdf::Sphere &sphere = dynamic_cast<const urdf::Sphere &>(*collision->geometry);
            geom_info->_vGeomData = sphere.radius * OpenRAVE::Vector(1, 1, 1);
            geom_info->_type = OpenRAVE::GT_Sphere;
            break;
        }

        case urdf::Geometry::BOX: {
            const urdf::Box &box = dynamic_cast<const urdf::Box &>(*collision->geometry);
            geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(box.dim.x, box.dim.y, box.dim.z);
            geom_info->_type = OpenRAVE::GT_Box;
            break;
        }

        case urdf::Geometry::CYLINDER: {
            const urdf::Cylinder &cylinder = dynamic_cast<const urdf::Cylinder &>(*collision->geometry);
            geom_info->_vGeomData = OpenRAVE::Vector(cylinder.radius, cylinder.length, 0);
            geom_info->_type = OpenRAVE::GT_Cylinder;
            break;
        }
        }

        link_info->_vgeometryinfos.push_back(geom_info);
      }

      // Add the render geometry. We can't create a link with no collision
      // geometry, so we'll instead create a zero-radius sphere with the
      // desired render mesh.
      boost::shared_ptr<urdf::Visual> visual = link_ptr->visual;
      if (visual) {
        OpenRAVE::KinBody::GeometryInfoPtr geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
        geom_info->_t = URDFPoseToRaveTransform(visual->origin);
        geom_info->_type = OpenRAVE::GT_Sphere;
        geom_info->_vGeomData = OpenRAVE::Vector(0.0, 0.0, 0.0);
        geom_info->_bModifiable = false;
        geom_info->_bVisible = true;

        switch (visual->geometry->type) {
        case urdf::Geometry::MESH: {
            const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh&>(*visual->geometry);
            geom_info->_filenamerender = resolveURI(mesh.filename);
            geom_info->_vRenderScale = OpenRAVE::Vector(1.0, 1.0, 1.0);
            break;
        }

        default:
            RAVELOG_WARN("Link[%s]: Only trimeshes are supported for visual geometry.\n", link_ptr->name.c_str());
        }

        // If a material color is specified, use it.
        boost::shared_ptr<urdf::Material> material = visual->material;
        if (material) {
            geom_info->_vDiffuseColor = URDFColorToRaveVector(material->color);
            geom_info->_vAmbientColor = URDFColorToRaveVector(material->color);
        }
        link_info->_vgeometryinfos.push_back(geom_info);
      }
      
      // Mark this link as completed
      link_infos.push_back(link_info);
    }

    // Populate vector of joints
    std::string joint_name; 
    boost::shared_ptr<urdf::Joint> joint_ptr;

    // Parse the yaml file
    YAML::Node doc;
    std::vector<boost::shared_ptr<urdf::Joint> > ordered_joints;
    BOOST_FOREACH(boost::tie(joint_name, joint_ptr), model.joints_) {
        ordered_joints.push_back(joint_ptr);
    }

    BOOST_FOREACH(boost::shared_ptr<urdf::Joint> joint_ptr, ordered_joints) {
      OpenRAVE::KinBody::JointInfoPtr joint_info = boost::make_shared<OpenRAVE::KinBody::JointInfo>();
      joint_info->_name = joint_ptr->name;
      joint_info->_linkname0 = joint_ptr->parent_link_name;
      joint_info->_linkname1 = joint_ptr->child_link_name;
      joint_info->_vanchor = URDFVectorToRaveVector(joint_ptr->parent_to_joint_origin_transform.position);

      int urdf_joint_type = joint_ptr->type;
      if (urdf_joint_type == urdf::Joint::REVOLUTE || urdf_joint_type == urdf::Joint::CONTINUOUS) {
          if (joint_ptr->limits) {
              urdf_joint_type = urdf::Joint::REVOLUTE;
          } else {
              urdf_joint_type = urdf::Joint::CONTINUOUS;
          }
      }

      // Set the joint type. Some URDF joints correspond to disabled OpenRAVE
      // joints, so we'll appropriately set the corresponding IsActive flag.
      OpenRAVE::KinBody::JointType joint_type;
      bool enabled;
      boost::tie(joint_type, enabled) = URDFJointTypeToRaveJointType(urdf_joint_type);

      joint_info->_type = joint_type;
      joint_info->_bIsActive = enabled;

      // URDF only supports linear mimic joints with a constant offset. We map
      // that into the correct position (index 0) and velocity (index 1)
      // equations for OpenRAVE.
      boost::shared_ptr<urdf::JointMimic> mimic = joint_ptr->mimic;
      if (mimic) {
        joint_info->_vmimic[0] = boost::make_shared<OpenRAVE::KinBody::MimicInfo>();
        joint_info->_vmimic[0]->_equations[0] = boost::str(boost::format("%s*%0.6f+%0.6f")
                                                  % mimic->joint_name % mimic->multiplier % mimic->offset);
        joint_info->_vmimic[0]->_equations[1] = boost::str(boost::format("|%s %0.6f")
                                                  % mimic->joint_name % mimic->multiplier);
      }

      // Configure joint axis. Add an arbitrary axis if the joint is disabled.
      urdf::Vector3 joint_axis;
      if (enabled) {
        joint_axis = joint_ptr->parent_to_joint_origin_transform.rotation * joint_ptr->axis;
      } else {
        joint_axis = urdf::Vector3(1, 0, 0);
      }
      joint_info->_vaxes[0] = URDFVectorToRaveVector(joint_axis);
      
      // Configure joint limits.
      boost::shared_ptr<urdf::JointLimits> limits = joint_ptr->limits;
      if (limits) {
          // TODO: What about acceleration?
          joint_info->_vlowerlimit[0] = limits->lower;
          joint_info->_vupperlimit[0] = limits->upper;
          joint_info->_vmaxvel[0] = limits->velocity;
          joint_info->_vmaxtorque[0] = limits->effort;
      }
      // Fixed joints are just revolute joints with zero limits.
      else if (!enabled) {
          joint_info->_vlowerlimit[0] = 0;
          joint_info->_vupperlimit[0] = 0;
      }
      // This is a hack to get continuous joints to work. The limits default to
      // [0, 0], which inserts a fixed joint.
      else if (urdf_joint_type == urdf::Joint::CONTINUOUS) {
          joint_info->_vlowerlimit[0] = -2 * M_PI;
          joint_info->_vupperlimit[0] =  2 * M_PI;
      }

      joint_infos.push_back(joint_info);
    }
  }

void URDFLoader::ParseSRDF(srdf::Model &srdf, std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                                              std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos,
                                              std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> &manip_infos)
{
    std::map<std::string, OpenRAVE::KinBody::LinkInfoPtr> link_map;
    BOOST_FOREACH (OpenRAVE::KinBody::LinkInfoPtr link_info, link_infos) {
        link_map[link_info->_name] = link_info;
    }

    std::string link1_name, link2_name;
    BOOST_FOREACH(boost::tie(link1_name, link2_name), srdf.getDisabledCollisions()) {
        OpenRAVE::KinBody::LinkInfoPtr link1_info = link_map[link1_name];
        if (!link1_info) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no link named %s.", link1_name.c_str(),
                                            OpenRAVE::ORE_Failed);
        }

        OpenRAVE::KinBody::LinkInfoPtr link2_info = link_map[link2_name];
        if (!link2_info) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no link named %s.", link2_name.c_str(),
                                            OpenRAVE::ORE_Failed);
        }

        link1_info->_vForcedAdjacentLinks.push_back(link2_name);
        link2_info->_vForcedAdjacentLinks.push_back(link1_name);
    }

    // TODO: Create manipulators.
    // TODO: Add CHOMP spheres.
}
  
  /** Opens a URDF file and returns a robot in OpenRAVE */
  bool URDFLoader::load(std::ostream &soutput, std::istream &sinput)
  {
    // Get filename from input arguments
    std::string input_urdf, input_srdf;
    sinput >> input_urdf >> input_srdf;

    std::cout << "| " << input_urdf << " | " << input_srdf << " |" << std::endl;

    OpenRAVE::KinBodyPtr body;
    std::string name;

    // Load the URDF file.
    urdf::Model urdf_model;
    if (!urdf_model.initFile(input_urdf)) {
      throw OpenRAVE::openrave_exception("Failed to open URDF file.");
    }

    std::vector<OpenRAVE::KinBody::LinkInfoPtr> link_infos;
    std::vector<OpenRAVE::KinBody::JointInfoPtr> joint_infos;
    ParseURDF(urdf_model, link_infos, joint_infos);

    // Optionally load the SRDF to create a Robot.
    if (!input_srdf.empty()) {
        std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> manip_infos;
        std::vector<OpenRAVE::RobotBase::AttachedSensorInfoPtr> sensor_infos;

        srdf::Model srdf_model;
        if (!srdf_model.initFile(urdf_model, input_srdf)) {
            throw OpenRAVE::openrave_exception("Failed to open SRDF file.");
        }

        ParseSRDF(srdf_model, link_infos, joint_infos, manip_infos);

        // Cast all of the vector contents to const.
        std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const = MakeConst(link_infos);
        std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const = MakeConst(joint_infos);
        std::vector<OpenRAVE::RobotBase::ManipulatorInfoConstPtr> manip_infos_const = MakeConst(manip_infos);
        std::vector<OpenRAVE::RobotBase::AttachedSensorInfoConstPtr> sensor_infos_const = MakeConst(sensor_infos);

        OpenRAVE::RobotBasePtr robot = OpenRAVE::RaveCreateRobot(GetEnv(), "");
        robot->Init(link_infos_const, joint_infos_const, manip_infos_const, sensor_infos_const);
        body = robot;
        // TODO: Set the name.
    }
    // It's just a URDF file, so create a KinBody.
    else {
        std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const = MakeConst(link_infos);
        std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const = MakeConst(joint_infos);

        body = OpenRAVE::RaveCreateKinBody(GetEnv(), "");
        body->Init(link_infos_const, joint_infos_const);
    }

    body->SetName(urdf_model.getName());
    GetEnv()->Add(body, true);
    soutput << body->GetName(); 
    return true;
  }

} 

