/** \file urdf_loader.cpp
 * \brief Implementation of a URDF loading plugin for OpenRAVE
 * \author Michael Koval
 * \date 2013
 */
#include "urdf_loader.h"
#include "boostfs_helpers.h"
#include "picojson.h"

#include <iterator>

#include <boost/typeof/std/utility.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>

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

void GetURDFRootLinks(
        std::vector<boost::shared_ptr<urdf::Link const> > const &links,
        std::vector<boost::shared_ptr<urdf::Link const> >       *roots)
{
    typedef boost::shared_ptr<urdf::Link const> LinkConstPtr;

    BOOST_ASSERT(roots);
    std::set<LinkConstPtr> const links_set(links.begin(), links.end());

    BOOST_FOREACH (LinkConstPtr link, links) {
        LinkConstPtr parent_link = link->getParent();
        if (links_set.count(parent_link) == 0) {
            roots->push_back(link);
        }
    }
}

srdf::Model::Group const &GetSRDFGroup(
    std::map<std::string, srdf::Model::Group> const &groups,
    std::string const &name)
{
    BOOST_AUTO(it, groups.find(name));
    if (it == groups.end()) {
        throw std::runtime_error(boost::str(
                boost::format("Group '%s' does not exist.") % name));
    }
    return it->second;
}

void ExtractSRDFGroup(urdf::Model const &urdf, srdf::Model::Group const &group,
                      std::vector<boost::shared_ptr<urdf::Link const> > *links,
                      std::vector<boost::shared_ptr<urdf::Joint const> > *joints)
{
    typedef boost::shared_ptr<urdf::Link const> LinkConstPtr;
    typedef boost::shared_ptr<urdf::Joint const> JointConstPtr;

    BOOST_ASSERT(links);
    BOOST_ASSERT(joints);

    if (!group.chains_.empty()) {
        std::string base_link_name, tip_link_name;
        BOOST_FOREACH (boost::tie(base_link_name, tip_link_name), group.chains_) {
            // Crawl the kinematic chain from bottom-up.
            LinkConstPtr base_link = urdf.getLink(base_link_name);
            LinkConstPtr tip_link = urdf.getLink(tip_link_name);
            LinkConstPtr link_ptr = tip_link;
            BOOST_ASSERT(base_link);
            BOOST_ASSERT(tip_link);

            do {
                JointConstPtr parent_joint = link_ptr->parent_joint;
                links->push_back(link_ptr);
                joints->push_back(parent_joint);
                link_ptr = link_ptr->getParent();
            } while (link_ptr && link_ptr != base_link);

            if (!link_ptr) {
                throw std::runtime_error(boost::str(
                    boost::format("Invalid chain in group '%s':"
                                  " Link '%s' is not a parent of '%s'.")
                        % group.name_ % base_link_name % tip_link_name));
            }

            links->push_back(link_ptr);
        }
    }

    if (!group.joints_.empty()) {
        // Joints implicitly include their child link.
        BOOST_FOREACH (std::string const &joint_name, group.joints_) {
            JointConstPtr joint = urdf.getJoint(joint_name);
            LinkConstPtr child_link = urdf.getLink(joint->child_link_name);
            links->push_back(child_link);
            joints->push_back(joint);
        }
    }

    if (!group.links_.empty()) {
        // Links implicitly include their parent joint.
        BOOST_FOREACH (std::string const &link_name, group.links_) {
            LinkConstPtr link = urdf.getLink(link_name);
            JointConstPtr parent_joint = link->parent_joint;

            links->push_back(link);
            if (parent_joint) {
                joints->push_back(parent_joint);
            }
        }
    }
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{
    info.interfacenames[OpenRAVE::PT_Module].push_back("URDF");
}

void DestroyPlugin()
{
}

namespace or_urdf {

template <class T>
std::vector<boost::shared_ptr<T const> > MakeConst(
        std::vector<boost::shared_ptr<T> > const &vconst)
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

  /** Converts URDF joint to an OpenRAVE joint string and a boolean
      representing whether the joint is moving or fixed */
std::pair<OpenRAVE::KinBody::JointType, bool> URDFJointTypeToRaveJointType(int type)
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

    // TODO: Fill the rest of these joint types in!
    case urdf::Joint::PLANAR:
        throw std::runtime_error("Planar joints are not supported.");

    case urdf::Joint::FLOATING:
        throw std::runtime_error("Floating joints are not supported.");

    case urdf::Joint::UNKNOWN:
        throw std::runtime_error("Unknown joints are not supported.");

    default:
        throw std::runtime_error(boost::str(
            boost::format("Unkonwn type of joint %d.") % type));
    }
}

void URDFLoader::ParseURDF(
                     urdf::Model &model,
                     std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                     std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos)
{
    // Populate list of links from URDF model. We'll force the root link to be
    // first.
    std::vector<boost::shared_ptr<urdf::Link> > link_vector;
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

        OpenRAVE::KinBody::LinkInfoPtr link_info
                = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();

        // TODO: Set "type" to "dynamic".
        link_info->_name = link_ptr->name;

        // Set inertial parameters
        boost::shared_ptr<urdf::Inertial> inertial = link_ptr->inertial;
        if (inertial) {
            // XXX: We should also specify the off-diagonal terms (ixy, iyz, ixz)
            // of the inertia tensor. We can do this in KinBody XML files, but I
            // cannot figure out how to do so through this API.
            link_info->_mass = inertial->mass;
            link_info->_tMassFrame = URDFPoseToRaveTransform(inertial->origin);
            link_info->_vinertiamoments = OpenRAVE::Vector(
                    inertial->ixx, inertial->iyy, inertial->izz);
        }

        // Set local transformation to be same as parent joint
        boost::shared_ptr<urdf::Joint> parent_joint = link_ptr->parent_joint;
        while (parent_joint) {
            link_info->_t = URDFPoseToRaveTransform(
                    parent_joint->parent_to_joint_origin_transform) * link_info->_t;
            boost::shared_ptr<urdf::Link const> parent_link
                    = model.getLink(parent_joint->parent_link_name);
            parent_joint = parent_link->parent_joint;
        }

        // Set information for collision geometry
        BOOST_FOREACH (boost::shared_ptr<urdf::Collision> collision,
                       link_ptr->collision_array) {
            OpenRAVE::KinBody::GeometryInfoPtr geom_info
                    = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();

            // TODO: Shouldn't we apply this transform?
            geom_info->_t = URDFPoseToRaveTransform(collision->origin);
            geom_info->_bVisible = false;
            geom_info->_bModifiable = false;

            switch (collision->geometry->type) {
                case urdf::Geometry::MESH: {
                    urdf::Mesh const &mesh
                            = dynamic_cast<const urdf::Mesh &>(*collision->geometry);
                    geom_info->_filenamecollision = resolveURI(mesh.filename);
                    geom_info->_type = OpenRAVE::GT_TriMesh;

                    // This doesn't seem to do anything, but we'll set it anyway.
                    geom_info->_vCollisionScale = URDFVectorToRaveVector(mesh.scale);

                    boost::shared_ptr<OpenRAVE::TriMesh> trimesh
                            = boost::make_shared<OpenRAVE::TriMesh>();
                    trimesh = GetEnv()->ReadTrimeshURI(trimesh,
                                                       geom_info->_filenamecollision);
                    if (trimesh) {
                        // The _vCollisionScale property does nothing, so we have to
                        // manually scale the mesh.
                        BOOST_FOREACH(OpenRAVE::Vector &vertex, trimesh->vertices) {
                            vertex *= geom_info->_vCollisionScale;
                        }

                        geom_info->_meshcollision = *trimesh;
                    } else {
                        RAVELOG_WARN("Link[%s]: Failed loading collision mesh %s\n",
                                     link_ptr->name.c_str(), geom_info->_filenamecollision.c_str());
                    }
                    break;
                }

                case urdf::Geometry::SPHERE: {
                    RAVELOG_WARN("Creating sphere!\n");
                    urdf::Sphere const &sphere
                            = dynamic_cast<const urdf::Sphere &>(*collision->geometry);
                    geom_info->_vGeomData = sphere.radius * OpenRAVE::Vector(1, 1, 1);
                    geom_info->_type = OpenRAVE::GT_Sphere;
                    break;
                }

                case urdf::Geometry::BOX: {
                    urdf::Box const &box
                            = dynamic_cast<const urdf::Box &>(*collision->geometry);
                    geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(box.dim.x, box.dim.y,
                                                                   box.dim.z);
                    geom_info->_type = OpenRAVE::GT_Box;
                    break;
                }

                case urdf::Geometry::CYLINDER: {
                    urdf::Cylinder const &cylinder
                            = dynamic_cast<const urdf::Cylinder &>(*collision->geometry);
                    geom_info->_vGeomData = OpenRAVE::Vector(cylinder.radius,
                                                             cylinder.length, 0);
                    geom_info->_type = OpenRAVE::GT_Cylinder;
                    break;
                }
            }

            link_info->_vgeometryinfos.push_back(geom_info);
        }

        // Add the render geometry. We can't create a link with no collision
        // geometry, so we'll instead create a zero-radius sphere with the
        // desired render mesh.
        // TODO: Why does GT_None crash OpenRAVE?
        boost::shared_ptr<urdf::Visual> visual = link_ptr->visual;
        if (visual) {
            OpenRAVE::KinBody::GeometryInfoPtr geom_info
                = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            geom_info->_t = URDFPoseToRaveTransform(visual->origin);
            geom_info->_type = OpenRAVE::GT_Sphere;
            geom_info->_vGeomData = OpenRAVE::Vector(0.0, 0.0, 0.0);
            geom_info->_bModifiable = false;
            geom_info->_bVisible = true;

            switch (visual->geometry->type) {
            case urdf::Geometry::MESH: {
                const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh&>(*visual->geometry);
                geom_info->_filenamerender = resolveURI(mesh.filename);
                geom_info->_vRenderScale = URDFVectorToRaveVector(mesh.scale);

                // If a material color is specified, use it.
                boost::shared_ptr<urdf::Material> material = visual->material;
                if (material) {
                    geom_info->_vDiffuseColor = URDFColorToRaveVector(material->color);
                    geom_info->_vAmbientColor = URDFColorToRaveVector(material->color);
                }

                // Add the render-only geometry to the standard geometry group for
                // backwards compatability with QtCoin.
                link_info->_vgeometryinfos.push_back(geom_info);

                // Create a group dedicated to visual geometry for or_rviz.
                OpenRAVE::KinBody::GeometryInfoPtr geom_info_clone
                    = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>(*geom_info);
                link_info->_mapExtraGeometries["visual"].push_back(geom_info_clone);
                break;
            }

            default:
                RAVELOG_WARN("Link[%s]: Only trimeshes are supported for visual geometry.\n", link_ptr->name.c_str());
            }
        }

        // Verify that the "visual" and "spheres" groups always exist. Recall
        // that accessing an element with operator[] creates it using the default
        // no-arg constructor if it does not already exist.
        link_info->_mapExtraGeometries["visual"];
        link_info->_mapExtraGeometries["spheres"];

        link_infos.push_back(link_info);
    }

    // Populate vector of joints
    std::string joint_name;
    boost::shared_ptr<urdf::Joint> joint_ptr;

    // Parse the joint properties
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

        int const urdf_joint_type = joint_ptr->type;

        // Set the joint type. Some URDF joints correspond to disabled OpenRAVE
        // joints, so we'll appropriately set the corresponding IsActive flag.
        OpenRAVE::KinBody::JointType joint_type;
        bool enabled;
        boost::tie(joint_type, enabled) = URDFJointTypeToRaveJointType(urdf_joint_type);
        joint_info->_type = joint_type;
        joint_info->_bIsActive = enabled;
        joint_info->_bIsCircular[0] = (urdf_joint_type == urdf::Joint::CONTINUOUS);

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
        // Default to +/- 2*PI. This is the same default used by OpenRAVE for
        // revolute joints.
        else {
            joint_info->_vlowerlimit[0] = -M_PI;
            joint_info->_vupperlimit[0] =  M_PI;
        }

        // Force continuous joints to have +/- PI limits. Otherwise, the internal
        // _vcircularlowerlimit and _vcircularupperlimit values will be set to
        // zero. This causes OpenRAVE::utils::NormalizeCircularAngle to crash.
        if (urdf_joint_type == urdf::Joint::CONTINUOUS) {
            joint_info->_vlowerlimit[0] = -M_PI;
            joint_info->_vupperlimit[0] =  M_PI;
        }

        joint_infos.push_back(joint_info);
    }
}

void URDFLoader::ParseSRDF(urdf::Model const &urdf, srdf::Model const &srdf,
                           std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                           std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos,
                           std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> &manip_infos)
{
    std::map<std::string, OpenRAVE::KinBody::LinkInfoPtr> link_map;
    BOOST_FOREACH (OpenRAVE::KinBody::LinkInfoPtr link_info, link_infos) {
        link_map[link_info->_name] = link_info;
    }

    // Link adjacencies.
    size_t num_adjacent = 0;
    BOOST_FOREACH (srdf::Model::DisabledCollision const &link_pair,
                   srdf.getDisabledCollisionPairs()) {
        std::string const &link1_name = link_pair.link1_;
        OpenRAVE::KinBody::LinkInfoPtr link1_info = link_map[link1_name];
        if (!link1_info) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no link named %s.", link1_name.c_str(),
                                            OpenRAVE::ORE_Failed);
        }

        std::string const &link2_name = link_pair.link2_;
        OpenRAVE::KinBody::LinkInfoPtr link2_info = link_map[link2_name];
        if (!link2_info) {
            throw OPENRAVE_EXCEPTION_FORMAT("There is no link named %s.", link2_name.c_str(),
                                            OpenRAVE::ORE_Failed);
        }

        link1_info->_vForcedAdjacentLinks.push_back(link2_name);
        link2_info->_vForcedAdjacentLinks.push_back(link1_name);
        num_adjacent++;
    }
    RAVELOG_DEBUG("Disabled collisions between %d pairs of joints.\n",
                   num_adjacent);

    // TODO: Passive joints.

    // Build an index of the SRDF groups. This is necessary because the srdf
    // package provides very low-level access to the file's contents.
    std::vector<srdf::Model::Group> const &raw_groups = srdf.getGroups();
    std::map<std::string, srdf::Model::Group> groups;

    BOOST_FOREACH(srdf::Model::Group const &group, raw_groups) {
        BOOST_AUTO(result, groups.insert(std::make_pair(group.name_, group)));
        if (!result.second) {
            throw std::runtime_error(boost::str(
                boost::format("Duplicate SRDF group '%s'.") % group.name_));
        }
    }

    // Create manipulators.
    BOOST_FOREACH (srdf::Model::EndEffector const &end_effector, srdf.getEndEffectors()) {
        typedef boost::shared_ptr<urdf::Link const> LinkConstPtr;
        typedef boost::shared_ptr<urdf::Joint const> JointConstPtr;

        // Get the end-effector group its associated mainpulator. We assume
        // that the parent group of an end-effector is a manipulator.
        RAVELOG_DEBUG("Loading '%s' end-effector group '%s'.\n",
                     end_effector.name_.c_str(),
                     end_effector.component_group_.c_str());
        srdf::Model::Group const &ee_group = GetSRDFGroup(groups, end_effector.component_group_);

        if (end_effector.parent_group_.empty()) {
            throw std::runtime_error(boost::str(
                boost::format("End-effector '%s' has no parent group.\n")
                    % end_effector.name_));
        }
        RAVELOG_DEBUG("Loading '%s' manipulator group '%s'.\n",
                     end_effector.name_.c_str(),
                     end_effector.parent_group_.c_str());
        srdf::Model::Group const &manip_group = GetSRDFGroup(groups, end_effector.parent_group_);

        // Compute the root link of the manipulator.
        std::vector<LinkConstPtr> manip_links, manip_root_links;
        std::vector<JointConstPtr> manip_joints;
        ExtractSRDFGroup(urdf, manip_group, &manip_links, &manip_joints);
        GetURDFRootLinks(manip_links, &manip_root_links);
        RAVELOG_DEBUG("Manipulator group '%s' contains %d links and %d joints.\n",
                     manip_group.name_.c_str(), manip_links.size(), manip_joints.size());

        if (manip_root_links.size() != 1) {
            throw std::runtime_error(boost::str(
                boost::format("Manipulator '%s' has %d root links; must have exactly one.")
                    % manip_group.name_.c_str() % manip_root_links.size()));
        }
        LinkConstPtr manip_root_link = manip_root_links.front();
        BOOST_ASSERT(manip_root_link);
        RAVELOG_DEBUG("Detected '%s' as root link of manipulator group '%s'.\n",
                     manip_root_link->name.c_str(), manip_group.name_.c_str());

        // Find the parent link of the end-effector. This serves as the tip
        // link of the manipulator.
        // TODO: std::map<std::string, OpenRAVE::KinBody::LinkInfoPtr> link_map;
        OpenRAVE::KinBody::LinkInfoPtr ee_root_link;
        BOOST_AUTO(it, link_map.find(end_effector.parent_link_));
        if (it != link_map.end()) {
            ee_root_link = it->second;
        } else {
            throw std::runtime_error(boost::str(
                boost::format("Unable to find end-effector parent "
                              " link '%s'.") % end_effector.parent_link_));
        }
        RAVELOG_DEBUG("Found manipulator tip link '%s'.\n",
                     ee_root_link->_name.c_str());

        // Find active joints in the end-effector group. We will treat these as
        // gripper joints.
        std::vector<LinkConstPtr> ee_links, ee_root_links;
        std::vector<JointConstPtr> ee_joints;
        ExtractSRDFGroup(urdf, ee_group, &ee_links, &ee_joints);

        std::set<JointConstPtr> gripper_joints;
        size_t num_rejected_gripper_joints = 0;

        BOOST_FOREACH (JointConstPtr ee_joint, ee_joints) {
            bool const is_mimic = !!ee_joint->mimic;
            bool const is_fixed = ee_joint->type == urdf::Joint::FIXED;

            if (!is_mimic && !is_fixed) {
                gripper_joints.insert(gripper_joints.begin(), ee_joint);
            } else {
                num_rejected_gripper_joints++;
            }
        }

        RAVELOG_DEBUG("Found %d gripper joints for manipulator '%s';"
                     " ignored %d passive/mimic joints.\n",
                     gripper_joints.size(), manip_group.name_.c_str(),
                     num_rejected_gripper_joints);

        // Generate the OpenRAVE manipulator.
        // TODO: What about the closing direction?
        // TODO: What about the end-effector direction?
        BOOST_AUTO(manip_info,
                   boost::make_shared<OpenRAVE::RobotBase::ManipulatorInfo>());
        manip_info->_name = end_effector.name_;
        manip_info->_sBaseLinkName = manip_root_link->name;
        manip_info->_vdirection = OpenRAVE::Vector(0, 0, 1);
        manip_info->_vChuckingDirection.resize(gripper_joints.size(), 0.0);
        manip_info->_sEffectorLinkName = ee_root_link->_name;

        BOOST_FOREACH (JointConstPtr joint, gripper_joints) {
            manip_info->_vGripperJointNames.push_back(joint->name);
        }
        manip_infos.push_back(manip_info);
    }

    // Add the sphere approximation to a geometry group.
    BOOST_FOREACH (srdf::Model::LinkSpheres const &spheres,
                   srdf.getLinkSphereApproximations()) {
        // Find the corresponding OpenRAVE link.
        OpenRAVE::KinBody::LinkInfoPtr link_info;
        BOOST_FOREACH (OpenRAVE::KinBody::LinkInfoPtr candidate, link_infos) {
            if (candidate->_name == spheres.link_) {
                link_info = candidate;
                break;
            }
        }

        if (!link_info) {
            throw std::runtime_error(boost::str(
                boost::format("Unable to create sphere geometry for link '%s':"
                              " Link does not exist.") % spheres.link_));
        }

        // Get the transform for the collision geometry. Spheres are specified
        // in the collision frame.
        typedef boost::shared_ptr<urdf::Link const> LinkConstPtr;
        LinkConstPtr const urdf_link = urdf.getLink(spheres.link_);
        BOOST_ASSERT(urdf_link);
        boost::shared_ptr<urdf::Collision> const collision = urdf_link->collision;
        BOOST_ASSERT(collision);
        OpenRAVE::Transform const collision_transform
            = URDFPoseToRaveTransform(collision->origin);

        // Add the spheres to a separate collision group.
        std::vector<OpenRAVE::KinBody::GeometryInfoPtr> &sphere_infos
                = link_info->_mapExtraGeometries["spheres"];
        BOOST_FOREACH (srdf::Model::Sphere const &sphere, spheres.spheres_) {
            // TODO: Spheres should be in the collision frame.
            OpenRAVE::KinBody::GeometryInfoPtr sphere_info
                    = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            sphere_info->_bVisible = true;
            sphere_info->_bModifiable = false;
            sphere_info->_type = OpenRAVE::GT_Sphere;
            sphere_info->_vGeomData = sphere.radius_ * OpenRAVE::Vector(1, 1, 1);
            sphere_info->_t.trans = collision_transform * OpenRAVE::Vector(
                    sphere.center_x_, sphere.center_y_, sphere.center_z_);
            sphere_infos.push_back(sphere_info);
        }
    }
}

void URDFLoader::ProcessGeometryGroupTagsFromURDF(
                       TiXmlDocument &xml_doc,
                       std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos)
{
    TiXmlHandle xml_handle(&xml_doc);

    // <robot> element
    TiXmlHandle robot_handle(0);
    {
        TiXmlElement* robot_element = xml_handle.FirstChild("robot").Element();
        if (!robot_element) {
            throw std::runtime_error("Could not find the 'robot' element in the xml file.");
        }

        robot_handle = TiXmlHandle(robot_element);
    }

    // Store a set of unique <geometry_group> names found in the URDF
    std::set<std::string> geometry_group_names_set;

    // Parse the URDF,
    // if a specific link has a <geometry_group> tag
    // then add that group to the OpenRAVE link_infos:

    // <link> elements
    for (TiXmlElement* link_element = robot_handle.FirstChild("link").Element();
         link_element;
         link_element = link_element->NextSiblingElement("link"))
    {
        const char* link_name = link_element->Attribute("name");
        if (!link_name) {
            throw std::runtime_error("Link is missing 'name' attribute.");
        }

        TiXmlHandle link_handle = TiXmlHandle(link_element);

        // Find the corresponding OpenRAVE link
        OpenRAVE::KinBody::LinkInfoPtr link_info;
        BOOST_FOREACH (OpenRAVE::KinBody::LinkInfoPtr candidate,
                       link_infos) {
            if (candidate->_name == link_name) {
                link_info = candidate;
                break;
            }
        }
        if (!link_info) {
            throw std::runtime_error(boost::str(
                boost::format("Unable to find a corresponding link '%s'")
                % link_name));
        }

        // <geometry_group> element
        const char* geometry_group_name;
        for (TiXmlElement* geometry_group_element = link_handle.FirstChild("geometry_group").Element();
             geometry_group_element;
             geometry_group_element = geometry_group_element->NextSiblingElement("geometry_group"))
        {
            geometry_group_name = geometry_group_element->Attribute("name");
            if (!geometry_group_name) {
                throw std::runtime_error(boost::str(
                    boost::format("Unable to get geometry_group name for link '%s':")
                    % link_name));
            }

            // Add this geometry group to our set of groups
            geometry_group_names_set.insert(geometry_group_name);

            TiXmlHandle geometry_group_handle = TiXmlHandle(geometry_group_element);

            // <origin> element
            urdf::Pose origin_pose;
            for (TiXmlElement* origin_element = geometry_group_handle.FirstChild("origin").Element();
                 origin_element;
                 origin_element = origin_element->NextSiblingElement("origin"))
            {
                std::string rpy_str = std::string("0 0 0");
                const char* rpy = origin_element->Attribute("rpy");
                if (rpy) {
                    rpy_str = std::string(rpy);
                }

                std::string xyz_str = std::string("0 0 0");
                const char* xyz = origin_element->Attribute("xyz");
                if (xyz) {
                    xyz_str = std::string(xyz);
                }

                origin_pose.rotation.init(rpy_str);
                origin_pose.position.init(xyz_str);

                break; // only find first <origin> element
            }

            // <geometry> element
            for (TiXmlElement* geometry_element = geometry_group_handle.FirstChild("geometry").Element();
                 geometry_element;
                 geometry_element = geometry_element->NextSiblingElement("geometry"))
            {
                TiXmlHandle geometry_handle = TiXmlHandle(geometry_element);

                // <mesh> element
                for (TiXmlElement* mesh_element = geometry_handle.FirstChild("mesh").Element();
                     mesh_element;
                     mesh_element = mesh_element->NextSiblingElement("mesh"))
                {
                    const char* mesh_filename = mesh_element->Attribute("filename");
                    if (!mesh_filename) {
                        throw std::runtime_error(boost::str(
                            boost::format("Unable to get 'filename' attribute for"
                                          " mesh element for link '%s':")
                                          % link_name));
                    }

                    // Add this mesh to a separate collision geometry group
                    std::vector<OpenRAVE::KinBody::GeometryInfoPtr> &geom_infos
                            = link_info->_mapExtraGeometries[geometry_group_name];

                    OpenRAVE::KinBody::GeometryInfoPtr geom_info
                        = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
                    geom_info->_t = URDFPoseToRaveTransform(origin_pose);
                    geom_info->_bModifiable = false;

                    // Set collision geometry to be visible, so it can be
                    // seen in the 'visual' group in or_rviz
                    geom_info->_bVisible = true;

                    geom_info->_type = OpenRAVE::GT_TriMesh;
                    geom_info->_filenamecollision = resolveURI(mesh_filename);

                    // The mesh <scale> tag is not parsed, also apparently it
                    // doesn't seem to do anything
                    //geom_info->_vCollisionScale = URDFVectorToRaveVector(mesh_scale);

                    boost::shared_ptr<OpenRAVE::TriMesh> trimesh =
                                              boost::make_shared<OpenRAVE::TriMesh>();
                    trimesh = GetEnv()->ReadTrimeshURI(trimesh,
                                                       geom_info->_filenamecollision);
                    if (trimesh) {
                        // The _vCollisionScale property does nothing, so we have to
                        // manually scale the mesh.
                        BOOST_FOREACH(OpenRAVE::Vector &vertex, trimesh->vertices) {
                            vertex *= geom_info->_vCollisionScale;
                        }
                        geom_info->_meshcollision = *trimesh;
                    }
                    else {
                        RAVELOG_WARN("Link %s: Failed loading collision mesh %s \n",
                                     link_name, geom_info->_filenamecollision.c_str());
                    }

                    geom_infos.push_back(geom_info);

                    break; // only find first <mesh> element
                }

                break; // only find first <geometry> element
            }
        } // end <collision> element
    } // end <link> element

    // Each OpenRAVE link must be a member of each geometry group,
    // so iterate over all the links and add them to each of the
    // geometry groups.
    // If a specific geometry group (and mesh) was already specified for
    // a link, calling calling mapExtraGeometries[] again does nothing.
    BOOST_FOREACH(OpenRAVE::KinBody::LinkInfoPtr link_info, link_infos) {
        BOOST_FOREACH(std::string geometry_group_name, geometry_group_names_set) {
            link_info->_mapExtraGeometries[geometry_group_name];
        }
    }
}

/** Opens a URDF file and returns a robot in OpenRAVE */
bool URDFLoader::loadURI(std::ostream &soutput, std::istream &sinput)
{
  try {
    // Get filename from input arguments
    std::string input_urdf_uri, input_srdf_uri;
    sinput >> input_urdf_uri >> input_srdf_uri;

    std::string const input_urdf = resolveURIorPath(input_urdf_uri);
    std::string const input_srdf = resolveURIorPath(input_srdf_uri);

    // Parse the URDF file
    urdf::Model urdf_model;
    if (!urdf_model.initFile(input_urdf)) {
      throw std::runtime_error("Failed to open URDF file.");
    }

    // Parse the SRDF file, if specified
    std::shared_ptr<srdf::Model> srdf_model;
    if (!input_srdf.empty()) {
      srdf_model = std::make_shared<srdf::Model>();
      srdf_model->initFile(urdf_model, input_srdf);
    }

    // Parse the URDF again to find <geometry_group> elements
    TiXmlDocument xml_doc(input_urdf.c_str());
    if (!xml_doc.LoadFile()) {
      throw std::runtime_error("Could not load the URDF file.");
    }

    std::string uri = srdf_model == nullptr ?
                          input_urdf_uri :
                          input_urdf_uri + " " + input_srdf_uri;
    soutput << loadModel(urdf_model, xml_doc, srdf_model, uri);
    return true;
  } catch (std::runtime_error const &e) {
    RAVELOG_ERROR("Failed loading URDF model: %s\n", e.what());
    return false;
  }
}

/** load URDF and SRDF from file/URI with deprecated warning for "load" command
 * name
 */
bool URDFLoader::deprecatedLoad(std::ostream &soutput, std::istream &sinput)
{
  RAVELOG_WARN("URDFLoader 'load' command is deprecated. Use 'LoadURI' instead.\n");
  return loadURI(soutput, sinput);
}

/** Loads a JSON-wrapped URDF and optionally SRDF string and returns a robot in
 * OpenRAVE.
 *
 * The JSON object must have the following schema (srdf property is optional):
 *
 * { "urdf": "<?xml...", "srdf": "<?xml..." }
 */
bool URDFLoader::loadJsonString(std::ostream &soutput, std::istream &sinput)
{
  try {
    // load json wrapper
    std::string json_wrapper(std::istreambuf_iterator<char>(sinput), {});
    picojson::value json_v;
    std::string err = picojson::parse(json_v, json_wrapper);
    if (!err.empty()) {
      throw std::runtime_error("Failed to parse JSON wrapper: " + err);
    }

    if (!json_v.is<picojson::object>()) {
      throw std::runtime_error("JSON wrapper not JSON object");
    }
    picojson::object &obj = json_v.get<picojson::object>();

    // parse and and load URDF
    auto urdf_v = obj.find("urdf");
    if (urdf_v == obj.end()) {
      throw std::runtime_error("JSON wrapper has no \"urdf\" property");
    }
    if (!urdf_v->second.is<std::string>()) {
      throw std::runtime_error(
          "JSON wrapper \"urdf\" property is not a string");
    }
    std::string urdf_string = urdf_v->second.get<std::string>();

    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_string)) {
      throw std::runtime_error("Failed to load URDF from string.");
    }

    // optionally parse and load SRDF
    std::shared_ptr<srdf::Model> srdf_model;
    auto srdf_v = obj.find("srdf");
    if (srdf_v == obj.end()) {
      RAVELOG_INFO("JSON wrapper has no \"srdf\" property");
    } else {
      if (!srdf_v->second.is<std::string>()) {
        throw std::runtime_error(
            "JSON wrapper \"srdf\" property is not a string");
      }
      std::string srdf_string = srdf_v->second.get<std::string>();
      srdf_model = std::make_shared<srdf::Model>();

      if (!srdf_model->initString(urdf_model, srdf_string)) {
        throw std::runtime_error("Failed to load SRDF from string.");
      }
    }

    // Parse the URDF again to find <geometry_group> elements
    TiXmlDocument xml_doc;
    xml_doc.Parse(urdf_string.c_str());
    if (xml_doc.Error()) {
      std::string xmlerr = xml_doc.ErrorDesc();
      throw std::runtime_error("Could not parse the URDF file: " + xmlerr);
    }

    soutput << loadModel(urdf_model, xml_doc, srdf_model);
    return true;

  } catch (std::runtime_error const &e) {
    RAVELOG_ERROR("Failed loading URDF/SRDF model: %s\n", e.what());
    return false;
  }
}

std::string URDFLoader::loadModel(urdf::Model &urdf_model,
                                  TiXmlDocument &xml_doc,
                                  std::shared_ptr<srdf::Model> srdf_model,
                                  std::string uri)
{
  OpenRAVE::KinBodyPtr body;
  std::string name;

  std::vector<OpenRAVE::KinBody::LinkInfoPtr> link_infos;
  std::vector<OpenRAVE::KinBody::JointInfoPtr> joint_infos;
  ParseURDF(urdf_model, link_infos, joint_infos);

  std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> manip_infos;
  std::vector<OpenRAVE::RobotBase::AttachedSensorInfoPtr> sensor_infos;
  if (srdf_model != nullptr) {
    ParseSRDF(urdf_model, *srdf_model, link_infos, joint_infos, manip_infos);
  }

  ProcessGeometryGroupTagsFromURDF(xml_doc, link_infos);

  if (srdf_model != nullptr) {
    // Cast all of the vector contents to const.
    std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const =
        MakeConst(link_infos);
    std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const =
        MakeConst(joint_infos);
    std::vector<OpenRAVE::RobotBase::ManipulatorInfoConstPtr>
        manip_infos_const = MakeConst(manip_infos);
    std::vector<OpenRAVE::RobotBase::AttachedSensorInfoConstPtr>
        sensor_infos_const = MakeConst(sensor_infos);

    // TODO: Sort the joints to guarantee contiguous manipulators.

    OpenRAVE::RobotBasePtr robot = OpenRAVE::RaveCreateRobot(GetEnv(), "");
    robot->Init(link_infos_const, joint_infos_const, manip_infos_const,
                sensor_infos_const, uri);
    body = robot;
  }
  // It's just a URDF file, so create a KinBody.
  else {
    std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const =
        MakeConst(link_infos);
    std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const =
        MakeConst(joint_infos);

    body = OpenRAVE::RaveCreateKinBody(GetEnv(), "");
    body->Init(link_infos_const, joint_infos_const, uri);
  }

  body->SetName(urdf_model.getName());
  GetEnv()->Add(body, true);
  return body->GetName();
}

/** Resolves URIs for file:// and package:// paths */
std::string URDFLoader::resolveURI(const std::string &path) const
{
    std::string uri = path;

    if (uri.find("file://") == 0) {
        // Strip off the file://
        uri.erase(0, strlen("file://"));

        // Resolve the mesh path as a file URI
        boost::filesystem::path file_path(uri);
        return file_path.string();
    } else if (uri.find("package://") == 0) {
        return _catkin_finder.find(uri);
    } else {
        RAVELOG_WARN("Cannot handle this type of URI.\n");
        return "";
    }
}

std::string URDFLoader::resolveURIorPath(const std::string &path) const
{
    using boost::algorithm::starts_with;

    if (starts_with(path, "package://") || starts_with(path, "file://")) {
        return resolveURI(path);
    } else {
        return path;
    }
}

}
