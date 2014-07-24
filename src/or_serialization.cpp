#include <tinyxml.h>
#include <boost/format.hpp>
#include <openrave/openrave.h>

using boost::format;
using boost::str;
using OpenRAVE::KinBody;
using OpenRAVE::RobotBase;
using OpenRAVE::openrave_exception;
typedef KinBody::LinkInfo LinkInfo;
typedef KinBody::JointInfo JointInfo;
typedef KinBody::GeometryInfo GeometryInfo;
typedef KinBody::GeometryInfoPtr GeometryInfoPtr;
typedef RobotBase::ManipulatorInfo ManipulatorInfo;

void MakeText(TiXmlElement *parent, std::string const &name,
              std::string const &value)
{
    TiXmlElement *node = new TiXmlElement(name);
    node->LinkEndChild(new TiXmlText(value));
    parent->LinkEndChild(node);
}

std::string SerializeBoolean(bool flag)
{
    if (flag) {
        return "true";
    } else {
        return "false";
    }
}

std::string SerializeScalar(OpenRAVE::dReal x)
{
    std::stringstream ss;
    ss << x;
    return ss.str();
}

std::string SerializeVector3(OpenRAVE::Vector const &v)
{
    std::stringstream ss;
    ss << v[0] << " " << v[1] << " " << v[2];
    return ss.str();
}

std::string SerializeVector4(OpenRAVE::Vector const &v)
{
    std::stringstream ss;
    ss << v[0] << " " << v[1] << " " << v[2] << " " << v[3];
    return ss.str();
}

TiXmlElement *SerializeGeom(TiXmlElement *parent, GeometryInfo const &geom_info)
{
    BOOST_ASSERT(parent);

    TiXmlElement *geom_ele = new TiXmlElement("Geom");
    geom_ele->SetAttribute("visible", SerializeBoolean(geom_info._bVisible));
    geom_ele->SetAttribute("modifiable", SerializeBoolean(geom_info._bModifiable));
    MakeText(geom_ele, "Translation", SerializeVector4(geom_info._t.trans));
    MakeText(geom_ele, "Quat", SerializeVector4(geom_info._t.rot));
    MakeText(geom_ele, "AmbientColor", SerializeVector4(geom_info._vAmbientColor));
    MakeText(geom_ele, "DiffuseColor", SerializeVector4(geom_info._vDiffuseColor));
    MakeText(geom_ele, "Transparency", SerializeScalar(geom_info._fTransparency));

    // TODO: Collision scale.
    // TODO: Render scale.

    if (!geom_info._filenamerender.empty()) {
        MakeText(geom_ele, "Render", geom_info._filenamerender);
    }

    std::string type;
    switch (geom_info._type) {
    case OpenRAVE::GT_None:
        delete geom_ele;
        return NULL;

    case OpenRAVE::GT_Box:
        geom_ele->SetAttribute("type", "box");
        MakeText(geom_ele, "Extents", SerializeVector3(geom_info._vGeomData));
        break;

    case OpenRAVE::GT_Sphere:
        geom_ele->SetAttribute("type", "sphere");
        MakeText(geom_ele, "Radius", SerializeScalar(geom_info._vGeomData[0]));
        break;

    case OpenRAVE::GT_Cylinder:
        geom_ele->SetAttribute("type", "cylinder");
        MakeText(geom_ele, "Radius", SerializeScalar(geom_info._vGeomData[0]));
        MakeText(geom_ele, "Height", SerializeScalar(geom_info._vGeomData[1]));
        break;

    case OpenRAVE::GT_TriMesh:
        geom_ele->SetAttribute("type", "trimesh");
        MakeText(geom_ele, "Data", geom_info._filenamecollision);
        break;

    default:
        RAVELOG_WARN("Unknown type of geometry '%d'.\n", geom_info._type);
    }

    parent->LinkEndChild(geom_ele);
    return geom_ele;
}

TiXmlElement *SerializeLink(TiXmlElement *parent, LinkInfo const &link_info)
{
    BOOST_ASSERT(parent);

    std::string const type_str = (link_info._bStatic) ? "static" : "dynamic";

    TiXmlElement *link_ele = new TiXmlElement("Body");
    link_ele->SetAttribute("name", link_info._name);
    link_ele->SetAttribute("type", type_str);
    link_ele->SetAttribute("enable", SerializeBoolean(link_info._bIsEnabled));
    MakeText(link_ele, "Translation", SerializeVector4(link_info._t.trans));
    MakeText(link_ele, "Quat", SerializeVector4(link_info._t.rot));

    // TODO: Serialize _tMassFrame and _vinertiamoments.
    // TODO: Serialize _vForcedAdjacentLinks.

    for (GeometryInfoPtr const &geom_info : link_info._vgeometryinfos) {
        SerializeGeom(parent, *geom_info);
    }

    parent->LinkEndChild(link_ele);
    return link_ele;
}

TiXmlElement *SerializeJoint(TiXmlElement *parent, JointInfo const &joint_info)
{
    BOOST_ASSERT(parent);

    TiXmlElement *joint_ele = new TiXmlElement("Joint");
    joint_ele->SetAttribute("name", joint_info._name);
    joint_ele->SetAttribute("enable", SerializeBoolean(joint_info._bIsActive));
    joint_ele->SetAttribute("circular", SerializeBoolean(joint_info._bIsCircular[0]));

    MakeText(joint_ele, "Link", joint_info._linkname0);
    MakeText(joint_ele, "Link", joint_info._linkname1);
    MakeText(joint_ele, "Anchor", SerializeVector3(joint_info._vanchor));
    MakeText(joint_ele, "Axis", SerializeVector3(joint_info._vaxes[0]));

    BOOST_ASSERT(joint_info._vcurrentvalues.size() == 1);
    MakeText(joint_ele, "Initial", SerializeScalar(joint_info._vcurrentvalues[0]));
    MakeText(joint_ele, "Limits", SerializeScalar(joint_info._vlowerlimit[0])
                          + " " + SerializeScalar(joint_info._vupperlimit[1]));

    if (joint_info._vmaxvel[0] != 0) {
        MakeText(joint_ele, "MaxVel", SerializeScalar(joint_info._vmaxvel[0]));
    }
    if (joint_info._vmaxaccel[0] != 0) {
        MakeText(joint_ele, "MaxAccel", SerializeScalar(joint_info._vmaxaccel[0]));
    }
    if (joint_info._vmaxtorque[0] != 0) {
        MakeText(joint_ele, "MaxTorque", SerializeScalar(joint_info._vmaxtorque[0]));
    }
    if (joint_info._vresolution[0] != 0) {
        MakeText(joint_ele, "Resolution", SerializeScalar(joint_info._vresolution[0]));
    }
    if (joint_info._vweights[0] != 0) {
        MakeText(joint_ele, "Weight", SerializeScalar(joint_info._vweights[0]));
    }

    switch (joint_info._type) {
    case KinBody::JointNone:
        joint_ele->SetAttribute("type", "none");
        break;
        
    case KinBody::JointRevolute: // = KinBody::JointHinge
        joint_ele->SetAttribute("type", "revolute");
        break;

    case KinBody::JointPrismatic: // = KinBody::JointSlider:
        joint_ele->SetAttribute("type", "prismatic");
        break;

    case KinBody::JointRR:
    case KinBody::JointRP:
    case KinBody::JointPR:
    case KinBody::JointPP:
        delete joint_ele;
        throw openrave_exception(str(
            format("Serializing multi-DOF joint type %d is not implemented.")
                % joint_info._type
            ), OpenRAVE::ORE_NotImplemented
        );
        
    default:
        delete joint_ele;
        if (joint_info._type | KinBody::JointSpecialBit) {
            throw openrave_exception(str(
                format("Serializing special joint type %d is not implemented.")
                    % joint_info._type
                ), OpenRAVE::ORE_NotImplemented
            );
        } else {
            throw openrave_exception(str(
                format("Unknown joint type %d.") % joint_info._type),
                OpenRAVE::ORE_Failed
            );
        }
    }

    parent->LinkEndChild(joint_ele);
    return joint_ele;
}


