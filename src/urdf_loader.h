/** \file urdf_loader.h
 * \brief Interface to a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */
#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include <openrave/openrave.h>
#include <boost/bind.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include "catkin_finder.h"

namespace or_urdf
{
  class URDFLoader : public OpenRAVE::ModuleBase
  {
  public:
    /** Opens a URDF file and returns a robot in OpenRAVE */
    bool load(std::ostream& sout, std::istream& sin);
    
    URDFLoader(OpenRAVE::EnvironmentBasePtr env);
    
    virtual ~URDFLoader();

    void Destroy();

    void ParseURDF(urdf::Model &model, std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                   std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos);

    void ParseSRDF(urdf::Model const &urdf,
                   srdf::Model const &srdf,
                   std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
                   std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos,
                   std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> &manip_infos);

    void ProcessGeometryGroupTagsFromURDF(
                   TiXmlDocument &xml_doc,
                   std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos);

  private:
    /** Reference to OpenRAVE environment, filled in on construction */
    OpenRAVE::EnvironmentBasePtr _env;
    or_urdf::CatkinFinder _catkin_finder;

    std::string resolveURI(const std::string &path) const;
    std::string resolveURIorPath(const std::string &path) const;
  };
  
} /* namespace or_urdf */

#endif // URDF_LOADER_H
