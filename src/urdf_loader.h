/** \file urdf_loader.h
 * \brief Interface to a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */
#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include <memory>
#include <string>

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <urdf/model.h>
#include <srdfdom/model.h>

#include <tinyxml.h>

#include "catkin_finder.h"

namespace or_urdf
{
  class URDFLoader : public OpenRAVE::ModuleBase
  {
  public:
    /** Opens a URDF file and returns a robot in OpenRAVE */
    bool loadURI(std::ostream& sout, std::istream& sin);
    bool loadJsonString(std::ostream& sout, std::istream& sin);
    bool deprecatedLoad(std::ostream&, std::istream& sin);
    
    /** Constructs plugin and registers functions */
    URDFLoader(OpenRAVE::EnvironmentBasePtr env) : OpenRAVE::ModuleBase(env)
    {
      __description = "URDFLoader: Loader that imports URDF files.";
      _env = env;

      RegisterCommand("LoadURI", boost::bind(&URDFLoader::loadURI, this, _1, _2),
                      "load URDF and SRDF from file/URI");
      RegisterCommand("LoadString", boost::bind(&URDFLoader::loadJsonString, this, _1, _2),
                      "load URDF and SRDF from json string wrapping XML");
      RegisterCommand("load", boost::bind(&URDFLoader::deprecatedLoad, this, _1, _2),
                      "Deprecated method name to load URDF and SRDF from file/URI");
    }

    void Destroy() { RAVELOG_INFO("URDF loader unloaded from environment\n"); }
    
    virtual ~URDFLoader() {}

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
    
    /* This is called on env.LoadProblem(m, 'command') */
    int main(const std::string& cmd) { RAVELOG_INFO("URDF loader initialized with command: %s\n", cmd.c_str()); return 0; }

  private:
    /** Reference to OpenRAVE environment, filled in on construction */
    OpenRAVE::EnvironmentBasePtr _env;
    or_urdf::CatkinFinder _catkin_finder;

    std::string loadModel(urdf::Model &urdf_model, TiXmlDocument &xml_doc,
                     std::shared_ptr<srdf::Model> srdf_model = nullptr,
                     std::string uri = std::string());
    std::string resolveURI(const std::string &path) const;
    std::string resolveURIorPath(const std::string &path) const;
  };
  
} /* namespace or_urdf */

#endif // URDF_LOADER_H
