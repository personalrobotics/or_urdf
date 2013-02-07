/** \file urdf_loader.h
 * \brief Interface to a URDF loading plugin for OpenRAVE
 * \author Pras Velagapudi
 * \date 2013
 */

/* (C) Copyright 2013 Carnegie Mellon University */

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>

namespace urdf_loader
{

  class URDFLoader : public OpenRAVE::ModuleBase
  {
  public:
    OpenRAVE::EnvironmentBasePtr _env; /* filled on module creation */
    
    /** Opens a URDF file and returns a robot in OpenRAVE */
    bool load(ostream& sout, istream& sin);
    
    /** Constructs plugin and registers functions */
    URDFLoader(OpenRAVE::EnvironmentBasePtr env) : OpenRAVE::ModuleBase(env)
    {
      __description = "URDFLoader: Loader that imports URDF files.";
      _env = env;

      RegisterCommand("load", orcwrap(boost::bind(&URDFLoader::load, this, _1, _2)), "loads URDF from file");     
    }
    
    virtual ~URDFLoader() {}

    void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
    
    /* This is called on env.LoadProblem(m, 'command') */
    int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
  };
  
} /* namespace urdf_loader */
