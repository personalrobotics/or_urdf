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
    /** Opens a URDF file and returns a robot in OpenRAVE */
    bool load(std::ostream& sout, std::istream& sin);
    
    /** Constructs plugin and registers functions */
    URDFLoader(OpenRAVE::EnvironmentBasePtr env) : OpenRAVE::ModuleBase(env)
    {
      __description = "URDFLoader: Loader that imports URDF files.";
      _env = env;

      RegisterCommand("load", boost::bind(&URDFLoader::load, this, _1, _2), "loads URDF from file");     
    }
    
    virtual ~URDFLoader() {}

    void Destroy() { RAVELOG_INFO("URDF loader unloaded from environment\n"); }
    
    /* This is called on env.LoadProblem(m, 'command') */
    int main(const std::string& cmd) { RAVELOG_INFO("URDF loader initialized with command: %s\n", cmd.c_str()); return 0; }

  private:
    /** Reference to OpenRAVE environment, filled in on construction */
    OpenRAVE::EnvironmentBasePtr _env;
  };
  
} /* namespace urdf_loader */
