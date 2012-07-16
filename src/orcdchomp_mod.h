/** \file orcdchomp_mod.h
 * \brief Interface to the orcdchomp module, an implementation of CHOMP
 *        using libcd.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

/* requires:
 *  - openrave/openrave.h
 * */

#include "orcwrap.h"

namespace orcdchomp
{

struct sdf;

/* the module itself */
class mod : public OpenRAVE::ModuleBase
{
public:
   OpenRAVE::EnvironmentBasePtr e; /* filled on module creation */
   int n_sdfs;
   struct sdf * sdfs;
   
   int viewspheres(int argc, char * argv[], std::ostream& sout);
   int computedistancefield(int argc, char * argv[], std::ostream& sout);
   int runchomp(int argc, char * argv[], std::ostream& sout);

   mod(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ModuleBase(penv)
   {
      __description = "orcdchomp: implementation chomp using libcd";
      RegisterCommand("viewspheres",orcwrap(boost::bind(&mod::viewspheres,this,_1,_2,_3)),"view spheres");
      RegisterCommand("computedistancefield",orcwrap(boost::bind(&mod::computedistancefield,this,_1,_2,_3)),"compute distance field");
      RegisterCommand("runchomp",orcwrap(boost::bind(&mod::runchomp,this,_1,_2,_3)),"run chomp");
      
      this->e = penv;
      this->n_sdfs = 0;
      this->sdfs = 0;
   }
   virtual ~mod() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

} /* namespace orcdchomp */
