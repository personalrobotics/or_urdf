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
   int create(int argc, char * argv[], std::ostream& sout);
   int iterate(int argc, char * argv[], std::ostream& sout);
   int gettraj(int argc, char * argv[], std::ostream& sout);
   int destroy(int argc, char * argv[], std::ostream& sout);

   mod(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ModuleBase(penv)
   {
      __description = "orcdchomp: implementation chomp using libcd";
      RegisterCommand("viewspheres",orcwrap(boost::bind(&mod::viewspheres,this,_1,_2,_3)),"view spheres");
      RegisterCommand("computedistancefield",orcwrap(boost::bind(&mod::computedistancefield,this,_1,_2,_3)),"compute distance field");
      RegisterCommand("create",orcwrap(boost::bind(&mod::create,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("iterate",orcwrap(boost::bind(&mod::iterate,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("gettraj",orcwrap(boost::bind(&mod::gettraj,this,_1,_2,_3)),"create a chomp run");
      RegisterCommand("destroy",orcwrap(boost::bind(&mod::destroy,this,_1,_2,_3)),"create a chomp run");
      
      this->e = penv;
      this->n_sdfs = 0;
      this->sdfs = 0;
   }
   virtual ~mod() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

void run_destroy(struct run * r);

struct tsr
{
   int manipindex;
   char bodyandlink[32];
   double T0w[7];
   double Twe[7];
   double Bw[6][2];
};

int tsr_create_parse(struct tsr ** tp, char * str);
void tsr_destroy(struct tsr * t);

} /* namespace orcdchomp */
