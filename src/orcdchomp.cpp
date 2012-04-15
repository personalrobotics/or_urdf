#include <cstdio>
#include <cstring>
#include <boost/bind.hpp>
#include <cblas.h>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <openrave/plugin.h>
extern "C" {
#include <libcd/grid.h>
#include <libcd/gs.h>
#include <libcd/kin.h>
#include <libcd/mat.h>
#include <libcd/png.h>
#include <libcd/util.h>
#include <libcd-gscpp.h>
}

#define TAU (6.283185307179586)

using namespace std;
using namespace OpenRAVE;


/* ======================================================================== *
 * module skeleton, openrave integration
 */

/* the module itself */
class orcdchomp : public ModuleBase
{
public:
   EnvironmentBasePtr e; /* filled on module creation */
   cd_grid * g;
   
   bool computedistancefield(ostream& sout, istream& sinput);

   orcdchomp(EnvironmentBasePtr penv) : ModuleBase(penv)
   {
      __description = "orcdchomp: implement chomp using libcd";
      RegisterCommand("computedistancefield",boost::bind(&orcdchomp::computedistancefield,this,_1,_2),"compute distance field");
      this->e = penv;
      this->g = 0;
   }
   virtual ~orcdchomp() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

/* export as an openrave module */
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if((type == PT_Module)&&(interfacename == "orcdchomp"))
        return InterfaceBasePtr(new orcdchomp(penv));
    return InterfaceBasePtr();
}
void GetPluginAttributesValidated(PLUGININFO& info)
   { info.interfacenames[PT_Module].push_back("orcdchomp"); }
OPENRAVE_PLUGIN_API void DestroyPlugin()
   { RAVELOG_INFO("destroying plugin\n"); }


/* ======================================================================== *
 * useful utilities
 */

char * str_from_istream(istream& sinput)
{
   char * str;
   std::ostringstream oss;
   oss << sinput.rdbuf();
   str = (char *) malloc(strlen(oss.str().c_str())+1);
   if (!str) return 0;
   strcpy(str, oss.str().c_str());
   return str;
}

/* skips spaces, then prefix;
 * returns 1 on success, 0 on failure */
int strp_skipprefix(char ** strp, char * prefix)
{
   int len;
   char * str;
   str = *strp;
   while (*str==' ') str++;
   len = strlen(prefix);
   if (strncmp(str,prefix,len))
      return 0;
   *strp = str + len;
   return 1;
}


/* ======================================================================== *
 * module commands
 */

/* computedistancefield robot Herb2
 * computes a distance field in the vicinity of the passed robot
 * */
bool orcdchomp::computedistancefield(ostream& sout, istream& sinput)
{
   int err;
   int i;
   int j;
   int k;
   char * in;
   char * cur;
   char buf[64];
   int len;
   EnvironmentMutex::scoped_lock lockenv;
   RobotBase * robot;
   double temp;
   
   robot = 0;
   
   /* lock environment */
   lockenv = EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* parse arguments into manip, n_frees, object, placement_tsr, grasp_tsr */
   in = str_from_istream(sinput);
   if (!in) { RAVELOG_ERROR("Out of memory!\n"); return false; }
   cur = in;
   while (1)
   {
      if (strp_skipprefix(&cur, (char *)"robot"))
      {
         if (robot) { RAVELOG_ERROR("Only one robot can be passed!\n"); free(in); return false; }
         sscanf(cur, " %s%n", buf, &len); cur += len;
         robot = this->e->GetRobot(buf).get();
         if (!robot) { RAVELOG_ERROR("Could not find robot with name %s!\n",buf); free(in); return false; }
         RAVELOG_INFO("Using robot %s.\n", robot->GetName().c_str());
         continue;
      }
      break;
   }
   if (cur[0]) RAVELOG_ERROR("remaining string: |%s|! continuing ...\n", cur);
   free(in);
   
   /* check that we have everything */
   if (!robot)
      { RAVELOG_ERROR("Did not pass all required args!\n"); return false; }
      
   if (this->g) cd_grid_destroy(this->g);
   
   /* Create a new grid located around the current robot; 100x100x100
    * for now, this is axis-aligned. */
   temp = 0.0; /* free space */
   err = cd_grid_create(&this->g, &temp, sizeof(double), 3, 10, 10, 10);
   if (err) { this->g=0; RAVELOG_ERROR("Not enough memory for distance field!\n"); return false; }
   
   /* fill it with cubes! */
   {
      OpenRAVE::KinBodyPtr b = OpenRAVE::RaveCreateKinBody(this->e, "cube");
      
      std::vector<AABB> vaabbs(1);
      vaabbs[0].pos = OpenRAVE::Vector(0.0, 0.1, 0.2);
      vaabbs[0].extents = OpenRAVE::Vector(0.1, 0.2, 0.3);
      b->InitFromBoxes(vaabbs, 1); /* draw */
      
      this->e->AddKinBody(b);
   }
   
   
   return true;
}
