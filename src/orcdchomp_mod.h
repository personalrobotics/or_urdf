
/* requires:
 *  - openrave/openrave.h
 * */

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
   
   bool viewspheres(std::ostream& sout, std::istream& sinput);
   bool computedistancefield(std::ostream& sout, std::istream& sinput);
   bool runchomp(std::ostream& sout, std::istream& sinput);

   mod(OpenRAVE::EnvironmentBasePtr penv) : OpenRAVE::ModuleBase(penv)
   {
      __description = "orcdchomp: implement chomp using libcd";
      RegisterCommand("viewspheres",boost::bind(&mod::viewspheres,this,_1,_2),"view spheres");
      RegisterCommand("computedistancefield",boost::bind(&mod::computedistancefield,this,_1,_2),"compute distance field");
      RegisterCommand("runchomp",boost::bind(&mod::runchomp,this,_1,_2),"run chomp");
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
