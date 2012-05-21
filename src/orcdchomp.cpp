#include <cstdio>
#include <cstring>
#include <boost/bind.hpp>
#include <cblas.h>
#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <openrave/plugin.h>
extern "C" {
#include <libcd/chomp.h>
#include <libcd/grid.h>
#include <libcd/kin.h>
#include <libcd/mat.h>
#include <libcd/util.h>
}

#define N_INTPOINTS (99)

#define EPSILON (0.1) /* in meters */
#define EPSILON_SELF (0.04) /* in meters */
#define OBS_FACTOR (200.0)
#define OBS_FACTOR_SELF (10.0)

using namespace OpenRAVE;

#if 0
/* default openrave wam7 */
static struct sphere spheres[] =
{
   /* shoulder spheres */
   {"wam0", -1, {0.22, 0.14, 0.346}, 0.15},
   /* upper arm spheres */
   {"wam2", -1, {0.0, -0.20, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.30, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.40, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.50, 0.0}, 0.06},
   /* elbow knuckle spheres */
   {"wam3", -1, {0.045, 0.0, 0.55}, 0.06},
   /* forearm spheres */
   {"wam4", -1, {-0.045, -0.2, 0.0}, 0.06},
   {"wam4", -1, {-0.045, -0.1, 0.0}, 0.06},
   {"wam4", -1, {-0.045, -0.3, 0.0}, 0.06},
   /* hand sphere */
   {"wam6", -1, {0.0, -0.06-0.04, 0.0}, 0.06},
   /* finger spheres (inner links) */
   {"Finger0-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   {"Finger1-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   {"Finger2-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   /* finger spheres (tip links) */
   {"Finger0-2", -1, { 0.05, 0.0, 0.0}, 0.04},
   {"Finger1-2", -1, { 0.05, 0.0, 0.0}, 0.04},
   {"Finger2-2", -1, { 0.05, 0.0, 0.0}, 0.04}
};
#endif


/* ======================================================================== *
 * type definitions, module skeleton, things exposed to openrave
 */

namespace {

struct sphere
{
   struct sphere * next;
   /* parsed from xml */
   char linkname[32];
   double pos[3];
   double radius;
   /* solved for later */
   int linkindex; /* solved for on init */
};

/* the robot-attached data class */
class rdata : public XMLReadable
{
public:
   rdata() : XMLReadable("orcdchomp") { this->spheres = 0; }
   ~rdata()
   {
      struct sphere * s;
      while (this->spheres)
      {
         s=this->spheres->next;
         free(this->spheres);
         this->spheres = s;
      }
   }
   struct sphere * spheres;
};

/* the rdata-parser */
class rdata_parser : public BaseXMLReader
{
public:
   rdata_parser(boost::shared_ptr<rdata> passed_d, const AttributesList& atts)
   {
      /* save or construct the rdata object */
      this->d = passed_d;
      if(!this->d) this->d.reset(new rdata());
      /* get ready */
      this->inside_spheres = false;
   }

   virtual XMLReadablePtr GetReadable() { return this->d; }

   virtual ProcessElement startElement(const std::string& name, const AttributesList& atts)
   {
      if (name == "spheres")
      {
         if (this->inside_spheres) RAVELOG_ERROR("you can't have <spheres> inside <spheres>!\n");
         this->inside_spheres = true;
         return PE_Support;
      }
      if (name == "sphere")
      {
         struct sphere * s;
         if (!this->inside_spheres) { RAVELOG_ERROR("you can't have <sphere> not inside <spheres>!\n"); return PE_Pass; }
         s = (struct sphere *) malloc(sizeof(struct sphere));
         for(AttributesList::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt)
         {
            if (itatt->first=="link")
               strcpy(s->linkname, itatt->second.c_str());
            else if (itatt->first=="radius")
               s->radius = strtod(itatt->second.c_str(), 0);
            else if (itatt->first=="pos")
               sscanf(itatt->second.c_str(), "%lf %lf %lf", &s->pos[0], &s->pos[1], &s->pos[2]);
            else
               RAVELOG_ERROR("unknown attribute %s=%s!\n",itatt->first.c_str(),itatt->second.c_str());
         }
         /* insert at head of rdata list */
         s->next = this->d->spheres;
         this->d->spheres = s;
         return PE_Support;
      }
      return PE_Pass;
   }
   
   virtual void characters(const std::string& ch) { }
   
   virtual bool endElement(const std::string& name)
   {
      if (name == "orcdchomp") return true;
      if (name == "spheres")
      {
         if (!this->inside_spheres) RAVELOG_ERROR("you can't have </spheres> without matching <spheres>!\n");
         this->inside_spheres = false;
      }
      else if (name == "sphere")
      {
         if (!this->inside_spheres) RAVELOG_ERROR("you can't have </sphere> not inside <spheres>!\n");
      }
      else
         RAVELOG_ERROR("unknown field %s\n", name.c_str());
      return false;
   }

   boost::shared_ptr<rdata> d;
   bool inside_spheres;
};

/* the module itself */
class mod : public ModuleBase
{
public:
   EnvironmentBasePtr e; /* filled on module creation */
   double g_pos[3]; /* location of the zero-corner of the grid */
   cd_grid * g_sdf;
   
   bool viewspheres(std::ostream& sout, std::istream& sinput);
   bool computedistancefield(std::ostream& sout, std::istream& sinput);
   bool runchomp(std::ostream& sout, std::istream& sinput);

   mod(EnvironmentBasePtr penv) : ModuleBase(penv)
   {
      __description = "orcdchomp: implement chomp using libcd";
      RegisterCommand("viewspheres",boost::bind(&mod::viewspheres,this,_1,_2),"view spheres");
      RegisterCommand("computedistancefield",boost::bind(&mod::computedistancefield,this,_1,_2),"compute distance field");
      RegisterCommand("runchomp",boost::bind(&mod::runchomp,this,_1,_2),"run chomp");
      this->e = penv;
      this->g_sdf = 0;
   }
   virtual ~mod() {}
   void Destroy() { RAVELOG_INFO("module unloaded from environment\n"); }
   /* This is called on e.LoadProblem(m, 'command') */
   int main(const std::string& cmd) { RAVELOG_INFO("module init cmd: %s\n", cmd.c_str()); return 0; }
};

} /* anonymous namespace */


/* ======================================================================== *
 * globals and direct openrave callables
 */

namespace {
static boost::shared_ptr<void> reg_reader;
static BaseXMLReaderPtr rdata_parser_maker(InterfaceBasePtr ptr, const AttributesList& atts)
   { return BaseXMLReaderPtr(new rdata_parser(boost::shared_ptr<rdata>(),atts)); }
}

/* export as an openrave module */
void GetPluginAttributesValidated(PLUGININFO& info)
{ 
   if(!reg_reader) reg_reader = RaveRegisterXMLReader(PT_Robot,"orcdchomp",rdata_parser_maker);
   info.interfacenames[PT_Module].push_back("orcdchomp");
}
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
   if((type == PT_Module)&&(interfacename == "orcdchomp"))
      return InterfaceBasePtr(new mod(penv));
   return InterfaceBasePtr();
}
OPENRAVE_PLUGIN_API void DestroyPlugin()
   { RAVELOG_INFO("destroying plugin\n"); }


/* ======================================================================== *
 * implementation below!
 */

namespace {

/* ======================================================================== *
 * useful utilities
 */

char * str_from_istream(std::istream& sinput)
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

/* For now, this wraps all dimensions */
int grid_flood_fill(struct cd_grid * g, size_t index_start, int (*replace)(void *, void *), void * rptr)
{
   struct stack
   {
      struct stack * next;
      size_t index;
   };
   
   int ret;
   struct stack * s;
   struct stack * popped;
   size_t index;
   int ni;
   int pm;
   int * subs;
   int sub_save;
   
   subs = (int *) malloc(g->n * sizeof(int));
   if (!subs) return -1;
   
   /* Start with a stack with one node */
   s = (struct stack *) malloc(sizeof(struct stack));
   if (!s) { free(subs); return -1; }
   s->next = 0;
   s->index = index_start;
   
   /* Flood fill! */
   ret = 0;
   while (s)
   {
      /* Pop */
      popped = s;
      s = s->next;
      index = popped->index;
      free(popped);
      
      /* Attempt replace (continue if failed) */
      if (!replace(cd_grid_get_index(g,index), rptr)) continue;
      
      cd_grid_index_to_subs(g, index, subs);
      
      /* Add neighbors to stack */
      for (ni=0; ni<g->n; ni++)
      for (pm=0; pm<2; pm++)
      {
         sub_save = subs[ni];
         subs[ni] += pm==0 ? -1 : 1;
         /* don't wrap */
         if (subs[ni] < 0) { subs[ni] = sub_save; continue; } /*subs[ni] += g->sizes[ni];*/
         if (subs[ni] >= g->sizes[ni]) { subs[ni] = sub_save; continue; } /*subs[ni] -= g->sizes[ni];*/
         cd_grid_index_from_subs(g, &index, subs);
         subs[ni] = sub_save;
         
         popped = (struct stack *) malloc(sizeof(struct stack));
         if (!popped) { ret = -1; continue; }
         popped->next = s;
         popped->index = index;
         s = popped;
      }
   }
   
   free(subs);
   return ret;
}

int replace_1_to_0(double * val, void * rptr)
{
   if (*val == 1.0)
   {
      *val = 0.0;
      return 1;
   }
   return 0;
}


/* ======================================================================== *
 * module commands
 */

bool mod::viewspheres(std::ostream& sout, std::istream& sinput)
{
   int i;
   EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   RobotBasePtr r;
   char buf[1024];
   struct sphere * s;
   struct sphere * spheres;
   
   /* parse arguments into robot */
   {
      char * in;
      char * cur;
      char buf[64];
      int len;
      in = str_from_istream(sinput);
      if (!in) { throw openrave_exception("Out of memory!"); }
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (r.get()) { free(in); throw openrave_exception("Only one robot can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            r = this->e->GetRobot(buf)/*.get()*/;
            if (!r.get()) { free(in); throw openrave_exception("Could not find robot with that name!"); }
            RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!r.get()) { throw openrave_exception("Did not pass all required args!"); }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<rdata> d = boost::dynamic_pointer_cast<rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { throw openrave_exception("robot does not have a <orcdchomp> tag defined!"); }
      spheres = d->spheres;
   }
   
   /* view each sphere */
   for (s=spheres,i=0; s; s=s->next,i++)
   {
      /* make some sweet spheres! */
      OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
      sprintf(buf, "orcdchomp_sphere_%d", i);
      sbody->SetName(buf);
      /* set its dimensions */
      {
         std::vector< OpenRAVE::Vector > svec;
         Transform t = r->GetLink(s->linkname)->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
         v.w = s->radius; /* radius */
         svec.push_back(v);
         sbody->InitFromSpheres(svec, true);
      }
      /* add the sphere */
      this->e->AddKinBody(sbody);
   }
   
   return true;
}

/* computedistancefield robot Herb2
 * computes a distance field in the vicinity of the passed robot
 * */
bool mod::computedistancefield(std::ostream& sout, std::istream& sinput)
{
   int err;
   EnvironmentMutex::scoped_lock lockenv;
   RobotBase * robot;
   double temp;
   OpenRAVE::KinBodyPtr cube;
   size_t idx;
   struct cd_grid * g_obs;
   
   /* lock environment */
   lockenv = EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   robot = 0;
   
   /* parse arguments into robot */
   {
      char * in;
      char * cur;
      char buf[64];
      int len;
      in = str_from_istream(sinput);
      if (!in) { throw openrave_exception("Out of memory!"); }
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (robot) { free(in); throw openrave_exception("Only one robot can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            robot = this->e->GetRobot(buf).get();
            if (!robot) { free(in); throw openrave_exception("Could not find robot with that name!"); }
            RAVELOG_INFO("Using robot %s.\n", robot->GetName().c_str());
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!robot) throw openrave_exception("Did not pass all required args!");

   /* the grid position is 2m in -x, 2m in -y, and 1m in -z from the robot location */
   Transform t = robot->GetTransform();
   this->g_pos[0] = t.trans.x - 2.0;
   this->g_pos[1] = t.trans.y - 2.0;
   this->g_pos[2] = t.trans.z - 1.0;
   
   /* Create a new grid located around the current robot; 100x100x100
    * for now, this is axis-aligned.
    * the box is centered around (0,0,1).
    * the box has extents +/- 2.0m, so each cube is 4cm on a side. */
   temp = 1.0; /* free space */
   err = cd_grid_create(&g_obs, &temp, sizeof(double), 3, 100, 100, 100);
   if (err) throw openrave_exception("Not enough memory for distance field!");
   cd_mat_fill(g_obs->lengths, 3, 1, 4.0, 4.0, 4.0);
   
   /* create the cube */
   cube = OpenRAVE::RaveCreateKinBody(this->e);
   cube->SetName("cube");
   
   /* set its dimensions */
   {
      std::vector<AABB> vaabbs(1);
      vaabbs[0].extents = OpenRAVE::Vector(0.02, 0.02, 0.02); /* extents = half side lengths */
      cube->InitFromBoxes(vaabbs, 1);
   }
   
   /* add the cube */
   this->e->AddKinBody(cube);

   int cols = 0;
   
   /* go through the grid, testing for collision as we go;
    * collisions are HUGE_VAL, free are 1.0 */
   printf("computing occupancy grid ...\n");
   for (idx=0; idx<g_obs->ncells; idx++)
   {
      double center[3];
      Transform t;
      
      if (idx % 100000 == 0)
         printf("idx=%d ...\n", (int)idx);
      
      /* set cube location */
      t.identity();
      cd_grid_center_index(g_obs, idx, center);
      t.trans.x = this->g_pos[0] + center[0];
      t.trans.y = this->g_pos[1] + center[1];
      t.trans.z = this->g_pos[2] + center[2];
      cube->SetTransform(t);
      
      /* do collision check */
      if (this->e->CheckCollision(cube))
      {
         *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
         cols++;
      }
   }

   printf("found %d collisions!\n",cols);
   
   /* remove cube */
   this->e->Remove(cube);
   
   /* we assume the point at the very top is free;
    * do flood fill to set all cells that are 1.0 -> 0.0 */
   printf("performing flood fill ...\n");
   {
      double point[3] = {2.0, 2.0, 3.999}; /* center, at the top */
      cd_grid_lookup_index(g_obs, point, &idx);
   }
   grid_flood_fill(g_obs, idx, (int (*)(void *, void *))replace_1_to_0, 0);
   
   /* change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles) */
   for (idx=0; idx<g_obs->ncells; idx++)
      if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
         *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
   
   /* compute the signed distance field (in the module instance) */
   printf("computing signed distance field ...\n");
   if (this->g_sdf) cd_grid_destroy(this->g_sdf);
   cd_grid_double_bin_sdf(&this->g_sdf, g_obs);
   cd_grid_destroy(g_obs);
   
   return true;
}


struct cost_helper
{
   int n; /* config space dimensionality */
   double * g_pos;
   struct cd_grid * g_sdf;
   RobotBase * r;
   int * adofindices;
   struct sphere * spheres;
   double * J; /* space for the jacobian; 3xn */
   double * J2;
   
   /* ugh */
   EnvironmentBasePtr e;
};

/* cost over a bunch of body points */
int sphere_cost(struct cost_helper * h, double * c_point, double * c_vel, double * costp)
{
   int i;
   int j;
   double x_vel[3];
   double g_point[3];
   double dist;
   double cost;
   double cost_sphere;
   struct sphere * s;
   struct sphere * s2;
   
   /* put the robot in the config */
   std::vector<dReal> vec(c_point, c_point+h->n);
   h->r->SetActiveDOFValues(vec);

   cost = 0.0;
   
   for (s=h->spheres; s; s=s->next)
   {
      /* get sphere center */
      Transform t = h->r->GetLink(s->linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
      
      /* compute the manipulator jacobian at this point, at this link */
      boost::multi_array< dReal, 2 > orjacobian;
      h->r->CalculateJacobian(s->linkindex, v, orjacobian);
      /* copy the active columns of orjacobian into our J */
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            h->J[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      /* compute the current workspace velocity of the sphere */
      cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, h->n,
         1.0, h->J,h->n, c_vel,1, 0.0, x_vel,1);
      
      /* transform into grid frame */
      g_point[0] = v.x - h->g_pos[0];
      g_point[1] = v.y - h->g_pos[1];
      g_point[2] = v.z - h->g_pos[2];
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->g_sdf, g_point, &dist);
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= s->radius;
      
      /* convert to a cost */
      if (dist < 0.0)
         cost_sphere = OBS_FACTOR * (0.5 * EPSILON - dist);
      else if (dist < EPSILON)
         cost_sphere = OBS_FACTOR * (0.5/EPSILON) * (dist-EPSILON) * (dist-EPSILON);
      else
         cost_sphere = 0.0;
      
      /* consider effects from all other spheres (i.e. self collision) */
      for (s2=h->spheres; s2; s2=s2->next)
      {
         /* skip spheres on the same link */
         if (s->linkindex == s2->linkindex) continue;
         
         /* skip spheres far enough away from us */
         dist = s->radius + s2->radius + EPSILON_SELF;
         OpenRAVE::Vector v_from_other = v - h->r->GetLink(s2->linkname)->GetTransform() * OpenRAVE::Vector(s2->pos);
         if (v_from_other.lengthsqr3() > dist*dist) continue;
         
         /* compute the cost */
         dist = sqrt(v_from_other.lengthsqr3()) - s->radius - s2->radius;
         if (dist < 0.0)
            cost_sphere += OBS_FACTOR_SELF * (0.5 * EPSILON_SELF - dist);
         else
            cost_sphere += OBS_FACTOR_SELF * (0.5/EPSILON_SELF) * (dist-EPSILON_SELF) * (dist-EPSILON_SELF);
      }
      
      /* scale by sphere velocity */
      cost_sphere *= cblas_dnrm2(3, x_vel, 1);
      cost += cost_sphere;
   }
   
   *costp = cost;
   return 0;
}

int sphere_cost_grad(struct cost_helper * h, double * c_point, double * c_vel, double * c_grad)
{
   int i;
   int j;
   double x_vel[3];
   double g_point[3];
   double dist;
   double g_grad[3];
   double x_grad[3];
   double proj;
   double x_vel2;
   boost::multi_array< dReal, 2 > orjacobian;
   struct sphere * s;
   struct sphere * s2;
   
   /* put the robot in the config */
   std::vector<dReal> vec(c_point, c_point+h->n);
   h->r->SetActiveDOFValues(vec);
   
   /* start with a zero config-space gradient */
   cd_mat_set_zero(c_grad, h->n, 1);
   
   for (s=h->spheres; s; s=s->next)
   {
      /* get sphere center */
      Transform t = h->r->GetLink(s->linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */

      /* compute the manipulator jacobian at this point, at this link */
      h->r->CalculateJacobian(s->linkindex, v, orjacobian);
      /* copy the active columns of orjacobian into our J */
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            h->J[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      /* compute the current workspace velocity of the sphere */
      cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, h->n,
         1.0, h->J,h->n, c_vel,1, 0.0, x_vel,1);

      /* transform sphere center into grid frame */
      g_point[0] = v.x - h->g_pos[0];
      g_point[1] = v.y - h->g_pos[1];
      g_point[2] = v.z - h->g_pos[2];
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->g_sdf, g_point, &dist);
      /* get sdf gradient */
      cd_grid_double_grad(h->g_sdf, g_point, g_grad); /* this will be a unit vector away from closest obs */
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= s->radius;
      /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
      cd_mat_memcpy(x_grad, g_grad, 3, 1);
      if (dist < 0.0)
         cd_mat_scale(x_grad, 3, 1, -1.0);
      else if (dist < EPSILON)
         cd_mat_scale(x_grad, 3, 1, dist/EPSILON - 1.0);
      else
         cd_mat_set_zero(x_grad, 3, 1);
      cd_mat_scale(x_grad, 3, 1, OBS_FACTOR);
      
      /* subtract from x_grad vector projection onto x_vel */
      x_vel2 = cblas_ddot(3, x_vel,1, x_vel,1);
      if (x_vel2 > 0.000001)
      {
         proj = cblas_ddot(3, x_grad,1, x_vel,1) / x_vel2;
         cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
      }
      
      /* multiply into c_grad through JT */
      cblas_dgemv(CblasRowMajor, CblasTrans, 3, h->n,
         1.0, h->J,h->n, x_grad,1, 1.0, c_grad,1);


      /* consider effects from all other spheres (i.e. self collision) */
      for (s2=h->spheres; s2; s2=s2->next)
      {
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (s->linkindex == s2->linkindex) continue;
         
         /* skip spheres far enough away from us */
         dist = s->radius + s2->radius + EPSILON_SELF;
         OpenRAVE::Vector v_from_other = v - h->r->GetLink(s2->linkname)->GetTransform() * OpenRAVE::Vector(s2->pos);
         if (v_from_other.lengthsqr3() > dist*dist) continue;
         
         /* make unit vector (g_grad) away from other sphere */
         dist = sqrt(v_from_other.lengthsqr3());
         g_grad[0] = v_from_other[0] / dist;
         g_grad[1] = v_from_other[1] / dist;
         g_grad[2] = v_from_other[2] / dist;
         
         /* compute distance in collision */
         dist -= s->radius - s2->radius;
         
         /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
         cd_mat_memcpy(x_grad, g_grad, 3, 1);
         if (dist < 0.0)
            cd_mat_scale(x_grad, 3, 1, -1.0);
         else if (dist < EPSILON_SELF)
            cd_mat_scale(x_grad, 3, 1, dist/EPSILON_SELF - 1.0);
         cd_mat_scale(x_grad, 3, 1, OBS_FACTOR_SELF);
         
         /* subtract from x_grad vector projection onto x_vel */
         x_vel2 = cblas_ddot(3, x_vel,1, x_vel,1);
         if (x_vel2 > 0.000001)
         {
            proj = cblas_ddot(3, x_grad,1, x_vel,1) / x_vel2;
            cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
         }
         
         /* J2 = J - jacobian of other sphere*/
         cd_mat_memcpy(h->J2, h->J, 3, h->n);
         h->r->CalculateJacobian(s2->linkindex, v, orjacobian);
         for (i=0; i<3; i++)
            for (j=0; j<h->n; j++)
               h->J2[i*h->n+j] -= orjacobian[i][h->adofindices[j]];
         
         /* multiply into c_grad through JT */
         /* I HAVE NO IDEA WHY THERES A -1 HERE! */
         cblas_dgemv(CblasRowMajor, CblasTrans, 3, h->n,
            -1.0, h->J2,h->n, x_grad,1, 1.0, c_grad,1);
      }
   }
   
   return 0;
}

/* runchomp robot Herb2
 * run chomp from the current config to the passed goal config
 * uses the active dofs of the passed robot
 * initialized with a straight-line trajectory
 * */
bool mod::runchomp(std::ostream& sout, std::istream& sinput)
{
   int i;
   int j;
   int iter;
   int err;
   EnvironmentMutex::scoped_lock lockenv;
   /* parameters from the command line */
   RobotBasePtr r;
   int n_dof;
   double * adofgoal;
   int n_adofgoal;
   int n_iter;
   double lambda;
   /* stuff we compute later */
   int * adofindices;
   struct cd_chomp * c;
   struct cost_helper h;
   double * Gjlimit;
   double * GjlimitAinv;
   std::vector< dReal > vec_jlimit_lower;
   std::vector< dReal > vec_jlimit_upper;
   struct sphere * s;
   struct sphere * spheres;
   TrajectoryBasePtr starttraj;
   
   /* lock environment */
   lockenv = EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /*r = 0;*/
   adofgoal = 0;
   n_iter = 10;
   lambda = 10.0;
   
   /* parse arguments into robot */
   {
      char * in;
      char * cur;
      char buf[64];
      int len;
      in = str_from_istream(sinput);
      if (!in) throw openrave_exception("Out of memory!");
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (r.get()) { free(adofgoal); free(in); throw openrave_exception("Only one robot can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            r = this->e->GetRobot(buf)/*.get()*/;
            if (!r.get()) { free(adofgoal); free(in); throw openrave_exception("Could not find robot that name!"); }
            RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"adofgoal"))
         {
            if (adofgoal) { free(adofgoal); free(in); throw openrave_exception("Only one adofgoal can be passed!"); }
            if (starttraj.get()) { free(adofgoal); free(in); throw openrave_exception("Cannot pass both adofgoal and starttraj!"); }
            sscanf(cur, " %d%n", &n_adofgoal, &len); cur += len;
            if (n_adofgoal <= 0) { free(adofgoal); free(in); throw openrave_exception("n_adofgoal must be >0!"); }
            adofgoal = (double *) malloc(n_adofgoal * sizeof(double));
            for (j=0; j<n_adofgoal; j++)
            {
               sscanf(cur, " %lf%n", &adofgoal[j], &len); cur += len;
            }
            cd_mat_vec_print("parsed adofgoal: ", adofgoal, n_adofgoal);
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"n_iter"))
         {
            sscanf(cur, " %d%n", &n_iter, &len); cur += len;
            if (n_iter < 0) { free(adofgoal); free(in); throw openrave_exception("n_iter must be >=0!"); }
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"lambda"))
         {
            sscanf(cur, " %lf%n", &lambda, &len); cur += len;
            if (lambda < 0.01) { free(adofgoal); free(in); throw openrave_exception("lambda must be >=0.01!"); }
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"starttraj"))
         {
            int ser_len;
            if (starttraj.get()) { free(adofgoal); free(in); throw openrave_exception("Only one starttraj can be passed!"); }
            if (adofgoal) { free(adofgoal); free(in); throw openrave_exception("Cannot pass both adofgoal and starttraj!"); }
            sscanf(cur, " %d%n", &ser_len, &len); cur += len;
            /* skip a space */
            if (*cur != ' ') { free(adofgoal); free(in); throw openrave_exception("syntax is starttraj numchars <string>!"); }
            cur++;
            for (j=0; j<ser_len; j++) if (!cur[j]) break;
            if (j<ser_len) { free(adofgoal); free(in); throw openrave_exception("not enough characters in string!"); }
            /* create trajectory */
            starttraj = RaveCreateTrajectory(this->e);
            std::istringstream ser_iss(std::string(cur,ser_len));
            starttraj->deserialize(ser_iss);
            cur += ser_len;
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check validity of input arguments ... */
   
   if (!r.get())
      { free(adofgoal); throw openrave_exception("Did not pass a robot!"); }
   
   if (!adofgoal && !starttraj.get())
      { free(adofgoal); throw openrave_exception("Did not pass either adofgoal or starttraj!"); }
   
   if (!this->g_sdf)
      { free(adofgoal); throw openrave_exception("A signed distance field has not yet been computed!"); }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<rdata> d = boost::dynamic_pointer_cast<rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { free(adofgoal); throw openrave_exception("robot does not have a <orcdchomp> tag defined!"); }
      spheres = d->spheres;
   }
   
   n_dof = r->GetActiveDOF();
   
   /* allocate adofindices */
   adofindices = (int *) malloc(r->GetActiveDOF() * sizeof(int));
   {
      std::vector<int> vec = r->GetActiveDOFIndices();
      printf("adofindices:");
      for (i=0; i<r->GetActiveDOF(); i++)
      {
         adofindices[i] = vec[i];
         printf(" %d", adofindices[i]);
      }
      printf("\n");
   }
   
   /* check that n_adofgoal matches active dof */
   if (adofgoal && (n_dof != n_adofgoal))
      { printf("n_dof: %d; n_adofgoal: %d\n", n_dof, n_adofgoal);
         free(adofgoal); throw openrave_exception("size of adofgoal does not match active dofs!"); }
   
   /* check that starttraj has the right ConfigurationSpecification? */
   
   /* get joint limits */
   Gjlimit = (double *) malloc(N_INTPOINTS * n_dof * sizeof(double));
   GjlimitAinv = (double *) malloc(N_INTPOINTS * n_dof * sizeof(double));
   r->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   
   /* compute sphere link indices */
   for (s=spheres; s; s=s->next)
      s->linkindex = r->GetLink(s->linkname)->GetIndex();
   
   /* initialize the cost helper */
   h.n = n_dof;
   h.r = r.get();
   h.spheres = spheres;
   h.g_pos = this->g_pos;
   h.g_sdf = this->g_sdf;
   h.adofindices = adofindices;
   h.J = (double *) malloc(3*n_dof*sizeof(double));
   h.J2 = (double *) malloc(3*n_dof*sizeof(double));
   h.e = this->e;
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, n_dof, N_INTPOINTS, 1, &h,
      (int (*)(void *, double *, double *, double *))sphere_cost,
      (int (*)(void *, double *, double *, double *))sphere_cost_grad);
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofgoal); free(adofindices); throw openrave_exception("Error creating chomp instance."); }
   /*c->lambda = 1000000.0;*/
   c->lambda = lambda;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   /* initialize trajectory */
   if (starttraj.get())
   {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      for (i=0; i<N_INTPOINTS+2; i++)
      {
         std::vector<dReal> vec;
         starttraj->Sample(vec, i*starttraj->GetDuration()/(N_INTPOINTS+1), r->GetActiveConfigurationSpecification());
         for (j=0; j<n_dof; j++)
            c->T_ext_points[i][j] = vec[j];
      }
   }
   else
   {
      std::vector<dReal> vec;
      double percentage;
      RAVELOG_INFO("Initializing from a straight-line trajectory ...\n");
      /* starting point */
      r->GetActiveDOFValues(vec);
      for (j=0; j<n_dof; j++)
         c->T_ext_points[0][j] = vec[j];
      /* ending point */
      cd_mat_memcpy(c->T_ext_points[N_INTPOINTS+1], adofgoal, n_dof, 1);
      /* all points in between */
      cd_mat_set_zero(&c->T_ext_points[1][0], N_INTPOINTS, n_dof);
      for (i=1; i<N_INTPOINTS+1; i++)
      {
         percentage = 1.0*i/(N_INTPOINTS+1); /* between 0.0 (starting point) to 1.0 (ending point) */
         cblas_daxpy(n_dof, 1.0-percentage, c->T_ext_points[0],1,             c->T_ext_points[i],1);
         cblas_daxpy(n_dof,     percentage, c->T_ext_points[N_INTPOINTS+1],1, c->T_ext_points[i],1);
      }
   }
   free(adofgoal);
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); throw openrave_exception("Error initializing chomp instance."); }
   
   printf("iterating CHOMP once ...\n");
   for (iter=0; iter<n_iter; iter++)
   {
      double cost_total;
      double cost_obs;
      double cost_smooth;
#if 0
      /* lambda increases over time */
      c->lambda = 10.0 * exp(0.1 * iter);
      printf("lambda: %f\n", c->lambda);
#endif
      cd_chomp_cost(c, &cost_total, &cost_obs, &cost_smooth);
      printf("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f\n", iter, cost_total, cost_obs, cost_smooth);
      cd_chomp_iterate(c);
      
      /* handle joint limits */
      while (1)
      {
         double largest_violation;
         size_t largest_idx;
         
         /* find largest violation, and build Gjlimit matrix */
         largest_violation = 0.0;
         largest_idx = 0;
         cd_mat_set_zero(Gjlimit, c->m, c->n);
         for (i=0; i<c->m; i++)
         for (j=0; j<c->n; j++)
         {
            if (c->T_points[i][j] < vec_jlimit_lower[adofindices[j]])
            {
               Gjlimit[i*c->n+j] = vec_jlimit_lower[adofindices[j]] - c->T_points[i][j];
               if (fabs(Gjlimit[i*c->n+j]) > largest_violation)
               {
                  largest_violation = fabs(Gjlimit[i*c->n+j]);
                  largest_idx = i*c->n+j;
               }
            }
            if (c->T_points[i][j] > vec_jlimit_upper[adofindices[j]])
            {
               Gjlimit[i*c->n+j] = vec_jlimit_upper[adofindices[j]] - c->T_points[i][j];
               if (fabs(Gjlimit[i*c->n+j]) > largest_violation)
               {
                  largest_violation = fabs(Gjlimit[i*c->n+j]);
                  largest_idx = i*c->n+j;
               }
            }
         }
         if (largest_violation == 0.0) break;
         
         /* pre-multiply by Ainv */
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
            1.0, c->Ainv,c->m, Gjlimit,c->n, 0.0, GjlimitAinv,c->n);
            
         /* compute scalar necessary to make trajectory satisfy limit at largest_idx */
         cblas_daxpy(c->m * c->n,
                     1.01 * Gjlimit[largest_idx] / GjlimitAinv[largest_idx],
                     GjlimitAinv,1, c->T,1);
      }
   }
   
   printf("done!\n");
   
   /* create an openrave trajectory from the result, and send to sout */
   TrajectoryBasePtr t = RaveCreateTrajectory(this->e);
   t->Init(r->GetActiveConfigurationSpecification());
   for (i=0; i<N_INTPOINTS+2; i++)
   {
      std::vector<dReal> vec(c->T_ext_points[i], c->T_ext_points[i]+n_dof);
      t->Insert(i, vec);
   }
   
   printf("timing trajectory ...\n");
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
   /* new openrave added a fmaxaccelmult parameter (number 5) */
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,r,false,0.2,0.2,"","");
#else
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,r,false,0.2,"");
#endif

   printf("checking trajectory for collision ...\n");
   {
      double time;
      CollisionReportPtr report(new CollisionReport());
      for (time=0.0; time<t->GetDuration(); time+=0.01)
      {
         std::vector< dReal > point;
         t->Sample(point, time);
         r->SetActiveDOFValues(point);
         if (this->e->CheckCollision(r,report))
         {
            RAVELOG_ERROR("Collision: %s\n", report->__str__().c_str());
#if 0
            { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); throw openrave_exception("Resulting trajectory is in collision!"); }
#endif
         }
      }
   }
   
   t->serialize(sout);
   
   cd_chomp_destroy(c);
   
   free(GjlimitAinv);
   free(Gjlimit);
   free(h.J);
   free(h.J2);
   free(adofindices);

   printf("runchomp done! returning ...\n");
   return true;
}

} /* anonymous namespace */
