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

#define CUBE_EXTENT (0.01)

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
class rdata : public OpenRAVE::XMLReadable
{
public:
   rdata() : OpenRAVE::XMLReadable("orcdchomp") { this->spheres = 0; }
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
class rdata_parser : public OpenRAVE::BaseXMLReader
{
public:
   rdata_parser(boost::shared_ptr<rdata> passed_d, const OpenRAVE::AttributesList& atts)
   {
      /* save or construct the rdata object */
      this->d = passed_d;
      if(!this->d) this->d.reset(new rdata());
      /* get ready */
      this->inside_spheres = false;
   }

   virtual OpenRAVE::XMLReadablePtr GetReadable() { return this->d; }

   virtual ProcessElement startElement(const std::string& name, const OpenRAVE::AttributesList& atts)
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
         for(OpenRAVE::AttributesList::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt)
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
class mod : public OpenRAVE::ModuleBase
{
public:
   OpenRAVE::EnvironmentBasePtr e; /* filled on module creation */
   char kinbody_gsdf[256];
   double pose_gsdf[7]; /* pose of the grid w.r.t. the kinbody frame */
   cd_grid * g_sdf;
   
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
static OpenRAVE::BaseXMLReaderPtr rdata_parser_maker(OpenRAVE::InterfaceBasePtr ptr, const OpenRAVE::AttributesList& atts)
   { return OpenRAVE::BaseXMLReaderPtr(new rdata_parser(boost::shared_ptr<rdata>(),atts)); }
}

/* export as an openrave module */
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info)
{ 
   if(!reg_reader) reg_reader = OpenRAVE::RaveRegisterXMLReader(OpenRAVE::PT_Robot,"orcdchomp",rdata_parser_maker);
   info.interfacenames[OpenRAVE::PT_Module].push_back("orcdchomp");
}
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string& interfacename, std::istream& sinput, OpenRAVE::EnvironmentBasePtr penv)
{
   if((type == OpenRAVE::PT_Module)&&(interfacename == "orcdchomp"))
      return OpenRAVE::InterfaceBasePtr(new mod(penv));
   return OpenRAVE::InterfaceBasePtr();
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
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   OpenRAVE::RobotBasePtr r;
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
      if (!in) { throw OpenRAVE::openrave_exception("Out of memory!"); }
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (r.get()) { free(in); throw OpenRAVE::openrave_exception("Only one robot can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            r = this->e->GetRobot(buf)/*.get()*/;
            if (!r.get()) { free(in); throw OpenRAVE::openrave_exception("Could not find robot with that name!"); }
            RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!r.get()) { throw OpenRAVE::openrave_exception("Did not pass all required args!"); }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<rdata> d = boost::dynamic_pointer_cast<rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { throw OpenRAVE::openrave_exception("robot does not have a <orcdchomp> tag defined!"); }
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
         OpenRAVE::Transform t = r->GetLink(s->linkname)->GetTransform();
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
 * 
 * note: does aabb include disabled bodies? probably, but we might hope not ...
 * */
bool mod::computedistancefield(std::ostream& sout, std::istream& sinput)
{
   int i;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr kinbody;
   double temp;
   OpenRAVE::KinBodyPtr cube;
   size_t idx;
   struct cd_grid * g_obs;
   int gsdf_sizearray[3];
   OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
   double pose_world_gsdf[7];
   double pose_cube[7];
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* parse arguments into kinbody */
   {
      char * in;
      char * cur;
      char buf[64];
      int len;
      in = str_from_istream(sinput);
      if (!in) { throw OpenRAVE::openrave_exception("Out of memory!"); }
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"kinbody"))
         {
            if (kinbody) { free(in); throw OpenRAVE::openrave_exception("Only one kinbody can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            kinbody = this->e->GetKinBody(buf);
            if (!kinbody.get()) { free(in); throw OpenRAVE::openrave_exception("Could not find kinbody with that name!"); }
            RAVELOG_INFO("Using kinbody %s.\n", kinbody->GetName().c_str());
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!kinbody.get()) throw OpenRAVE::openrave_exception("Did not pass all required args!");
   
   strcpy(this->kinbody_gsdf, kinbody->GetName().c_str());

   /* compute aabb when object is at world origin */
   {
      OpenRAVE::KinBody::KinBodyStateSaver statesaver(kinbody);
      kinbody->SetTransform(OpenRAVE::Transform());
      aabb = kinbody->ComputeAABB();
   }
   printf("    pos: %f %f %f\n", aabb.pos[0], aabb.pos[1], aabb.pos[2]);
   printf("extents: %f %f %f\n", aabb.extents[0], aabb.extents[1], aabb.extents[2]);

   /* calculate dimension sizes (number of cells) */
   for (i=0; i<3; i++)
   {
      gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+0.3) / CUBE_EXTENT); /* 0.3m padding on each side, 4cm on a side */
      printf("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
   }
   
   /* Create a new grid located around the current kinbody; 100x100x100
    * for now, this is axis-aligned.
    * the box is centered around (0,0,1).
    * the box has extents +/- 2.0m, so each cube is 4cm on a side. */
   temp = 1.0; /* free space */
   err = cd_grid_create_sizearray(&g_obs, &temp, sizeof(double), 3, gsdf_sizearray);
   if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
   
   /* set side lengths */
   for (i=0; i<3; i++)
      g_obs->lengths[i] = gsdf_sizearray[i] * 2.0 * CUBE_EXTENT;
   cd_mat_vec_print("g_obs->lengths: ", g_obs->lengths, 3);
   
   /* set pose of grid w.r.t. kinbody frame */
   cd_kin_pose_identity(pose_gsdf);
   for (i=0; i<3; i++)
      pose_gsdf[i] = aabb.pos[i] - 0.5 * g_obs->lengths[i];
   cd_mat_vec_print("pose_gsdf: ",pose_gsdf, 7);
   
   /* create the cube */
   cube = OpenRAVE::RaveCreateKinBody(this->e);
   cube->SetName("cube");
   
   /* set its dimensions */
   {
      std::vector<OpenRAVE::AABB> vaabbs(1);
      vaabbs[0].extents = OpenRAVE::Vector(CUBE_EXTENT, CUBE_EXTENT, CUBE_EXTENT); /* extents = half side lengths */
      cube->InitFromBoxes(vaabbs, 1);
   }
   
   /* add the cube */
   this->e->AddKinBody(cube);
   
   /* get the pose_world_gsdf = pose_world_kinbody * pose_kinbody_gsdf */
   {
      OpenRAVE::Transform t = kinbody->GetTransform();
      pose_world_gsdf[0] = t.trans.x;
      pose_world_gsdf[1] = t.trans.y;
      pose_world_gsdf[2] = t.trans.z;
      pose_world_gsdf[3] = t.rot.y;
      pose_world_gsdf[4] = t.rot.z;
      pose_world_gsdf[5] = t.rot.w;
      pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(pose_world_gsdf, pose_gsdf, pose_world_gsdf);
   }

   int collisions = 0;
   
   /* go through the grid, testing for collision as we go;
    * collisions are HUGE_VAL, free are 1.0 */
   printf("computing occupancy grid ...\n");
   for (idx=0; idx<g_obs->ncells; idx++)
   {
      OpenRAVE::Transform t;
      
      if (idx % 100000 == 0)
         printf("idx=%d ...\n", (int)idx);
      
      /* set cube location */
      t.identity();
      cd_kin_pose_identity(pose_cube);
      cd_grid_center_index(g_obs, idx, pose_cube);
      cd_kin_pose_compose(pose_world_gsdf, pose_cube, pose_cube);
      t.trans.x = pose_cube[0];
      t.trans.y = pose_cube[1];
      t.trans.z = pose_cube[2];
      t.rot.y = pose_cube[3];
      t.rot.z = pose_cube[4];
      t.rot.w = pose_cube[5];
      t.rot.x = pose_cube[6];
      cube->SetTransform(t);
      
      /* do collision check */
      if (this->e->CheckCollision(cube))
      {
         *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
         collisions++;
      }
   }

   printf("found %d/%d collisions!\n", collisions, g_obs->ncells);
   
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
   double pose_world_gsdf[7];
   double pose_gsdf_world[7];
   struct cd_grid * g_sdf;
   OpenRAVE::RobotBase * r;
   int * adofindices;
   struct sphere * spheres;
   double * J; /* space for the jacobian; 3xn */
   double * J2;
   
   /* ugh */
   OpenRAVE::EnvironmentBasePtr e;
};

int sphere_cost(struct cost_helper * h, double * c_point, double * c_vel, double * costp, double * c_grad)
{
   int i;
   int j;
   double x_vel[3];
   double x_vel_norm;
   double g_point[3];
   double dist;
   double cost;
   double cost_sphere;
   double g_grad[3];
   double x_grad[3];
   double proj;
   double x_vel2;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   struct sphere * s;
   struct sphere * s2;
   
   /* put the robot in the config */
   std::vector<OpenRAVE::dReal> vec(c_point, c_point+h->n);
   h->r->SetActiveDOFValues(vec);
   
   /* start with a zero cost and config-space gradient */
   cost = 0.0;
   if (c_grad) cd_mat_set_zero(c_grad, h->n, 1);
   
   /* the cost and its gradient are summed over each sphere on the robot */
   for (s=h->spheres; s; s=s->next)
   {
      cost_sphere = 0.0;
      
      /* get sphere center */
      OpenRAVE::Transform t = h->r->GetLink(s->linkname)->GetTransform();
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
      x_vel_norm = cblas_dnrm2(3, x_vel, 1);

      /* transform sphere center into grid frame */
      g_point[0] = v.x;
      g_point[1] = v.y;
      g_point[2] = v.z;
      cd_kin_pose_compos(h->pose_gsdf_world, g_point, g_point);
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->g_sdf, g_point, &dist);
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= s->radius;
      
      if (costp)
      {
         /* convert to a cost */
         if (dist < 0.0)
            cost_sphere += OBS_FACTOR * (0.5 * EPSILON - dist);
         else if (dist < EPSILON)
            cost_sphere += OBS_FACTOR * (0.5/EPSILON) * (dist-EPSILON) * (dist-EPSILON);
      }
      
      if (c_grad)
      {
         /* get sdf gradient */
         cd_grid_double_grad(h->g_sdf, g_point, g_grad); /* this will be a unit vector away from closest obs */
         cd_kin_pose_compose_vec(h->pose_world_gsdf, g_grad, g_grad); /* now in world frame */
         
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
      }

      /* consider effects from all other spheres (i.e. self collision) */
      for (s2=h->spheres; s2; s2=s2->next)
      {
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (s->linkindex == s2->linkindex) continue;
         
         /* skip spheres far enough away from us */
         dist = s->radius + s2->radius + EPSILON_SELF;
         OpenRAVE::Vector v_from_other = v - h->r->GetLink(s2->linkname)->GetTransform() * OpenRAVE::Vector(s2->pos);
         if (v_from_other.lengthsqr3() > dist*dist) continue;
         
         if (costp)
         {
            /* compute the cost */
            dist = sqrt(v_from_other.lengthsqr3()) - s->radius - s2->radius;
            if (dist < 0.0)
               cost_sphere += OBS_FACTOR_SELF * (0.5 * EPSILON_SELF - dist);
            else
               cost_sphere += OBS_FACTOR_SELF * (0.5/EPSILON_SELF) * (dist-EPSILON_SELF) * (dist-EPSILON_SELF);
         }
         
         if (c_grad)
         {
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
      
      if (costp)
      {
         /* scale by sphere velocity */
         cost_sphere *= x_vel_norm;
         cost += cost_sphere;
      }
   }
   
   if (costp) *costp = cost;
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
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   /* parameters from the command line */
   OpenRAVE::RobotBasePtr r;
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
   std::vector< OpenRAVE::dReal > vec_jlimit_lower;
   std::vector< OpenRAVE::dReal > vec_jlimit_upper;
   struct sphere * s;
   struct sphere * spheres;
   OpenRAVE::TrajectoryBasePtr starttraj;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
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
      if (!in) throw OpenRAVE::openrave_exception("Out of memory!");
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (r.get()) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Only one robot can be passed!"); }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            r = this->e->GetRobot(buf)/*.get()*/;
            if (!r.get()) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Could not find robot that name!"); }
            RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"adofgoal"))
         {
            if (adofgoal) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Only one adofgoal can be passed!"); }
            if (starttraj.get()) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Cannot pass both adofgoal and starttraj!"); }
            sscanf(cur, " %d%n", &n_adofgoal, &len); cur += len;
            if (n_adofgoal <= 0) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("n_adofgoal must be >0!"); }
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
            if (n_iter < 0) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("n_iter must be >=0!"); }
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"lambda"))
         {
            sscanf(cur, " %lf%n", &lambda, &len); cur += len;
            if (lambda < 0.01) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("lambda must be >=0.01!"); }
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"starttraj"))
         {
            int ser_len;
            if (starttraj.get()) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Only one starttraj can be passed!"); }
            if (adofgoal) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("Cannot pass both adofgoal and starttraj!"); }
            sscanf(cur, " %d%n", &ser_len, &len); cur += len;
            /* skip a space */
            if (*cur != ' ') { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("syntax is starttraj numchars <string>!"); }
            cur++;
            for (j=0; j<ser_len; j++) if (!cur[j]) break;
            if (j<ser_len) { free(adofgoal); free(in); throw OpenRAVE::openrave_exception("not enough characters in string!"); }
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
      { free(adofgoal); throw OpenRAVE::openrave_exception("Did not pass a robot!"); }
   
   if (!adofgoal && !starttraj.get())
      { free(adofgoal); throw OpenRAVE::openrave_exception("Did not pass either adofgoal or starttraj!"); }
   
   if (!this->g_sdf)
      { free(adofgoal); throw OpenRAVE::openrave_exception("A signed distance field has not yet been computed!"); }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<rdata> d = boost::dynamic_pointer_cast<rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { free(adofgoal); throw OpenRAVE::openrave_exception("robot does not have a <orcdchomp> tag defined!"); }
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
         free(adofgoal); throw OpenRAVE::openrave_exception("size of adofgoal does not match active dofs!"); }
   
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
   h.g_sdf = this->g_sdf;
   h.adofindices = adofindices;
   h.J = (double *) malloc(3*n_dof*sizeof(double));
   h.J2 = (double *) malloc(3*n_dof*sizeof(double));
   h.e = this->e;
   
   /* compute location of the world in g_sdf frame */
   {
      OpenRAVE::Transform t = this->e->GetKinBody(this->kinbody_gsdf)->GetTransform();
      h.pose_world_gsdf[0] = t.trans.x;
      h.pose_world_gsdf[1] = t.trans.y;
      h.pose_world_gsdf[2] = t.trans.z;
      h.pose_world_gsdf[3] = t.rot.y;
      h.pose_world_gsdf[4] = t.rot.z;
      h.pose_world_gsdf[5] = t.rot.w;
      h.pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(h.pose_world_gsdf, this->pose_gsdf, h.pose_world_gsdf);
      cd_kin_pose_invert(h.pose_world_gsdf, h.pose_gsdf_world);
   }
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, n_dof, N_INTPOINTS, 1, &h,
      (int (*)(void *, double *, double *, double *, double *))sphere_cost);
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofgoal); free(adofindices); throw OpenRAVE::openrave_exception("Error creating chomp instance."); }
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
         std::vector<OpenRAVE::dReal> vec;
         starttraj->Sample(vec, i*starttraj->GetDuration()/(N_INTPOINTS+1), r->GetActiveConfigurationSpecification());
         for (j=0; j<n_dof; j++)
            c->T_ext_points[i][j] = vec[j];
      }
   }
   else
   {
      std::vector<OpenRAVE::dReal> vec;
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
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); throw OpenRAVE::openrave_exception("Error initializing chomp instance."); }
   
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
   OpenRAVE::TrajectoryBasePtr t = OpenRAVE::RaveCreateTrajectory(this->e);
   t->Init(r->GetActiveConfigurationSpecification());
   for (i=0; i<N_INTPOINTS+2; i++)
   {
      std::vector<OpenRAVE::dReal> vec(c->T_ext_points[i], c->T_ext_points[i]+n_dof);
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
      OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
      for (time=0.0; time<t->GetDuration(); time+=0.01)
      {
         std::vector< OpenRAVE::dReal > point;
         t->Sample(point, time);
         r->SetActiveDOFValues(point);
         if (this->e->CheckCollision(r,report))
         {
            RAVELOG_ERROR("Collision: %s\n", report->__str__().c_str());
#if 1
            { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); throw OpenRAVE::openrave_exception("Resulting trajectory is in collision!"); }
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
