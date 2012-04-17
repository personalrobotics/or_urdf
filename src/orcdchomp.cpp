#include <cstdio>
#include <cstring>
#include <boost/bind.hpp>
#include <cblas.h>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <openrave/plugin.h>
extern "C" {
#include <libcd/chomp.h>
#include <libcd/grid.h>
#include <libcd/gs.h>
#include <libcd/kin.h>
#include <libcd/mat.h>
#include <libcd/png.h>
#include <libcd/util.h>
#include <libcd-gscpp.h>
}

#define TAU (6.283185307179586)

#define N_INTPOINTS (99)

#define EPSILON (0.1) /* in meters */
#define OBS_FACTOR (30.0)

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
   cd_grid * g_sdf;
   
   bool computedistancefield(ostream& sout, istream& sinput);
   bool runchomp(ostream& sout, istream& sinput);

   orcdchomp(EnvironmentBasePtr penv) : ModuleBase(penv)
   {
      __description = "orcdchomp: implement chomp using libcd";
      RegisterCommand("computedistancefield",boost::bind(&orcdchomp::computedistancefield,this,_1,_2),"compute distance field");
      RegisterCommand("runchomp",boost::bind(&orcdchomp::runchomp,this,_1,_2),"run chomp");
      this->e = penv;
      this->g_sdf = 0;
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

/* computedistancefield robot Herb2
 * computes a distance field in the vicinity of the passed robot
 * */
bool orcdchomp::computedistancefield(ostream& sout, istream& sinput)
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
   }
   
   /* check that we have everything */
   if (!robot)
      { RAVELOG_ERROR("Did not pass all required args!\n"); return false; }
   
   /* Create a new grid located around the current robot; 100x100x100
    * for now, this is axis-aligned.
    * the box is centered around (0,0,1).
    * the box has extents +/- 2.0m, so each cube is 4cm on a side. */
   temp = 1.0; /* free space */
   err = cd_grid_create(&g_obs, &temp, sizeof(double), 3, 100, 100, 100);
   if (err) { RAVELOG_ERROR("Not enough memory for distance field!\n"); return false; }
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
      t.trans.x = center[0] - 2.0;
      t.trans.y = center[1] - 2.0;
      t.trans.z = center[2] - 1.0;
      cube->SetTransform(t);
      
      /* do collision check */
      if (this->e->CheckCollision(cube))
         *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
   }
   
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
   struct cd_grid * g_sdf;
   RobotBase * r;
   int * adofindices;
   double * J; /* space for the jacobian; 3xn */
   
   /* ugh */
   EnvironmentBasePtr e;
};

struct sphere
{
   char linkname[32];
   int linkindex; /* solved for on init */
   double pos[3];
   double radius;
   
};

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

/* cost over a bunch of body points */
int sphere_cost(struct cost_helper * h, double * c_point, double * c_vel, double * costp)
{
   int i;
   int j;
   int si;
   double x_vel[3];
   double g_point[3];
   double dist;
   double cost;
   double cost_sphere;
   
   /* put the robot in the config */
   std::vector<dReal> vec(c_point, c_point+h->n);
   h->r->SetActiveDOFValues(vec);
   
#if 0
   /* add up the costs of each sphere */
   for (i=0; i<sizeof(spheres)/sizeof(spheres[0]); i++)
   {
      /* make some sweet spheres! */
      OpenRAVE::KinBodyPtr s = OpenRAVE::RaveCreateKinBody(h->e);
      sprintf(buf, "sphere_%d", i);
      s->SetName(buf);
      /* set its dimensions */
      {
         std::vector< OpenRAVE::Vector > svec;
         Transform t = h->r->GetLink(spheres[i].linkname)->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(spheres[i].pos); /* copies 3 values */
         v.w = spheres[i].radius; /* radius */
         svec.push_back(v);
         s->InitFromSpheres(svec, true);
      }
      /* add the sphere */
      h->e->AddKinBody(s);
   }
#endif

   cost = 0.0;
   
   for (si=0; si<(int)(sizeof(spheres)/sizeof(spheres[0])); si++)
   {
      /* get sphere center */
      Transform t = h->r->GetLink(spheres[si].linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(spheres[si].pos); /* copies 3 values */
      
      /* compute the manipulator jacobian at this point, at this link */
      boost::multi_array< dReal, 2 > orjacobian;
      h->r->CalculateJacobian(spheres[si].linkindex, v, orjacobian);
      /* copy the active columns of orjacobian into our J */
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            h->J[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      /* compute the current workspace velocity of the sphere */
      cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, h->n,
         1.0, h->J,h->n, c_vel,1, 0.0, x_vel,1);
      
      /* transform into grid frame */
      g_point[0] = v.x + 2.0;
      g_point[1] = v.y + 2.0;
      g_point[2] = v.z + 1.0;
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->g_sdf, g_point, &dist);
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= spheres[si].radius;
      /* convert to a cost */
      if (dist < 0.0)
         cost_sphere = 0.5 * EPSILON - dist;
      else if (dist < EPSILON)
         cost_sphere = (0.5/EPSILON) * (dist-EPSILON) * (dist-EPSILON);
      else
         cost_sphere = 0.0;
      cost_sphere *= OBS_FACTOR;
      
      /* scale by sphere velocity */
      cost_sphere *= cblas_dnrm2(3, x_vel, 1);
      
      cost += cost_sphere;
   }
   
   *costp = cost;
   return 0;
}

int sphere_cost_grad(struct cost_helper * h, double * c_point, double * c_vel, double * c_grad)
{
   int si;
   int i;
   int j;
   double x_vel[3];
   double g_point[3];
   double dist;
   double g_grad[3];
   double x_grad[3];
   double proj;
   double x_vel2;
   
   /* put the robot in the config */
   std::vector<dReal> vec(c_point, c_point+h->n);
   h->r->SetActiveDOFValues(vec);
   
   /* start with a zero config-space gradient */
   cd_mat_set_zero(c_grad, h->n, 1);
   
   for (si=0; si<(int)(sizeof(spheres)/sizeof(spheres[0])); si++)
   {
      /* get sphere center */
      Transform t = h->r->GetLink(spheres[si].linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(spheres[si].pos); /* copies 3 values */

      /* compute the manipulator jacobian at this point, at this link */
      boost::multi_array< dReal, 2 > orjacobian;
      h->r->CalculateJacobian(spheres[si].linkindex, v, orjacobian);
      /* copy the active columns of orjacobian into our J */
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            h->J[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      /* compute the current workspace velocity of the sphere */
      cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, h->n,
         1.0, h->J,h->n, c_vel,1, 0.0, x_vel,1);

      /* transform sphere center into grid frame */
      g_point[0] = v.x + 2.0;
      g_point[1] = v.y + 2.0;
      g_point[2] = v.z + 1.0;
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->g_sdf, g_point, &dist);
      /* get sdf gradient */
      cd_grid_double_grad(h->g_sdf, g_point, g_grad); /* this will be a unit vector to closest obs */
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= spheres[si].radius;
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
   
   return 0;
}

/* runchomp robot Herb2
 * run chomp from the current config to the passed goal config
 * uses the active dofs of the passed robot
 * initialized with a straight-line trajectory
 * */
bool orcdchomp::runchomp(ostream& sout, istream& sinput)
{
   int si;
   int i;
   int j;
   int iter;
   int err;
   EnvironmentMutex::scoped_lock lockenv;
   RobotBasePtr r;
   double * adofgoal;
   int n_adofgoal;
   int * adofindices;
   struct cd_chomp * c;
   struct cost_helper h;
   double * Gjlimit;
   double * GjlimitAinv;
   std::vector< dReal > vec_jlimit_lower;
   std::vector< dReal > vec_jlimit_upper;
   
   /* lock environment */
   lockenv = EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /*r = 0;*/
   adofgoal = 0;
   
   /* parse arguments into robot */
   {
      char * in;
      char * cur;
      char buf[64];
      int len;
      in = str_from_istream(sinput);
      if (!in) { RAVELOG_ERROR("Out of memory!\n"); return false; }
      cur = in;
      while (1)
      {
         if (strp_skipprefix(&cur, (char *)"robot"))
         {
            if (r.get()) { RAVELOG_ERROR("Only one robot can be passed!\n"); free(adofgoal); free(in); return false; }
            sscanf(cur, " %s%n", buf, &len); cur += len;
            r = this->e->GetRobot(buf)/*.get()*/;
            if (!r.get()) { RAVELOG_ERROR("Could not find robot with name %s!\n",buf); free(adofgoal); free(in); return false; }
            RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"adofgoal"))
         {
            if (adofgoal) { RAVELOG_ERROR("Only one adofgoal can be passed!\n"); free(adofgoal); free(in); return false; }
            sscanf(cur, " %d%n", &n_adofgoal, &len); cur += len;
            if (n_adofgoal <= 0) { RAVELOG_ERROR("n_adofgoal must be >0!\n"); free(adofgoal); free(in); return false; }
            adofgoal = (double *) malloc(n_adofgoal * sizeof(double));
            for (j=0; j<n_adofgoal; j++)
            {
               sscanf(cur, " %lf%n", &adofgoal[j], &len); cur += len;
            }
            cd_mat_vec_print("parsed adofgoal: ", adofgoal, n_adofgoal);
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_ERROR("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!r.get() || !adofgoal)
      { RAVELOG_ERROR("Did not pass all required args!\n"); free(adofgoal); return false; }
   
#if 0
   if (!this->g_sdf)
      { RAVELOG_ERROR("A signed distance field has not yet been computed!\n"); free(adofgoal); return false; }
#endif
   
   /* check that n_adofgoal matches active dof */
   if (n_adofgoal != r->GetActiveDOF())
      { RAVELOG_ERROR("size of adofgoal does not match active dofs!\n"); free(adofgoal); return false; }
   
   /* allocate adofindices */
   adofindices = (int *) malloc(n_adofgoal * sizeof(int));
   {
      std::vector<int> vec = r->GetActiveDOFIndices();
      for (i=0; i<n_adofgoal; i++)
         adofindices[i] = vec[i];
   }
   
   Gjlimit = (double *) malloc(N_INTPOINTS * n_adofgoal * sizeof(double));
   GjlimitAinv = (double *) malloc(N_INTPOINTS * n_adofgoal * sizeof(double));
   r->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   
   /* compute sphere link indices */
   for (si=0; si<(int)(sizeof(spheres)/sizeof(spheres[0])); si++)
      spheres[si].linkindex = r->GetLink(spheres[si].linkname)->GetIndex();
   
   /* initialize the cost helper */
   h.n = n_adofgoal;
   h.r = r.get();
   h.g_sdf = this->g_sdf;
   h.adofindices = adofindices;
   h.J = (double *) malloc(3*n_adofgoal*sizeof(double));
   h.e = this->e;
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, n_adofgoal, N_INTPOINTS, 1, &h,
      (int (*)(void *, double *, double *, double *))sphere_cost,
      (int (*)(void *, double *, double *, double *))sphere_cost_grad);
   if (err) { RAVELOG_ERROR("Error creating chomp instance.\n"); free(Gjlimit); free(GjlimitAinv); free(h.J); free(adofgoal); free(adofindices); return false; }
   /*c->lambda = 1000000.0;*/
   c->lambda = 5.0;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   /* initialize trajectory */
   {
      std::vector<dReal> vec;
      double percentage;
      /* starting point */
      r->GetActiveDOFValues(vec);
      for (j=0; j<n_adofgoal; j++)
         c->T_ext_points[0][j] = vec[j];
      /* ending point */
      cd_mat_memcpy(c->T_ext_points[N_INTPOINTS+1], adofgoal, n_adofgoal, 1);
      /* all points in between */
      cd_mat_set_zero(&c->T_ext_points[1][0], N_INTPOINTS, n_adofgoal);
      for (i=1; i<N_INTPOINTS+1; i++)
      {
         percentage = 1.0*i/(N_INTPOINTS+1); /* between 0.0 (starting point) to 1.0 (ending point) */
         cblas_daxpy(n_adofgoal, 1.0-percentage, c->T_ext_points[0],1,             c->T_ext_points[i],1);
         cblas_daxpy(n_adofgoal,     percentage, c->T_ext_points[N_INTPOINTS+1],1, c->T_ext_points[i],1);
      }
   }
   free(adofgoal);
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { RAVELOG_ERROR("Error initializing chomp instance.\n"); free(Gjlimit); free(GjlimitAinv); free(h.J); free(adofindices); cd_chomp_destroy(c); return false; }
   
   printf("iterating CHOMP once ...\n");
   for (iter=0; iter<100; iter++)
   {
      double cost_total;
      double cost_obs;
      double cost_smooth;
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
   
   printf("creating trajectory ...\n");
   /* create an openrave trajectory from the result, and send to sout */
   TrajectoryBasePtr t = RaveCreateTrajectory(this->e);
   t->Init(r->GetActiveConfigurationSpecification());
   for (i=0; i<N_INTPOINTS+2; i++)
   {
      std::vector<dReal> vec(c->T_ext_points[i], c->T_ext_points[i]+n_adofgoal);
      t->Insert(i, vec);
   }
   printf("done!\n");
   
   printf("smoothing trajectory ...\n");
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,r,false,0.2,"","");
   printf("done!\n");
   
   printf("serializing trajectory ...\n");
   t->serialize(sout);
   printf("done!\n");
   
   cd_chomp_destroy(c);
   
   free(GjlimitAinv);
   free(Gjlimit);
   free(h.J);
   free(adofindices);
   return true;
}
