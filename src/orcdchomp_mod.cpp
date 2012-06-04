
#include <time.h>
#include <cblas.h>

extern "C" {
#include <libcd/chomp.h>
#include <libcd/grid.h>
#include <libcd/kin.h>
#include <libcd/mat.h>
#include <libcd/util.h>
}

#include <openrave/openrave.h>
#include <openrave/planningutils.h>

#include "orcdchomp_rdata.h"
#include "orcdchomp_mod.h"


namespace orcdchomp
{
   
struct sdf
{
   char kinbody_name[256];
   double pose[7]; /* pose of the grid w.r.t. the kinbody frame */
   cd_grid * grid;
};


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
   struct orcdchomp::sphere * s;
   struct orcdchomp::sphere * spheres;
   
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
      boost::shared_ptr<orcdchomp::rdata> d = boost::dynamic_pointer_cast<orcdchomp::rdata>(r->GetReadableInterface("orcdchomp"));
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
 * computes a distance field in the vicinity of the passed kinbody
 * 
 * note: does aabb include disabled bodies? probably, but we might hope not ...
 * */
bool mod::computedistancefield(std::ostream& sout, std::istream& sinput)
{
   int i;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr kinbody;
   /* parameters */
   double cube_extent;
   char cache_filename[1024];
   /* other */
   double temp;
   OpenRAVE::KinBodyPtr cube;
   size_t idx;
   struct cd_grid * g_obs;
   int gsdf_sizearray[3];
   OpenRAVE::geometry::aabb< OpenRAVE::dReal > aabb;
   double pose_world_gsdf[7];
   double pose_cube[7];
   struct sdf sdf_new;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   cube_extent = 0.02;
   cache_filename[0] = 0;
   
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
            RAVELOG_INFO("Using kinbody |%s|.\n", kinbody->GetName().c_str());
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"cube_extent"))
         {
            sscanf(cur, " %lf%n", &cube_extent, &len); cur += len;
            RAVELOG_INFO("Using cube_extent |%f|.\n", cube_extent);
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"cache_filename"))
         {
            sscanf(cur, " %1023s%n", cache_filename, &len); cur += len;
            RAVELOG_INFO("Using cache_filename |%s|.\n", cache_filename);
            continue;
         }
         break;
      }
      if (cur[0]) RAVELOG_WARN("remaining string: |%s|! continuing ...\n", cur);
      free(in);
   }
   
   /* check that we have everything */
   if (!kinbody.get()) throw OpenRAVE::openrave_exception("Did not pass all required args!");
   
   /* make sure we don't already have an sdf loaded for this kinbody */
   for (i=0; i<this->n_sdfs; i++)
      if (strcmp(this->sdfs[i].kinbody_name, kinbody->GetName().c_str()) == 0)
         break;
   if (i<this->n_sdfs)
      throw OpenRAVE::openrave_exception("We already have an sdf for this kinbody!");
   
   /* are we making our own sdf object? */
   sdf_new.grid = 0;
   
   /* attempt to load the sdf from file */
   if (cache_filename[0]) do
   {
      FILE * fp;
      printf("reading sdf from file %s ...\n", cache_filename);
      fp = fopen(cache_filename, "rb");
      if (!fp) { printf("could not read from file!\n"); break; }
      /* read struct sdf */
      i = fread(sdf_new.kinbody_name, sizeof(sdf_new.kinbody_name), 1, fp);
      i = fread(sdf_new.pose, sizeof(sdf_new.pose), 1, fp);
      /* read grid */
      sdf_new.grid = (struct cd_grid *) malloc(sizeof(struct cd_grid));
      i = fread(&sdf_new.grid->n, sizeof(sdf_new.grid->n), 1, fp);
      sdf_new.grid->sizes = (int *) malloc(sizeof(sdf_new.grid->sizes[0]) * sdf_new.grid->n);
      i = fread(sdf_new.grid->sizes, sizeof(sdf_new.grid->sizes[0]), sdf_new.grid->n, fp);
      i = fread(&sdf_new.grid->ncells, sizeof(sdf_new.grid->ncells), 1, fp);
      i = fread(&sdf_new.grid->cell_size, sizeof(sdf_new.grid->cell_size), 1, fp);
      sdf_new.grid->data = (char *) malloc(sdf_new.grid->cell_size * sdf_new.grid->ncells);
      i = fread(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
      sdf_new.grid->lengths = (double *) malloc(sizeof(sdf_new.grid->lengths[0]) * sdf_new.grid->n);
      i = fread(sdf_new.grid->lengths, sizeof(sdf_new.grid->lengths[0]), sdf_new.grid->n, fp);
      fclose(fp);
   } while (0);
   
   /* create sdf_new object */
   if (!sdf_new.grid)
   {
      /* copy in name */
      strcpy(sdf_new.kinbody_name, kinbody->GetName().c_str());

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
         /* 0.15m padding (was 0.3m) on each side
          * (this is the radius of the biggest herb2 spehere)
          * (note: extents are half the side lengths!) */
         gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+1.0) / cube_extent);
         printf("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
      }
      
      /* Create a new grid located around the current kinbody;
       * per-dimension sizes set above */
      temp = 1.0; /* free space */
      err = cd_grid_create_sizearray(&g_obs, &temp, sizeof(double), 3, gsdf_sizearray);
      if (err) throw OpenRAVE::openrave_exception("Not enough memory for distance field!");
      
      /* set side lengths */
      for (i=0; i<3; i++)
         g_obs->lengths[i] = gsdf_sizearray[i] * 2.0 * cube_extent;
      cd_mat_vec_print("g_obs->lengths: ", g_obs->lengths, 3);
      
      /* set pose of grid w.r.t. kinbody frame */
      cd_kin_pose_identity(sdf_new.pose);
      for (i=0; i<3; i++)
         sdf_new.pose[i] = aabb.pos[i] - 0.5 * g_obs->lengths[i];
      cd_mat_vec_print("pose_gsdf: ", sdf_new.pose, 7);
      
      /* create the cube */
      cube = OpenRAVE::RaveCreateKinBody(this->e);
      cube->SetName("cube");
      
      /* set its dimensions */
      {
         std::vector<OpenRAVE::AABB> vaabbs(1);
         vaabbs[0].extents = OpenRAVE::Vector(cube_extent, cube_extent, cube_extent); /* extents = half side lengths */
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
         cd_kin_pose_compose(pose_world_gsdf, sdf_new.pose, pose_world_gsdf);
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

      printf("found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
      
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
      cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);
      cd_grid_destroy(g_obs);
      
      /* if we were passed a cache_filename, save what we just computed! */
      if (cache_filename[0])
      {
         FILE * fp;
         printf("saving sdf to file %s ...\n", cache_filename);
         fp = fopen(cache_filename, "wb");
         /* write struct sdf */
         fwrite(sdf_new.kinbody_name, sizeof(sdf_new.kinbody_name), 1, fp);
         fwrite(sdf_new.pose, sizeof(sdf_new.pose), 1, fp);
         /* write grid */
         fwrite(&sdf_new.grid->n, sizeof(sdf_new.grid->n), 1, fp);
         fwrite(sdf_new.grid->sizes, sizeof(sdf_new.grid->sizes[0]), sdf_new.grid->n, fp);
         fwrite(&sdf_new.grid->ncells, sizeof(sdf_new.grid->ncells), 1, fp);
         fwrite(&sdf_new.grid->cell_size, sizeof(sdf_new.grid->cell_size), 1, fp);
         fwrite(sdf_new.grid->data, sdf_new.grid->cell_size, sdf_new.grid->ncells, fp);
         fwrite(sdf_new.grid->lengths, sizeof(sdf_new.grid->lengths[0]), sdf_new.grid->n, fp);
         fclose(fp);
      }
   }
   
   /* allocate a new sdf struct, and copy the new one there! */
   this->sdfs = (struct sdf *) realloc(this->sdfs, (this->n_sdfs+1)*sizeof(struct sdf));
   this->sdfs[this->n_sdfs] = sdf_new;
   this->n_sdfs++;
   
   return true;
}

/* a rooted sdf (fixed in the world) */
struct cost_helper_rsdf
{
   double pose_world_gsdf[7];
   double pose_gsdf_world[7];
   struct cd_grid * grid;
};

struct cost_helper
{
   int n; /* config space dimensionality */
   int n_rsdfs;
   struct cost_helper_rsdf * rsdfs;
   OpenRAVE::RobotBase * r;
   int * adofindices;
   struct orcdchomp::sphere * spheres_all;
   int n_spheres_active;
   struct orcdchomp::sphere ** spheres_active;
   double * J; /* space for the jacobian; 3xn */
   double * J2;
   
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   
   clock_t ticks_fk;
   clock_t ticks_jacobian;
   clock_t ticks_selfcol;
};

int sphere_cost(struct cost_helper * h, double * c_point, double * c_vel, double * costp, double * c_grad)
{
   int i;
   int j;
   int err;
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
   struct orcdchomp::sphere * s;
   struct orcdchomp::sphere * s2;
   int sdfi_best;
   double sdfi_best_dist;
   int sai;
   clock_t tic;
   clock_t toc;
   
   /* put the robot in the config */
   std::vector<OpenRAVE::dReal> vec(c_point, c_point+h->n);
   tic = clock();
   h->r->SetActiveDOFValues(vec);
   toc = clock();
   h->ticks_fk += (toc - tic);

   /* start with a zero cost and config-space gradient */
   cost = 0.0;
   if (c_grad) cd_mat_set_zero(c_grad, h->n, 1);
   
   /* the cost and its gradient are summed over each sphere on the robot */
   for (sai=0; sai<h->n_spheres_active; sai++)
   {
      s = h->spheres_active[sai];
      
      cost_sphere = 0.0;
      
      /* get sphere center */
      OpenRAVE::Transform t = h->r->GetLink(s->linkname)->GetTransform();
      OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */

      /* compute the manipulator jacobian at this point, at this link */
      tic = clock();
      h->r->CalculateJacobian(s->linkindex, v, orjacobian);
      toc = clock();
      h->ticks_jacobian += (toc - tic);
      /* copy the active columns of orjacobian into our J */
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            h->J[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      /* compute the current workspace velocity of the sphere */
      cblas_dgemv(CblasRowMajor, CblasNoTrans, 3, h->n,
         1.0, h->J,h->n, c_vel,1, 0.0, x_vel,1);
      x_vel_norm = cblas_dnrm2(3, x_vel, 1);
      
      /* compute which distance field is closest to an obstacle */
      sdfi_best_dist = HUGE_VAL;
      sdfi_best = -1;
      for (i=0; i<h->n_rsdfs; i++)
      {
         /* transform sphere center into grid frame */
         g_point[0] = v.x;
         g_point[1] = v.y;
         g_point[2] = v.z;
         cd_kin_pose_compos(h->rsdfs[i].pose_gsdf_world, g_point, g_point);
         /* get sdf value (from interp) */
         err = cd_grid_double_interp(h->rsdfs[i].grid, g_point, &dist);
         if (err)
            continue; /* not inside of this distance field at all! */
         if (dist < sdfi_best_dist)
         {
            sdfi_best_dist = dist;
            sdfi_best = i;
         }
      }
      if (sdfi_best == -1)
      {
         /* this sphere is not inside any distance field;
          * for now, assume that it contributes no cost or gradient. */
         continue;
      }

      /* transform sphere center into grid frame */
      g_point[0] = v.x;
      g_point[1] = v.y;
      g_point[2] = v.z;
      cd_kin_pose_compos(h->rsdfs[sdfi_best].pose_gsdf_world, g_point, g_point);
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->rsdfs[sdfi_best].grid, g_point, &dist);
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= s->radius;
      
      if (costp)
      {
         /* convert to a cost */
         if (dist < 0.0)
            cost_sphere += h->obs_factor * (0.5 * h->epsilon - dist);
         else if (dist < h->epsilon)
            cost_sphere += h->obs_factor * (0.5/h->epsilon) * (dist-h->epsilon) * (dist-h->epsilon);
      }
      
      if (c_grad)
      {
         /* get sdf gradient */
         cd_grid_double_grad(h->rsdfs[sdfi_best].grid, g_point, g_grad); /* this will be a unit vector away from closest obs */
         cd_kin_pose_compose_vec(h->rsdfs[sdfi_best].pose_world_gsdf, g_grad, g_grad); /* now in world frame */
         
         /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
         cd_mat_memcpy(x_grad, g_grad, 3, 1);
         if (dist < 0.0)
            cd_mat_scale(x_grad, 3, 1, -1.0);
         else if (dist < h->epsilon)
            cd_mat_scale(x_grad, 3, 1, dist/h->epsilon - 1.0);
         else
            cd_mat_set_zero(x_grad, 3, 1);
         cd_mat_scale(x_grad, 3, 1, h->obs_factor);
         
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
      tic = clock();
      for (s2=h->spheres_all; s2; s2=s2->next)
      {
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (s->linkindex == s2->linkindex) continue;
         
         /* skip spheres far enough away from us */
         dist = s->radius + s2->radius + h->epsilon_self;
         OpenRAVE::Vector v_from_other = v - h->r->GetLink(s2->linkname)->GetTransform() * OpenRAVE::Vector(s2->pos);
         if (v_from_other.lengthsqr3() > dist*dist) continue;
         
         if (costp)
         {
            /* compute the cost */
            dist = sqrt(v_from_other.lengthsqr3()) - s->radius - s2->radius;
            if (dist < 0.0)
               cost_sphere += h->obs_factor_self * (0.5 * h->epsilon_self - dist);
            else
               cost_sphere += h->obs_factor_self * (0.5/h->epsilon_self) * (dist-h->epsilon_self) * (dist-h->epsilon_self);
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
            else if (dist < h->epsilon_self)
               cd_mat_scale(x_grad, 3, 1, dist/h->epsilon_self - 1.0);
            cd_mat_scale(x_grad, 3, 1, h->obs_factor_self);
            
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
      toc = clock();
      h->ticks_selfcol += (toc - tic);
      
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
   int n_intpoints;
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   int no_collision_exception;
   int no_report_cost;
   /* stuff we compute later */
   int * adofindices;
   struct cd_chomp * c;
   struct cost_helper h;
   double * Gjlimit;
   double * GjlimitAinv;
   std::vector< OpenRAVE::dReal > vec_jlimit_lower;
   std::vector< OpenRAVE::dReal > vec_jlimit_upper;
   struct orcdchomp::sphere * s;
   struct orcdchomp::sphere * spheres_all;
   int n_spheres_active;
   struct orcdchomp::sphere ** spheres_active;
   OpenRAVE::TrajectoryBasePtr starttraj;
   double cost_total;
   double cost_obs;
   double cost_smooth;
   clock_t clock_start;
   clock_t clock_end;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /*r = 0;*/
   adofgoal = 0;
   n_iter = 10;
   lambda = 10.0;
   n_intpoints = 99;
   epsilon = 0.1; /* in meters */
   epsilon_self = 0.04; /* in meters */
   obs_factor = 200.0;
   obs_factor_self = 10.0;
   no_collision_exception = 0;
   no_report_cost = 0;
   
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
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"lambda"))
         {
            sscanf(cur, " %lf%n", &lambda, &len); cur += len;
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
         if (strp_skipprefix(&cur, (char *)"n_intpoints"))
         {
            sscanf(cur, " %d%n", &n_intpoints, &len); cur += len;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"epsilon"))
         {
            sscanf(cur, " %lf%n", &epsilon, &len); cur += len;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"epsilon_self"))
         {
            sscanf(cur, " %lf%n", &epsilon_self, &len); cur += len;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"obs_factor"))
         {
            sscanf(cur, " %lf%n", &obs_factor, &len); cur += len;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"obs_factor_self"))
         {
            sscanf(cur, " %lf%n", &obs_factor_self, &len); cur += len;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"no_collision_exception"))
         {
            no_collision_exception = 1;
            continue;
         }
         if (strp_skipprefix(&cur, (char *)"no_report_cost"))
         {
            no_report_cost = 1;
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
   
   if (!this->n_sdfs)
      { free(adofgoal); throw OpenRAVE::openrave_exception("No signed distance fields have yet been computed!"); }
   
   if (n_iter < 0)
      { free(adofgoal); throw OpenRAVE::openrave_exception("n_iter must be >=0!"); }
   
   if (lambda < 0.01)
      { free(adofgoal); throw OpenRAVE::openrave_exception("lambda must be >=0.01!"); }
   
   if (n_intpoints < 1)
      { free(adofgoal); throw OpenRAVE::openrave_exception("n_intpoints must be >=1!"); }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<orcdchomp::rdata> d = boost::dynamic_pointer_cast<orcdchomp::rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { free(adofgoal); throw OpenRAVE::openrave_exception("robot does not have a <orcdchomp> tag defined!"); }
      spheres_all = d->spheres;
   }
   
   n_dof = r->GetActiveDOF();
   
   /* allocate adofindices */
   adofindices = (int *) malloc(n_dof * sizeof(int));
   {
      std::vector<int> vec = r->GetActiveDOFIndices();
      printf("adofindices:");
      for (i=0; i<n_dof; i++)
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
   Gjlimit = (double *) malloc(n_intpoints * n_dof * sizeof(double));
   GjlimitAinv = (double *) malloc(n_intpoints * n_dof * sizeof(double));
   r->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   
   /* (re-)compute sphere link indices */
   for (s=spheres_all; s; s=s->next)
      s->linkindex = r->GetLink(s->linkname)->GetIndex();
   
   /* compute which spheres are affected by the active dofs */
   n_spheres_active = 0;
   spheres_active = 0;
   for (s=spheres_all; s; s=s->next)
   {
      for (i=0; i<n_dof; i++)
         if (r->DoesAffect(adofindices[i], s->linkindex))
            break;
      if (i==n_dof)
         continue;
      spheres_active = (struct orcdchomp::sphere **) realloc(spheres_active, (n_spheres_active+1)*sizeof(struct orcdchomp::sphere *));
      spheres_active[n_spheres_active] = s;
      n_spheres_active++;
   }
   
   /* initialize the cost helper */
   h.n = n_dof;
   h.r = r.get();
   h.spheres_all = spheres_all;
   h.n_spheres_active = n_spheres_active;
   h.spheres_active = spheres_active;
   h.adofindices = adofindices;
   h.J = (double *) malloc(3*n_dof*sizeof(double));
   h.J2 = (double *) malloc(3*n_dof*sizeof(double));
   h.epsilon = epsilon;
   h.epsilon_self = epsilon_self;
   h.obs_factor = obs_factor;
   h.obs_factor_self = obs_factor_self;
   h.ticks_jacobian = 0;
   h.ticks_fk = 0;
   h.ticks_selfcol = 0;
   
   /* compute the rooted sdfs ... */
   h.n_rsdfs = this->n_sdfs;
   h.rsdfs = (struct cost_helper_rsdf *) malloc(this->n_sdfs * sizeof(struct cost_helper_rsdf));
   for (i=0; i<this->n_sdfs; i++)
   {
      h.rsdfs[i].grid = this->sdfs[i].grid;
      OpenRAVE::Transform t = this->e->GetKinBody(this->sdfs[i].kinbody_name)->GetTransform();
      h.rsdfs[i].pose_world_gsdf[0] = t.trans.x;
      h.rsdfs[i].pose_world_gsdf[1] = t.trans.y;
      h.rsdfs[i].pose_world_gsdf[2] = t.trans.z;
      h.rsdfs[i].pose_world_gsdf[3] = t.rot.y;
      h.rsdfs[i].pose_world_gsdf[4] = t.rot.z;
      h.rsdfs[i].pose_world_gsdf[5] = t.rot.w;
      h.rsdfs[i].pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(h.rsdfs[i].pose_world_gsdf, this->sdfs[i].pose, h.rsdfs[i].pose_world_gsdf);
      cd_kin_pose_invert(h.rsdfs[i].pose_world_gsdf, h.rsdfs[i].pose_gsdf_world);
   }
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, n_dof, n_intpoints, 1, &h,
      (int (*)(void *, double *, double *, double *, double *))sphere_cost);
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofgoal); free(adofindices); free(spheres_active); throw OpenRAVE::openrave_exception("Error creating chomp instance."); }
   /*c->lambda = 1000000.0;*/
   c->lambda = lambda;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   /* initialize trajectory */
   if (starttraj.get())
   {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      for (i=0; i<n_intpoints+2; i++)
      {
         std::vector<OpenRAVE::dReal> vec;
         starttraj->Sample(vec, i*starttraj->GetDuration()/(n_intpoints+1), r->GetActiveConfigurationSpecification());
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
      cd_mat_memcpy(c->T_ext_points[n_intpoints+1], adofgoal, n_dof, 1);
      /* all points in between */
      cd_mat_set_zero(&c->T_ext_points[1][0], n_intpoints, n_dof);
      for (i=1; i<n_intpoints+1; i++)
      {
         percentage = 1.0*i/(n_intpoints+1); /* between 0.0 (starting point) to 1.0 (ending point) */
         cblas_daxpy(n_dof, 1.0-percentage, c->T_ext_points[0],1,             c->T_ext_points[i],1);
         cblas_daxpy(n_dof,     percentage, c->T_ext_points[n_intpoints+1],1, c->T_ext_points[i],1);
      }
   }
   free(adofgoal);
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); free(spheres_active); throw OpenRAVE::openrave_exception("Error initializing chomp instance."); }
   
   RAVELOG_INFO("iterating CHOMP ...\n");
   clock_start = clock();
   for (iter=0; iter<n_iter; iter++)
   {
#if 0
      /* lambda increases over time */
      c->lambda = 10.0 * exp(0.1 * iter);
      printf("lambda: %f\n", c->lambda);
#endif

      if (no_report_cost)
      {
         cd_chomp_iterate(c, 1, 0, 0, 0);
      }
      else
      {
         cd_chomp_iterate(c, 1, &cost_total, &cost_obs, &cost_smooth);
         printf("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f\n", iter, cost_total, cost_obs, cost_smooth);
      }
      
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
   clock_end = clock();
   
   cd_chomp_iterate(c, 0, &cost_total, &cost_obs, &cost_smooth);
   printf("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f [FINAL]\n", iter, cost_total, cost_obs, cost_smooth);
   
   printf("done!\n");
   
   printf("Clock time for %d iterations: %.3f\n", iter, (1.0*clock_end/CLOCKS_PER_SEC)-(1.0*clock_start/CLOCKS_PER_SEC));
   printf("Time breakdown:\n");
   printf("  ticks_vels       %.3f\n", 1.0*c->ticks_vels/CLOCKS_PER_SEC);
   printf("  ticks_callbacks  %.3f\n", 1.0*c->ticks_callbacks/CLOCKS_PER_SEC);
   printf("    ticks_fk       %.3f\n", 1.0*h.ticks_fk/CLOCKS_PER_SEC);
   printf("    ticks_jacobian   %.3f\n", 1.0*h.ticks_jacobian/CLOCKS_PER_SEC);
   printf("    ticks_selfcol    %.3f\n", 1.0*h.ticks_selfcol/CLOCKS_PER_SEC);
   printf("  ticks_smoothgrad %.3f\n", 1.0*c->ticks_smoothgrad/CLOCKS_PER_SEC);
   printf("  ticks_smoothcost %.3f\n", 1.0*c->ticks_smoothcost/CLOCKS_PER_SEC);
   
   /* create an openrave trajectory from the result, and send to sout */
   OpenRAVE::TrajectoryBasePtr t = OpenRAVE::RaveCreateTrajectory(this->e);
   t->Init(r->GetActiveConfigurationSpecification());
   for (i=0; i<n_intpoints+2; i++)
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
            if (!no_collision_exception)
               { free(Gjlimit); free(GjlimitAinv); free(h.J); free(h.J2); free(adofindices); cd_chomp_destroy(c); free(spheres_active); throw OpenRAVE::openrave_exception("Resulting trajectory is in collision!"); }
         }
      }
   }
   
   t->serialize(sout);
   
   cd_chomp_destroy(c);
   
   free(GjlimitAinv);
   free(Gjlimit);
   free(h.rsdfs);
   free(h.J);
   free(h.J2);
   free(adofindices);
   free(spheres_active);

   printf("runchomp done! returning ...\n");
   return true;
}

} /* namespace orcdchomp */
