/** \file orcdchomp_mod.cpp
 * \brief Implementation of the orcdchomp module, an implementation of CHOMP
 *        using libcd.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

#include <time.h>
#include <cblas.h>

extern "C" {
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <libcd/chomp.h>
#include <libcd/grid.h>
#include <libcd/grid_flood.h>
#include <libcd/kin.h>
#include <libcd/mat.h>
#include <libcd/os.h>
#include <libcd/spatial.h>
#include <libcd/util.h>
#include <libcd/util_shparse.h>
}

#include <openrave/openrave.h>
#include <openrave/planningutils.h>

#include "orcdchomp_rdata.h"
#include "orcdchomp_mod.h"

#define DEBUG_TIMING

#ifdef DEBUG_TIMING
#  define TIC() { struct timespec tic; struct timespec toc; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &tic);
#  define TOC(tsptr) clock_gettime(CLOCK_THREAD_CPUTIME_ID, &toc); CD_OS_TIMESPEC_SUB(&toc, &tic); CD_OS_TIMESPEC_ADD(tsptr, &toc); }
#else
#  define TIC()
#  define TOC(tsptr)
#endif


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

int mod::viewspheres(int argc, char * argv[], std::ostream& sout)
{
   int i;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   OpenRAVE::RobotBasePtr r;
   char buf[1024];
   struct orcdchomp::sphere * s;
   struct orcdchomp::sphere * spheres;
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"robot")==0 && i+1<argc)
      {
         if (r.get()) throw OpenRAVE::openrave_exception("Only one robot can be passed!");
         RAVELOG_INFO("Getting robot named |%s|.\n", argv[i+1]);
         r = this->e->GetRobot(argv[++i]);
         if (!r.get()) throw OpenRAVE::openrave_exception("Could not find robot with that name!");
         RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
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
   
   return 0;
}


/* computedistancefield robot Herb2
 * computes a distance field in the vicinity of the passed kinbody
 * 
 * note: does aabb include disabled bodies? probably, but we might hope not ...
 * */
int mod::computedistancefield(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int err;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::KinBodyPtr kinbody;
   /* parameters */
   double cube_extent;
   double aabb_padding;
   char * cache_filename;
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
   aabb_padding = 0.2;
   cache_filename = 0;

   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"kinbody")==0 && i+1<argc)
      {
         if (kinbody.get()) throw OpenRAVE::openrave_exception("Only one kinbody can be passed!");
         kinbody = this->e->GetKinBody(argv[++i]);
         if (!kinbody.get()) throw OpenRAVE::openrave_exception("Could not find kinbody with that name!");
      }
      else if (strcmp(argv[i],"aabb_padding")==0 && i+1<argc)
         aabb_padding = atof(argv[++i]);
      else if (strcmp(argv[i],"cube_extent")==0 && i+1<argc)
         cube_extent = atof(argv[++i]);
      else if (strcmp(argv[i],"cache_filename")==0 && i+1<argc)
         cache_filename = argv[++i];
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_INFO("Using kinbody %s.\n", kinbody->GetName().c_str());
   RAVELOG_INFO("Using aabb_padding |%f|.\n", aabb_padding);
   RAVELOG_INFO("Using cube_extent |%f|.\n", cube_extent);
   RAVELOG_INFO("Using cache_filename |%s|.\n", cache_filename ? cache_filename : "none passed");

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
   if (cache_filename) do
   {
      FILE * fp;
      RAVELOG_INFO("reading sdf from file %s ...\n", cache_filename);
      fp = fopen(cache_filename, "rb");
      if (!fp) { RAVELOG_ERROR("could not read from file!\n"); break; }
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
      RAVELOG_INFO("    pos: %f %f %f\n", aabb.pos[0], aabb.pos[1], aabb.pos[2]);
      RAVELOG_INFO("extents: %f %f %f\n", aabb.extents[0], aabb.extents[1], aabb.extents[2]);

      /* calculate dimension sizes (number of cells) */
      for (i=0; i<3; i++)
      {
         /* 0.15m padding (was 0.3m) on each side
          * (this is the radius of the biggest herb2 spehere)
          * (note: extents are half the side lengths!) */
         gsdf_sizearray[i] = (int) ceil((aabb.extents[i]+aabb_padding) / cube_extent);
         RAVELOG_INFO("gsdf_sizearray[%d]: %d\n", i, gsdf_sizearray[i]);
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
      RAVELOG_INFO("computing occupancy grid ...\n");
      for (idx=0; idx<g_obs->ncells; idx++)
      {
         OpenRAVE::Transform t;
         
         if (idx % 100000 == 0)
            RAVELOG_INFO("idx=%d (%5.1f%%)...\n", (int)idx, (100.0*((double)idx)/((double)g_obs->ncells)));
         
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

      RAVELOG_INFO("found %d/%d collisions!\n", collisions, (int)(g_obs->ncells));
      
      /* remove cube */
      this->e->Remove(cube);
      
      /* we assume the point at the very top is free;
       * do flood fill to set all cells that are 1.0 -> 0.0 */
      RAVELOG_INFO("performing flood fill ...\n");
      {
         double point[3] = {2.0, 2.0, 3.999}; /* center, at the top */
         cd_grid_lookup_index(g_obs, point, &idx);
      }
      cd_grid_flood_fill(g_obs, idx, 0, (int (*)(void *, void *))replace_1_to_0, 0);
      
      /* change any remaining 1.0 cells to HUGE_VAL (assumed inside of obstacles) */
      for (idx=0; idx<g_obs->ncells; idx++)
         if (*(double *)cd_grid_get_index(g_obs, idx) == 1.0)
            *(double *)cd_grid_get_index(g_obs, idx) = HUGE_VAL;
      
      /* compute the signed distance field (in the module instance) */
      RAVELOG_INFO("computing signed distance field ...\n");
      cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);
      cd_grid_destroy(g_obs);
      
      /* if we were passed a cache_filename, save what we just computed! */
      if (cache_filename)
      {
         FILE * fp;
         RAVELOG_INFO("saving sdf to file %s ...\n", cache_filename);
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
   
   return 0;
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
   double * traj;
   int n_points;
   int m;
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
   struct tsr * start_tsr;
   int start_tsr_enabled[6]; /* xyzrpy */
   int start_tsr_k;
   
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   
   /* each of these filled by sphere_cost_pre */
   double dt;
   double * sphere_poss_all; /* [trajpoint(external)][sphereid][xyz] */
   double * sphere_poss; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_vels; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_accs; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_jacs; /* [trajpoint(moving)][sphereid][3xn] */
   
   struct timespec ticks_fk;
   struct timespec ticks_jacobians;
   struct timespec ticks_selfcol;
};

int sphere_cost_pre(struct cost_helper * h, int m, double ** T_points)
{
   int ti;
   int ti_mov;
   int i;
   int j;
   int sai;
   struct orcdchomp::sphere * s;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double * internal;
   
   /* compute positions of all spheres (including external points) */
   for (ti=0; ti<h->n_points; ti++)
   {
      /* put the robot in the config */
      std::vector<OpenRAVE::dReal> vec(&h->traj[ti*h->n], &h->traj[(ti+1)*h->n]);
      TIC()
      h->r->SetActiveDOFValues(vec);
      TOC(&h->ticks_fk)
      
      for (sai=0; sai<h->n_spheres_active; sai++)
      {
         s = h->spheres_active[sai];
         OpenRAVE::Transform t = h->r->GetLink(s->linkname)->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
         /* save sphere positions */
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 0] = v.x;
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 1] = v.y;
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 2] = v.z;
         /* get the moving point index */
         if (h->m == h->n_points - 2)
            ti_mov = ti-1;
         else
            ti_mov = ti;
         /* compute linear jacobians for all moving points */
         if (ti_mov < 0 || h->m <= ti_mov)
            continue;
         TIC()
         h->r->CalculateJacobian(s->linkindex, v, orjacobian);
         TOC(&h->ticks_jacobians)
         /* copy the active columns of orjacobian into our J */
         for (i=0; i<3; i++)
            for (j=0; j<h->n; j++)
               h->sphere_jacs[ti_mov*h->n_spheres_active*3*h->n + sai*3*h->n + i*h->n+j] = orjacobian[i][h->adofindices[j]];
      }
   }
   
   /* compute velocities for all moving points */
   /* start my computing internal velocities */
   internal = h->sphere_vels;
   if (h->m != h->n_points - 2)
      internal += h->n_spheres_active*3;
   cd_mat_memcpy(internal, h->sphere_poss_all + 2*h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_sub(internal, h->sphere_poss_all, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, 1.0/(2.0*h->dt));
   /* next do the start vel */
   if (h->m != h->n_points - 2)
   {
      cd_mat_memcpy(h->sphere_vels, h->sphere_poss + h->n_spheres_active*3, 1, h->n_spheres_active*3);
      cd_mat_sub(h->sphere_vels, h->sphere_poss, 1, h->n_spheres_active*3);
      cd_mat_scale(h->sphere_vels, 1, h->n_spheres_active*3, 1.0/(h->dt));
   }
   
   /* compute accelerations for all moving points */
   /* start my computing internal accelerations */
   internal = h->sphere_accs;
   if (h->m != h->n_points - 2)
      internal += h->n_spheres_active*3;
   cd_mat_memcpy(internal, h->sphere_poss + h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, -2.0);
   cd_mat_add(internal, h->sphere_poss, h->n_points-2, h->n_spheres_active*3);
   cd_mat_add(internal, h->sphere_poss + 2*h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, 1.0/(h->dt * h->dt));
   /* simply copy 1st accel into the 0th accel for now */
   if (h->m != h->n_points - 2)
      cd_mat_memcpy(h->sphere_accs, h->sphere_accs + h->n_spheres_active*3, 1, h->n_spheres_active*3);
   
   return 0;
}

int sphere_cost(struct cost_helper * h, int ti, double * c_point, double * c_vel, double * costp, double * c_grad)
{
   int i;
   int err;
   double * x_vel;
   double x_vel_norm;
   double g_point[3];
   double dist;
   double cost;
   double cost_sphere;
   double g_grad[3];
   double x_grad[3];
   double proj;
   double x_curv[3];
   struct orcdchomp::sphere * s;
   struct orcdchomp::sphere * s2;
   int sdfi_best;
   double sdfi_best_dist;
   int sai;
   int sai2;
   double v_from_other[3];
   
   /* start with a zero cost and config-space gradient */
   cost = 0.0;
   if (c_grad) cd_mat_set_zero(c_grad, h->n, 1);
   
   /* the cost and its gradient are summed over each sphere on the robot */
   for (sai=0; sai<h->n_spheres_active; sai++)
   {
      s = h->spheres_active[sai];
      
      cost_sphere = 0.0;

      /* compute the current workspace velocity of the sphere */
      x_vel = h->sphere_vels + ti*(h->n_spheres_active*3) + sai*3;
      x_vel_norm = cblas_dnrm2(3, x_vel, 1);
      
      /* compute which distance field is closest to an obstacle */
      sdfi_best_dist = HUGE_VAL;
      sdfi_best = -1;
      for (i=0; i<h->n_rsdfs; i++)
      {
         /* transform sphere center into grid frame */
         cd_kin_pose_compos(h->rsdfs[i].pose_gsdf_world,
            h->sphere_poss + ti*(h->n_spheres_active*3) + sai*3,
            g_point);
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
      cd_kin_pose_compos(h->rsdfs[sdfi_best].pose_gsdf_world,
         h->sphere_poss + ti*(h->n_spheres_active*3) + sai*3,
         g_point);
      /* get sdf value (from interp) */
      cd_grid_double_interp(h->rsdfs[sdfi_best].grid, g_point, &dist);
      /* subtract radius to get distance of closest sphere point to closest obstacle */
      dist -= s->radius;
      
      /* convert to a cost, scaled by sphere velocity */
      if (dist < 0.0)
         cost_sphere += x_vel_norm * h->obs_factor * (0.5 * h->epsilon - dist);
      else if (dist < h->epsilon)
         cost_sphere += x_vel_norm * h->obs_factor * (0.5/h->epsilon) * (dist-h->epsilon) * (dist-h->epsilon);
      
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
         if (x_vel_norm > 0.000001)
         {
            proj = cblas_ddot(3, x_grad,1, x_vel,1) / (x_vel_norm * x_vel_norm);
            cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
         }
         
         /* compute curvature vector */
         cd_mat_memcpy(x_curv, h->sphere_accs + ti*(h->n_spheres_active*3) + sai*3, 3, 1);
         if (x_vel_norm > 0.000001)
         {
            proj = cblas_ddot(3, x_curv,1, x_vel,1) / (x_vel_norm * x_vel_norm);
            cblas_daxpy(3, -proj, x_vel,1, x_curv,1);
         }
         cd_mat_scale(x_curv, 3, 1, 1.0 / (x_vel_norm * x_vel_norm));
         /* add that in to x_grad */
         cblas_daxpy(3, -cost_sphere, x_curv,1, x_grad,1);
         
         /* multiply into c_grad through JT, scaled by sphere velocity */
         cblas_dgemv(CblasRowMajor, CblasTrans, 3, h->n,
            x_vel_norm, h->sphere_jacs + ti*h->n_spheres_active*3*h->n + sai*3*h->n,h->n, x_grad,1, 1.0, c_grad,1);
      }

      /* consider effects from all other spheres (i.e. self collision) */
      TIC()
      for (sai2=0; sai2<h->n_spheres_active; sai2++)
      {
         s2 = h->spheres_active[sai2];
         
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (s->linkindex == s2->linkindex) continue;
         
         /* skip spheres far enough away from us */
         cd_mat_memcpy(v_from_other, h->sphere_poss + ti*(h->n_spheres_active*3) + sai*3, 3, 1);
         cd_mat_sub(v_from_other, h->sphere_poss + ti*(h->n_spheres_active*3) + sai2*3, 3, 1);
         dist = cblas_dnrm2(3, v_from_other,1);
         if (dist > s->radius + s2->radius + h->epsilon_self) continue;
         
         if (c_grad)
         {
            /* make unit vector (g_grad) away from other sphere */
            g_grad[0] = v_from_other[0] / dist;
            g_grad[1] = v_from_other[1] / dist;
            g_grad[2] = v_from_other[2] / dist;
         }
         
         dist -= s->radius + s2->radius;
         
         if (costp)
         {
            /* compute the cost */
            if (dist < 0.0)
               cost_sphere += x_vel_norm * h->obs_factor_self * (0.5 * h->epsilon_self - dist);
            else
               cost_sphere += x_vel_norm * h->obs_factor_self * (0.5/h->epsilon_self) * (dist-h->epsilon_self) * (dist-h->epsilon_self);
         }
         
         if (c_grad)
         {
            /* convert sdf g_grad to x_grad (w.r.t. cost) according to dist */
            cd_mat_memcpy(x_grad, g_grad, 3, 1);
            if (dist < 0.0)
               cd_mat_scale(x_grad, 3, 1, -1.0);
            else if (dist < h->epsilon_self)
               cd_mat_scale(x_grad, 3, 1, dist/h->epsilon_self - 1.0);
            cd_mat_scale(x_grad, 3, 1, h->obs_factor_self);
            
            /* subtract from x_grad vector projection onto x_vel */
            if (x_vel_norm > 0.000001)
            {
               proj = cblas_ddot(3, x_grad,1, x_vel,1) / (x_vel_norm * x_vel_norm);
               cblas_daxpy(3, -proj, x_vel,1, x_grad,1);
            }
            
            /* J2 = J - jacobian of other sphere*/
            cd_mat_memcpy(h->J2, h->sphere_jacs + ti*h->n_spheres_active*3*h->n + sai*3*h->n, 3, h->n);
            cd_mat_sub(h->J2, h->sphere_jacs + ti*h->n_spheres_active*3*h->n + sai2*3*h->n, 3, h->n);
            
            /* multiply into c_grad through JT */
            /* I HAVE NO IDEA WHY THERES A -1 HERE! */
            cblas_dgemv(CblasRowMajor, CblasTrans, 3, h->n,
               -1.0, h->J2,h->n, x_grad,1, 1.0, c_grad,1);
         }
      }
      TOC(&h->ticks_selfcol)
      
      cost += cost_sphere;
   }
   
   /* we potentially add the ee_force cost and its gradient ... */
   
   
   
   if (costp) *costp = cost;
   return 0;
}

int con_start_tsr(struct cost_helper * h, int ti, double * point, double * con_val, double * con_jacobian)
{
   int tsri;
   int ki;
   int i;
   int j;
   double pose_ee[7];
   double pose_obj[7];
   double pose_ee_obj[7];
   double pose_table_world[7];
   double pose_table_obj[7];
   double xyzypr_table_obj[7];
   
   double * spajac_world;
   int ee_link_index;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double xm_table_world[6][6];
   double jac_inverse[7][6];
   double pose_to_xyzypr_jac[6][7];
   double * full_result;
   double temp6x6a[6][6];
   double temp6x6b[6][6];
   
   /* put the arm in this configuration */
   std::vector<OpenRAVE::dReal> vec(point, point+h->n);
   h->r->SetActiveDOFValues(vec);
   
   /* get the end-effector transform */
   OpenRAVE::Transform t = h->r->GetActiveManipulator()->GetEndEffectorTransform();
   pose_ee[0] = t.trans.x;
   pose_ee[1] = t.trans.y;
   pose_ee[2] = t.trans.z;
   pose_ee[3] = t.rot.y;
   pose_ee[4] = t.rot.z;
   pose_ee[5] = t.rot.w;
   pose_ee[6] = t.rot.x;
   
   /* get the object pose */
   cd_kin_pose_invert(h->start_tsr->Twe, pose_ee_obj);
   cd_kin_pose_compose(pose_ee, pose_ee_obj, pose_obj);
   
   /* get the pose of the world w.r.t. the table */
   cd_kin_pose_invert(h->start_tsr->T0w, pose_table_world);
   
   /* get the pose of the object w.r.t. the table */
   cd_kin_pose_compose(pose_table_world, pose_obj, pose_table_obj);
   
   /* convert to xyzypr */
   cd_kin_pose_to_xyzypr(pose_table_obj, xyzypr_table_obj);
   
   /* fill the constraint value vector */
   ki=0;
   for (tsri=0; tsri<6; tsri++) if (h->start_tsr_enabled[tsri])
   {
      con_val[ki] = xyzypr_table_obj[tsri<3?tsri:8-tsri];
      ki++;
   }
   
   if (con_jacobian)
   {
      /* compute the spatial Jacobian! */
      spajac_world = (double *) malloc(6 * h->n * sizeof(double));
      full_result = (double *) malloc(6 * h->n * sizeof(double));
      
      ee_link_index = h->r->GetActiveManipulator()->GetEndEffector()->GetIndex();
      
      /* copy the active columns of (rotational) orjacobian into our J */
      h->r->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            spajac_world[i*h->n+j] = orjacobian[i][h->adofindices[j]];
      
      /* copy the active columns of (translational) orjacobian into our J */
      h->r->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<h->n; j++)
            spajac_world[(3+i)*h->n+j] = orjacobian[i][h->adofindices[j]];
            
      /* compute the spatial transform to get velocities in table frame */
      cd_spatial_xm_from_pose(xm_table_world, pose_table_world);
      
      /* calculate the pose derivative Jacobian matrix */
      cd_spatial_pose_jac_inverse(pose_table_obj, jac_inverse);
      
      /* calculate the xyzypr Jacobian matrix */
      cd_kin_pose_to_xyzypr_J(pose_table_obj, pose_to_xyzypr_jac);
      
      /* do the matrix multiplications! */
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 7,
         1.0, *pose_to_xyzypr_jac,7, *jac_inverse,6, 0.0, *temp6x6a,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6,
         1.0, *temp6x6a,6, *xm_table_world,6, 0.0, *temp6x6b,6);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, h->n, 6,
         1.0, *temp6x6b,6, spajac_world,h->n, 0.0, full_result,h->n);
      
      /* fill the constraint Jacbobian matrix */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (h->start_tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*h->n, full_result+(tsri<3?tsri:8-tsri)*h->n, 1, h->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}

/* runchomp robot Herb2
 * run chomp from the current config to the passed goal config
 * uses the active dofs of the passed robot
 * initialized with a straight-line trajectory
 * */
int mod::runchomp(int argc, char * argv[], std::ostream& sout)
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
   double max_time;
   double lambda;
   int n_points; /* points, including endpoints if any */
   int use_momentum;
   int use_hmc;
   double hmc_resample_lambda;
   unsigned int seed;
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   int no_collision_exception;
   int no_collision_details;
   char * dat_filename;
   char * trajs_fileformstr;
   double ee_force[3];
   double ee_force_at[3];
   double * ee_torque_weights;
   int ee_torque_weights_n;
   /* stuff we compute later */
   char trajs_filename[1024];
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
   FILE * fp_dat;
   const char * exc;
   OpenRAVE::TrajectoryBasePtr t;
   struct timespec ticks;
   struct timespec ticks_tic;
   struct timespec ticks_toc;
   int hmc_resample_iter;
   double hmc_alpha;
   gsl_rng * rng;
   
   /* the exception string */
   exc = 0;
   
   /* initialize all mallocs to zero */
   c = 0;
   GjlimitAinv = 0;
   Gjlimit = 0;
   h.traj = 0;
   h.rsdfs = 0;
   h.J = 0;
   h.J2 = 0;
   h.sphere_poss_all = 0;
   h.sphere_vels = 0;
   h.sphere_accs = 0;
   h.sphere_jacs = 0;
   adofindices = 0;
   spheres_active = 0;
   fp_dat = 0;
   rng = 0;
   
   /* lock environment */
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* default parameters */
   adofgoal = 0;
   h.start_tsr = 0;
   n_iter = 10;
   max_time = HUGE_VAL;
   lambda = 10.0;
   n_points = 101;
   use_momentum = 0;
   use_hmc = 0;
   hmc_resample_lambda = 0.02; /* parameter of exponential distribution over iterations between resamples */
   seed = 0;
   epsilon = 0.1; /* in meters */
   epsilon_self = 0.04; /* in meters */
   obs_factor = 200.0;
   obs_factor_self = 10.0;
   no_collision_exception = 0;
   no_collision_details = 0;
   dat_filename = 0;
   trajs_fileformstr = 0;
   cd_mat_set_zero(ee_force, 3, 1);
   cd_mat_set_zero(ee_force_at, 3, 1);
   ee_torque_weights = 0;
   ee_torque_weights_n = 0;
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"robot")==0 && i+1<argc)
      {
         if (r.get()) { exc = "Only one robot can be passed!"; goto error; }
         r = this->e->GetRobot(argv[++i]);
         if (!r.get()) { exc = "Could not find robot with that name!"; goto error; }
      }
      else if (strcmp(argv[i],"adofgoal")==0 && i+1<argc)
      {
         if (adofgoal) { exc = "Only one adofgoal can be passed!"; goto error; }
         if (starttraj.get()) { exc = "Cannot pass both adofgoal and starttraj!"; goto error; }
         {
            char ** adofgoal_argv;
            cd_util_shparse(argv[++i], &n_adofgoal, &adofgoal_argv);
            adofgoal = (double *) malloc(n_adofgoal * sizeof(double));
            for (j=0; j<n_adofgoal; j++)
               adofgoal[j] = atof(adofgoal_argv[j]);
            free(adofgoal_argv);
         }
         cd_mat_vec_print("parsed adofgoal: ", adofgoal, n_adofgoal);
      }
      else if (strcmp(argv[i],"start_tsr")==0 && i+1<argc)
      {
         err = tsr_create_parse(&h.start_tsr, argv[++i]);
         if (err) { exc = "Cannot parse TSR!"; goto error; }
      }
      else if (strcmp(argv[i],"n_iter")==0 && i+1<argc)
         n_iter = atoi(argv[++i]);
      else if (strcmp(argv[i],"lambda")==0 && i+1<argc)
         lambda = atof(argv[++i]);
      else if (strcmp(argv[i],"max_time")==0 && i+1<argc)
         max_time = atof(argv[++i]);
      else if (strcmp(argv[i],"starttraj")==0 && i+1<argc)
      {
         if (starttraj.get()) { exc = "Only one starttraj can be passed!"; goto error; }
         if (adofgoal) { exc = "Cannot pass both adofgoal and starttraj!"; goto error; }
         starttraj = RaveCreateTrajectory(this->e);
         std::string my_string(argv[++i]);
         std::istringstream ser_iss(my_string);
         starttraj->deserialize(ser_iss);
      }
      else if (strcmp(argv[i],"n_points")==0 && i+1<argc)
         n_points = atoi(argv[++i]);
      else if (strcmp(argv[i],"use_momentum")==0)
         use_momentum = 1;
      else if (strcmp(argv[i],"use_hmc")==0)
         use_hmc = 1;
      else if (strcmp(argv[i],"hmc_resample_lambda")==0 && i+1<argc)
         hmc_resample_lambda = atof(argv[++i]);
      else if (strcmp(argv[i],"seed")==0 && i+1<argc)
         sscanf(argv[++i], "%u", &seed);
      else if (strcmp(argv[i],"epsilon")==0 && i+1<argc)
         epsilon = atof(argv[++i]);
      else if (strcmp(argv[i],"epsilon_self")==0 && i+1<argc)
         epsilon_self = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor")==0 && i+1<argc)
         obs_factor = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor_self")==0 && i+1<argc)
         obs_factor_self = atof(argv[++i]);
      else if (strcmp(argv[i],"no_collision_exception")==0)
         no_collision_exception = 1;
      else if (strcmp(argv[i],"no_collision_details")==0)
         no_collision_details = 1;
      else if (strcmp(argv[i],"dat_filename")==0 && i+1<argc)
         dat_filename = argv[++i];
      else if (strcmp(argv[i],"trajs_fileformstr")==0 && i+1<argc)
         trajs_fileformstr = argv[++i];
      else if (strcmp(argv[i],"ee_force")==0 && i+1<argc)
      {
         int ee_force_argc;
         char ** ee_force_argv;
         cd_util_shparse(argv[++i], &ee_force_argc, &ee_force_argv);
         if (ee_force_argc == 1)
         {
            ee_force[0] = 0.0;
            ee_force[1] = 0.0;
            ee_force[2] = -atof(ee_force_argv[0]);
         }
         else if (ee_force_argc == 3)
         {
            ee_force[0] = atof(ee_force_argv[0]);
            ee_force[1] = atof(ee_force_argv[1]);
            ee_force[2] = atof(ee_force_argv[2]);
         }
         else
            { exc = "ee_force must be length 1 or 3!"; goto error; }
      }
      else if (strcmp(argv[i],"ee_force_at")==0 && i+1<argc)
      {
         int ee_force_at_argc;
         char ** ee_force_at_argv;
         cd_util_shparse(argv[++i], &ee_force_at_argc, &ee_force_at_argv);
         if (ee_force_at_argc != 3)
            { exc = "ee_force_at must be length 3!"; goto error; }
         ee_force_at[0] = atof(ee_force_at_argv[0]);
         ee_force_at[1] = atof(ee_force_at_argv[1]);
         ee_force_at[2] = atof(ee_force_at_argv[2]);
      }
      else if (strcmp(argv[i],"ee_torque_weights")==0 && i+1<argc)
      {
         char ** my_argv;
         if (ee_torque_weights) { exc = "Only one ee_torque_weights can be passed!"; goto error; }
         cd_util_shparse(argv[++i], &ee_torque_weights_n, &my_argv);
         ee_torque_weights = (double *) malloc(ee_torque_weights_n * sizeof(double));
         for (j=0; j<ee_torque_weights_n; j++)
            ee_torque_weights[j] = atof(my_argv[j]);
         free(my_argv);
         cd_mat_vec_print("parsed ee_torque_weights: ", ee_torque_weights, ee_torque_weights_n);
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_INFO("Using robot %s.\n", r->GetName().c_str());
   if (dat_filename) RAVELOG_INFO("Using dat_filename |%s|.\n", dat_filename);
   if (trajs_fileformstr) RAVELOG_INFO("Using trajs_fileformstr |%s|.\n", trajs_fileformstr);
   
   /* check validity of input arguments ... */
   if (!r.get()) { exc = "Did not pass a robot!"; goto error; }
   if (!adofgoal && !starttraj.get()) { exc = "Did not pass either adofgoal or starttraj!"; goto error; }
   if (!this->n_sdfs) { exc = "No signed distance fields have yet been computed!"; goto error; }
   if (n_iter < 0) { exc = "n_iter must be >=0!"; goto error; }
   if (lambda < 0.01) { exc = "lambda must be >=0.01!"; goto error; }
   if (n_points < 3) { exc = "n_points must be >=3!"; goto error; }
   
   n_dof = r->GetActiveDOF();
   
   /* check that n_adofgoal matches active dof */
   if (adofgoal && (n_dof != n_adofgoal))
   {
      RAVELOG_INFO("n_dof: %d; n_adofgoal: %d\n", n_dof, n_adofgoal);
      exc = "size of adofgoal does not match active dofs!";
      goto error;
   }
   
   /* check that ee_torque_weights_n matches */
   if (ee_torque_weights && (ee_torque_weights_n != n_dof))
   {
      RAVELOG_INFO("n_dof: %d; ee_torque_weights_n: %d\n", n_dof, ee_torque_weights_n);
      exc = "size of ee_torque_weights does not match active dofs!";
      goto error;
   }
   
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
   
   /* ensure that if we're doing the ee_force math, the active dofs are
    * only revolute dofs (since we do our own jacobian math) */
   if (ee_torque_weights)
   {
      printf("ee_torque_weights check for revolute only ...\n");
      for (i=0; i<n_dof; i++)
      {
         printf("joint type: %d\n", r->GetJointFromDOFIndex(adofindices[i])->GetType());
      }
   }
   
   /* ensure the robot has spheres defined */
   {
      boost::shared_ptr<orcdchomp::rdata> d = boost::dynamic_pointer_cast<orcdchomp::rdata>(r->GetReadableInterface("orcdchomp"));
      if (!d) { exc = "robot does not have a <orcdchomp> tag defined!"; goto error; }
      spheres_all = d->spheres;
   }
   
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
   
   /* allocate rng */
   rng = gsl_rng_alloc(gsl_rng_default);
   gsl_rng_set(rng, seed);
   
   if (dat_filename)
   {
      fp_dat = fopen(dat_filename, "w");
      if (!fp_dat) { exc = "could not open dat_filename file for writing!"; goto error; }
   }
   
   /* OK, input arguments are parsed. Start timing! */
   CD_OS_TIMESPEC_SET_ZERO(&ticks);
   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
   
   /* check that starttraj has the right ConfigurationSpecification? */
   
   /* initialize the cost helper */
   h.n_points = n_points;
   h.m = n_points-2;
   if (h.start_tsr) h.m++;
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
   cd_os_timespec_set_zero(&h.ticks_jacobians);
   cd_os_timespec_set_zero(&h.ticks_fk);
   cd_os_timespec_set_zero(&h.ticks_selfcol);
   h.sphere_poss_all = (double *) malloc((n_points)*n_spheres_active*3*sizeof(double));
   if (h.start_tsr)
      h.sphere_poss = h.sphere_poss_all;
   else
      h.sphere_poss = h.sphere_poss_all + n_spheres_active*3;
   h.sphere_vels = (double *) malloc((h.m)*n_spheres_active*3*sizeof(double));
   h.sphere_accs = (double *) malloc((h.m)*n_spheres_active*3*sizeof(double));
   h.sphere_jacs = (double *) malloc((h.m)*n_spheres_active*(3*n_dof)*sizeof(double));
   
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
   
   /* get joint limit stuff (allocated for all moving points) */
   Gjlimit = (double *) malloc(h.m * n_dof * sizeof(double));
   GjlimitAinv = (double *) malloc(h.m * n_dof * sizeof(double));
   r->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   
   /* create the trajectory */
   h.traj = (double *) malloc(n_points*n_dof*sizeof(double));
   
   /* initialize trajectory */
   if (starttraj.get())
   {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      for (i=0; i<n_points; i++)
      {
         std::vector<OpenRAVE::dReal> vec;
         starttraj->Sample(vec, i*starttraj->GetDuration()/(n_points-1), r->GetActiveConfigurationSpecification());
         for (j=0; j<n_dof; j++)
            h.traj[i*n_dof+j] = vec[j];
      }
   }
   else
   {
      std::vector<OpenRAVE::dReal> start;
      RAVELOG_INFO("Initializing from a straight-line trajectory ...\n");
      cd_mat_set_zero(h.traj, n_points, n_dof);
      /* starting point */
      r->GetActiveDOFValues(start);
      for (i=0; i<n_points; i++)
         for (j=0; j<n_dof; j++)
            h.traj[i*n_dof+j] = start[j] + (adofgoal[j]-start[j])*i/(n_points-1);
   }
      
   /* calculate the dimensionality of the constraint */
   if (h.start_tsr)
   {
      printf("start_tsr");
      h.start_tsr_k = 0;
      for (i=0; i<6; i++)
      {
         if (h.start_tsr->Bw[i][0] == 0.0 && h.start_tsr->Bw[i][1] == 0.0)
         {
            h.start_tsr_enabled[i] = 1;
            h.start_tsr_k++;
         }
         else
            h.start_tsr_enabled[i] = 0;
         printf(" %d", h.start_tsr_enabled[i]);
      }
      printf("\n");
   }
   
   /* evaluate the constraint on the start and end points */
   if (h.start_tsr)
   {
      double * con_val;
      con_val = (double *) malloc(h.start_tsr_k * sizeof(double));
      printf("evaluating constraint on first point ...\n");
      con_start_tsr(&h, 0, h.traj, con_val, 0);
      cd_mat_vec_print("con_val: ", con_val, h.start_tsr_k);
      free(con_val);
   }
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, n_dof, h.m, 1, &h.traj[(h.start_tsr?0:1)*n_dof], n_dof);
   if (err) { exc = "error creating chomp instance!"; goto error; }
   /* set up trajectory */
   c->dt = 1.0/(n_points-1);
   if (h.start_tsr)
   {
      c->inits[0] = 0;
      cd_chomp_add_constraint(c, h.start_tsr_k, 0, &h,
      (int (*)(void * cptr, int i, double * point, double * con_val, double * con_jacobian))con_start_tsr);
   }
   else
      c->inits[0] = &h.traj[0*n_dof];
   c->finals[0] = &h.traj[(n_points-1)*n_dof];
   /* set up obstacle cost */
   c->cptr = &h;
   c->cost_pre = (int (*)(void *, int, double **))sphere_cost_pre;
   c->cost = (int (*)(void *, int, double *, double *, double *, double *))sphere_cost;
   /*c->lambda = 1000000.0;*/
   c->lambda = lambda;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   if (use_momentum)
      c->use_momentum = 1;
   
   if (use_hmc)
      hmc_resample_iter = 0;
      /*hmc_iters_until_resample = log(cd_util_rand_double(0.0, 1.0)) / hmc_resample_lambda;*/
   
   free(adofgoal);
   adofgoal = 0;
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { exc = "Error initializing chomp instance."; goto error; }

   h.dt = c->dt;
   
   RAVELOG_INFO("iterating CHOMP ...\n");
   for (iter=0; iter<n_iter; iter++)
   {
#if 0
      /* lambda increases over time */
      c->lambda = 10.0 * exp(0.1 * iter);
      RAVELOG_INFO("lambda: %f\n", c->lambda);
#endif

      /* resample momentum if using hmc */
      if (use_hmc && iter == hmc_resample_iter)
      {
         hmc_alpha = 100.0 * exp(0.02 * iter);
         printf("resampling momentum with alpha = %f ...\n", hmc_alpha);
         
         /* the momentum term is now AG */
         for (i=0; i<c->m; i++)
            for (j=0; j<c->n; j++)
               c->AG[i*c->n+j] = gsl_ran_gaussian(rng, 1.0/sqrt(hmc_alpha));
         c->leapfrog_first = 1;
         
         /* set new resampling iter */
         hmc_resample_iter += 1 + (int) (- log(gsl_rng_uniform(rng)) / hmc_resample_lambda);
      }
      
      /* dump the intermediate trajectory before each iteration */
      if (trajs_fileformstr)
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks, &ticks_toc);
         sprintf(trajs_filename, trajs_fileformstr, iter);
         t = OpenRAVE::RaveCreateTrajectory(this->e);
         t->Init(r->GetActiveConfigurationSpecification());
         for (i=0; i<n_points; i++)
         {
            std::vector<OpenRAVE::dReal> vec(&h.traj[i*n_dof], &h.traj[i*n_dof+n_dof]);
            t->Insert(i, vec);
         }
         std::ofstream f(trajs_filename);
         f << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1); /// have to do this or otherwise precision gets lost
         t->serialize(f);
         f.close();
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      }

      cd_chomp_iterate(c, 1, &cost_total, &cost_obs, &cost_smooth);
      RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f\n", iter, cost_total, cost_obs, cost_smooth);
      
      /* dump stats to data file (note these stats are for trajectory before this iteration) */
      if (fp_dat)
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks_toc, &ticks);
         fprintf(fp_dat, "%d %f %f %f %f\n",
            iter, CD_OS_TIMESPEC_DOUBLE(&ticks_toc), cost_total, cost_obs, cost_smooth);
      }
      
      /* handle joint limits */
      { int num_limadjs; num_limadjs=0;
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
         /*printf("num_limadjs: %d\n", num_limadjs);*/
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
            1.0, c->Ainv,c->m, Gjlimit,c->n, 0.0, GjlimitAinv,c->n);
            
         /* compute scalar necessary to make trajectory satisfy limit at largest_idx */
         cblas_daxpy(c->m * c->n,
                     1.01 * Gjlimit[largest_idx] / GjlimitAinv[largest_idx],
                     GjlimitAinv,1, c->T,1);
         
         num_limadjs++;
         if (num_limadjs == 100)
         {
            printf("ran too many joint limit fixes! aborting ...\n");
            abort();
         }
      }}
      
      /* quit if we're over time! */
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks_toc, &ticks);
         if (CD_OS_TIMESPEC_DOUBLE(&ticks_toc) > max_time)
            break;
      }
   }
   
   cd_chomp_iterate(c, 0, &cost_total, &cost_obs, &cost_smooth);
   RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f [FINAL]\n", iter, cost_total, cost_obs, cost_smooth);
   
   RAVELOG_INFO("done!\n");
   
   /*printf("Clock time for %d iterations: %.8f\n", iter, cd_os_timespec_double(&ticks_iterations));*/
   RAVELOG_INFO("Time breakdown:\n");
   RAVELOG_INFO("  ticks_vels         %.8f\n", cd_os_timespec_double(&c->ticks_vels));
   RAVELOG_INFO("  ticks_callback_pre %.8f\n", cd_os_timespec_double(&c->ticks_callback_pre));
   RAVELOG_INFO("  ticks_callbacks    %.8f\n", cd_os_timespec_double(&c->ticks_callbacks));
   RAVELOG_INFO("    ticks_fk           %.8f\n", cd_os_timespec_double(&h.ticks_fk));
   RAVELOG_INFO("    ticks_jacobians    %.8f\n", cd_os_timespec_double(&h.ticks_jacobians));
   RAVELOG_INFO("    ticks_selfcol      %.8f\n", cd_os_timespec_double(&h.ticks_selfcol));
   RAVELOG_INFO("  ticks_smoothgrad   %.8f\n", cd_os_timespec_double(&c->ticks_smoothgrad));
   RAVELOG_INFO("  ticks_smoothcost   %.8f\n", cd_os_timespec_double(&c->ticks_smoothcost));
   
   /* create an openrave trajectory from the result, and send to sout */
   t = OpenRAVE::RaveCreateTrajectory(this->e);
   t->Init(r->GetActiveConfigurationSpecification());
   for (i=0; i<n_points; i++)
   {
      std::vector<OpenRAVE::dReal> vec(&h.traj[i*n_dof], &h.traj[(i+1)*n_dof]);
      t->Insert(i, vec);
   }
   
   RAVELOG_INFO("timing trajectory ...\n");
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
   /* new openrave added a fmaxaccelmult parameter (number 5) */
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,r,false,0.2,0.2,"","");
#else
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,r,false,0.2,"");
#endif
   RAVELOG_INFO("checking trajectory for collision ...\n");
   {
      int collides = 0;
      double time;
      OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
      for (time=0.0; time<t->GetDuration(); time+=0.01)
      {
         std::vector< OpenRAVE::dReal > point;
         t->Sample(point, time);
         r->SetActiveDOFValues(point);
         if (this->e->CheckCollision(r,report))
         {
            collides = 1;
            if (!no_collision_details) RAVELOG_ERROR("Collision: %s\n", report->__str__().c_str());
            if (!no_collision_exception) { exc = "Resulting trajectory is in collision!"; goto error; }
         }
      }
      if (collides)
         RAVELOG_ERROR("   trajectory collides!\n");
   }
   
   t->serialize(sout);
   
error:
   if (c) cd_chomp_destroy(c);
   free(GjlimitAinv);
   free(Gjlimit);
   free(h.traj);
   free(h.rsdfs);
   free(h.J);
   free(h.J2);
   free(h.sphere_poss_all);
   free(h.sphere_vels);
   free(h.sphere_accs);
   free(h.sphere_jacs);
   free(adofindices);
   free(spheres_active);
   free(adofgoal);
   free(ee_torque_weights);
   if (h.start_tsr) tsr_destroy(h.start_tsr);
   if (rng) gsl_rng_free(rng);
   if (fp_dat) fclose(fp_dat);
   
   if (exc)
      throw OpenRAVE::openrave_exception(exc);

   RAVELOG_INFO("runchomp done! returning ...\n");
   return 0;
}

int tsr_create_parse(struct tsr ** tp, char * str)
{
   int ret;
   struct tsr * t;
   double AR[3][3];
   double Ad[3];
   double BR[3][3];
   double Bd[3];
   
   t = (struct tsr *) malloc(sizeof(struct tsr));
   if (!t) return -1;
   
   ret = sscanf(str,
      "%d %31s"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf"
      " %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
      &t->manipindex,
      t->bodyandlink,
      &AR[0][0], &AR[1][0], &AR[2][0],
      &AR[0][1], &AR[1][1], &AR[2][1],
      &AR[0][2], &AR[1][2], &AR[2][2],
      &Ad[0], &Ad[1], &Ad[2],
      &BR[0][0], &BR[1][0], &BR[2][0],
      &BR[0][1], &BR[1][1], &BR[2][1],
      &BR[0][2], &BR[1][2], &BR[2][2],
      &Bd[0], &Bd[1], &Bd[2],
      &t->Bw[0][0], &t->Bw[0][1],
      &t->Bw[1][0], &t->Bw[1][1],
      &t->Bw[2][0], &t->Bw[2][1],
      &t->Bw[3][0], &t->Bw[3][1],
      &t->Bw[4][0], &t->Bw[4][1],
      &t->Bw[5][0], &t->Bw[5][1]);
   if (ret != 38) { free(t); return -2; }
   
   /* Convert from dr to pose */
   cd_kin_pose_from_dR(t->T0w, Ad, AR);
   
   cd_kin_pose_from_dR(t->Twe, Bd, BR);
   
   *tp = t;
   return 0;
}

void tsr_destroy(struct tsr * t)
{
   free(t);
}

} /* namespace orcdchomp */
