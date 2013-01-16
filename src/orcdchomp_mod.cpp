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
#include "libcd/chomp.h"
#include "libcd/grid.h"
#include "libcd/grid_flood.h"
#include "libcd/kin.h"
#include "libcd/mat.h"
#include "libcd/os.h"
#include "libcd/spatial.h"
#include "libcd/util.h"
#include "libcd/util_shparse.h"
}

#include <openrave/openrave.h>
#include <openrave/planningutils.h>

#include "orcdchomp_kdata.h"
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
   int si;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   OpenRAVE::RobotBasePtr r;
   char buf[1024];
   struct orcdchomp::sphereelem * el;
   struct orcdchomp::sphere * s;
   std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
   OpenRAVE::KinBodyPtr k;
   
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
   
   /* view each sphere */
   r->GetGrabbed(vgrabbed);
   /*vgrabbed.push_front(r);*/
   vgrabbed.insert(vgrabbed.begin(), r);
   si = 0;
   for (i=0; i<(int)(vgrabbed.size()); i++)
   {
      k = vgrabbed[i];
      /* get kinbody spheres */
      boost::shared_ptr<orcdchomp::kdata> d = boost::dynamic_pointer_cast<orcdchomp::kdata>(k->GetReadableInterface("orcdchomp"));
      if (!d) { throw OpenRAVE::openrave_exception("kinbody does not have a <orcdchomp> tag defined!"); }
      for (el=d->sphereelems; el; el=el->next)
      {
         s = el->s;
         /* make some sweet spheres! */
         OpenRAVE::KinBodyPtr sbody = OpenRAVE::RaveCreateKinBody(this->e);
         sprintf(buf, "orcdchomp_sphere_%d", si);
         sbody->SetName(buf);
         /* set its dimensions */
         {
            std::vector< OpenRAVE::Vector > svec;
            OpenRAVE::Transform t = k->GetLink(s->linkname)->GetTransform();
            OpenRAVE::Vector v = t * OpenRAVE::Vector(s->pos); /* copies 3 values */
            v.w = s->radius; /* radius */
            svec.push_back(v);
            sbody->InitFromSpheres(svec, true);
         }
         /* add the sphere */
         this->e->AddKinBody(sbody);
         si++;
      }
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
      struct timespec ticks_tic;
      struct timespec ticks_toc;

      /* start timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

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
      
      /* stop timing voxel grid computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("total voxel grid computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

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
      
      /* stop timing flood fill computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("total flood fill computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

      /* start timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

      /* compute the signed distance field (in the module instance) */
      RAVELOG_INFO("computing signed distance field ...\n");
      cd_grid_double_bin_sdf(&sdf_new.grid, g_obs);

      /* stop timing sdf computation */
      clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
      CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
      RAVELOG_INFO("total sdf computation time: %f seconds.\n", CD_OS_TIMESPEC_DOUBLE(&ticks_toc));

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
struct run_rsdf
{
   double pose_world_gsdf[7];
   double pose_gsdf_world[7];
   struct cd_grid * grid;
};

struct run_sphere_active
{
   struct run_sphere_active * next;
   double radius;
   OpenRAVE::KinBody::Link * robot_link;
   int robot_linkindex;
   double pos_wrt_link[3];
};

/* this encodes a run of chomp, which is setup, iterated, and checked. */
struct run
{
   /* the trajectory */
   double * traj;
   int n_points; /* points, including endpoints if any */
   
   /* for interfacing to openrave */
   OpenRAVE::RobotBase * robot;
   int * adofindices;
   
   /* obstacle parameters */
   double epsilon;
   double epsilon_self;
   double obs_factor;
   double obs_factor_self;
   
   /* spheres */
   struct run_sphere_active * spheres_active;
   int n_spheres_active;
   
   /* space for cached sphere info */
   double * sphere_poss_all; /* [trajpoint(external)][sphereid][xyz] */
   double * sphere_poss; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_vels; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_accs; /* [trajpoint(moving)][sphereid][3xn] */
   double * sphere_jacs; /* [trajpoint(moving)][sphereid][3xn] */
   
   /* rooted sdfs */
   int n_rsdfs;
   struct run_rsdf * rsdfs;
   
   /* optional start stuff */
   struct tsr * start_tsr;
   int start_tsr_enabled[6]; /* xyzrpy */
   int start_tsr_k;
   int (*start_cost)(void * ptr, int n, double * point, double * cost, double * grad);
   void * start_cost_ptr;
   
   /* optional every_n constraint */
   struct tsr * everyn_tsr;
   int everyn_tsr_enabled[6]; /* xyzrpy */
   int everyn_tsr_k;
   int (*everyn_cost)(void * ptr, int n, double * point, double * cost, double * grad);
   void * everyn_cost_ptr;
   
   /* ee_force stuff */
   double ee_force[3];
   double ee_force_at[3];
   double * ee_torque_weights;
   int ee_torque_weights_n;
   
   /* hmc/resampling stuff */
   int use_hmc;
   int hmc_resample_iter;
   double hmc_resample_lambda;
   gsl_rng * rng;
   
   /* space for cost function computation */
   double * J; /* space for the jacobian; 3xn */
   double * J2;
   
   /* timing */
   struct timespec ticks_fk;
   struct timespec ticks_jacobians;
   struct timespec ticks_selfcol;
   
   /* logging */
   FILE * fp_dat;
   
   /* the chomp itself */
   struct cd_chomp * c;
   int iter;
};

int sphere_cost_pre(struct run * h, struct cd_chomp * c, int m, double ** T_points)
{
   int ti;
   int ti_mov;
   int i;
   int j;
   int sai;
   struct run_sphere_active * sact;
   boost::multi_array< OpenRAVE::dReal, 2 > orjacobian;
   double * internal;
   
   /* compute positions of all active spheres (including external traj points) */
   for (ti=0; ti<h->n_points; ti++)
   {
      /* put the robot in the config */
      std::vector<OpenRAVE::dReal> vec(&h->traj[ti*c->n], &h->traj[(ti+1)*c->n]);
      TIC()
      h->robot->SetActiveDOFValues(vec);
      TOC(&h->ticks_fk)
      
      for (sact=h->spheres_active,sai=0; sact; sact=sact->next,sai++)
      {
         OpenRAVE::Transform t = sact->robot_link->GetTransform();
         OpenRAVE::Vector v = t * OpenRAVE::Vector(sact->pos_wrt_link); /* copies 3 values */
         /* save sphere positions */
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 0] = v.x;
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 1] = v.y;
         h->sphere_poss_all[ti*(h->n_spheres_active*3) + sai*3 + 2] = v.z;
         /* get the moving point index */
         if (c->m == h->n_points - 2)
            ti_mov = ti-1;
         else
            ti_mov = ti;
         /* compute linear jacobians for all moving points */
         if (ti_mov < 0 || c->m <= ti_mov)
            continue;
         TIC()
         h->robot->CalculateJacobian(sact->robot_linkindex, v, orjacobian);
         TOC(&h->ticks_jacobians)
         /* copy the active columns of orjacobian into our J */
         for (i=0; i<3; i++)
            for (j=0; j<c->n; j++)
               h->sphere_jacs[ti_mov*h->n_spheres_active*3*c->n + sai*3*c->n + i*c->n+j] = orjacobian[i][h->adofindices[j]];
      }
   }
   
   /* compute velocities for all moving points */
   /* start my computing internal velocities */
   internal = h->sphere_vels;
   if (c->m != h->n_points - 2)
      internal += h->n_spheres_active*3;
   cd_mat_memcpy(internal, h->sphere_poss_all + 2*h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_sub(internal, h->sphere_poss_all, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, 1.0/(2.0*c->dt));
   /* next do the start vel */
   if (c->m != h->n_points - 2)
   {
      cd_mat_memcpy(h->sphere_vels, h->sphere_poss_all + h->n_spheres_active*3, 1, h->n_spheres_active*3);
      cd_mat_sub(h->sphere_vels, h->sphere_poss_all, 1, h->n_spheres_active*3);
      cd_mat_scale(h->sphere_vels, 1, h->n_spheres_active*3, 1.0/(c->dt));
   }
   
   /* compute accelerations for all moving points */
   /* start my computing internal accelerations */
   internal = h->sphere_accs;
   if (c->m != h->n_points - 2)
      internal += h->n_spheres_active*3;
   cd_mat_memcpy(internal, h->sphere_poss_all + h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, -2.0);
   cd_mat_add(internal, h->sphere_poss_all, h->n_points-2, h->n_spheres_active*3);
   cd_mat_add(internal, h->sphere_poss_all + 2*h->n_spheres_active*3, h->n_points-2, h->n_spheres_active*3);
   cd_mat_scale(internal, h->n_points-2, h->n_spheres_active*3, 1.0/(c->dt * c->dt));
   /* simply copy 1st accel into the 0th accel for now */
   if (c->m != h->n_points - 2)
      cd_mat_memcpy(h->sphere_accs, h->sphere_accs + h->n_spheres_active*3, 1, h->n_spheres_active*3);
   
   return 0;
}

int sphere_cost(struct run * h, struct cd_chomp * c, int ti, double * c_point, double * c_vel, double * costp, double * c_grad)
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
   struct run_sphere_active * sact;
   struct run_sphere_active * sact2;
   int sdfi_best;
   double sdfi_best_dist;
   int sai;
   int sai2;
   double v_from_other[3];
   
   /* start with a zero cost and config-space gradient */
   cost = 0.0;
   if (c_grad) cd_mat_set_zero(c_grad, c->n, 1);
   
   /* the cost and its gradient are summed over each active sphere on the robot */
   for (sact=h->spheres_active,sai=0; sact; sact=sact->next,sai++)
   {
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
      dist -= sact->radius;
      
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
         cblas_dgemv(CblasRowMajor, CblasTrans, 3, c->n,
            x_vel_norm, h->sphere_jacs + ti*h->n_spheres_active*3*c->n + sai*3*c->n,c->n, x_grad,1, 1.0, c_grad,1);
      }

      /* consider effects from all other spheres (i.e. self collision) */
      TIC()
      for (sact2=h->spheres_active,sai2=0; sact2; sact2=sact2->next,sai2++)
      {
         /* skip spheres on the same link (their Js would be identical anyways) */
         if (sact->robot_linkindex == sact2->robot_linkindex) continue;
         
         /* skip spheres far enough away from us */
         cd_mat_memcpy(v_from_other, h->sphere_poss + ti*(h->n_spheres_active*3) + sai*3, 3, 1);
         cd_mat_sub(v_from_other, h->sphere_poss + ti*(h->n_spheres_active*3) + sai2*3, 3, 1);
         dist = cblas_dnrm2(3, v_from_other,1);
         if (dist > sact->radius + sact2->radius + h->epsilon_self) continue;
         
         if (c_grad)
         {
            /* make unit vector (g_grad) away from other sphere */
            g_grad[0] = v_from_other[0] / dist;
            g_grad[1] = v_from_other[1] / dist;
            g_grad[2] = v_from_other[2] / dist;
         }
         
         dist -= sact->radius + sact2->radius;
         
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
            cd_mat_memcpy(h->J2, h->sphere_jacs + ti*h->n_spheres_active*3*c->n + sai*3*c->n, 3, c->n);
            cd_mat_sub(h->J2, h->sphere_jacs + ti*h->n_spheres_active*3*c->n + sai2*3*c->n, 3, c->n);
            
            /* multiply into c_grad through JT */
            /* I HAVE NO IDEA WHY THERES A -1 HERE! */
            cblas_dgemv(CblasRowMajor, CblasTrans, 3, c->n,
               -1.0, h->J2,c->n, x_grad,1, 1.0, c_grad,1);
         }
      }
      TOC(&h->ticks_selfcol)
      
      cost += cost_sphere;
   }
   
   /* we potentially add the ee_force cost and its gradient ... */
   
   
   
   if (costp) *costp = cost;
   return 0;
}

int con_everyn_tsr(struct run * h, struct cd_chomp * c, int ti, double * point, double * con_val, double * con_jacobian)
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
   std::vector<OpenRAVE::dReal> vec(point, point + c->n);
   h->robot->SetActiveDOFValues(vec);
   
   /* get the end-effector transform */
   OpenRAVE::Transform t = h->robot->GetActiveManipulator()->GetEndEffectorTransform();
   pose_ee[0] = t.trans.x;
   pose_ee[1] = t.trans.y;
   pose_ee[2] = t.trans.z;
   pose_ee[3] = t.rot.y;
   pose_ee[4] = t.rot.z;
   pose_ee[5] = t.rot.w;
   pose_ee[6] = t.rot.x;
   
   /* get the object pose */
   cd_kin_pose_invert(h->everyn_tsr->Twe, pose_ee_obj);
   cd_kin_pose_compose(pose_ee, pose_ee_obj, pose_obj);
   
   /* get the pose of the world w.r.t. the table */
   cd_kin_pose_invert(h->everyn_tsr->T0w, pose_table_world);
   
   /* get the pose of the object w.r.t. the table */
   cd_kin_pose_compose(pose_table_world, pose_obj, pose_table_obj);
   
   /* convert to xyzypr */
   cd_kin_pose_to_xyzypr(pose_table_obj, xyzypr_table_obj);
   
   /* fill the constraint value vector */
   ki=0;
   for (tsri=0; tsri<6; tsri++) if (h->everyn_tsr_enabled[tsri])
   {
      con_val[ki] = xyzypr_table_obj[tsri<3?tsri:8-tsri];
      ki++;
   }
   
   if (con_jacobian)
   {
      /* compute the spatial Jacobian! */
      spajac_world = (double *) malloc(6 * c->n * sizeof(double));
      full_result = (double *) malloc(6 * c->n * sizeof(double));
      
      ee_link_index = h->robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
      
      /* copy the active columns of (rotational) orjacobian into our J */
      h->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[i*c->n+j] = orjacobian[i][h->adofindices[j]];
      
      /* copy the active columns of (translational) orjacobian into our J */
      h->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[(3+i)*c->n+j] = orjacobian[i][h->adofindices[j]];
            
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
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, c->n, 6,
         1.0, *temp6x6b,6, spajac_world,c->n, 0.0, full_result,c->n);
      
      /* fill the constraint Jacbobian matrix */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (h->everyn_tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*c->n, full_result+(tsri<3?tsri:8-tsri)*c->n, 1, c->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}

int con_start_tsr(struct run * h, struct cd_chomp * c, int ti, double * point, double * con_val, double * con_jacobian)
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
   std::vector<OpenRAVE::dReal> vec(point, point + c->n);
   h->robot->SetActiveDOFValues(vec);
   
   /* get the end-effector transform */
   OpenRAVE::Transform t = h->robot->GetActiveManipulator()->GetEndEffectorTransform();
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
      spajac_world = (double *) malloc(6 * c->n * sizeof(double));
      full_result = (double *) malloc(6 * c->n * sizeof(double));
      
      ee_link_index = h->robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
      
      /* copy the active columns of (rotational) orjacobian into our J */
      h->robot->CalculateAngularVelocityJacobian(ee_link_index, orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[i*c->n+j] = orjacobian[i][h->adofindices[j]];
      
      /* copy the active columns of (translational) orjacobian into our J */
      h->robot->CalculateJacobian(ee_link_index, OpenRAVE::Vector(0.0, 0.0, 0.0), orjacobian);
      for (i=0; i<3; i++)
         for (j=0; j<c->n; j++)
            spajac_world[(3+i)*c->n+j] = orjacobian[i][h->adofindices[j]];
            
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
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, c->n, 6,
         1.0, *temp6x6b,6, spajac_world,c->n, 0.0, full_result,c->n);
      
      /* fill the constraint Jacbobian matrix */
      ki=0;
      for (tsri=0; tsri<6; tsri++) if (h->start_tsr_enabled[tsri])
      {
         cd_mat_memcpy(con_jacobian+ki*c->n, full_result+(tsri<3?tsri:8-tsri)*c->n, 1, c->n);
         ki++;
      }
      
      free(spajac_world);
      free(full_result);
   }
   
   return 0;
}

/* implements a cost and gradient on the start point,
 * implemented by the passed start_cost callback function */
int cost_extra_start(struct run * r, struct cd_chomp * c, double * T, double * costp, double * G)
{
   if (!r->start_cost) return 0;
   r->start_cost(r->start_cost_ptr, c->n, T, costp, G);
   return 0;
}

/* runchomp robot Herb2
 * run chomp from the current config to the passed goal config
 * uses the active dofs of the passed robot
 * initialized with a straight-line trajectory
 * */
int mod::create(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int j;
   int err;
   int nscan;
   struct orcdchomp::sphere * s;
   const char * exc = 0;
   
   /* our object(s) */
   struct run * r = 0;
   struct cd_chomp * c = 0;
   
   /* temporary args from the command line */
   int n_adofgoal = 0; /* size read from arguments */
   double * adofgoal = 0;
   unsigned int seed = 0;
   char * dat_filename = 0;
   
   /* loaded into chomp */
   int m;
   int n; /* chomp dof */
   double lambda = 10.0;
   int use_momentum = 0;
   
   /* lock environment; other temporaries */
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv(this->e->GetMutex());
   std::vector< OpenRAVE::dReal > vec_jlimit_lower;
   std::vector< OpenRAVE::dReal > vec_jlimit_upper;
   OpenRAVE::TrajectoryBasePtr starttraj;
   
   r = (struct run *) malloc(sizeof(struct run));
   if (!r) throw OpenRAVE::openrave_exception("no memory!");
   
   /* initialize */
   r->traj = 0;
   r->n_points = 101;
   r->robot = 0;
   r->adofindices = 0;
   r->epsilon = 0.1; /* in meters */
   r->epsilon_self = 0.04; /* in meters */
   r->obs_factor = 200.0;
   r->obs_factor_self = 10.0;
   r->spheres_active = 0;
   r->n_spheres_active = 0;
   r->sphere_poss_all = 0;
   r->sphere_vels = 0;
   r->sphere_accs = 0;
   r->sphere_jacs = 0;
   r->n_rsdfs = 0;
   r->rsdfs = 0;
   r->start_tsr = 0;
   for (i=0; i<6; i++) r->start_tsr_enabled[i] = 0;
   r->start_tsr_k = 0;
   r->start_cost = 0;
   r->start_cost_ptr = 0;
   r->everyn_tsr = 0;
   for (i=0; i<6; i++) r->everyn_tsr_enabled[i] = 0;
   r->everyn_tsr_k = 0;
   r->everyn_cost = 0;
   r->everyn_cost_ptr = 0;
   cd_mat_set_zero(r->ee_force, 3, 1);
   cd_mat_set_zero(r->ee_force_at, 3, 1);
   r->ee_torque_weights = 0;
   r->ee_torque_weights_n = 0;
   r->use_hmc = 0;
   r->hmc_resample_iter = 0;
   r->hmc_resample_lambda = 0.02; /* parameter of exponential distribution over iterations between resamples */
   r->rng = 0;
   r->J = 0;
   r->J2 = 0;
   cd_os_timespec_set_zero(&r->ticks_fk);
   cd_os_timespec_set_zero(&r->ticks_jacobians);
   cd_os_timespec_set_zero(&r->ticks_selfcol);
   r->fp_dat = 0;
   r->c = 0;
   r->iter = 0;
   
   /* parse command line arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"robot")==0 && i+1<argc)
      {
         
         if (r->robot) { exc = "Only one robot can be passed!"; goto error; }
         OpenRAVE::RobotBasePtr rob = this->e->GetRobot(argv[++i]);
         r->robot = rob.get();
         if (!r->robot) { exc = "Could not find robot with that name!"; goto error; }
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
         err = tsr_create_parse(&r->start_tsr, argv[++i]);
         if (err) { exc = "Cannot parse start_tsr TSR!"; goto error; }
      }
      else if (strcmp(argv[i],"everyn_tsr")==0 && i+1<argc)
      {
         err = tsr_create_parse(&r->everyn_tsr, argv[++i]);
         if (err) { exc = "Cannot parse everyn_tsr TSR!"; goto error; }
      }
      else if (strcmp(argv[i],"start_cost")==0 && i+1<argc)
      {
         nscan = sscanf(argv[++i], "%p %p", &r->start_cost, &r->start_cost_ptr);
         if (nscan != 2) { exc = "Cannot parse start_cost callback!"; goto error; }
      }
      else if (strcmp(argv[i],"lambda")==0 && i+1<argc)
         lambda = atof(argv[++i]);
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
         r->n_points = atoi(argv[++i]);
      else if (strcmp(argv[i],"use_momentum")==0)
         use_momentum = 1;
      else if (strcmp(argv[i],"use_hmc")==0)
         r->use_hmc = 1;
      else if (strcmp(argv[i],"hmc_resample_lambda")==0 && i+1<argc)
         r->hmc_resample_lambda = atof(argv[++i]);
      else if (strcmp(argv[i],"seed")==0 && i+1<argc)
         sscanf(argv[++i], "%u", &seed);
      else if (strcmp(argv[i],"epsilon")==0 && i+1<argc)
         r->epsilon = atof(argv[++i]);
      else if (strcmp(argv[i],"epsilon_self")==0 && i+1<argc)
         r->epsilon_self = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor")==0 && i+1<argc)
         r->obs_factor = atof(argv[++i]);
      else if (strcmp(argv[i],"obs_factor_self")==0 && i+1<argc)
         r->obs_factor_self = atof(argv[++i]);
      else if (strcmp(argv[i],"dat_filename")==0 && i+1<argc)
         dat_filename = argv[++i];
      else if (strcmp(argv[i],"ee_force")==0 && i+1<argc)
      {
         int ee_force_argc;
         char ** ee_force_argv;
         cd_util_shparse(argv[++i], &ee_force_argc, &ee_force_argv);
         if (ee_force_argc == 1)
         {
            r->ee_force[0] = 0.0;
            r->ee_force[1] = 0.0;
            r->ee_force[2] = -atof(ee_force_argv[0]);
         }
         else if (ee_force_argc == 3)
         {
            r->ee_force[0] = atof(ee_force_argv[0]);
            r->ee_force[1] = atof(ee_force_argv[1]);
            r->ee_force[2] = atof(ee_force_argv[2]);
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
         r->ee_force_at[0] = atof(ee_force_at_argv[0]);
         r->ee_force_at[1] = atof(ee_force_at_argv[1]);
         r->ee_force_at[2] = atof(ee_force_at_argv[2]);
      }
      else if (strcmp(argv[i],"ee_torque_weights")==0 && i+1<argc)
      {
         char ** my_argv;
         if (r->ee_torque_weights) { exc = "Only one ee_torque_weights can be passed!"; goto error; }
         cd_util_shparse(argv[++i], &r->ee_torque_weights_n, &my_argv);
         r->ee_torque_weights = (double *) malloc(r->ee_torque_weights_n * sizeof(double));
         for (j=0; j<r->ee_torque_weights_n; j++)
            r->ee_torque_weights[j] = atof(my_argv[j]);
         free(my_argv);
         cd_mat_vec_print("parsed ee_torque_weights: ", r->ee_torque_weights, r->ee_torque_weights_n);
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   RAVELOG_INFO("Using robot %s.\n", r->robot->GetName().c_str());
   if (dat_filename) RAVELOG_INFO("Using dat_filename |%s|.\n", dat_filename);
   
   /* check validity of input arguments ... */
   if (!r->robot) { exc = "Did not pass a robot!"; goto error; }
   if (!adofgoal && !starttraj.get()) { exc = "Did not pass either adofgoal or starttraj!"; goto error; }
   if (!this->n_sdfs) { exc = "No signed distance fields have yet been computed!"; goto error; }
   if (lambda < 0.01) { exc = "lambda must be >=0.01!"; goto error; }
   if (r->n_points < 3) { exc = "n_points must be >=3!"; goto error; }
   
   n = r->robot->GetActiveDOF();
   
   /* check that n_adofgoal matches active dof */
   if (adofgoal && (n != n_adofgoal))
   {
      RAVELOG_INFO("n_dof: %d; n_adofgoal: %d\n", n, n_adofgoal);
      exc = "size of adofgoal does not match active dofs!";
      goto error;
   }
   
   /* check that ee_torque_weights_n matches */
   if (r->ee_torque_weights && (r->ee_torque_weights_n != n))
   {
      RAVELOG_INFO("n_dof: %d; ee_torque_weights_n: %d\n", n, r->ee_torque_weights_n);
      exc = "size of ee_torque_weights does not match active dofs!";
      goto error;
   }
   
   /* allocate adofindices */
   r->adofindices = (int *) malloc(n * sizeof(int));
   {
      std::vector<int> vec = r->robot->GetActiveDOFIndices();
      printf("adofindices:");
      for (j=0; j<n; j++)
      {
         r->adofindices[j] = vec[j];
         printf(" %d", r->adofindices[j]);
      }
      printf("\n");
   }
   
   /* ensure that if we're doing the ee_force math, the active dofs are
    * only revolute dofs (since we do our own jacobian math) */
   if (r->ee_torque_weights)
   {
      printf("ee_torque_weights check for revolute only ...\n");
      for (j=0; j<n; j++)
      {
         printf("joint type: %d\n", r->robot->GetJointFromDOFIndex(r->adofindices[j])->GetType());
      }
   }
   
   /* allocate sphere stuff */
   {
      struct run_sphere_active * sact;
      struct sphereelem * sel;
      OpenRAVE::KinBody::Link * link;
      int linkindex;
      std::vector<OpenRAVE::KinBodyPtr> vgrabbed;
      OpenRAVE::KinBodyPtr k;
      boost::shared_ptr<orcdchomp::kdata> d;

      /* consider the robot kinbody, as well as all grabbed bodies */
      r->robot->GetGrabbed(vgrabbed);
      vgrabbed.insert(vgrabbed.begin(), this->e->GetRobot(r->robot->GetName()));
      
      r->n_spheres_active = 0;
      for (i=0; i<(int)(vgrabbed.size()); i++)
      {
         k = vgrabbed[i];
         
         /* get kinbody spheres */
         d = boost::dynamic_pointer_cast<orcdchomp::kdata>(k->GetReadableInterface("orcdchomp"));
         if (!d) { throw OpenRAVE::openrave_exception("kinbody does not have a <orcdchomp> tag defined!"); }
         for (sel=d->sphereelems; sel; sel=sel->next)
         {
            /* what link is this sphere attached to? */
            if (k.get()==r->robot)
               link = r->robot->GetLink(sel->s->linkname).get();
            else
               link = r->robot->IsGrabbing(k).get();
            linkindex = link->GetIndex();
            /* if not affected by active dofs, skip it! */
            for (j=0; j<n; j++)
               if (r->robot->DoesAffect(r->adofindices[j], linkindex))
                  break;
            if (!(j<n))
            {
               if (k.get()==r->robot)
                  continue;
               else
                  break;
            }
            /* ok, this is an active sphere; add it */
            sact = (struct run_sphere_active *) malloc(sizeof(struct run_sphere_active));
            sact->radius = sel->s->radius;
            sact->robot_link = link;
            sact->robot_linkindex = linkindex;
            if (k.get()==r->robot)
            {
               cd_mat_memcpy(sact->pos_wrt_link, sel->s->pos, 3, 1);
            }
            else
            {
               OpenRAVE::Transform T_w_klink = k->GetLink(sel->s->linkname)->GetTransform();
               OpenRAVE::Transform T_w_rlink = link->GetTransform();
               OpenRAVE::Vector v = T_w_rlink.inverse() * T_w_klink * OpenRAVE::Vector(sel->s->pos); /* copies 3 values */
               sact->pos_wrt_link[0] = v.x;
               sact->pos_wrt_link[1] = v.y;
               sact->pos_wrt_link[2] = v.z;
            }
            sact->next = r->spheres_active;
            r->spheres_active = sact;
            r->n_spheres_active++;
         }
      }
      printf("found %d active spheres\n", r->n_spheres_active);
   }
   
   /* allocate rng */
   r->rng = gsl_rng_alloc(gsl_rng_default);
   gsl_rng_set(r->rng, seed);
   
   if (dat_filename)
   {
      r->fp_dat = fopen(dat_filename, "w");
      if (!r->fp_dat) { exc = "could not open dat_filename file for writing!"; goto error; }
   }
   
   /* check that starttraj has the right ConfigurationSpecification? */
   
   /* initialize the cost helper */
   m = r->n_points-2;
   if (r->start_tsr) m++;
   r->J = (double *) malloc(3*n*sizeof(double));
   r->J2 = (double *) malloc(3*n*sizeof(double));
   r->sphere_poss_all = (double *) malloc((r->n_points)*(r->n_spheres_active)*3*sizeof(double));
   if (r->start_tsr)
      r->sphere_poss = r->sphere_poss_all;
   else
      r->sphere_poss = r->sphere_poss_all + (r->n_spheres_active)*3;
   r->sphere_vels = (double *) malloc(m*(r->n_spheres_active)*3*sizeof(double));
   r->sphere_accs = (double *) malloc(m*(r->n_spheres_active)*3*sizeof(double));
   r->sphere_jacs = (double *) malloc(m*(r->n_spheres_active)*(3*n)*sizeof(double));
   
   /* compute the rooted sdfs ... */
   r->n_rsdfs = this->n_sdfs;
   r->rsdfs = (struct run_rsdf *) malloc(this->n_sdfs * sizeof(struct run_rsdf));
   for (i=0; i<this->n_sdfs; i++)
   {
      r->rsdfs[i].grid = this->sdfs[i].grid;
      OpenRAVE::Transform t = this->e->GetKinBody(this->sdfs[i].kinbody_name)->GetTransform();
      r->rsdfs[i].pose_world_gsdf[0] = t.trans.x;
      r->rsdfs[i].pose_world_gsdf[1] = t.trans.y;
      r->rsdfs[i].pose_world_gsdf[2] = t.trans.z;
      r->rsdfs[i].pose_world_gsdf[3] = t.rot.y;
      r->rsdfs[i].pose_world_gsdf[4] = t.rot.z;
      r->rsdfs[i].pose_world_gsdf[5] = t.rot.w;
      r->rsdfs[i].pose_world_gsdf[6] = t.rot.x;
      cd_kin_pose_compose(r->rsdfs[i].pose_world_gsdf, this->sdfs[i].pose, r->rsdfs[i].pose_world_gsdf);
      cd_kin_pose_invert(r->rsdfs[i].pose_world_gsdf, r->rsdfs[i].pose_gsdf_world);
   }
   
   /* create the trajectory */
   r->traj = (double *) malloc((r->n_points)*n*sizeof(double));
   
   /* initialize trajectory */
   if (starttraj.get())
   {
      RAVELOG_INFO("Initializing from a passed trajectory ...\n");
      for (i=0; i<r->n_points; i++)
      {
         std::vector<OpenRAVE::dReal> vec;
         starttraj->Sample(vec, i*starttraj->GetDuration()/((r->n_points)-1), r->robot->GetActiveConfigurationSpecification());
         for (j=0; j<n; j++)
            r->traj[i*n+j] = vec[j];
      }
   }
   else
   {
      std::vector<OpenRAVE::dReal> start;
      RAVELOG_INFO("Initializing from a straight-line trajectory ...\n");
      cd_mat_set_zero(r->traj, (r->n_points), n);
      /* starting point */
      r->robot->GetActiveDOFValues(start);
      for (i=0; i<r->n_points; i++)
         for (j=0; j<n; j++)
            r->traj[i*n+j] = start[j] + (adofgoal[j]-start[j])*i/(r->n_points-1);
   }
      
   /* calculate the dimensionality of the constraint */
   if (r->start_tsr)
   {
      printf("start_tsr");
      r->start_tsr_k = 0;
      for (i=0; i<6; i++)
      {
         if (r->start_tsr->Bw[i][0] == 0.0 && r->start_tsr->Bw[i][1] == 0.0)
         {
            r->start_tsr_enabled[i] = 1;
            r->start_tsr_k++;
         }
         else
            r->start_tsr_enabled[i] = 0;
         printf(" %d", r->start_tsr_enabled[i]);
      }
      printf("\n");
   }
   
   /* calculate the dimensionality of the constraint */
   if (r->everyn_tsr)
   {
      printf("everyn_tsr");
      r->everyn_tsr_k = 0;
      for (i=0; i<6; i++)
      {
         if (r->everyn_tsr->Bw[i][0] == 0.0 && r->everyn_tsr->Bw[i][1] == 0.0)
         {
            r->everyn_tsr_enabled[i] = 1;
            r->everyn_tsr_k++;
         }
         else
            r->everyn_tsr_enabled[i] = 0;
         printf(" %d", r->everyn_tsr_enabled[i]);
      }
      printf("\n");
   }
   
   /* ok, ready to go! create a chomp solver */
   err = cd_chomp_create(&c, m, n, 1, &r->traj[(r->start_tsr?0:1)*n], n);
   if (err) { exc = "error creating chomp instance!"; goto error; }
   
   /* evaluate the constraint on the start and end points */
   if (r->start_tsr)
   {
      double * con_val;
      con_val = (double *) malloc(r->start_tsr_k * sizeof(double));
      printf("evaluating start_tsr constraint on first point ...\n");
      con_start_tsr(r, c, 0, r->traj, con_val, 0);
      cd_mat_vec_print("con_val: ", con_val, r->start_tsr_k);
      free(con_val);
   }
   
   /* evaluate the everyn constraint on the start and end points */
   if (r->everyn_tsr)
   {
      double * con_val;
      con_val = (double *) malloc(r->everyn_tsr_k * sizeof(double));
      printf("evaluating everyn_tsr constraint on middle point m=%d ...\n", m/2);
      con_everyn_tsr(r, c, m/2, &r->traj[(m/2)*n], con_val, 0);
      cd_mat_vec_print("con_val: ", con_val, r->everyn_tsr_k);
      free(con_val);
   }
   
   /* set up trajectory */
   c->dt = 1.0/((r->n_points)-1);
   
   if (r->start_tsr)
   {
      c->inits[0] = 0;
      cd_chomp_add_constraint(c, r->start_tsr_k, 0, r,
         (int (*)(void * cptr, struct cd_chomp *, int i, double * point, double * con_val, double * con_jacobian))con_start_tsr);
   }
   else
      c->inits[0] = &r->traj[0*n];
   
   c->finals[0] = &r->traj[((r->n_points)-1)*n];
   
   if (r->everyn_tsr)
   {
      int con_i;
      for (con_i=0; con_i<m; con_i+=1)
      {
         RAVELOG_INFO("Adding everyn_tsr constraint to traj point i=%d\n", con_i);
         cd_chomp_add_constraint(c, r->everyn_tsr_k, con_i, r,
            (int (*)(void * cptr, struct cd_chomp *, int i, double * point, double * con_val, double * con_jacobian))con_everyn_tsr);
         /*if (con_i == 51) break;*/
      }
   }
   
   /* set up obstacle cost */
   c->cptr = r;
   c->cost_pre = (int (*)(void *, struct cd_chomp *, int, double **))sphere_cost_pre;
   c->cost = (int (*)(void *, struct cd_chomp *, int, double *, double *, double *, double *))sphere_cost;
   
   /* set up extra cost */
   if (r->start_cost)
      c->cost_extra = (int (*)(void *, struct cd_chomp *, double *, double *, double *))cost_extra_start;
   
   /*c->lambda = 1000000.0;*/
   c->lambda = lambda;
   /* this parameter affects how fast things settle;
    * 1.0e1 ~ 90% smooth in ~10 iterations
    * bigger, means much slower convergence */
   
   if (use_momentum)
      c->use_momentum = 1;
   
   if (r->use_hmc)
      r->hmc_resample_iter = 0;
      /*hmc_iters_until_resample = log(cd_util_rand_double(0.0, 1.0)) / hmc_resample_lambda;*/
   
   /* set up joint limits (allocated for all moving points) */
   r->robot->GetDOFLimits(vec_jlimit_lower, vec_jlimit_upper);
   for (j=0; j<n; j++)
   {
      c->jlimit_lower[j] = vec_jlimit_lower[r->adofindices[j]];
      c->jlimit_upper[j] = vec_jlimit_upper[r->adofindices[j]];
   }
   
   free(adofgoal);
   adofgoal = 0;
   
   /* Initialize CHOMP */
   err = cd_chomp_init(c);
   if (err) { exc = "Error initializing chomp instance."; goto error; }
   
   /* save the chomp */
   r->c = c;
   
   /* return */
   {
      char buf[128];
      sprintf(buf, "%p", r);
      sout.write(buf, strlen(buf));
   }
   
error:
   if (exc)
   {
      run_destroy(r);
      throw OpenRAVE::openrave_exception(exc);
   }

   RAVELOG_INFO("create done! returning ...\n");
   return 0;
}

int mod::iterate(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int j;
   int nscan;
   /* input args */
   struct run * r = 0;
   int n_iter = 1;
   double max_time = HUGE_VAL;
   char * trajs_fileformstr = 0;
   /* other */
   struct cd_chomp * c;
   double hmc_alpha;
   double cost_total;
   double cost_obs;
   double cost_smooth;
   char trajs_filename[1024];
   struct timespec ticks;
   struct timespec ticks_tic;
   struct timespec ticks_toc;
   OpenRAVE::TrajectoryBasePtr t;
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;

   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one r can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else if (strcmp(argv[i],"n_iter")==0 && i+1<argc)
         n_iter = atoi(argv[++i]);
      else if (strcmp(argv[i],"max_time")==0 && i+1<argc)
         max_time = atof(argv[++i]);
      else if (strcmp(argv[i],"trajs_fileformstr")==0 && i+1<argc)
         trajs_fileformstr = argv[++i];
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   /* check argument values */
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   if (n_iter < 0) throw OpenRAVE::openrave_exception("n_iter must be >=0!");

   if (trajs_fileformstr) RAVELOG_INFO("Using trajs_fileformstr |%s|.\n", trajs_fileformstr);
   
   /* convenience stuff */
   c = r->c;
   
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* start timing! */
   CD_OS_TIMESPEC_SET_ZERO(&ticks);
   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);

   RAVELOG_INFO("iterating CHOMP ...\n");
   for (r->iter=0; r->iter<n_iter; r->iter++)
   {
      /* resample momentum if using hmc */
      if (r->use_hmc && r->iter == r->hmc_resample_iter)
      {
         hmc_alpha = 100.0 * exp(0.02 * r->iter);
         printf("resampling momentum with alpha = %f ...\n", hmc_alpha);
         
         /* the momentum term is now AG */
         for (i=0; i<c->m; i++)
            for (j=0; j<c->n; j++)
               c->AG[i*c->n+j] = gsl_ran_gaussian(r->rng, 1.0/sqrt(hmc_alpha));
         c->leapfrog_first = 1;
         
         /* set new resampling iter */
         r->hmc_resample_iter += 1 + (int) (- log(gsl_rng_uniform(r->rng)) / r->hmc_resample_lambda);
      }
      
      /* dump the intermediate trajectory before each iteration */
      if (trajs_fileformstr)
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks, &ticks_toc);
         sprintf(trajs_filename, trajs_fileformstr, r->iter);
         t = OpenRAVE::RaveCreateTrajectory(this->e);
         t->Init(r->robot->GetActiveConfigurationSpecification());
         for (i=0; i<r->n_points; i++)
         {
            std::vector<OpenRAVE::dReal> vec(&r->traj[i*c->n], &r->traj[i*c->n + c->n]);
            t->Insert(i, vec);
         }
         std::ofstream f(trajs_filename);
         f << std::setprecision(std::numeric_limits<OpenRAVE::dReal>::digits10+1); /// have to do this or otherwise precision gets lost
         t->serialize(f);
         f.close();
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_tic);
      }

      cd_chomp_iterate(c, 1, &cost_total, &cost_obs, &cost_smooth);
      RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f\n", r->iter, cost_total, cost_obs, cost_smooth);
      
      /* dump stats to data file (note these stats are for trajectory before this iteration) */
      if (r->fp_dat)
      {
         clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ticks_toc);
         CD_OS_TIMESPEC_SUB(&ticks_toc, &ticks_tic);
         CD_OS_TIMESPEC_ADD(&ticks_toc, &ticks);
         fprintf(r->fp_dat, "%d %f %f %f %f\n",
            r->iter, CD_OS_TIMESPEC_DOUBLE(&ticks_toc), cost_total, cost_obs, cost_smooth);
      }
      
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
   RAVELOG_INFO("iter:%2d cost_total:%f cost_obs:%f cost_smooth:%f [FINAL]\n", r->iter, cost_total, cost_obs, cost_smooth);
   
   RAVELOG_INFO("done!\n");
   
   /*printf("Clock time for %d iterations: %.8f\n", iter, cd_os_timespec_double(&ticks_iterations));*/
   RAVELOG_INFO("Time breakdown:\n");
   RAVELOG_INFO("  ticks_vels         %.8f\n", cd_os_timespec_double(&c->ticks_vels));
   RAVELOG_INFO("  ticks_callback_pre %.8f\n", cd_os_timespec_double(&c->ticks_callback_pre));
   RAVELOG_INFO("  ticks_callbacks    %.8f\n", cd_os_timespec_double(&c->ticks_callbacks));
   RAVELOG_INFO("    ticks_fk           %.8f\n", cd_os_timespec_double(&r->ticks_fk));
   RAVELOG_INFO("    ticks_jacobians    %.8f\n", cd_os_timespec_double(&r->ticks_jacobians));
   RAVELOG_INFO("    ticks_selfcol      %.8f\n", cd_os_timespec_double(&r->ticks_selfcol));
   RAVELOG_INFO("  ticks_smoothgrad   %.8f\n", cd_os_timespec_double(&c->ticks_smoothgrad));
   RAVELOG_INFO("  ticks_smoothcost   %.8f\n", cd_os_timespec_double(&c->ticks_smoothcost));
   
   sout << cost_total;

   return 0;
}

int mod::gettraj(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int nscan;
   /* args */
   struct run * r = 0;
   int no_collision_check = 0;
   int no_collision_exception = 0;
   int no_collision_details = 0;
   /* other */
   OpenRAVE::EnvironmentMutex::scoped_lock lockenv;
   OpenRAVE::TrajectoryBasePtr t;
   OpenRAVE::RobotBasePtr boostrobot;
   
   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one r can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else if (strcmp(argv[i],"no_collision_check")==0)
         no_collision_check = 1;
      else if (strcmp(argv[i],"no_collision_exception")==0)
         no_collision_exception = 1;
      else if (strcmp(argv[i],"no_collision_details")==0)
         no_collision_details = 1;
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   boostrobot = this->e->GetRobot(r->robot->GetName());
   
   lockenv = OpenRAVE::EnvironmentMutex::scoped_lock(this->e->GetMutex());
   
   /* create an openrave trajectory from the result, and send to sout */
   t = OpenRAVE::RaveCreateTrajectory(this->e);
   t->Init(r->robot->GetActiveConfigurationSpecification());
   for (i=0; i<r->n_points; i++)
   {
      std::vector<OpenRAVE::dReal> vec(&r->traj[i*r->c->n], &r->traj[(i+1)*r->c->n]);
      t->Insert(i, vec);
   }
   
   RAVELOG_INFO("timing trajectory ...\n");
#if OPENRAVE_VERSION >= OPENRAVE_VERSION_COMBINED(0,7,0)
   /* new openrave added a fmaxaccelmult parameter (number 5) */
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,0.2,0.2,"","");
#else
   OpenRAVE::planningutils::RetimeActiveDOFTrajectory(t,boostrobot,false,0.2,"");
#endif

   if (!no_collision_check)
   {
      RAVELOG_INFO("checking trajectory for collision ...\n");
      int collides = 0;
      double time;
      OpenRAVE::CollisionReportPtr report(new OpenRAVE::CollisionReport());
      for (time=0.0; time<t->GetDuration(); time+=0.01)
      {
         std::vector< OpenRAVE::dReal > point;
         t->Sample(point, time);
         r->robot->SetActiveDOFValues(point);
         if (this->e->CheckCollision(boostrobot,report))
         {
            collides = 1;
            if (!no_collision_details) RAVELOG_ERROR("Collision: %s\n", report->__str__().c_str());
            if (!no_collision_exception) throw OpenRAVE::openrave_exception("Resulting trajectory is in collision!");
         }
      }
      if (collides)
         RAVELOG_ERROR("   trajectory collides!\n");
   }
   
   t->serialize(sout);
   
   return 0;
}

int mod::destroy(int argc, char * argv[], std::ostream& sout)
{
   int i;
   int nscan;
   struct run * r = 0;
   /* parse arguments */
   for (i=1; i<argc; i++)
   {
      if (strcmp(argv[i],"run")==0 && i+1<argc)
      {
         if (r) throw OpenRAVE::openrave_exception("Only one run can be passed!");
         nscan = sscanf(argv[++i], "%p", &r);
         if (nscan != 1) throw OpenRAVE::openrave_exception("Could not parse r!");
      }
      else break;
   }
   if (i<argc)
   {
      for (; i<argc; i++) RAVELOG_ERROR("argument %s not known!\n", argv[i]);
      throw OpenRAVE::openrave_exception("Bad arguments!");
   }
   if (!r) throw OpenRAVE::openrave_exception("you must pass a created run!");
   run_destroy(r);
   return 0;
}

void run_destroy(struct run * r)
{
   free(r->traj);
   free(r->rsdfs);
   free(r->J);
   free(r->J2);
   free(r->sphere_poss_all);
   free(r->sphere_vels);
   free(r->sphere_accs);
   free(r->sphere_jacs);
   free(r->adofindices);
   free(r->spheres_active);
   free(r->ee_torque_weights);
   if (r->fp_dat) fclose(r->fp_dat);
   if (r->rng) gsl_rng_free(r->rng);
   tsr_destroy(r->start_tsr);
   cd_chomp_free(r->c);
   free(r);
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
