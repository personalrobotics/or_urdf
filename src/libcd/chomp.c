/** \file chomp.c
 * \brief Implementation of cd_chomp, a version of CHOMP (Covariant
 *        Hamiltonian Optimization for Motion Planning), a trajectory
 *        optimizer developed by Nathan Ratliff, Matthew Zucker,
 *        J. Andrew Bagnell, and Siddhartha Srinivasa.
 * \author Christopher Dellin
 * \date 2011-02-21 CD: File created.
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h> /* requires POSIX.1b struct timespec */
#include <cblas.h>
#include <lapacke.h>
#include "mat.h"
#include "os.h"
#include "chomp.h"

int cd_chomp_create(struct cd_chomp ** cp, int m, int n, int D, double * T, int ldt)
{
   struct cd_chomp * c;
   int i;
   
   /* Allocate structure */
   c = (struct cd_chomp *) malloc(sizeof(struct cd_chomp));
   if (!c) return -1;
   c->n = n;
   c->m = m;
   c->lambda = 1.0;
   c->dt = 1.0/(m+1);
   c->T = T;
   c->ldt = ldt;
   c->T_points = 0;
   c->G = 0;
   c->G_points = 0;
   c->AG = 0;
   c->AG_points = 0;
   c->D = D;
   c->wds = 0;
   c->inits = 0;
   c->finals = 0;
   c->A = 0;
   c->Ainv = 0;
   c->B = 0;
   c->jlimit_lower = 0;
   c->jlimit_upper = 0;
   c->cost_nxn = 0;
   c->cost_mxn = 0;
   c->Kvels = 0;
   c->Evels = 0;
   c->vels = 0;
   c->Gjlimit = 0;
   c->GjlimitAinv = 0;
   c->cptr = 0;
   c->cost_pre = 0;
   c->cost = 0;
   c->cost_extra = 0;
   c->use_momentum = 0;
   c->leapfrog_first = 1;
   cd_os_timespec_set_zero(&c->ticks_vels);
   cd_os_timespec_set_zero(&c->ticks_callback_pre);
   cd_os_timespec_set_zero(&c->ticks_callbacks);
   cd_os_timespec_set_zero(&c->ticks_smoothgrad);
   cd_os_timespec_set_zero(&c->ticks_smoothcost);
   /* constraint stuff */
   c->cons = 0;
   c->cons_k = 0;
   c->cons_h = 0;
   c->cons_Jcol = 0;
   c->cons_JAJT = 0;
   c->cons_ipiv = 0;
   c->cons_delta = 0;
   
   /* Allocate T_points pointers */
   c->T_points = (double **) malloc(m * sizeof(double *));
   if (!c->T_points) { cd_chomp_free(c); return -1; }
   for (i=0; i<m; i++)
      c->T_points[i] = &c->T[i*ldt];
   
   /* Allocate gradient */
   c->G = (double *) malloc(m * n * sizeof(double));
   c->G_points = (double **) malloc(m * sizeof(double *));
   if (!c->G || !c->G_points) { cd_chomp_free(c); return -1; }
   for (i=0; i<m; i++)
      c->G_points[i] = &c->G[i*n];
   
   /* Allocate Ainv-spread gradient (also momentum) */
   c->AG = (double *) malloc(m * n * sizeof(double));
   c->AG_points = (double **) malloc(m * sizeof(double *));
   if (!c->AG || !c->AG_points) { cd_chomp_free(c); return -1; }
   for (i=0; i<m; i++)
      c->AG_points[i] = &c->AG[i*n];
   for (i=0; i<m*n; i++)
      c->AG[i] = 0.0;
   
   /* wds SHOULD BE CALCULATED IN init! */
   
   /* Allocate wds */
   if (D)
   {
      c->wds = (double *) malloc(D*sizeof(double));
      if (!c->wds) { cd_chomp_free(c); return -1; }
      /* The resulting K[d]*T+E[d] yields a vector of d-derivatives;
       * when we dot-product them (i.e. V^T V) we'll get sum-squared-derivatives;
       * set wd to be 1/number of points in order to get average deriv value */
      for (i=0; i<D; i++)
         c->wds[i] = (i<D-1) ? 0.0 : 1.0;
   }
   
   /* Allocate inits, finals; defaults to 0.0 everywhere */
   c->initsfinals = (double *) malloc((2*D)*n*sizeof(double *));
   c->inits = (double **) malloc(D * sizeof(double *));
   c->finals = (double **) malloc(D * sizeof(double *));
   if (!c->initsfinals || !c->inits || !c->finals) { cd_chomp_free(c); return -1; }
   cd_mat_set_zero(c->initsfinals, 2*D, n);
   for (i=0; i<D; i++)
   {
      c->inits[i] = &c->initsfinals[(2*i)*n];
      c->finals[i] = &c->initsfinals[(2*i+1)*n];
   }
   
   /* Allocate A and Ainv */
   c->A = (double *) malloc(m * m * sizeof(double));
   c->Ainv = (double *) malloc(m * m * sizeof(double));
   c->B = (double *) malloc(m * n * sizeof(double));
   c->cost_nxn = (double *) malloc(n * n * sizeof(double));
   c->cost_mxn = (double *) malloc(m * n * sizeof(double));
   c->vels = (double *) malloc(m * n * sizeof(double));
   if (!c->A || !c->Ainv || !c->B || !c->cost_nxn || !c->cost_mxn || !c->vels)
      { cd_chomp_free(c); return -1; }
   /* A, B, and trC start at 0.0;
    * before init, the user can fill them with whatever they want */
   cd_mat_set_zero(c->A, m, m);
   cd_mat_set_zero(c->B, m, n);
   c->trC = 0.0;
   
   /* allocate joint limit stuff */
   c->jlimit_lower = (double *) malloc(n * sizeof(double));
   c->jlimit_upper = (double *) malloc(n * sizeof(double));
   c->Gjlimit = (double *) malloc(m * n * sizeof(double));
   c->GjlimitAinv = (double *) malloc(m * n * sizeof(double));
   if (!c->jlimit_lower || !c->jlimit_upper || !c->Gjlimit || !c->GjlimitAinv)
      { cd_chomp_free(c); return -1; }
   for (i=0; i<n; i++)
   {
      c->jlimit_lower[i] = -HUGE_VAL;
      c->jlimit_upper[i] =  HUGE_VAL;
   }
   
   /* Allocate Kvels and Evels */
   c->Kvels = (double *) malloc(m * m * sizeof(double));
   c->Evels = (double *) malloc(m * n * sizeof(double));
   if (!c->Kvels || !c->Evels) { cd_chomp_free(c); return -1; }
   
   *cp = c;
   return 0;
}

void cd_chomp_free(struct cd_chomp * c)
{
   struct cd_chomp_con * con;
   if (!c) return;
   free(c->cons_h);
   free(c->cons_Jcol);
   free(c->cons_JAJT);
   free(c->cons_ipiv);
   free(c->cons_delta);
   while (c->cons)
   {
      con = c->cons;
      c->cons = c->cons->next;
      free(con);
   }
   free(c->wds);
   free(c->initsfinals);
   free(c->inits);
   free(c->finals);
   free(c->A);
   free(c->Ainv);
   free(c->B);
   free(c->jlimit_lower);
   free(c->jlimit_upper);
   free(c->Gjlimit);
   free(c->GjlimitAinv);
   free(c->cost_nxn);
   free(c->cost_mxn);
   free(c->Kvels);
   free(c->Evels);
   free(c->vels);
   free(c->G);
   free(c->G_points);
   free(c->AG);
   free(c->AG_points);
   free(c->T_points);
   free(c);
}

int cd_chomp_add_constraint(struct cd_chomp * c, int k, int i, void * cptr,
   int (*con_eval)(void * cptr, struct cd_chomp * c, int i, double * point, double * con_val, double * con_jacobian))
{
   struct cd_chomp_con * con;
   con = (struct cd_chomp_con *) malloc(sizeof(struct cd_chomp_con));
   if (!con) return -1;
   con->k = k;
   con->i = i;
   con->cptr = cptr;
   con->con_eval = con_eval;
   con->h = 0;
   con->J = 0;
   con->next = c->cons;
   c->cons = con;
   return 0;
}

/* this adds the default KEs for this chomp to A, B, and trC;
 * this also calculates D0 and E0 even if D is 0,
 * because we need it to compute velocities later */
int cd_chomp_add_KEs(struct cd_chomp * c)
{
   int ret;
   int i;
   int d;
   int * num_derivs_alloc;
   int * num_derivs;
   double ** Ks;
   double ** Es;
   double * diff; /* Temp suitable-sized finite-differencing matrix */
   
   ret = 0;
   
   /*c->wds[i] = 1.0 / (c->m+1+i)*/
   
   /* Allocate space for Ks, Es */
   num_derivs_alloc = (int *) malloc((c->D+1) * sizeof(int));
   Ks = (double **) malloc(c->D * sizeof(double *));
   Es = (double **) malloc(c->D * sizeof(double *));
   if (!num_derivs_alloc || !Ks || !Es) { free(num_derivs_alloc); free(Ks); free(Es); return -1; }
   num_derivs_alloc[0] = c->m;
   num_derivs = &num_derivs_alloc[1];
   for (d=0; d<c->D; d++) { Ks[d] = 0; Es[d] = 0; }
   
   /* Allocate and set each K, E */
   for (d=0; d<c->D; d++)
   {
      num_derivs[d] = num_derivs[d-1] - 1 + (c->inits[d]?1:0) + (c->finals[d]?1:0);
      Ks[d] = (double *) malloc(num_derivs[d] * c->m * sizeof(double));
      Es[d] = (double *) malloc(num_derivs[d] * c->n * sizeof(double));
      if (!Ks[d] || !Es[d]) { ret = -1; goto error; }
      
      /* Create suitable finite-differencing matrix */
      diff = (double *) malloc(num_derivs[d] * num_derivs[d-1] * sizeof(double));
      if (!diff) { ret = -1; goto error; }
      cd_mat_set_zero(diff, num_derivs[d], num_derivs[d-1]);
      cd_mat_set_zero(Es[d], num_derivs[d], c->n);
      
      /* potential first row */
      if (c->inits[d])
      {
         diff[0*num_derivs[d-1]+0] = 1.0/c->dt;
         cblas_daxpy(c->n, -1.0/c->dt, c->inits[d],1,  &Es[d][0],1);
      }
      
      /* fill each row of the middle */
      for (i=0; i<num_derivs[d-1]-1; i++)
      {
         diff[((c->inits[d]?1:0)+i)*num_derivs[d-1]+i]   = -1.0/c->dt;
         diff[((c->inits[d]?1:0)+i)*num_derivs[d-1]+i+1] =  1.0/c->dt;
      }
      
      /* potential last row */
      if (c->finals[d])
      {
         diff[(num_derivs[d]-1)*num_derivs[d-1]+(num_derivs[d-1]-1)] = -1.0/c->dt;
         cblas_daxpy(c->n,  1.0/c->dt, c->finals[d],1, &Es[d][(num_derivs[d]-1)*(c->n)],1);
      }
      
      /* Set K */
      if (d==0)
         cd_mat_memcpy(Ks[d], diff, num_derivs[d], num_derivs[d-1]);
      else
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, num_derivs[d], c->m, num_derivs[d-1],
            1.0, diff,num_derivs[d-1], Ks[d-1],c->m, 0.0,Ks[d],c->m);
      
      /* Add in from prior Es */
      if (d > 0)
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, num_derivs[d], c->n, num_derivs[d-1],
            1.0, diff,num_derivs[d-1], Es[d-1],c->n, 1.0,Es[d],c->n);
      
      free(diff);
   }
   
   /* Calculate A as weighted sum of K^T K's */
   cd_mat_set_zero(c->A, c->m, c->m);
   for (d=0; d<c->D; d++)
      cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, c->m, c->m, num_derivs[d],
         c->wds[d]/num_derivs[d], Ks[d],c->m, Ks[d],c->m, 1.0,c->A,c->m);
   
   /* Calculate B = weighed K^T E */
   cd_mat_set_zero(c->B, c->m, c->n);
   for (d=0; d<c->D; d++)
      cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, c->m, c->n, num_derivs[d],
         c->wds[d]/num_derivs[d], Ks[d],c->m, Es[d],c->n, 1.0,c->B,c->n);
   
   /* Calculate trC = 0.5 * weighted tr(E^T E) */
   cd_mat_set_zero(c->cost_nxn, c->n, c->n);
   for (d=0; d<c->D; d++)
      cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, c->n, c->n, num_derivs[d],
         c->wds[d]/num_derivs[d], Es[d],c->n, Es[d],c->n, 1.0,c->cost_nxn,c->n);
   c->trC = 0.5 * cd_mat_trace(c->cost_nxn, c->n, c->n);
   
error:
   /* note that we're not freeing K0 or E0 */
   for (d=0; d<c->D; d++) free(Ks[d]);
   for (d=0; d<c->D; d++) free(Es[d]);
   free(Ks);
   free(Es);
   free(num_derivs_alloc);
   return ret;
}

int cd_chomp_init(struct cd_chomp * c)
{
   int i;
   int err;
   struct cd_chomp_con * con;
   
   /* compute Kvels and Evels (note that inits[0] and finals[0] might be 0! */
   cd_mat_set_zero(c->Kvels, c->m, c->m);
   cd_mat_set_zero(c->Evels, c->m, c->n);
   for (i=0; i<c->m; i++)
   {
      if (i==0)
      {
         if (c->inits[0])
         {
            c->Kvels[0*c->m+1] = 0.5 / c->dt;
            cd_mat_memcpy(&c->Evels[0*c->n], c->inits[0], c->n, 1);
            cd_mat_scale(&c->Evels[0*c->n], c->n, 1, -0.5/c->dt);
         }
         else
         {
            c->Kvels[0*c->m+1] = 1.0 / c->dt;
            c->Kvels[0*c->m+0] = -1.0 / c->dt;
         }
      }
      else if (i < c->m-1)
      {
         c->Kvels[i*c->m+i+1] = 0.5 / c->dt;
         c->Kvels[i*c->m+i-1] = -0.5 / c->dt;
      }
      else
      {
         if (c->finals[0])
         {
            cd_mat_memcpy(&c->Evels[i*c->n], c->finals[0], c->n, 1);
            cd_mat_scale(&c->Evels[i*c->n], c->n, 1, 0.5/c->dt);
            c->Kvels[i*c->m+i-1] = -0.5 / c->dt;
         }
         else
         {
            c->Kvels[i*c->m+i-0] = 1.0 / c->dt;
            c->Kvels[i*c->m+i-1] = -1.0 / c->dt;
         }
      }
   }
   
   /* if D is nonzero, add into A the default KEs using inits, weights, dt, etc */
   err = cd_chomp_add_KEs(c);
   if (err) return -1;
   
   /* Calculate A^-1 from A */
   {
      int * ipiv;
      ipiv = (int *) malloc(c->m * sizeof(int));
      if (!ipiv) return -1;
      memcpy(c->Ainv, c->A, c->m * c->m * sizeof(double));
      err = LAPACKE_dgetrf(LAPACK_ROW_MAJOR, c->m, c->m, c->Ainv, c->m, ipiv);
      if (err) { free(ipiv); return -2; }
      err = LAPACKE_dgetri(LAPACK_ROW_MAJOR, c->m, c->Ainv, c->m, ipiv);
      if (err) { free(ipiv); return -2; }
      free(ipiv);
   }
   
   /* allocate constraint stuff from c->cons list */
   c->cons_k = 0;
   for (con=c->cons; con; con=con->next)
      c->cons_k += con->k;
   if (c->cons_k)
   {
      printf("cd_chomp_init: total per-point constraint dimensionality: %d\n", c->cons_k);
      c->cons_h = (double *) malloc(c->cons_k * sizeof(double));
      c->cons_Jcol = (double *) malloc(c->cons_k * c->n * sizeof(double));
      c->cons_JAJT = (double *) malloc(c->cons_k * c->cons_k * sizeof(double));
      c->cons_ipiv = (int *) malloc(c->cons_k * sizeof(int));
      c->cons_delta = (double *) malloc(c->n * sizeof(double));
      if (!c->cons_h || !c->cons_Jcol || !c->cons_JAJT || !c->cons_ipiv || !c->cons_delta) return -1;
      c->cons_k = 0;
      for (con=c->cons; con; con=con->next)
      {
         con->h = c->cons_h + c->cons_k;
         con->J = c->cons_Jcol + (c->cons_k * c->n);
         c->cons_k += con->k;
      }
   }
   
   return 0;
}

int cd_chomp_iterate(struct cd_chomp * c, int do_iteration, double * costp_total, double * costp_obs, double * costp_smooth)
{
   int i;
   int j;
   int err;
   double cost_point;
   double cost_obs = 0.0;
   double cost_smooth = 0.0;
   struct timespec tic;
   struct timespec toc;
   int num_limadjs;
   
   /* Compute average velocities at each point */
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   cd_mat_memcpy(c->vels, c->Evels, c->m, c->n);
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
      1.0, c->Kvels,c->m, c->T,c->ldt, 1.0,c->vels,c->n);
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   cd_os_timespec_sub(&toc, &tic);
   cd_os_timespec_add(&c->ticks_vels, &toc);
   
   /* get obstacle costs and/or gradients from user's callback */
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   if (c->cost_pre)
      c->cost_pre(c->cptr, c, c->m, c->T_points);
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   cd_os_timespec_sub(&toc, &tic);
   cd_os_timespec_add(&c->ticks_callback_pre, &toc);
   
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
   if (do_iteration) cd_mat_set_zero(c->G, c->m, c->n);
   if (c->cost) for (i=0; i<c->m; i++)
   {
      /* cost callback */
      /* compute obstacle costs (weighted by body-point Cartesian velocities) */
      /* compute obstacle cost gradients for each internal trajectory point */
      c->cost(c->cptr, c, i, c->T_points[i], &c->vels[i*(c->n)],
         ((costp_total || costp_obs) ? &cost_point : 0),
         (do_iteration ? c->G_points[i] : 0));
      /* accumulate obstacle cost */
      if (costp_total || costp_obs)
         cost_obs += cost_point;
   }
   
   /* divide by number of points (so you're getting average obstacle cost,
    * w.r.t. workspace arc length */
   if (costp_total || costp_obs)
      cost_obs /= c->m;
   cd_mat_scale(c->G, c->m, c->n, 1.0/c->m);
   
   if (c->cost_extra)
   {
      c->cost_extra(c->cptr, c, c->T,
         ((costp_total || costp_obs) ? &cost_point : 0),
         (do_iteration ? c->G : 0));
      cost_obs += cost_point;
   }
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
   cd_os_timespec_sub(&toc, &tic);
   cd_os_timespec_add(&c->ticks_callbacks, &toc);
   
   /* do chomp iteration itself */
   if (do_iteration)
   {
      /* Add on prior (smoothness) gradient */
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
         1.0, c->A,c->m, c->T,c->ldt, 1.0,c->G,c->n);
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      cd_os_timespec_sub(&toc, &tic);
      cd_os_timespec_add(&c->ticks_smoothgrad, &toc);
      cd_mat_add(c->G, c->B, c->m, c->n);
      
      /* COMP */
      if (!c->use_momentum)
      {
         /* Apply this trajectory gradient through Ainv to the trajectory */
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
            1.0, c->Ainv,c->m, c->G,c->n, 0.0,c->AG,c->n);
      }
      else
      {
         /* Hamiltonian simulation (with velocity) */
         
         /* Apply this trajectory gradient through Ainv to the momentum */
         if (c->leapfrog_first)
         {
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
               0.5/c->lambda, c->Ainv,c->m, c->G,c->n, 1.0,c->AG,c->n);
            c->leapfrog_first = 0;
         }
         else
         {
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
               1.0/c->lambda, c->Ainv,c->m, c->G,c->n, 1.0,c->AG,c->n);
         }
      }
      
      /* do the constraint math */
      if (c->cons_k)
      {
         struct cd_chomp_con * con1;
         struct cd_chomp_con * con2;
         
         /* evaluate each point constraint into h and Jcol */
         for (con1=c->cons; con1; con1=con1->next)
            con1->con_eval(con1->cptr, c, con1->i, c->T_points[con1->i], con1->h, con1->J);
         
         /* add into cons_h the gradient component C AG (scaled by -1.0/c->lambda) */
         for (con1=c->cons; con1; con1=con1->next)
            cblas_dgemv(CblasRowMajor, CblasNoTrans, con1->k, c->n,
               -1.0/c->lambda, con1->J,c->n, c->AG_points[con1->i],1, 1.0,con1->h,1);
         
         /* build c->cons_JAJT */
         for (con1=c->cons; con1; con1=con1->next)
         for (con2=c->cons; con2; con2=con2->next)
         {
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, con1->k, con2->k, c->n,
               c->Ainv[con1->i * c->m + con2->i],
               con1->J, c->n, con2->J, c->n,
               0.0, &c->cons_JAJT[((con1->h)-(c->cons_h))*c->cons_k + ((con2->h)-(c->cons_h))], c->cons_k);
         }
         
         /* linear algebra solve, using lapack dgesv driver routine
          * (double general Ax=b solver using PLU factorization) */
         err = LAPACKE_dgesv(LAPACK_ROW_MAJOR, c->cons_k, 1,
                             c->cons_JAJT, c->cons_k, c->cons_ipiv,
                             c->cons_h, 1);
         if (err)
         {
#if 0
            int ii;
            for (ii=0; ii<c->cons_k; ii++)
               cd_mat_vec_print("c->cons_JAJT: ", &c->cons_JAJT[ii*c->cons_k], c->cons_k);
#endif
            printf("constraint inversion error!\n");
         }
         
         /* push the updates back through Ainv to the trajectory! */
         for (con1=c->cons; con1; con1=con1->next)
         {
            cblas_dgemv(CblasRowMajor, CblasTrans, con1->k, c->n,
               1.0, con1->J,c->n, con1->h,1, 0.0,c->cons_delta,1);
            cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, 1,
               -1.0, &c->Ainv[con1->i],c->m, c->cons_delta,c->n, 1.0,c->T,c->ldt);
         }
      }
      
      /* Add AG into the trajectory (from non-constraint part) */
      for (i=0; i<c->m; i++)
         cblas_daxpy(c->n, -1.0/c->lambda, &c->AG[i*c->n],1, &c->T[i*c->ldt],1);
      
      /* handle joint limit violations */
      for (num_limadjs=0; num_limadjs<1000; num_limadjs++)
      {
         double largest_violation;
         size_t largest_idx;
         
         /* find largest violation, and build Gjlimit matrix */
         largest_violation = 0.0;
         largest_idx = 0;
         cd_mat_set_zero(c->Gjlimit, c->m, c->n);
         for (i=0; i<c->m; i++)
         for (j=0; j<c->n; j++)
         {
            if (c->T_points[i][j] < c->jlimit_lower[j])
            {
               c->Gjlimit[i*c->n+j] = c->jlimit_lower[j] - c->T_points[i][j];
               if (fabs(c->Gjlimit[i*c->n+j]) > largest_violation)
               {
                  largest_violation = fabs(c->Gjlimit[i*c->n+j]);
                  largest_idx = i*c->n+j;
               }
            }
            if (c->T_points[i][j] > c->jlimit_upper[j])
            {
               c->Gjlimit[i*c->n+j] = c->jlimit_upper[j] - c->T_points[i][j];
               if (fabs(c->Gjlimit[i*c->n+j]) > largest_violation)
               {
                  largest_violation = fabs(c->Gjlimit[i*c->n+j]);
                  largest_idx = i*c->n+j;
               }
            }
         }
         if (largest_violation == 0.0) break;
         
         /* pre-multiply by Ainv */
         /*printf("num_limadjs: %d\n", num_limadjs);*/
         cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
            1.0, c->Ainv,c->m, c->Gjlimit,c->n, 0.0, c->GjlimitAinv,c->n);
            
         /* compute scalar necessary to make trajectory satisfy limit at largest_idx */
         cblas_daxpy(c->m * c->n,
                     1.01 * c->Gjlimit[largest_idx] / c->GjlimitAinv[largest_idx],
                     c->GjlimitAinv,1, c->T,1);
      }
      if (!(num_limadjs<1000))
      {
         printf("ran too many joint limit fixes! aborting ...\n");
         abort();
      }
   }
   
   /* compute smoothness cost (if we're asked to);
    * uses the new updated trajectory T */
   if (costp_total || costp_smooth)
   {
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tic);
      cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, c->m, c->n, c->m,
         1.0, c->A,c->m, c->T,c->ldt, 0.0,c->cost_mxn,c->n);
      cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, c->n, c->n, c->m,
         0.5, c->T,c->ldt, c->cost_mxn,c->n, 0.0,c->cost_nxn,c->n);
      cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, c->n, c->n, c->m,
         1.0, c->B,c->n, c->T,c->ldt, 1.0,c->cost_nxn,c->n);
      cost_smooth = cd_mat_trace(c->cost_nxn, c->n, c->n) + c->trC;
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &toc);
      cd_os_timespec_sub(&toc, &tic);
      cd_os_timespec_add(&c->ticks_smoothcost, &toc);
   }
   
   if (costp_total) *costp_total = cost_obs + cost_smooth;
   if (costp_obs) *costp_obs = cost_obs;
   if (costp_smooth) *costp_smooth = cost_smooth;
   return 0;
}
