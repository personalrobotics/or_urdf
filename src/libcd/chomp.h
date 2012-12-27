/** \file chomp.h
 * \brief Interface to cd_chomp, a version of CHOMP (Covariant
 *        Hamiltonian Optimization for Motion Planning), a trajectory
 *        optimizer developed by Nathan Ratliff, Matthew Zucker,
 *        J. Andrew Bagnell, and Siddhartha Srinivasa.
 *        Siddhartha Srinivasa.
 * \author Christopher Dellin
 * \date 2011-02-21 CD: File created.
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

/* requires time.h
 *   (with POSIX.1b Real-time extensions (IEEE Std 1003.1b-1993))
 *   for struct timespec
 *   you can get this in gcc with #define _POSIX_C_SOURCE (199309L) */

/* todo: test this with m=1, m=2, m=0 even? */

struct cd_chomp_con;

/* This is a simple waypoint-based implementation of CHOMP without the H. */
struct cd_chomp
{
   int n; /* dimension of (configuration) space */
   int m; /* number of internal waypoints (excluding endpoints) */
   double lambda; /* -1/lambda is update rate, see chomp paper */
   double dt; /* set by default so traj lasts 1.0 second; used to calc wds in init_KEs_default */
   /* Trajectory */
   double * T; /* mxn; pointer to externally-managed memory */
   int ldt;
   double ** T_points; /* pointers to each point */
   /* Temporary space to store trajectory gradient (wrt external cost) */
   double * G; /* format equivalent to traj above, mxn */
   double ** G_points; /* array of m pointers */
   double * AG; /* mxn, holds Ainv-spread trajectory gradient; akin to M (momentum) */
   double ** AG_points; /* array of m pointers */
   /* Smoothness metrics */
   int D; /* D from chomp, the number of finite differencing matrices K used */
   double * wds; /* K weights (length D) */
   double ** inits; /* the first of these point into T_ext, the rest are malloc'ed */
   double ** finals;
   double * initsfinals; /* alloc'ed arrays used by default */
   /* Smoothness metric inverse (recompute with cd_chomp_init_recalc()) */
   double * A; /* mxm */
   double * Ainv; /* mxm */
   double * B; /* mxn */
   double trC; /* C is nxn */
   /* joint limits */
   double * jlimit_lower;
   double * jlimit_upper;
   /* Temporaries used for obs cost and its gradient */
   double * Kvels;
   double * Evels;
   double * vels;
   /* Temporaries used to calculate the trajectory cost */
   double * cost_nxn;
   double * cost_mxn;
   /* temporaries for joint limits */
   double * Gjlimit;
   double * GjlimitAinv;
   /* External cost function (e.g. obstacles) */
   void * cptr;
   int (*cost_pre)(void * cptr, struct cd_chomp * c, int m, double ** T_points); /* internal points only */
   int (*cost)(void * cptr, struct cd_chomp * c, int ti, double * point, double * vel, double * costp, double * grad);
   /* for some reason, this should set the cost, and increment the gradient! */
   int (*cost_extra)(void * cptr, struct cd_chomp * c, double * T, double * costp, double * G);
   /* Momentum (optional) */
   int use_momentum;
   int leapfrog_first;
   /* Hard constraint (from goal set paper) */
   struct cd_chomp_con * cons;
   /*   the following are allocated on init() */
   int cons_k;
   double * cons_h;
   double * cons_Jcol; /* stacked parts of J specific to each point; k_cons x n */
   double * cons_JAJT;
   int * cons_ipiv;
   double * cons_delta;
   /* for timing */
   struct timespec ticks_vels;
   struct timespec ticks_callback_pre;
   struct timespec ticks_callbacks;
   struct timespec ticks_smoothgrad;
   struct timespec ticks_smoothcost;
};

/* Step 1: create a problem of the correct dimension
 * set D=0 if you want to set up your own A, B, and trC;
 * otherwise, init() will calculate default KEs itself given your weights */
int cd_chomp_create(struct cd_chomp ** cp, int m, int n, int D, double * T, int ldt);
void cd_chomp_free(struct cd_chomp * c);

/* After create, but before init, you should do the following:
 *  - set up inits and finals; you could, for example, point them
 *    above and below the trajectory array T passed to create()
 *    (note that setting them to 0 means leave them free!)
 *  - seed the trajectory (if you have not already done so)
 *  - set dt (if you want something other than the default)
 *  - alloc your own Ks and Es if you dont like the defaults
 *  - add constraints (as below) */

struct cd_chomp_con
{
   struct cd_chomp_con * next;
   int k;
   int i;
   void * cptr;
   int (*con_eval)(void * cptr, struct cd_chomp * c, int i, double * point, double * con_val, double * con_jacobian);
   /* pointers to cd_chomp's allocations */
   double * h;
   double * J;
};

int cd_chomp_add_constraint(struct cd_chomp * c, int k, int i, void * cptr,
   int (*con_eval)(void * cptr, struct cd_chomp * c, int i, double * point, double * con_val, double * con_jacobian));

/* this calls the two below in succession */
/* for KEs, this takes into account the value of dt (and m) and inits and finals,
 * and sets the KE matrices to their default average-squared-derivatives form */
/* this calculates A (from Ks and Es) and Ainv and constraint stuff
 * this also calculates K0 and E0 for calculating velocities, from dt */
int cd_chomp_init(struct cd_chomp * c);

int cd_chomp_iterate(struct cd_chomp * c, int do_iteration, double * costp_total, double * costp_obs, double * costp_smooth);
