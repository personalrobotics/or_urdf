/** \file spatial.h
 * \brief Interface to cd_spatial, a collection of useful routines for
 *        spatial vector algrbra.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#ifndef CD_SPATIAL_H
#define CD_SPATIAL_H

int cd_spatial_xm_to_pose(double xm[6][6], double pose[7]);
int cd_spatial_xf_to_pose(double xf[6][6], double pose[7]);

int cd_spatial_xm_from_pose(double xm[6][6], double pose[7]);
int cd_spatial_xf_from_pose(double xf[6][6], double pose[7]);

/* transform an body inertia from frame b to frame a */
int cd_spatial_inertia_x(double pose_ab[7], double inertia_b[6][6], double inertia_a[6][6]);

/* where would a body be after moving with constant velocity for
 * unit time?
 * aka twists, pose (or H) gives the pose of the body-fixed frame
 * originally located ato the origin */
int cd_spatial_pose_from_spavel_unittime(double pose[7], double spavel[6]);
int cd_spatial_H_from_spavel_unittime(double H[4][4], double spavel[6]);

int cd_spatial_x_invert(double x[6][6]);

int cd_spatial_v_to_pos(double vel[6], double pos[3]);
int cd_spatial_v_from_pos(double vel[6], double pos[3]);
int cd_spatial_f_to_pos(double force[6], double pos[3]);
int cd_spatial_f_from_pos(double force[6], double pos[3]);

/* v = J \dot{r} */
int cd_spatial_pose_jac(double pose[7], double jac[6][7]);
/* \dot{r} = J^{-1} v */
int cd_spatial_pose_jac_inverse(double pose[7], double jac_inverse[7][6]);

int cd_spatial_inertia_from_com(double inertia[6][6], double mass, double com[3], double Icom[3][3]);
int cd_spatial_inertia_to_com(double inertia[6][6], double * massp, double com[3], double Icom[3][3]);
int cd_spatial_inertia_sphere_solid(double pos[3], double mass, double radius, double inertia[6][6]);

int cd_spatial_vxIv(double v[6], double I[6][6], double result[6]);

int cd_spatial_spring_damper(double pose[7], double vel[6],
   double pose_ref[7], double vel_ref[6], double force[6],
   double Klin, double Blin, double Kang, double Bang);

/* Older stuff, used in cd_sdfast_spatial_get_k */
int cd_spatial_mat_bullet(double m[6][10], double v[6]);
int cd_spatial_mat_crossf(double m[6][6], double v[6]);

#endif /* CD_SPATIAL_H */
