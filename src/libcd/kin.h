/** \file kin.h
 * \brief Interface to cd_kin, a collection of useful routines for
 *        kinematics.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#ifndef CD_KIN_H
#define CD_KIN_H


int cd_kin_quat_identity(double quat[4]);
int cd_kin_pose_identity(double pose[7]);

int cd_kin_quat_normalize(double quat[4]);
int cd_kin_pose_normalize(double pose[7]);

int cd_kin_quat_flip_closerto(double quat[4], double target[4]);
int cd_kin_pose_flip_closerto(double pose[7], double target[7]);


/* composition */
/* qab * qbc = qac
 * aka Hamilton product;
 * it's OK if output is either of the inputs */
int cd_kin_quat_compose(double quat_ab[4], double quat_bc[4], double quat_ac[4]);
int cd_kin_pose_compose(double pose_ab[7], double pose_bc[7], double pose_ac[7]);

/* Do the same, but for a 3d pos, not a full pose */
int cd_kin_pose_compos(double pose_ab[7], double pos_bc[3], double pos_ac[3]);

/* this is rotation, vac = q vbc q* */
int cd_kin_quat_compose_vec(double quat_ab[4], double vec_bc[3], double vec_ac[3]);
/* Do the same, but just rotate a 3d vector (vel,acc), not a position vector */
int cd_kin_pose_compose_vec(double pose_ab[7], double vec_bc[3], double vec_ac[3]);

/* note conf equals inverse for unit quaternions */
int cd_kin_quat_invert(double quat_in[4], double quat_out[4]);
int cd_kin_pose_invert(double pose_in[7], double pose_out[7]);


/* conversion to/from rotation matrices */
int cd_kin_quat_to_R(double quat[4], double R[3][3]);
int cd_kin_pose_to_H(double pose[7], double H[4][4], int fill_bottom);
int cd_kin_pose_to_dR(double pose[7], double d[3], double R[3][3]);

int cd_kin_quat_from_R(double quat[4], double R[3][3]);
int cd_kin_pose_from_H(double pose[7], double H[3][4]);
int cd_kin_pose_from_dR(double pose[7], double d[3], double R[3][3]);


/* conversion to/from axis-angle */
int cd_kin_quat_to_axisangle(double quat[4], double axis[3], double *angle);
int cd_kin_quat_from_axisangle(double quat[4], double axis[3], double angle);

/* this is the equivalent of cd_kin_quat_compose_vecs above */
int cd_kin_axisangle_rotate(double axis[3], double angle,
   double pos_in[3], double pos_out[3]);

int cd_kin_axisangle_to_R(double axis[3], double angle, double R[3][3]);


/* conversion to/from yaw-pitch-roll airplane euler angles */
int cd_kin_quat_to_ypr(double quat[4], double ypr[3]);
int cd_kin_pose_to_xyzypr(double pose[7], double xyzypr[6]);

int cd_kin_quat_to_ypr_J(double quat[4], double J[3][4]);
int cd_kin_pose_to_xyzypr_J(double pose[7], double J[6][7]);

int cd_kin_quat_from_ypr(double quat[4], double ypr[3]);
int cd_kin_pose_from_xyzypr(double pose[7], double xyzypr[6]);


/* convert between pose and pos+quat */
int cd_kin_pose_to_pos_quat(double pose[7], double pos[3], double quat[4]);
int cd_kin_pose_from_pos_quat(double pose[7], double pos[3], double quat[4]);


/* get an arbitrary pose from specification */
int cd_kin_pose_from_op(double pose[7], double from[3], double to[3], double * lenp);
int cd_kin_pose_from_op_diff(double pose[7], double from[3], double to_diff[3], double * lenp);

#endif /* CD_KIN_H */
