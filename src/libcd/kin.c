/** \file kin.c
 * \brief Implementation of cd_kin, a collection of useful routines for
 *        kinematics.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#include <math.h>
#include <cblas.h>
#include "mat.h"
#include "kin.h"

#define TAU (6.2831853071795864769252867665590057683943387987502116)

int cd_kin_quat_identity(double quat[4])
{
   quat[0] = 0.0;
   quat[1] = 0.0;
   quat[2] = 0.0;
   quat[3] = 1.0;
   return 0;
}

int cd_kin_pose_identity(double pose[7])
{
   pose[0] = 0.0;
   pose[1] = 0.0;
   pose[2] = 0.0;
   pose[3] = 0.0;
   pose[4] = 0.0;
   pose[5] = 0.0;
   pose[6] = 1.0;
   return 0;
}

/* Note - should we check for len = 0.0? */
int cd_kin_quat_normalize(double quat[4])
{
   double len;
   len = cblas_dnrm2(4, quat, 1);
   cblas_dscal(4, 1.0/len, quat, 1);
   return 0;
}

/* Note - should we check for len = 0.0? */
int cd_kin_pose_normalize(double pose[7])
{
   double len;
   len = cblas_dnrm2(4, pose+3, 1);
   cblas_dscal(4, 1.0/len, pose+3, 1);
   return 0;
}

int cd_kin_quat_flip_closerto(double quat[4], double target[4])
{
   int i;
   double orig_diff;
   double flipped_diff;
   double orig_sumsq = 0.0;
   double flipped_sumsq = 0.0;
   for (i=0; i<4; i++)
   {
      orig_diff = quat[i] - target[i];
      flipped_diff = - quat[i] - target[i];
      orig_sumsq += orig_diff * orig_diff;
      flipped_sumsq += flipped_diff * flipped_diff;
   }
   if (flipped_sumsq < orig_sumsq)
   {
      for (i=0; i<4; i++)
         quat[i] *= -1.0;
   }
   return 0;
}

int cd_kin_pose_flip_closerto(double pose[7], double target[7])
{
   int i;
   double orig_diff;
   double flipped_diff;
   double orig_sumsq = 0.0;
   double flipped_sumsq = 0.0;
   for (i=3; i<7; i++)
   {
      orig_diff = pose[i] - target[i];
      flipped_diff = - pose[i] - target[i];
      orig_sumsq += orig_diff * orig_diff;
      flipped_sumsq += flipped_diff * flipped_diff;
   }
   if (flipped_sumsq < orig_sumsq)
   {
      for (i=3; i<7; i++)
         pose[i] *= -1.0;
   }
   return 0;
}

int cd_kin_quat_compose(double quat_ab[4], double quat_bc[4], double quat_ac[4])
{
   double qabx, qaby, qabz, qabw;
   double qbcx, qbcy, qbcz, qbcw;
   qabx = quat_ab[0];
   qaby = quat_ab[1];
   qabz = quat_ab[2];
   qabw = quat_ab[3];
   qbcx = quat_bc[0];
   qbcy = quat_bc[1];
   qbcz = quat_bc[2];
   qbcw = quat_bc[3];
   /* Quaternion part is simple composition */
   quat_ac[0] = qabw*qbcx + qabx*qbcw + qaby*qbcz - qabz*qbcy;
   quat_ac[1] = qabw*qbcy - qabx*qbcz + qaby*qbcw + qabz*qbcx;
   quat_ac[2] = qabw*qbcz + qabx*qbcy - qaby*qbcx + qabz*qbcw;
   quat_ac[3] = qabw*qbcw - qabx*qbcx - qaby*qbcy - qabz*qbcz;
   return 0;
}

int cd_kin_pose_compose(double pose_ab[7], double pose_bc[7], double pose_ac[7])
{
   double qabx, qaby, qabz, qabw;
   double qbcx, qbcy, qbcz, qbcw;
   double x_in, y_in, z_in;
   double x_out, y_out, z_out;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   qabx = pose_ab[3];
   qaby = pose_ab[4];
   qabz = pose_ab[5];
   qabw = pose_ab[6];
   qbcx = pose_bc[3];
   qbcy = pose_bc[4];
   qbcz = pose_bc[5];
   qbcw = pose_bc[6];
   /* Quaternion part is simple composition */
   pose_ac[3] = qabw*qbcx + qabx*qbcw + qaby*qbcz - qabz*qbcy;
   pose_ac[4] = qabw*qbcy - qabx*qbcz + qaby*qbcw + qabz*qbcx;
   pose_ac[5] = qabw*qbcz + qabx*qbcy - qaby*qbcx + qabz*qbcw;
   pose_ac[6] = qabw*qbcw - qabx*qbcx - qaby*qbcy - qabz*qbcz;
   /* Rotate xyz_bc into a frame, rotate by qab */
   x_in = pose_bc[0];
   y_in = pose_bc[1];
   z_in = pose_bc[2];
   qx2 = qabx*qabx;
   qy2 = qaby*qaby;
   qz2 = qabz*qabz;
   qw2 = qabw*qabw;
   qxqy = qabx*qaby;
   qxqz = qabx*qabz;
   qxqw = qabx*qabw;
   qyqz = qaby*qabz;
   qyqw = qaby*qabw;
   qzqw = qabz*qabw;
   x_out = x_in*(qx2-qy2-qz2+qw2) + 2*y_in*(qxqy-qzqw) + 2*z_in*(qxqz+qyqw);
   y_out = 2*x_in*(qxqy+qzqw) + y_in*(-qx2+qy2-qz2+qw2) + 2*z_in*(qyqz-qxqw);
   z_out = 2*x_in*(qxqz-qyqw) + 2*y_in*(qyqz+qxqw) + z_in*(-qx2-qy2+qz2+qw2);
   /* xyz part is the above rotated xyc_bc plus xyz_ab */
   pose_ac[0] = x_out + pose_ab[0];
   pose_ac[1] = y_out + pose_ab[1];
   pose_ac[2] = z_out + pose_ab[2];
   return 0;
}

int cd_kin_pose_compos(double pose_ab[7], double pos_bc[3], double pos_ac[3])
{
   double qabx, qaby, qabz, qabw;
   double x_in, y_in, z_in;
   double x_out, y_out, z_out;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   qabx = pose_ab[3];
   qaby = pose_ab[4];
   qabz = pose_ab[5];
   qabw = pose_ab[6];
   /* Rotate xyz_bc into a frame, rotate by qab */
   x_in = pos_bc[0];
   y_in = pos_bc[1];
   z_in = pos_bc[2];
   qx2 = qabx*qabx;
   qy2 = qaby*qaby;
   qz2 = qabz*qabz;
   qw2 = qabw*qabw;
   qxqy = qabx*qaby;
   qxqz = qabx*qabz;
   qxqw = qabx*qabw;
   qyqz = qaby*qabz;
   qyqw = qaby*qabw;
   qzqw = qabz*qabw;
   x_out = x_in*(qx2-qy2-qz2+qw2) + 2*y_in*(qxqy-qzqw) + 2*z_in*(qxqz+qyqw);
   y_out = 2*x_in*(qxqy+qzqw) + y_in*(-qx2+qy2-qz2+qw2) + 2*z_in*(qyqz-qxqw);
   z_out = 2*x_in*(qxqz-qyqw) + 2*y_in*(qyqz+qxqw) + z_in*(-qx2-qy2+qz2+qw2);
   /* xyz part is the above rotated xyc_bc plus xyz_ab */
   pos_ac[0] = x_out + pose_ab[0];
   pos_ac[1] = y_out + pose_ab[1];
   pos_ac[2] = z_out + pose_ab[2];
   return 0;
}

int cd_kin_quat_compose_vec(double quat_ab[4], double vec_bc[3], double vec_ac[3])
{
   double qabx, qaby, qabz, qabw;
   double x_in, y_in, z_in;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   qabx = quat_ab[0];
   qaby = quat_ab[1];
   qabz = quat_ab[2];
   qabw = quat_ab[3];
   /* Rotate xyz_bc into a frame, rotate by qab */
   x_in = vec_bc[0];
   y_in = vec_bc[1];
   z_in = vec_bc[2];
   qx2 = qabx*qabx;
   qy2 = qaby*qaby;
   qz2 = qabz*qabz;
   qw2 = qabw*qabw;
   qxqy = qabx*qaby;
   qxqz = qabx*qabz;
   qxqw = qabx*qabw;
   qyqz = qaby*qabz;
   qyqw = qaby*qabw;
   qzqw = qabz*qabw;
   vec_ac[0] = x_in*(qx2-qy2-qz2+qw2) + 2*y_in*(qxqy-qzqw) + 2*z_in*(qxqz+qyqw);
   vec_ac[1] = 2*x_in*(qxqy+qzqw) + y_in*(-qx2+qy2-qz2+qw2) + 2*z_in*(qyqz-qxqw);
   vec_ac[2] = 2*x_in*(qxqz-qyqw) + 2*y_in*(qyqz+qxqw) + z_in*(-qx2-qy2+qz2+qw2);
   return 0;
}

/* Do the same, but just rotate a 3d vector (vel,acc), not a position vector */
int cd_kin_pose_compose_vec(double pose_ab[7], double vec_bc[3], double vec_ac[3])
{
   double qabx, qaby, qabz, qabw;
   double x_in, y_in, z_in;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   qabx = pose_ab[3];
   qaby = pose_ab[4];
   qabz = pose_ab[5];
   qabw = pose_ab[6];
   /* Rotate xyz_bc into a frame, rotate by qab */
   x_in = vec_bc[0];
   y_in = vec_bc[1];
   z_in = vec_bc[2];
   qx2 = qabx*qabx;
   qy2 = qaby*qaby;
   qz2 = qabz*qabz;
   qw2 = qabw*qabw;
   qxqy = qabx*qaby;
   qxqz = qabx*qabz;
   qxqw = qabx*qabw;
   qyqz = qaby*qabz;
   qyqw = qaby*qabw;
   qzqw = qabz*qabw;
   vec_ac[0] = x_in*(qx2-qy2-qz2+qw2) + 2*y_in*(qxqy-qzqw) + 2*z_in*(qxqz+qyqw);
   vec_ac[1] = 2*x_in*(qxqy+qzqw) + y_in*(-qx2+qy2-qz2+qw2) + 2*z_in*(qyqz-qxqw);
   vec_ac[2] = 2*x_in*(qxqz-qyqw) + 2*y_in*(qyqz+qxqw) + z_in*(-qx2-qy2+qz2+qw2);
   return 0;
}

int cd_kin_quat_invert(double quat_in[4], double quat_out[4])
{
   double qx, qy, qz, qw;
   /* Invert quaternion (assume normalized) */
   qx = -quat_in[0];
   qy = -quat_in[1];
   qz = -quat_in[2];
   qw =  quat_in[3];
   quat_out[0] = qx;
   quat_out[1] = qy;
   quat_out[2] = qz;
   quat_out[3] = qw;
   return 0;
}

int cd_kin_pose_invert(double pose_in[7], double pose_out[7])
{
   double x_in, y_in, z_in;
   double x_out, y_out, z_out;
   double qx, qy, qz, qw;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   /* Get from input */
   x_in = pose_in[0];
   y_in = pose_in[1];
   z_in = pose_in[2];
   /* Invert quaternion (assume normalized) */
   qx = -pose_in[3];
   qy = -pose_in[4];
   qz = -pose_in[5];
   qw =  pose_in[6];
   /* Apply inverted quaternion to xyz_in */
   qx2 = qx*qx;
   qy2 = qy*qy;
   qz2 = qz*qz;
   qw2 = qw*qw;
   qxqy = qx*qy;
   qxqz = qx*qz;
   qxqw = qx*qw;
   qyqz = qy*qz;
   qyqw = qy*qw;
   qzqw = qz*qw;
   x_out = x_in*(qx2-qy2-qz2+qw2) + 2*y_in*(qxqy-qzqw) + 2*z_in*(qxqz+qyqw);
   y_out = 2*x_in*(qxqy+qzqw) + y_in*(-qx2+qy2-qz2+qw2) + 2*z_in*(qyqz-qxqw);
   z_out = 2*x_in*(qxqz-qyqw) + 2*y_in*(qyqz+qxqw) + z_in*(-qx2-qy2+qz2+qw2);
   /* Apply negative x_out to output */
   pose_out[0] = -x_out;
   pose_out[1] = -y_out;
   pose_out[2] = -z_out;
   pose_out[3] = qx;
   pose_out[4] = qy;
   pose_out[5] = qz;
   pose_out[6] = qw;
   return 0;
}

/* Taken from http:/www.j3d.org/matrix_faq/matrfaq_latest.html#Q54
 * Author: matrix_faq@j3d.org, hexapod@(no-spam)netcom.com
 * Formerly ftp://ftp.netcom.com/pub/he/hexapod/index.html,
 *          http://www.glue.umd.edu/~rsrodger;
 * Checked (and modified) against
 *   http://en.wikipedia.org/wiki/Rotation_representation_(mathematics)
 *
 * Further Note, from http://en.wikipedia.org/wiki/Rotation_matrix:
 * Efficient computation:
 *
 * Nq = w^2 + x^2 + y^2 + z^2
 * if Nq > 0.0 then s = 2/Nq else s = 0.0
 * X = x*s; Y = y*s; Z = z*s
 * wX = w*X; wY = w*Y; wZ = w*Z
 * xX = x*X; xY = x*Y; xZ = x*Z
 * yY = y*Y; yZ = y*Z; zZ = z*Z
 * [ 1.0-(yY+zZ)   xY-wZ      xZ+wY    ]
 * [    xY+wZ   1.0-(xX+zZ)   yZ-wX    ]
 * [    xZ-wY      yZ+wX   1.0-(xX+yY) ]
 */
int cd_kin_quat_to_R(double quat[4], double R[3][3])
{
   double xx, xy, xz, xw, yy, yz, yw, zz, zw;
   xx = quat[0] * quat[0];
   xy = quat[0] * quat[1];
   xz = quat[0] * quat[2];
   xw = quat[0] * quat[3];
   yy = quat[1] * quat[1];
   yz = quat[1] * quat[2];
   yw = quat[1] * quat[3];
   zz = quat[2] * quat[2];
   zw = quat[2] * quat[3];
   R[0][0] = 1 - 2 * (yy + zz);
   R[0][1] = 2 * (xy - zw);
   R[0][2] = 2 * (xz + yw);
   R[1][0] = 2 * (xy + zw);
   R[1][1] = 1 - 2 * (xx + zz);
   R[1][2] = 2 * (yz - xw);
   R[2][0] = 2 * (xz - yw);
   R[2][1] = 2 * (yz + xw);
   R[2][2] = 1 - 2 * (xx + yy);
   return 0;
}

int cd_kin_pose_to_H(double pose[7], double H[4][4], int fill_bottom)
{
   double xx, xy, xz, xw, yy, yz, yw, zz, zw;
   xx = pose[3] * pose[3];
   xy = pose[3] * pose[4];
   xz = pose[3] * pose[5];
   xw = pose[3] * pose[6];
   yy = pose[4] * pose[4];
   yz = pose[4] * pose[5];
   yw = pose[4] * pose[6];
   zz = pose[5] * pose[5];
   zw = pose[5] * pose[6];
   H[0][0] = 1 - 2 * (yy + zz);
   H[0][1] = 2 * (xy - zw);
   H[0][2] = 2 * (xz + yw);
   H[1][0] = 2 * (xy + zw);
   H[1][1] = 1 - 2 * (xx + zz);
   H[1][2] = 2 * (yz - xw);
   H[2][0] = 2 * (xz - yw);
   H[2][1] = 2 * (yz + xw);
   H[2][2] = 1 - 2 * (xx + yy);
   H[0][3] = pose[0];
   H[1][3] = pose[1];
   H[2][3] = pose[2];
   if (fill_bottom)
   {
      H[3][0] = 0.0;
      H[3][1] = 0.0;
      H[3][2] = 0.0;
      H[3][3] = 1.0;
   }
   return 0;
}

int cd_kin_pose_to_dR(double pose[7], double d[3], double R[3][3])
{
   cd_kin_quat_to_R(pose+3, R);
   d[0] = pose[0];
   d[1] = pose[1];
   d[2] = pose[2];
   return 0;
}

int cd_kin_quat_from_R(double quat[4], double R[3][3])
{
   double xx4, yy4, zz4, ww4; /* 4 times each component */
   double v4; /* 4/v with v the biggest of x, y, z, w */
   xx4 = 1.0 + R[0][0] - R[1][1] - R[2][2];
   yy4 = 1.0 - R[0][0] + R[1][1] - R[2][2];
   zz4 = 1.0 - R[0][0] - R[1][1] + R[2][2];
   ww4 = 1.0 + R[0][0] + R[1][1] + R[2][2];
   if (xx4 > yy4 && xx4 > zz4 && xx4 > ww4)
   {
      quat[0] = sqrt(0.25*xx4);
      v4 = 0.25 / quat[0];
      quat[1] = v4 * (R[1][0] + R[0][1]);
      quat[2] = v4 * (R[0][2] + R[2][0]);
      quat[3] = v4 * (R[2][1] - R[1][2]);
   }
   else if (yy4 > zz4 && yy4 > ww4)
   {
      quat[1] = sqrt(0.25*yy4);
      v4 = 0.25 / quat[1];
      quat[0] = v4 * (R[1][0] + R[0][1]);
      quat[2] = v4 * (R[2][1] + R[1][2]);
      quat[3] = v4 * (R[0][2] - R[2][0]);
   }
   else if (zz4 > ww4)
   {
      quat[2] = sqrt(0.25*zz4);
      v4 = 0.25 / quat[2];
      quat[0] = v4 * (R[0][2] + R[2][0]);
      quat[1] = v4 * (R[2][1] + R[1][2]);
      quat[3] = v4 * (R[1][0] - R[0][1]);
   }
   else
   {
      quat[3] = sqrt(0.25*ww4);
      v4 = 0.25 / quat[3];
      quat[0] = v4 * (R[2][1] - R[1][2]);
      quat[1] = v4 * (R[0][2] - R[2][0]);
      quat[2] = v4 * (R[1][0] - R[0][1]);
   }
   return 0;
}

int cd_kin_pose_from_H(double pose[7], double H[3][4])
{
   double t, r, s;
   t = 1.0 + H[0][0] + H[1][1] + H[2][2];
   if (t > 0.000001)
   {
      r = sqrt(t);
      s = 0.5 / r;
      pose[3] = (H[2][1] - H[1][2]) * s;
      pose[4] = (H[0][2] - H[2][0]) * s;
      pose[5] = (H[1][0] - H[0][1]) * s;
      pose[6] = 0.5 * r;
   }
   else if (H[0][0] > H[1][1] && H[0][0] > H[2][2])
   {
      /* Rxx largest */
      r = sqrt(1.0 + H[0][0] - H[1][1] - H[2][2]);
      s = 0.5 / r;
      pose[3] = 0.5 * r;
      pose[4] = (H[0][1] + H[1][0]) * s;
      pose[5] = (H[0][2] + H[2][0]) * s;
      pose[6] = (H[2][1] - H[1][2]) * s;
   }
   else if (H[1][1] > H[2][2])
   {
      /* Ryy largest */
      r = sqrt(1.0 - H[0][0] + H[1][1] - H[2][2]);
      s = 0.5 / r;
      pose[3] = (H[1][0] + H[0][1]) * s;
      pose[4] = 0.5 * r;
      pose[5] = (H[1][2] + H[2][1]) * s;
      pose[6] = (H[0][2] - H[2][0]) * s;
   }
   else
   {
      /* Rzz largest */
      r = sqrt(1.0 - H[0][0] - H[1][1] + H[2][2]);
      s = 0.5 / r;
      pose[3] = (H[2][0] + H[0][2]) * s;
      pose[4] = (H[2][1] + H[1][2]) * s;
      pose[5] = 0.5 * r;
      pose[6] = (H[1][0] - H[0][1]) * s;
   }
   pose[0] = H[0][3];
   pose[1] = H[1][3];
   pose[2] = H[2][3];   
   return 0;
}

int cd_kin_pose_from_dR(double pose[7], double d[3], double R[3][3])
{
   cd_kin_quat_from_R(pose+3, R);
   pose[0] = d[0];
   pose[1] = d[1];
   pose[2] = d[2];
   return 0;
}

int cd_kin_quat_to_axisangle(double quat[4], double axis[3], double *angle)
{
   double a2;
   double sina2inv;
   a2 = acos(quat[3]);
   *angle = 2.0*a2;
   sina2inv = 1.0/sin(a2);
   axis[0] = sina2inv * quat[0];
   axis[1] = sina2inv * quat[1];
   axis[2] = sina2inv * quat[2];
   return 0;
}

int cd_kin_quat_from_axisangle(double quat[4], double axis[3], double angle)
{
   double a2;
   double sina2;
   a2 = 0.5*angle;
   quat[3] = cos(a2);
   sina2 = sin(a2);
   quat[0] = sina2 * axis[0];
   quat[1] = sina2 * axis[1];
   quat[2] = sina2 * axis[2];
   return 0;
}

/* using Rodrigues' rotation formula 
 * http://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula */
int cd_kin_axisangle_rotate(double axis[3], double angle,
   double pos_in[3], double pos_out[3])
{
   double v[3];
   double axisdotv;
   cd_mat_memcpy(v, pos_in, 3, 1);
   cd_mat_set_zero(pos_out, 3, 1);
   cd_mat_cross(axis, v, pos_out);
   cd_mat_scale(pos_out, 3, 1, sin(angle));
   cblas_daxpy(3, cos(angle), v,1, pos_out,1);
   axisdotv = cblas_ddot(3, axis,1, v,1);
   cblas_daxpy(3, axisdotv*(1-cos(angle)), axis,1, pos_out,1);
   return 0;
}

/* exponential map, so(3) -> SO(3) */
int cd_kin_axisangle_to_R(double axis[3], double angle, double R[3][3])
{
   double x, y, z;
   double s;
   double cm;
   s = sin(angle);
   cm = 1.0 - cos(angle);
   x = axis[0];
   y = axis[1];
   z = axis[2];
   R[0][0] = 1.0 - cm*(y*y + z*z);
   R[0][1] = -s*z + cm*x*y;
   R[0][2] =  s*y + cm*x*z;
   R[1][0] =  s*z + cm*x*y;
   R[1][1] = 1.0 - cm*(x*x + z*z);
   R[1][2] = -s*x + cm*y*z;
   R[2][0] = -s*y + cm*x*z;
   R[2][1] =  s*x + cm*y*z;
   R[2][2] = 1.0 - cm*(x*x + y*y);
   return 0;
}

int cd_kin_quat_to_ypr(double quat[4], double ypr[3])
{
   double qx, qy, qz, qw;
   double sinp2;
   qx = quat[0];
   qy = quat[1];
   qz = quat[2];
   qw = quat[3];
   sinp2 = qw*qy-qz*qx;
   if (sinp2 > 0.49999)
   {
      ypr[0] = -2.0*atan2(qx,qw);
      ypr[1] = 0.25*TAU;
      ypr[2] = 0.0;
   }
   else if (sinp2 < -0.49999)
   {
      ypr[0] = 2.0*atan2(qx,qw);
      ypr[1] = -0.25*TAU;
      ypr[2] = 0.0;
   }
   else
   {
      ypr[0] = atan2(2.0*(qw*qz+qx*qy), 1.0 - 2.0*(qy*qy+qz*qz));
      ypr[1] = asin(2.0*sinp2);
      ypr[2] = atan2(2.0*(qw*qx+qy*qz), 1.0 - 2.0*(qx*qx+qy*qy));
   }
   return 0;
}

int cd_kin_pose_to_xyzypr(double pose[7], double xyzypr[6])
{
   double qx, qy, qz, qw;
   double sinp2;
   qx = pose[3];
   qy = pose[4];
   qz = pose[5];
   qw = pose[6];
   xyzypr[0] = pose[0];
   xyzypr[1] = pose[1];
   xyzypr[2] = pose[2];
   sinp2 = qw*qy-qz*qx;
   if (sinp2 > 0.49999)
   {
      xyzypr[3] = -2.0*atan2(qx,qw);
      xyzypr[4] = 0.25*TAU;
      xyzypr[5] = 0.0;
   }
   else if (sinp2 < -0.49999)
   {
      xyzypr[3] = 2.0*atan2(qx,qw);
      xyzypr[4] = -0.25*TAU;
      xyzypr[5] = 0.0;
   }
   else
   {
      xyzypr[3] = atan2(2.0*(qw*qz+qx*qy), 1.0 - 2.0*(qy*qy+qz*qz));
      xyzypr[4] = asin(2.0*sinp2);
      xyzypr[5] = atan2(2.0*(qw*qx+qy*qz), 1.0 - 2.0*(qx*qx+qy*qy));
   }
   return 0;
}

/* for now, dont deal with gimbal lock! */
int cd_kin_quat_to_ypr_J(double quat[4], double J[3][4])
{
   double qx, qy, qz, qw;
   double nu, de; /* numerator, denominator */
   double as; /* argsin arg */
   qx = quat[0];
   qy = quat[1];
   qz = quat[2];
   qw = quat[3];
   /* yaw */
   nu = 2.0*(qw*qz+qx*qy);
   de = 1.0 - 2.0*(qy*qy+qz*qz);
   J[0][0] = de/(de*de+nu*nu)*(2.0*qy);
   J[0][1] = de/(de*de+nu*nu)*(2.0*qx) - nu/(de*de+nu*nu)*(-2.0*2.0*qy);
   J[0][2] = de/(de*de+nu*nu)*(2.0*qw) - nu/(de*de+nu*nu)*(-2.0*2.0*qz);
   J[0][3] = de/(de*de+nu*nu)*(2.0*qz);
   /* pitch */
   as = 2.0 * (qw*qy-qz*qx);
   J[1][0] = 1.0/sqrt(1.0-as*as)*2.0*(-qz);
   J[1][1] = 1.0/sqrt(1.0-as*as)*2.0*( qw);
   J[1][2] = 1.0/sqrt(1.0-as*as)*2.0*(-qx);
   J[1][3] = 1.0/sqrt(1.0-as*as)*2.0*( qy);
   /* roll */
   nu = 2.0*(qw*qx+qy*qz);
   de = 1.0 - 2.0*(qx*qx+qy*qy);
   J[2][0] = de/(de*de+nu*nu)*(2.0*qw) - nu/(de*de+nu*nu)*(-2.0*2.0*qx);
   J[2][1] = de/(de*de+nu*nu)*(2.0*qz) - nu/(de*de+nu*nu)*(-2.0*2.0*qy);
   J[2][2] = de/(de*de+nu*nu)*(2.0*qy);
   J[2][3] = de/(de*de+nu*nu)*(2.0*qx);
   return 0;
}

/* for now, dont deal with gimbal lock! */
int cd_kin_pose_to_xyzypr_J(double pose[7], double J[6][7])
{
   double qx, qy, qz, qw;
   double nu, de; /* numerator, denominator */
   double as; /* argsin arg */
   qx = pose[3];
   qy = pose[4];
   qz = pose[5];
   qw = pose[6];
   /* position part */
   cd_mat_set_zero(*J, 6, 7);
   J[0][0] = 1.0;
   J[1][1] = 1.0;
   J[2][2] = 1.0;
   /* yaw */
   nu = 2.0*(qw*qz+qx*qy);
   de = 1.0 - 2.0*(qy*qy+qz*qz);
   J[3][3] = de/(de*de+nu*nu)*(2.0*qy);
   J[3][4] = de/(de*de+nu*nu)*(2.0*qx) - nu/(de*de+nu*nu)*(-2.0*2.0*qy);
   J[3][5] = de/(de*de+nu*nu)*(2.0*qw) - nu/(de*de+nu*nu)*(-2.0*2.0*qz);
   J[3][6] = de/(de*de+nu*nu)*(2.0*qz);
   /* pitch */
   as = 2.0 * (qw*qy-qz*qx);
   J[4][3] = 1.0/sqrt(1.0-as*as)*2.0*(-qz);
   J[4][4] = 1.0/sqrt(1.0-as*as)*2.0*( qw);
   J[4][5] = 1.0/sqrt(1.0-as*as)*2.0*(-qx);
   J[4][6] = 1.0/sqrt(1.0-as*as)*2.0*( qy);
   /* roll */
   nu = 2.0*(qw*qx+qy*qz);
   de = 1.0 - 2.0*(qx*qx+qy*qy);
   J[5][3] = de/(de*de+nu*nu)*(2.0*qw) - nu/(de*de+nu*nu)*(-2.0*2.0*qx);
   J[5][4] = de/(de*de+nu*nu)*(2.0*qz) - nu/(de*de+nu*nu)*(-2.0*2.0*qy);
   J[5][5] = de/(de*de+nu*nu)*(2.0*qy);
   J[5][6] = de/(de*de+nu*nu)*(2.0*qx);
   return 0;
}

int cd_kin_quat_from_ypr(double quat[4], double ypr[3])
{
   double cy2, sy2, cp2, sp2, cr2, sr2;
   cy2 = cos(0.5 * ypr[0]);
   sy2 = sin(0.5 * ypr[0]);
   cp2 = cos(0.5 * ypr[1]);
   sp2 = sin(0.5 * ypr[1]);
   cr2 = cos(0.5 * ypr[2]);
   sr2 = sin(0.5 * ypr[2]);
   quat[0] = -sy2*sp2*cr2 + cy2*cp2*sr2; /* qx */
   quat[1] =  cy2*sp2*cr2 + sy2*cp2*sr2; /* qy */
   quat[2] = -cy2*sp2*sr2 + sy2*cp2*cr2; /* qz */
   quat[3] =  sy2*sp2*sr2 + cy2*cp2*cr2; /* qw */
   return 0;
}

int cd_kin_pose_from_xyzypr(double pose[7], double xyzypr[6])
{
   double cy2, sy2, cp2, sp2, cr2, sr2;
   cy2 = cos(0.5 * xyzypr[3]);
   sy2 = sin(0.5 * xyzypr[3]);
   cp2 = cos(0.5 * xyzypr[4]);
   sp2 = sin(0.5 * xyzypr[4]);
   cr2 = cos(0.5 * xyzypr[5]);
   sr2 = sin(0.5 * xyzypr[5]);
   pose[0] = xyzypr[0];
   pose[1] = xyzypr[1];
   pose[2] = xyzypr[2];
   pose[3] = -sy2*sp2*cr2 + cy2*cp2*sr2; /* qx */
   pose[4] =  cy2*sp2*cr2 + sy2*cp2*sr2; /* qy */
   pose[5] = -cy2*sp2*sr2 + sy2*cp2*cr2; /* qz */
   pose[6] =  sy2*sp2*sr2 + cy2*cp2*cr2; /* qw */
   return 0;
}

int cd_kin_pose_to_pos_quat(double pose[7], double pos[3], double quat[4])
{
   int i;
   if (pos)  for (i=0; i<3; i++) pos[i] = pose[i];
   if (quat) for (i=0; i<4; i++) quat[i] = pose[3+i];
   return 0;
}

int cd_kin_pose_from_pos_quat(double pose[7], double pos[3], double quat[4])
{
   int i;
   if (pos) for (i=0; i<3; i++) pose[i] = pos[i];
   else     for (i=0; i<3; i++) pose[i] = 0.0;
   if (quat) for (i=0; i<4; i++) pose[3+i] = quat[i];
   else      for (i=0; i<4; i++) pose[3+i] = i==3 ? 1.0 : 0.0;
   return 0;
}

int cd_kin_pose_from_op(double pose[7], double from[3], double to[3], double * lenp)
{
   double to_diff[3];
   to_diff[0] = to[0];
   to_diff[1] = to[1];
   to_diff[2] = to[2];
   if (from)
   {
      to_diff[0] -= from[0];
      to_diff[1] -= from[1];
      to_diff[2] -= from[2];
   }
   cd_kin_pose_from_op_diff(pose, from, to_diff, lenp);
   return 0;
}

int cd_kin_pose_from_op_diff(double pose[7], double from[3], double to_diff[3], double * lenp)
{
   double d[3];
   double R[3][3];
   double len;
   
   if (from)
   {
      d[0] = from[0];
      d[1] = from[1];
      d[2] = from[2];
   }
   else
   {
      d[0] = 0.0;
      d[1] = 0.0;
      d[2] = 0.0;
   }
   
   /* Define Z axis in direction of arrow */
   len = cblas_dnrm2(3, to_diff, 1);
   if (lenp) *lenp = len;
   R[0][2] = to_diff[0] / len;
   R[1][2] = to_diff[1] / len;
   R[2][2] = to_diff[2] / len;

   /* Define other axes */
   if (fabs(R[0][2]) > 0.9)
   {
      /* Z is too close to e1, but sufficiently far from e2;
       * cross e2 with Z to get X (and normalize) */
      len = sqrt(R[2][2]*R[2][2] + R[0][2]*R[0][2]);
      R[0][0] = R[2][2] / len;
      R[1][0] = 0.0;
      R[2][0] = -R[0][2] / len;
      /* Then Y = Z x X */
      R[0][1] = R[1][2] * R[2][0] - R[2][2] * R[1][0];
      R[1][1] = R[2][2] * R[0][0] - R[0][2] * R[2][0];
      R[2][1] = R[0][2] * R[1][0] - R[1][2] * R[0][0];
   }
   else
   {
      /* Z is sufficiently far from e1;
       * cross Z with e1 to get Y (and normalize) */
      len = sqrt(R[2][2]*R[2][2] + R[1][2]*R[1][2]);
      R[0][1] = 0.0;
      R[1][1] = R[2][2] / len;
      R[2][1] = -R[1][2] / len;
      /* Then X = Y x Z */
      R[0][0] = R[1][1] * R[2][2] - R[2][1] * R[1][2];
      R[1][0] = R[2][1] * R[0][2] - R[0][1] * R[2][2];
      R[2][0] = R[0][1] * R[1][2] - R[1][1] * R[0][2];
   }
   
   cd_kin_pose_from_dR(pose, d, R);
   return 0;
}
