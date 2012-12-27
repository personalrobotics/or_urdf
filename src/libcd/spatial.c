/** \file spatial.c
 * \brief Implementation of cd_spatial, a collection of useful routines for
 *        spatial vector algrbra.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#include <math.h>
#include <cblas.h>

#include "kin.h"
#include "mat.h"
#include "spatial.h"

int cd_spatial_xm_to_pose(double xm[6][6], double pose[7])
{
   int i, j;
   double temp3x3[3][3];
   
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
      1.0, &xm[3][0],6, &xm[0][0],6, 0.0,&temp3x3[0][0],3);
   pose[0] = temp3x3[2][1];
   pose[1] = temp3x3[0][2];
   pose[2] = temp3x3[1][0];
   
   for (i=0; i<3; i++)
      for (j=0; j<3; j++)
         temp3x3[i][j] = xm[i][j];
   cd_kin_quat_from_R(pose+3, temp3x3);
   
   return 0;
}

int cd_spatial_xf_to_pose(double xf[6][6], double pose[7])
{
   int i, j;
   double temp3x3[3][3];
   
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 3, 3, 3,
      1.0, &xf[0][3],6, &xf[0][0],6, 0.0,&temp3x3[0][0],3);
   pose[0] = temp3x3[2][1];
   pose[1] = temp3x3[0][2];
   pose[2] = temp3x3[1][0];
   
   for (i=0; i<3; i++)
      for (j=0; j<3; j++)
         temp3x3[i][j] = xf[i][j];
   cd_kin_quat_from_R(pose+3, temp3x3);

   return 0;
}

int cd_spatial_xm_from_pose(double xm[6][6], double pose[7])
{
   int i, j;
   double R[3][3];
   double rx[3][3];
   
   cd_mat_set_zero(&xm[0][0], 6, 6);
   
   /* Fill rotation matrices */
   cd_kin_quat_to_R(pose+3, R);
   for (i=0; i<3; i++)
   for (j=0; j<3; j++)
   {
      xm[i][j] = R[i][j];
      xm[3+i][3+j] = R[i][j];
   }
   
   /* Get [rx], then fill [rx]R */
   rx[0][0] = 0.0;
   rx[0][1] = -pose[2];
   rx[0][2] =  pose[1];
   rx[1][0] =  pose[2];
   rx[1][1] = 0.0;
   rx[1][2] = -pose[0];
   rx[2][0] = -pose[1];
   rx[2][1] =  pose[0];
   rx[2][2] = 0.0;
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
      1.0, &rx[0][0],3, &R[0][0],3, 0.0,&xm[3][0],6);
      
   return 0;
}

int cd_spatial_xf_from_pose(double xf[6][6], double pose[7])
{
   int i, j;
   double R[3][3];
   double rx[3][3];
   
   cd_mat_set_zero(&xf[0][0], 6, 6);
   
   /* Fill rotation matrices */
   cd_kin_quat_to_R(pose+3, R);
   for (i=0; i<3; i++)
   for (j=0; j<3; j++)
   {
      xf[i][j] = R[i][j];
      xf[3+i][3+j] = R[i][j];
   }
   
   /* Get [rx], then fill [rx]R */
   rx[0][0] = 0.0;
   rx[0][1] = -pose[2];
   rx[0][2] =  pose[1];
   rx[1][0] =  pose[2];
   rx[1][1] = 0.0;
   rx[1][2] = -pose[0];
   rx[2][0] = -pose[1];
   rx[2][1] =  pose[0];
   rx[2][2] = 0.0;
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 3, 3,
      1.0, &rx[0][0],3, &R[0][0],3, 0.0,&xf[0][3],6);

   return 0;
}

int cd_spatial_inertia_x(double pose_ab[7], double inertia_b[6][6], double inertia_a[6][6])
{
   double pose_ba[7];
   double xm_ba[6][6];
   double i_x[6][6];
   cd_kin_pose_invert(pose_ab, pose_ba);
   cd_spatial_xm_from_pose(xm_ba, pose_ba);
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6,
      1.0, &inertia_b[0][0],6, &xm_ba[0][0],6, 0.0,&i_x[0][0],6);
   cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, 6, 6, 6,
      1.0, &xm_ba[0][0],6, &i_x[0][0],6, 0.0,&inertia_a[0][0],6);
   return 0;
}


int cd_spatial_pose_from_spavel_unittime(double pose[7], double spavel[6])
{
   double wdotv;
   double w2;
   double w;
   double c_cross;
   double c_v;
   double c_w;
   double axis[3];
   w2 = spavel[0]*spavel[0]+spavel[1]*spavel[1]+spavel[2]*spavel[2];
   wdotv = spavel[0]*spavel[3]+spavel[1]*spavel[4]+spavel[2]*spavel[5];
   if (w2 < 0.0000001)
   {
      /* what is the orientation stuff here? */
      cd_kin_quat_identity(pose+3);
      cblas_daxpy(3, 0.5-w2/48.0+w2*w2/3840.0-w2*w2*w2/645120.0,
         spavel,1, pose+3,1);
      pose[6] = 1.0 - w2/8.0 + w2*w2/384.0 - w2*w2*w2/46080.0;
      /* position coefficients */
      c_cross = 1.0/2.0 - w2/24.0  + w2*w2/720.0  - w2*w2*w2/40320.0;
      c_v = 1.0 - w2/6.0 + w2*w2/120.0 - w2*w2*w2/5040.0;
      c_w = 1.0/6.0 - w2/120.0 + w2*w2/5040.0 - w2*w2*w2/362880.0;
      c_w *= wdotv;
   }
   else
   {
      w = sqrt(w2);
      /* the orientation stuff is pretty easy */
      axis[0] = spavel[0] / w;
      axis[1] = spavel[1] / w;
      axis[2] = spavel[2] / w;
      cd_kin_quat_from_axisangle(pose+3, axis, w);
      /* position coefficients */
      c_cross = (1.0 - cos(w))/w2;
      c_v = sin(w)/w;
      c_w = (1.0 - c_v) * wdotv / w2;
   }
   /* do the position stuff */
   cd_mat_set_zero(pose, 3, 1);
   cd_mat_cross(spavel, spavel+3, pose);
   cd_mat_scale(pose, 3, 1, c_cross);
   cblas_daxpy(3, c_v, spavel+3,1, pose,1);
   cblas_daxpy(3, c_w, spavel,1, pose,1);
   return 0;
}

/* This is the exponential map from se(3) to SE(3)
 * spavel is twist coordinates of the pose
 * the H gives the pose of the body-fixed frame
 * originally at the origin */
int cd_spatial_H_from_spavel_unittime(double H[4][4], double spavel[6])
{
   double S[4][4];
   double S2[4][4];
   double wx, wy, wz, vx, vy, vz;
   double w2;
   double w;
   double s2;
   double s3;
   wx = spavel[0];
   wy = spavel[1];
   wz = spavel[2];
   vx = spavel[3];
   vy = spavel[4];
   vz = spavel[5];
   /* w */
   w2 = wx*wx + wy*wy + wz*wz;
   if (w2 < 0.0000001)
   {
      s2 = 1.0/2.0 - w2/24.0  + w2*w2/720.0  - w2*w2*w2/40320.0;
      s3 = 1.0/6.0 - w2/120.0 + w2*w2/5040.0 - w2*w2*w2/362880.0;
   }
   else
   {
      w = sqrt(w2);
      s2 = (1.0-cos(w)) / w2;
      s3 = (w-sin(w)) / (w*w2);
   }
   /* set S */
   S[0][0] = 0.0; S[0][1] = -wz; S[0][2] =  wy; S[0][3] =  vx;
   S[1][0] =  wz; S[1][1] = 0.0; S[1][2] = -wx; S[1][3] =  vy;
   S[2][0] = -wy; S[2][1] =  wx; S[2][2] = 0.0; S[2][3] =  vz;
   S[3][0] = 0.0; S[3][1] = 0.0; S[3][2] = 0.0; S[3][3] = 0.0;
   /* get S2 */
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 4, 4, 4,
      1.0, *S,4, *S,4, 0.0, *S2,4);
   /* set identity */
   cd_mat_set_diag(*H, 4, 4, 1.0);
   /* add in S (4x4 screw matrix) */
   cd_mat_add(*H, *S, 4, 4);
   /* Add in S^2 component */
   cblas_daxpy(12, s2, *S2,1, *H,1);
   /* Add in S^3 component */
   cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 4, 4, 4,
      s3, *S,4, *S2,4, 1.0, *H,4);
   return 0;
}


/* Inverting transposes each 3x3 block individually */
int cd_spatial_x_invert(double x[6][6])
{
   double temp;
   
#define SW(i,j,a,b) temp=x[i][j]; x[i][j]=x[a][b]; x[a][b]=temp;
#define T3(i,j) SW(i,j+1, i+1,j) SW(i,j+2, i+2,j) SW(i+1,j+2, i+2,j+1)
   
   T3(0,0)
   T3(3,0)
   T3(0,3)
   T3(3,3)
   
#undef SW
#undef T3

   return 0;
}

int cd_spatial_v_to_pos(double vel[6], double pos[3])
{
   cd_mat_cross(vel, pos, vel+3);
   return 0;
}

int cd_spatial_v_from_pos(double vel[6], double pos[3])
{
   cd_mat_cross(pos, vel, vel+3);
   return 0;
}

int cd_spatial_f_to_pos(double force[6], double pos[3])
{
   cd_mat_cross(force+3, pos, force);
   return 0;
}

int cd_spatial_f_from_pos(double force[6], double pos[3])
{
   cd_mat_cross(pos, force+3, force);
   return 0;
}


int cd_spatial_pose_jac(double pose[7], double jac[6][7])
{
   double x, y, z, qxt2, qyt2, qzt2, qwt2;
   x = pose[0];
   y = pose[1];
   z = pose[2];
   qxt2 = 2.0 * pose[3];
   qyt2 = 2.0 * pose[4];
   qzt2 = 2.0 * pose[5];
   qwt2 = 2.0 * pose[6];
   cd_mat_set_zero(&jac[0][0], 6, 7);
   /* Bottom left */
   jac[3][0] = 1.0;
   jac[4][1] = 1.0;
   jac[5][2] = 1.0;
   /* Top right */
   jac[0][3] =  qwt2;
   jac[0][4] = -qzt2;
   jac[0][5] =  qyt2;
   jac[0][6] = -qxt2;
   jac[1][3] =  qzt2;
   jac[1][4] =  qwt2;
   jac[1][5] = -qxt2;
   jac[1][6] = -qyt2;
   jac[2][3] = -qyt2;
   jac[2][4] =  qxt2;
   jac[2][5] =  qwt2;
   jac[2][6] = -qzt2;
   /* Bottom right */
   jac[3][3] = -z*qzt2 -y*qyt2;
   jac[3][4] = -z*qwt2 +y*qxt2;
   jac[3][5] =  z*qxt2 +y*qwt2;
   jac[3][6] =  z*qyt2 -y*qzt2;
   jac[4][3] =  z*qwt2 +x*qyt2;
   jac[4][4] = -z*qzt2 -x*qxt2;
   jac[4][5] =  z*qyt2 -x*qwt2;
   jac[4][6] = -z*qxt2 +x*qzt2;
   jac[5][3] = -y*qwt2 +x*qzt2;
   jac[5][4] =  y*qzt2 +x*qwt2;
   jac[5][5] = -y*qyt2 -x*qxt2;
   jac[5][6] =  y*qxt2 -x*qyt2;
   return 0;
}

int cd_spatial_pose_jac_inverse(double pose[7], double jac_inverse[7][6])
{
   double x, y, z, qxd2, qyd2, qzd2, qwd2;
   x = pose[0];
   y = pose[1];
   z = pose[2];
   qxd2 = 0.5 * pose[3];
   qyd2 = 0.5 * pose[4];
   qzd2 = 0.5 * pose[5];
   qwd2 = 0.5 * pose[6];
   cd_mat_set_zero(&jac_inverse[0][0], 7, 6);
   /* Set upper left */
   jac_inverse[0][1] =  z;
   jac_inverse[0][2] = -y;
   jac_inverse[1][0] = -z;
   jac_inverse[1][2] =  x;
   jac_inverse[2][0] =  y;
   jac_inverse[2][1] = -x;
   /* Set upper right */
   jac_inverse[0][3] = 1.0;
   jac_inverse[1][4] = 1.0;
   jac_inverse[2][5] = 1.0;
   /* Set lower left */
   jac_inverse[3][0] =  qwd2;
   jac_inverse[3][1] =  qzd2;
   jac_inverse[3][2] = -qyd2;
   jac_inverse[4][0] = -qzd2;
   jac_inverse[4][1] =  qwd2;
   jac_inverse[4][2] =  qxd2;
   jac_inverse[5][0] =  qyd2;
   jac_inverse[5][1] = -qxd2;
   jac_inverse[5][2] =  qwd2;
   jac_inverse[6][0] = -qxd2;
   jac_inverse[6][1] = -qyd2;
   jac_inverse[6][2] = -qzd2;
   return 0;
}

int cd_spatial_inertia_from_com(double inertia[6][6], double mass, double com[3], double Icom[3][3])
{
   double x, y, z;
   if (com)
   {
      x = com[0];
      y = com[1];
      z = com[2];
   }
   else
   {
      x = 0.0;
      y = 0.0;
      z = 0.0;
   }
   /* Fill inertia matrix */
   cd_mat_set_zero(&inertia[0][0], 6, 6);
   /* Top left */
   inertia[0][0] = Icom[0][0] + mass*y*y + mass*z*z;
   inertia[0][1] = Icom[0][1] - mass*x*y;
   inertia[0][2] = Icom[0][2] - mass*x*z;
   inertia[1][0] = Icom[1][0] - mass*x*y;
   inertia[1][1] = Icom[1][1] + mass*x*x + mass*z*z;
   inertia[1][2] = Icom[1][2] - mass*y*z;
   inertia[2][0] = Icom[2][0] - mass*x*z;
   inertia[2][1] = Icom[2][1] - mass*y*z;
   inertia[2][2] = Icom[2][2] + mass*x*x + mass*y*y;
   /* Top right */
   inertia[0][4] = -mass*z;
   inertia[0][5] =  mass*y;
   inertia[1][3] =  mass*z;
   inertia[1][5] = -mass*x;
   inertia[2][3] = -mass*y;
   inertia[2][4] =  mass*x;
   /* Bottom left */
   inertia[3][1] =  mass*z;
   inertia[3][2] = -mass*y;
   inertia[4][0] = -mass*z;
   inertia[4][2] =  mass*x;
   inertia[5][0] =  mass*y;
   inertia[5][1] = -mass*x;
   /* Bottom right */
   inertia[3][3] = mass;
   inertia[4][4] = mass;
   inertia[5][5] = mass;
   return 0;
}

int cd_spatial_inertia_to_com(double inertia[6][6], double * massp, double com[3], double Icom[3][3])
{
   double x, y, z;
   double mass;
   /* mass */
   mass = (inertia[3][3] + inertia[4][4] + inertia[5][5]) / 3.0;
   /* com location */
   x = 0.0;
   y = 0.0;
   z = 0.0;
                       z -= inertia[0][4]; y += inertia[0][5];
   z += inertia[1][3];                     x -= inertia[1][5];
   y -= inertia[2][3]; x += inertia[2][4];
                       z += inertia[3][1]; y -= inertia[3][2];
   z -= inertia[4][0];                     x += inertia[4][2];
   y += inertia[5][0]; x -= inertia[5][1];
   x /= 4.0 * mass;
   y /= 4.0 * mass;
   z /= 4.0 * mass;
   /* Icom */
   Icom[0][0] = inertia[0][0] - mass*y*y-+ mass*z*z;
   Icom[0][1] = inertia[0][1] + mass*x*y;
   Icom[0][2] = inertia[0][2] + mass*x*z;
   Icom[1][0] = inertia[1][0] + mass*x*y;
   Icom[1][1] = inertia[1][1] - mass*x*x - mass*z*z;
   Icom[1][2] = inertia[1][2] + mass*y*z;
   Icom[2][0] = inertia[2][0] + mass*x*z;
   Icom[2][1] = inertia[2][1] + mass*y*z;
   Icom[2][2] = inertia[2][2] - mass*x*x - mass*y*y;
   /* save */
   com[0] = x;
   com[1] = y;
   com[2] = z;
   *massp = mass;
   return 0;
}

int cd_spatial_inertia_sphere_solid(double pos[3], double mass, double radius, double inertia[6][6])
{
   double Ielem;
   double Icom[3][3];
   Ielem = 0.4 * mass * radius * radius;
   Icom[0][0] = Ielem; Icom[0][1] =   0.0; Icom[0][2] =   0.0;
   Icom[1][0] =   0.0; Icom[1][1] = Ielem; Icom[1][2] =   0.0;
   Icom[2][0] =   0.0; Icom[2][1] =   0.0; Icom[2][2] = Ielem;
   return cd_spatial_inertia_from_com(inertia, mass, pos, Icom);
}

int cd_spatial_vxIv(double v[6], double I[6][6], double result[6])
{
   double Iv[6];
   cblas_dgemv(CblasRowMajor, CblasNoTrans, 6, 6,
      1.0, &I[0][0],6, v,1, 0.0,Iv,1);
   cd_mat_cross(v, Iv, result);
   cd_mat_cross(v+3, Iv+3, result);
   cd_mat_cross(v, Iv+3, result+3);
   return 0;
}

int cd_spatial_spring_damper(double pose[7], double vel[6],
   double pose_ref[7], double vel_ref[6], double force[6],
   double Klin, double Blin, double Kang, double Bang)
{
   double cx, cy, cz;
   double cqx, cqy, cqz, cqw;
   double cwx, cwy, cwz;
   double cvx, cvy, cvz;
   double rx, ry, rz;
   double rqx, rqy, rqz, rqw;
   double rwx, rwy, rwz;
   double rvx, rvy, rvz;
   
   double rwqx, rwqy, rwqz, rwqw;
   double rcqx, rcqy, rcqz, rcqw;
   double angle, denom;
   double raax, raay, raaz;
   double qx2, qy2, qz2, qw2, qxqy, qxqz, qxqw, qyqz, qyqw, qzqw;
   double waax, waay, waaz;
   
   double fnx, fny, fnz, fx, fy, fz;
   
   /* Get current position[ orientation[ angular & linear velocity */
   cx = pose[0];
   cy = pose[1];
   cz = pose[2];
   cqx = pose[3];
   cqy = pose[4];
   cqz = pose[5];
   cqw = pose[6];
   cwx = vel[0];
   cwy = vel[1];
   cwz = vel[2];
   cvx = vel[3] + cz*cwy - cy*cwz;
   cvy = vel[4] - cz*cwx + cx*cwz;
   cvz = vel[5] + cy*cwx - cx*cwy;
   
   /* Get reference position[ orientation[ angular & linear velocity */
   rx = pose_ref[0];
   ry = pose_ref[1];
   rz = pose_ref[2];
   rqx = pose_ref[3];
   rqy = pose_ref[4];
   rqz = pose_ref[5];
   rqw = pose_ref[6];
   if (vel_ref)
   {
      rwx = vel_ref[0];
      rwy = vel_ref[1];
      rwz = vel_ref[2];
      rvx = vel_ref[3] + rz*rwy - ry*rwz;
      rvy = vel_ref[4] - rz*rwx + rx*rwz;
      rvz = vel_ref[5] + ry*rwx - rx*rwy;
   }
   else
   {
      rwx = 0.0;
      rwy = 0.0;
      rwz = 0.0;
      rvx = 0.0;
      rvy = 0.0;
      rvz = 0.0;
   }
   
   /* Negate reference orientation (now world orientation in ref frame) */
   rwqx = -rqx;
   rwqy = -rqy;
   rwqz = -rqz;
   rwqw =  rqw;
   /* Compose reference quaternion with current quaternion
    * (now is orientation of current in reference frame) */
   rcqx = rwqw*cqx + rwqx*cqw + rwqy*cqz - rwqz*cqy;
   rcqy = rwqw*cqy - rwqx*cqz + rwqy*cqw + rwqz*cqx;
   rcqz = rwqw*cqz + rwqx*cqy - rwqy*cqx + rwqz*cqw;
   rcqw = rwqw*cqw - rwqx*cqx - rwqy*cqy - rwqz*cqz;
   if (rcqw >= 1.0) rcqw = 1.0; /* This is needed in case input quaternions are not perfectly normalized to avoid nan bug */
   /* Convert quaterion to axis/angle */
   denom = sqrt(1.0 - rcqw*rcqw);
   if (denom == 0.0)
   {
      raax = 0.0;
      raay = 0.0;
      raaz = 0.0;
   }
   else
   {
      angle = 2.0 * acos(rcqw);
      raax = angle * rcqx / denom;
      raay = angle * rcqy / denom;
      raaz = angle * rcqz / denom;
   }
   /* Rotate this axis/angle vector to world frame */
   /* Rotate xyz_bc into a frame, rotate by rq */
   qx2 = rqx*rqx;
   qy2 = rqy*rqy;
   qz2 = rqz*rqz;
   qw2 = rqw*rqw;
   qxqy = rqx*rqy;
   qxqz = rqx*rqz;
   qxqw = rqx*rqw;
   qyqz = rqy*rqz;
   qyqw = rqy*rqw;
   qzqw = rqz*rqw;
   waax = raax*(qx2-qy2-qz2+qw2) + 2*raay*(qxqy-qzqw) + 2*raaz*(qxqz+qyqw);
   waay = 2*raax*(qxqy+qzqw) + raay*(-qx2+qy2-qz2+qw2) + 2*raaz*(qyqz-qxqw);
   waaz = 2*raax*(qxqz-qyqw) + 2*raay*(qyqz+qxqw) + raaz*(-qx2-qy2+qz2+qw2);
   
   /* Apply spatial force */
   fx = -Klin * (cx - rx) - Blin * (cvx - rvx);
   fy = -Klin * (cy - ry) - Blin * (cvy - rvy);
   fz = -Klin * (cz - rz) - Blin * (cvz - rvz);
   fnx = -Kang * (waax) - Bang * (cwx - rwx) - cz*fy + cy*fz;
   fny = -Kang * (waay) - Bang * (cwy - rwy) + cz*fx - cx*fz;
   fnz = -Kang * (waaz) - Bang * (cwz - rwz) - cy*fx + cx*fy;
   
   /* Save to force */
   force[0] += fnx;
   force[1] += fny;
   force[2] += fnz;
   force[3] += fx;
   force[4] += fy;
   force[5] += fz;
   
   return 0;
}

int cd_spatial_mat_bullet(double m[6][10], double v[6])
{
   /* First column */
   m[3][0] += v[3];
   m[4][0] += v[4];
   m[5][0] += v[5];
   /* Upper-left 3x3 */
   m[0][2] += v[5];
   m[0][3] += - v[4];
   m[1][1] += - v[5];
   m[1][3] += v[3];
   m[2][1] += v[4];
   m[2][2] += - v[3];
   /* Lower-left 3x3 */
   m[3][2] += - v[2];
   m[3][3] += v[1];
   m[4][1] += v[2];
   m[4][3] += - v[0];
   m[5][1] += - v[1];
   m[5][2] += v[0];
   /* Upper-right 3x6 */
   m[0][4] += v[0];
   m[0][5] += v[1];
   m[0][6] += v[2];
   m[1][5] += v[0];
   m[1][7] += v[1];
   m[1][8] += v[2];
   m[2][6] += v[0];
   m[2][8] += v[1];
   m[2][9] += v[2];
   return 0;
}

/* aka m = [vx*] */
int cd_spatial_mat_crossf(double m[6][6], double v[6])
{
   cd_mat_set_zero(&m[0][0],6,6);
   /* Upper Left */
   m[0][1] = -v[2];
   m[0][2] =  v[1];
   m[1][0] =  v[2];
   m[1][2] = -v[0];
   m[2][0] = -v[1];
   m[2][1] =  v[0];
   /* Bottom Right */
   m[3][4] = -v[2];
   m[3][5] =  v[1];
   m[4][3] =  v[2];
   m[4][5] = -v[0];
   m[5][3] = -v[1];
   m[5][4] =  v[0];
   /* Upper Right */
   m[0][4] = -v[5];
   m[0][5] =  v[4];
   m[1][3] =  v[5];
   m[1][5] = -v[3];
   m[2][3] = -v[4];
   m[2][4] =  v[3];
   return 0;
}
