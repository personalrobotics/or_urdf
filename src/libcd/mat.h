/** \file mat.h
 * \brief Interface to cd_mat, a collection of useful matrix routines.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

#ifndef CD_MAT_H
#define CD_MAT_H

#include <stdio.h>

int cd_mat_set_zero(double * A, int m, int n);
int cd_mat_set_diag(double * A, int m, int n, double value);
int cd_mat_transpose(double * A, int mn);
int cd_mat_fill(double * A, int m, int n, ...);
int cd_mat_memcpy(double * const dst, const double * const src, const int m, const int n);
int cd_mat_memcpy_transpose(double * dst, double * src, int m, int n);
int cd_mat_add(double * dst, double * src, int m, int n);
int cd_mat_sub(double * dst, double * src, int m, int n);
int cd_mat_scale(double * A, int m, int n, double fac);

double cd_mat_trace(double * A, int m, int n);

int cd_mat_cross(double a[3], double b[3], double res[3]);

/* THIS IS UGLY! */
char * cd_mat_vec_sprintf(char * buf, double * a, int n);

int cd_mat_vec_fprintf(FILE * stream, const char * prefix,
   const double * a, int n, const char * start, const char * dblfmt,
   const char * sep, const char * end, const char * suffix);

int cd_mat_vec_print(const char * prefix, const double * a, int n);

#endif /* CD_MAT_H */
