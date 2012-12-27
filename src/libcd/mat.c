/** \file mat.h
 * \brief Interface to cd_mat, a collection of useful matrix routines.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "mat.h"

int cd_mat_set_zero(double * A, int m, int n)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         A[i*n+j] = 0.0;
   return 0;
}

int cd_mat_set_diag(double * A, int m, int n, double value)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         A[i*n+j] = i==j ? value : 0.0;
   return 0;
}

int cd_mat_transpose(double * A, int mn)
{
   int i, j;
   double temp;
   for (i=0; i<mn; i++)
      for (j=i+1; j<mn; j++)
      {
         temp = A[i*mn+j];
         A[i*mn+j] = A[j*mn+i];
         A[j*mn+i] = temp;
      }
   return 0;
}

int cd_mat_fill(double * A, int m, int n, ...)
{
   int i, j;
   va_list ap;
   va_start(ap, n);
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         A[i*n+j] = va_arg(ap, double);
   va_end(ap);
   return 0;
}

int cd_mat_memcpy(double * const dst, const double * const src, const int m, const int n)
{
   memcpy(dst, src, m*n*sizeof(double));
   return 0;
}

int cd_mat_memcpy_transpose(double * dst, double * src, int m, int n)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         dst[i*n+j] = src[j*m+i];
   return 0;
}

int cd_mat_add(double * dst, double * src, int m, int n)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         dst[i*n+j] += src[i*n+j];
   return 0;
}

int cd_mat_sub(double * dst, double * src, int m, int n)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         dst[i*n+j] -= src[i*n+j];
   return 0;
}

int cd_mat_scale(double * A, int m, int n, double fac)
{
   int i, j;
   for (i=0; i<m; i++)
      for (j=0; j<n; j++)
         A[i*n+j] *= fac;
   return 0;
}

double cd_mat_trace(double * A, int m, int n)
{
   int i;
   double tr;
   tr = 0.0;
   for (i=0; i<m && i<n; i++)
      tr += A[i*n+i];
   return tr;
}

int cd_mat_cross(double a[3], double b[3], double res[3])
{
   res[0] += a[1]*b[2] - a[2]*b[1];
   res[1] += a[2]*b[0] - a[0]*b[2];
   res[2] += a[0]*b[1] - a[1]*b[0];
   return 0;
}

/* THIS IS UGLY! */
char * cd_mat_vec_sprintf(char * buf, double * a, int n)
{
   int j;
   buf[0] = '<';
   buf[1] = 0;
   for (j=0; j<n; j++)
      sprintf(buf+strlen(buf), "%8.4f,", a[j]);
   buf[strlen(buf)-1] = 0;
   strcat(buf,">");
   return buf;
}

int cd_mat_vec_fprintf(FILE * stream, const char * prefix,
   const double * a, int n, const char * start, const char * dblfmt,
   const char * sep, const char * end, const char * suffix)
{
   int j;
   FILE * x_stream = stream ? stream : stdout;
   const char * x_prefix = prefix ? prefix : "";
   const char * x_start = start ? start : "< ";
   const char * x_dblfmt = dblfmt ? dblfmt : "%8.4f";
   const char * x_sep = sep ? sep : ", ";
   const char * x_end = end ? end : " >";
   const char * x_suffix = suffix ? suffix : "\n";
   fputs(x_prefix, x_stream);
   fputs(x_start, x_stream);
   for (j=0; j<n; j++)
   {
      if (j>0) fputs(x_sep, x_stream);
      fprintf(x_stream, x_dblfmt, a[j]);
   }
   fputs(x_end, x_stream);
   fputs(x_suffix, x_stream);
   return 0;
}


int cd_mat_vec_print(const char * prefix, const double * a, int n)
{
   cd_mat_vec_fprintf(0, prefix, a, n, 0, 0, 0, 0, 0);
   return 0;
}
