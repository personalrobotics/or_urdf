/** \file grid.c
 * \brief Implementation of cd_grid, a multidimensional grid of values.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "grid.h"

/* Rectgrid, standard c-ordered indexing into nd grid:
 * grid(x,y,z) = grid[x][y][z] = grid[x*NY*NZ + y*NZ + z] */
int cd_grid_create(struct cd_grid ** gp,
   void * cell_init, int cell_size, int n, ...)
{
   size_t i;
   va_list argp;
   int * sizes;
   
   /* Fill sizes from n variadic arguments */
   sizes = (int *) malloc(n * sizeof(int));
   if (!sizes) { return -1; }
   va_start(argp, n);
   for (i=0; i<n; i++)
      sizes[i] = va_arg(argp, int);
   va_end(argp);
   
   return cd_grid_create_sizeown(gp, cell_init, cell_size, n, sizes);
}

int cd_grid_create_sizearray(struct cd_grid ** gp,
   void * cell_init, int cell_size, int n, int * sizes)
{
   int * new_sizes;
   new_sizes = (int *) malloc(n * sizeof(int));
   if (!new_sizes) { return -1; }
   memcpy(new_sizes, sizes, n * sizeof(int));
   return cd_grid_create_sizeown(gp, cell_init, cell_size, n, new_sizes);
}

int cd_grid_create_sizeown(struct cd_grid ** gp,
   void * cell_init, int cell_size, int n, int * sizes)
{
   size_t i;
   struct cd_grid * g;
   
   g = (struct cd_grid *) malloc(sizeof(struct cd_grid));
   if (!g) { free(sizes); return -1; }
   g->n = n;
   g->sizes = sizes;
   g->ncells = 0;
   g->cell_size = cell_size;
   g->data = 0;
   g->lengths = 0;
   
   /* Allocate cell data */
   g->ncells = 1;
   for (i=0; i<n; i++)
      g->ncells *= g->sizes[i];
   if (cell_size > 0)
   {
      g->data = (char *) malloc(g->ncells * cell_size);
      if (!g->data) { cd_grid_destroy(g); return -2; }
      
      /* Initialize cell data */
      for (i=0; i<g->ncells; i++)
         memcpy(g->data + i*cell_size, cell_init, cell_size);
   }
   
   /* Allocate lengths vector */
   g->lengths = (double *) malloc(n*sizeof(double));
   if (!g->lengths) { cd_grid_destroy(g); return -1; }
   for (i=0; i<n; i++) g->lengths[i] = 1.0;
   
   *gp = g;
   return 0;
}

int cd_grid_create_copy(struct cd_grid ** gp,
   struct cd_grid * gsrc)
{
   struct cd_grid * g;
   
   g = (struct cd_grid *) malloc(sizeof(struct cd_grid));
   if (!g) return -1;
   g->n = gsrc->n;
   g->sizes = 0;
   g->ncells = gsrc->ncells;
   g->cell_size = gsrc->cell_size;
   g->data = 0;
   
   /* Fill sizes from n variadic arguments */
   g->sizes = (int *) malloc(gsrc->n * sizeof(int));
   if (!g->sizes) { cd_grid_destroy(g); return -1; }
   memcpy(g->sizes, gsrc->sizes, gsrc->n * sizeof(int));
   
   /* Allocate cell data */
   if (g->cell_size > 0)
   {
      g->data = (char *) malloc(g->ncells * g->cell_size);
      if (!g->data) { cd_grid_destroy(g); return -2; }
      memcpy(g->data, gsrc->data, g->ncells * g->cell_size);
   }
   
   /* Allocate lengths vector */
   g->lengths = (double *) malloc(gsrc->n*sizeof(double));
   if (!g->lengths) { cd_grid_destroy(g); return -1; }
   memcpy(g->lengths, gsrc->lengths, gsrc->n*sizeof(double));
   
   *gp = g;
   return 0;
}

int cd_grid_destroy(struct cd_grid * g)
{
   free(g->data);
   free(g->sizes);
   free(g->lengths);
   free(g);
   return 0;
}

int cd_grid_index_to_subs(struct cd_grid * g, size_t index, int * subs)
{
   int ni;
   size_t index_remain;

   index_remain = index;
   for (ni=g->n-1; ni>=0; ni--)
   {
      subs[ni] = index_remain % g->sizes[ni];
      index_remain = index_remain / g->sizes[ni];
   }

   return 0;
}

int cd_grid_index_from_subs(struct cd_grid * g, size_t * index, int * subs)
{
   int ni;
   *index = subs[0];
   for (ni=1; ni<g->n; ni++)
   {
      *index *= g->sizes[ni];
      *index += subs[ni];
   }
   return 0;
}

int cd_grid_center_index(struct cd_grid * g,
   size_t index, double * center)
{
   int ni;
   int sub;
   size_t index_remain;
   
   index_remain = index;
   for (ni=g->n-1; ni>=0; ni--)
   {
      sub = index_remain % g->sizes[ni];
      index_remain = index_remain / g->sizes[ni];
      center[ni] = (0.5 + sub) / g->sizes[ni];
   }
   
   for (ni=0; ni<g->n; ni++) center[ni] *= g->lengths[ni];
   return 0;
}

int cd_grid_lookup_index(struct cd_grid * g,
   double * p, size_t * index)
{
   int ni;
   double x;
   int sub;
   *index = 0;
   for (ni=0; ni<g->n; ni++)
   {
      x = p[ni] / g->lengths[ni];
      if (x < 0.0) return 1;
      if (x > 1.0) return 1;
      sub = (int) floor(x * g->sizes[ni]);
      if (sub == g->sizes[ni]) sub--;
      *index *= g->sizes[ni];
      *index += sub;
   }
   return 0;
}

int cd_grid_lookup_subs(struct cd_grid * g,
   double * p, int * subs)
{
   int ni;
   int sub;
   double x;
   for (ni=0; ni<g->n; ni++)
   {
      x = p[ni] / g->lengths[ni];
      if (x < 0.0) return 1;
      if (x > 1.0) return 1;
      sub = (int) floor(x * g->sizes[ni]);
      if (sub == g->sizes[ni]) sub--;
      subs[ni] = sub;
   }
   return 0;
}

void * cd_grid_get_index(struct cd_grid * g, size_t index)
{
   return g->data + index*g->cell_size;
}

void * cd_grid_get_subs(struct cd_grid * g, int * subs)
{
   int ni;
   size_t index;
   index = subs[0];
   for (ni=1; ni<g->n; ni++)
   {
      index *= g->sizes[ni];
      index += subs[ni];
   }
   return g->data + index*g->cell_size;
}

void * cd_grid_get(struct cd_grid * g, ...)
{
   int ni;
   va_list argp;
   size_t index;
   va_start(argp, g);
   index = va_arg(argp, int);
   for (ni=1; ni<g->n; ni++)
   {
      index *= g->sizes[ni];
      index += va_arg(argp, int);
   }
   va_end(argp);
   return g->data + index*g->cell_size;
}


static int sedt_onedim(int n, double * func, double * trans, int trans_stride,
   int * v, double * z)
{
   int i;
   int np; /* np = num parabolas = k+1 */
   int q;
   double s;
   
   np = 0;
   
   /* Consider each cell, compute lower envelope */
   for (q=0; q<n; q++)
   {
      /* Ignore infinite-height parabolas */
      if (func[q] == HUGE_VAL)
         continue;
      
      if (np == 0)
      {
         np = 1;
         v[0] = q;
         z[0] = -HUGE_VAL;
         z[1] = HUGE_VAL;
         continue;
      }
      
      /* Knock off as many right-most parabolas as possible */
   retry:
      /* Compute s (intersection point) with rightmost parabola */
      s = func[q] + q*q;
      s -= func[v[np-1]] + v[np-1]*v[np-1];
      s /= 2.0 * (q - v[np-1]);
      if (s <= z[np-1])
      {
         np--;
         goto retry;
      }

      np++;
      v[np-1] = q;
      z[np-1] = s;
      z[np] = HUGE_VAL;
   }
   
   /* Fill in values */
   if (np == 0)
   {
      for (i=0; i<n; i++) trans[i*trans_stride] = HUGE_VAL;
      return 0;
   }
   
   np = 0;
   for (q=0; q<n; q++)
   {
      while (z[np+1] < q)
         np++;
      trans[q*trans_stride] = pow(q-v[np],2.0) + func[v[np]];
   }
   
   return 0;
}

int cd_grid_double_grad(struct cd_grid * g,
   double * p, double * grad)
{
   int err;
   int ni;
   size_t stride;
   size_t index;
   size_t index_remain;
   int sub;
   double center;
   double diff;
   
   /* Get index of the grid point we're in */
   err = cd_grid_lookup_index(g, p, &index);
   if (err) return err;
   
   /* Each dimension is independent;
    * Go in reverse order, accumulating stride as you go,
    * also calculating (for comparison) center as you go */
   stride = 1;
   index_remain = index;
   for (ni=g->n-1; ni>=0; ni--)
   {
      /* Get subscript, decumulate index_remain */
      sub = index_remain % g->sizes[ni];
      index_remain = index_remain / g->sizes[ni];
      /* Decide whether to use previous or next cell */
      if (sub == 0) goto use_next;
      if (sub == g->sizes[ni]-1) goto use_previous;
      /* Get center location */
      center = (0.5 + sub) / g->sizes[ni] * g->lengths[ni];
      if (p[ni] < center)
         goto use_previous;
      else
         goto use_next;
      /* Calculate the difference */
   use_previous:
      /* Use previous cell for gradient */
      diff = *(double *)cd_grid_get_index(g,index);
      diff -= *(double *)cd_grid_get_index(g,index - stride);
      goto save_continue;
   use_next:
      /* Use next cell for gradient */
      diff = *(double *)cd_grid_get_index(g,index + stride);
      diff -= *(double *)cd_grid_get_index(g,index);
      goto save_continue;
      /* Save to the grad vector, accumulate the stride */
   save_continue:
      grad[ni] = diff * g->sizes[ni] / g->lengths[ni];
      stride *= g->sizes[ni];
   }
   
   return 0;
}

int cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep)
{
   int err;
   int ni;
   size_t stride;
   size_t index;
   size_t index_remain;
   int sub;
   double center;
   double diff;
   double value;
   double grad;
   double value_after;
   double value_before;
   
   /* First, look up the value at the nearest center */
   err = cd_grid_lookup_index(g, p, &index);
   if (err) return err;
   value = *(double *)cd_grid_get_index(g,index);
   if (value == HUGE_VAL)
      { *valuep = HUGE_VAL; return 0; }
   
   /* This is identical to the gradient algorithm from above */
   stride = 1;
   index_remain = index;
   for (ni=g->n-1; ni>=0; ni--)
   {
      /* Get subscript, decumulate index_remain */
      sub = index_remain % g->sizes[ni];
      index_remain = index_remain / g->sizes[ni];
      /* Get center location */
      center = (0.5 + sub) / g->sizes[ni] * g->lengths[ni];
      /* Decide whether to use previous or next cell */
      if (sub == 0) goto use_next;
      if (sub == g->sizes[ni]-1) goto use_previous;
      if (p[ni] < center)
         goto use_previous;
      else
         goto use_next;
      /* Calculate the difference */
   use_previous:
      /* Use previous cell for gradient */
      value_after = *(double *)cd_grid_get_index(g,index);
      value_before = *(double *)cd_grid_get_index(g,index - stride);
      if (value_after == HUGE_VAL || value_before == HUGE_VAL)
         { *valuep = HUGE_VAL; return 0; }
      diff = value_after;
      diff -= value_before;
      goto save_continue;
   use_next:
      /* Use next cell for gradient */
      value_after = *(double *)cd_grid_get_index(g,index + stride);
      value_before = *(double *)cd_grid_get_index(g,index);
      if (value_after == HUGE_VAL || value_before == HUGE_VAL)
         { *valuep = HUGE_VAL; return 0; }
      diff = value_after;
      diff -= value_before;
      goto save_continue;
      /* Save to the grad vector, accumulate the stride */
   save_continue:
      /* Adjust the value based on the gradient */
      grad = diff * g->sizes[ni] / g->lengths[ni];
      value += grad * (p[ni] - center);
      stride *= g->sizes[ni];
   }
   
   *valuep = value;
   return 0;
}

int cd_grid_double_sedt(struct cd_grid ** gp, struct cd_grid * funcg)
{
   int i;
   int ret;
   int n;
   int err;
   int ni;
   struct cd_grid * g;
   int * subs;
   int ni2;
   double extent;
   double sizedivextent2;
   int * v;
   double * z;
   double * func;
   int dim_n;
   int dim_stride;
   double * trans;
   
   ret = 0;
   
   n = funcg->n;
   
   subs = 0;
   v = 0;
   z = 0;
   func = 0;
   
   /* Allocate space to keep n+1 subscripts
    * (the msb flips from 0 to 1 when we're done) */
   subs = (int *) malloc((n+1) * sizeof(int));
   if (!subs) { ret = -1; goto error; }
   
   /* Start with copy of sampled function */
   err = cd_grid_create_copy(&g, funcg);
   if (err) { ret = -1; goto error; }
   
   /* Operate the 1-dimensional squared euclidean distance transform
    * over each dimension in turn, updating the values for each slice */
   for (ni=0; ni<n; ni++)
   {
      /* Start out with all subscripts 0 */
      for (ni2=0; ni2<n+1; ni2++) subs[ni2] = 0;
      
      /* Set slice size (length of this dimension) */
      dim_n = g->sizes[ni];
      
      /* Calculate slice stride (product of all smaller dimension sizes) */
      dim_stride = 1;
      for (ni2=ni+1; ni2<n; ni2++)
         dim_stride *= g->sizes[ni2];
      
      /* Grab extent for this dimension */
      extent = funcg->lengths[ni];
      sizedivextent2 = pow(g->sizes[ni]/extent, 2.0);
      
      /* Set up temp arrays for the 1d transform */
      v = (int *) malloc(dim_n * sizeof(int));
      z = (double *) malloc((dim_n + 1) * sizeof(double));
      func = (double *) malloc(dim_n*sizeof(double));
      if (!v || !z || !func) { ret = -1; goto error; }
      
      /* Loop until we're done (the msb subscript flips) */
      while (subs[n]==0)
      {
         /* Get slice data pointer (first element) */
         trans = (double *)cd_grid_get_subs(g, subs);
         
         /* Perform 1d transform (scale by extent) */
         /* XXX BUG:
          * Scaling HUGE_VAL is not guarenteed to maintain HUGE_VAL
          * (e.g. on systems with no IEEE infinitiy) */
         for (i=0; i<dim_n; i++) func[i] = sizedivextent2 * trans[i*dim_stride];
         sedt_onedim(dim_n, func, trans, dim_stride, v, z);
         for (i=0; i<dim_n; i++) trans[i*dim_stride] /= sizedivextent2;
         
         /* Increment subscript, do carries
          * (don't increment ni1, keep it at 0) */
         subs[0]++;
         for (ni2=0; ni2<n; ni2++)
         {
            if (subs[ni2] == ((ni2==ni) ? 1 : g->sizes[ni2]))
            {
               subs[ni2+1]++;
               subs[ni2] = 0;
            }
         }
         
         /* Continue! */
      }
      
      free(v);
      free(z);
      free(func);
      v = 0;
      z = 0;
      func = 0;
   }
   
error:
   free(func);
   free(v);
   free(z);
   free(subs);
   if (ret == 0)
      *gp = g;
   else if (g)
      cd_grid_destroy(g);
   return ret;
}

int cd_grid_double_bin_sdf(struct cd_grid ** g_sdfp, struct cd_grid * g_obs)
{
   int ret;
   int err;
   struct cd_grid * g_vox_emp;
   struct cd_grid * g_vox_obs;
   struct cd_grid * g_sedt_emp;
   struct cd_grid * g_sedt_obs;
   struct cd_grid * g_sdf;
   size_t index;
   
   ret = 0;
   
   /* g_vox_emp is 0.0 in free space, HUGE_VAL elsewhere */
   g_vox_emp = g_obs;
   
   /* g_vox_obs is 0.0 in obstacles, HUGE_VAL elsewhere */
   err = cd_grid_create_copy(&g_vox_obs, g_vox_emp);
   if (err) { ret = -1; goto ret_after_g_vox_obs; }
   for (index=0; index<g_vox_emp->ncells; index++)
   {
      *(double *)cd_grid_get_index(g_vox_obs,index) =
         (*(double *)cd_grid_get_index(g_vox_emp,index) == 0.0) ? HUGE_VAL : 0.0;
   }
   
   /* Compute the Squared Euclidean Distance Transform of each voxel grid */
   err = cd_grid_double_sedt(&g_sedt_emp, g_vox_emp);
   if (err) { ret = -1; goto ret_after_g_sedt_emp; }
   err = cd_grid_double_sedt(&g_sedt_obs, g_vox_obs);
   if (err) { ret = -1; goto ret_after_g_sedt_obs; }
   
   /* Create signed distance field grid (obs-emp) */
   err = cd_grid_create_copy(&g_sdf, g_sedt_obs);
   if (err) { ret = -1; goto ret_error; }
   for (index=0; index<g_sdf->ncells; index++)
   {
      *(double *)cd_grid_get_index(g_sdf, index) =
         sqrt(*(double *)cd_grid_get_index(g_sdf, index)) - 
         sqrt(*(double *)cd_grid_get_index(g_sedt_emp, index));
   }
   
ret_error:
   cd_grid_destroy(g_sedt_obs);
ret_after_g_sedt_obs:
   cd_grid_destroy(g_sedt_emp);
ret_after_g_sedt_emp:
   cd_grid_destroy(g_vox_obs);
ret_after_g_vox_obs:
   if (ret == 0) *g_sdfp = g_sdf;
   return ret;
}
