/** \file grid.h
 * \brief Interface to cd_grid, a multidimensional grid of values.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

struct cd_grid
{
   /* Dimensionality of space */
   int n;
   /* Grid parameters */
   int * sizes;
   size_t ncells;
   /* The actual data */
   int cell_size;
   char * data;
   /* Actual grid side lengths (not 1x1x1...) */
   double * lengths;
};

int cd_grid_create(struct cd_grid ** gp, void * cell_init, int cell_size, int n, ...);
int cd_grid_create_sizearray(struct cd_grid ** gp, void * cell_init, int cell_size, int n, int * sizes);

/* note - this takes ownership of sizes, and will free it when the grid is destroyed! */
int cd_grid_create_sizeown(struct cd_grid ** gp, void * cell_init, int cell_size, int n, int * sizes);

int cd_grid_create_copy(struct cd_grid ** gp, struct cd_grid * gsrc);
int cd_grid_destroy(struct cd_grid * g);

/* convert between index and subs */
int cd_grid_index_to_subs(struct cd_grid * g, size_t index, int * subs);
int cd_grid_index_from_subs(struct cd_grid * g, size_t * index, int * subs);

/* Get center from index */
int cd_grid_center_index(struct cd_grid * g, size_t index, double * center);
   
/* Lookup cell index, subs from location
 * these will return 1 if it's out of range! */
int cd_grid_lookup_index(struct cd_grid * g, double * p, size_t * index);
int cd_grid_lookup_subs(struct cd_grid * g, double * p, int * subs);


/* Get cell from index, subs */
void * cd_grid_get_index(struct cd_grid * g, size_t index);
void * cd_grid_get_subs(struct cd_grid * g, int * subs);
void * cd_grid_get(struct cd_grid * g, ...);


/* Simple multilinear interpolation / discontinuous gradient
 * these will return 1 if the point is out of range! */
int cd_grid_double_interp(struct cd_grid * g, double * p, double * valuep);
int cd_grid_double_grad(struct cd_grid * g, double * p, double * grad);



/* Squared Euclidean Distance Transform
 * (see http://www.cs.cornell.edu/~dph/papers/dt.pdf)
 * Uses doubles! */
int cd_grid_double_sedt(struct cd_grid ** sedtgp, struct cd_grid * funcg);

/* Compute signed distance field */
int cd_grid_double_bin_sdf(struct cd_grid ** g_sdfp, struct cd_grid * g_obs);
