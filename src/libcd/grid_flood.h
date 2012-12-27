/** \file grid_flood.h
 * \brief Interface to cd_grid_flood, a flood-fill implementation.
 * \author Christopher Dellin
 * \date 2011-2012
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

/* requires:
 * #include <stdlib.h>
 * #include <libcd/grid.h>
 */

/* Modify the grid by performing a flood-fill starting with the given index.
 * Diagonal cells are considered adjacent (e.g. 8-connected in 2D).
 * By default, the grid does not wrap; if wrap_dim is passed, it should
 * have length grid.n, with non-zero elements denoting a wrapped dimension. */
int cd_grid_flood_fill(struct cd_grid * g, size_t index_start,
   int * wrap_dim,
   int (*replace)(void *, void *), void * rptr);
