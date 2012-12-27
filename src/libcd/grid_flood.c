/** \file grid_flood.c
 * \brief Implementation of cd_grid_flood, a flood-fill implementation.
 * \author Christopher Dellin
 * \date 2011-11-21: Initial implementation (from orikgroup)
 * \date 2012-07-15: Incorporated into libcd
 */

/* (C) Copyright 2011-2012 Carnegie Mellon University */

#include <stdlib.h>
#include "grid.h"
#include "grid_flood.h"

int cd_grid_flood_fill(struct cd_grid * g, size_t index_start,
   int * wrap_dim,
   int (*replace)(void *, void *), void * rptr)
{
   /* we maintain a stack of yet-to-be-visited adjacent cells */
   struct stack
   {
      struct stack * next;
      size_t index;
   };
   
   int ret;
   struct stack * s;
   struct stack * popped;
   size_t index;
   int ni;
   int pm;
   int * subs;
   int sub_save;
   
   subs = (int *) malloc(g->n * sizeof(int));
   if (!subs) return -1;
   
   /* Start with a stack with one node */
   s = (struct stack *) malloc(sizeof(struct stack));
   if (!s) { free(subs); return -1; }
   s->next = 0;
   s->index = index_start;
   
   /* Flood fill! */
   ret = 0;
   while (s)
   {
      /* Pop */
      popped = s;
      s = s->next;
      index = popped->index;
      free(popped);
      
      /* Attempt replace (continue if failed) */
      if (!replace(cd_grid_get_index(g,index), rptr)) continue;
      
      cd_grid_index_to_subs(g, index, subs);
      
      /* Add neighbors to stack */
      for (ni=0; ni<g->n; ni++)
      for (pm=0; pm<2; pm++)
      {
         sub_save = subs[ni];
         subs[ni] += pm==0 ? -1 : 1;
         /* bail if over the edge and not wrapping this dim */
         if ((subs[ni] < 0 || subs[ni] >= g->sizes[ni]) && (!wrap_dim || !wrap_dim[ni]))
         {
            subs[ni] = sub_save;
            continue;
         }
         /* wrap if necessary */
         if (subs[ni] < 0) subs[ni] += g->sizes[ni];
         if (subs[ni] >= g->sizes[ni]) subs[ni] -= g->sizes[ni];
         cd_grid_index_from_subs(g, &index, subs);
         subs[ni] = sub_save;
         
         popped = (struct stack *) malloc(sizeof(struct stack));
         if (!popped) { ret = -1; goto do_return; }
         popped->next = s;
         popped->index = index;
         s = popped;
      }
   }
   
do_return:
   /* free any remaining stack elements */
   while (s)
   {
      popped = s;
      s = s->next;
      free(popped);
   }
   
   free(subs);
   return ret;
}
