/** \file os.c
 * \brief Implementation of cd_os, a collection of os routines.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

/* note: we need POSIX.1b support in time.h */
#include <time.h>
#include "os.h"

int cd_os_timespec_set_zero(struct timespec * t)
{
   t->tv_sec = 0;
   t->tv_nsec = 0;
   return 0;
}

int cd_os_timespec_add(struct timespec * dst, const struct timespec * src)
{
   dst->tv_sec += src->tv_sec;
   dst->tv_nsec += src->tv_nsec;
   if (dst->tv_nsec > 999999999)
   {
      dst->tv_sec += 1;
      dst->tv_nsec -= 1000000000;
   }
   return 0;
}

int cd_os_timespec_sub(struct timespec * dst, const struct timespec * src)
{
   dst->tv_sec -= src->tv_sec;
   dst->tv_nsec -= src->tv_nsec;
   if (dst->tv_nsec < 0)
   {
      dst->tv_sec -= 1;
      dst->tv_nsec += 1000000000;
   }
   return 0;
}

double cd_os_timespec_double(const struct timespec * src)
{
   return src->tv_sec + ((double)(src->tv_nsec))/1000000000.0;
}
