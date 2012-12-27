/** \file util.c
 * \brief Implementation of cd_util, a set of general utility functions.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include "util.h"

void cd_util_exitmsg(int status, const char * templ, ...)
{
   va_list ap;
   va_start(ap, templ);
   vfprintf((status==EXIT_SUCCESS)?stdout:stderr, templ, ap);
   exit(status);
   va_end(ap);
   return;
}

double cd_util_rand_double(double min, double max)
{
   return min + (max - min) * ((double) rand() / (double) RAND_MAX);
}

/* From http://c-faq.com/lib/rand.931117.html
 * scs@eskimo.com (Steve Summit) 
 * Wed, 17 Nov 1993 06:10:36 GMT
 * Newsgroups: comp.lang.c
 * Subject: Re: help with random #s! */
int cd_util_rand_int(unsigned int n)
{
   return (double) rand() / ((double) RAND_MAX + 1) * n;
}
