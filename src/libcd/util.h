/** \file util.h
 * \brief Interface to cd_util, a set of general utility functions.
 * \author Christopher Dellin
 * \date 2010-2012
 */

/* (C) Copyright 2010-2012 Carnegie Mellon University */

#ifndef CD_UTIL_H
#define CD_UTIL_H

#define cd_util_round(in) floor((in) + 0.5)

void cd_util_exitmsg(int status, const char * templ, ...);

/* Note: This is inclusive on both ends! */
double cd_util_rand_double(double min, double max);

int cd_util_rand_int(unsigned int n);

#endif /* CD_UTIL_H */
