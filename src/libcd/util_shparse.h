/** \file util_shparse.h
 * \brief Interface to cd_util_shparse, a simple POSIX shell argument parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Christopher Dellin */

/* Simple POSIX shell argument parsing;
 * that is, turns a string like:
 *   grep -v -e can -e "can't" -e 'Say "Hi!"' -e "do\\don't"
 * into an argv array like:
 *   grep, -v, -e, can, -e, can't, -e, Say "Hi!", -e, do\don't
 * Characters that have special meaning are:
 *   space characters, newline, backslash, double quote, single quote
 */

int cd_util_shparse(char * in, int * argcp, char *** argvp);
