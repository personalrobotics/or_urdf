/** \file orcwrap.cpp
 * \brief Implementation of orcwrap, an OpenRAVE interface command parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

#include <istream>
#include <ostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <openrave/openrave.h>

extern "C" {
#include "libcd/util_shparse.h"
}

#include "orcwrap.h"

static bool orcwrap_call(
   const char * cmd,
   boost::function<int (int, char * [], std::ostream&)> fn,
   std::ostream& sout, std::istream& sinput)
{
   char * in;
   int argc;
   char ** argv;
   int ret;
   
   std::ostringstream oss;
   oss << cmd << " " << sinput.rdbuf();
   in = (char *) malloc(strlen(oss.str().c_str())+1);
   if (!in) return false;
   strcpy(in, oss.str().c_str());
   
   cd_util_shparse(in, &argc, &argv);
   
   try
   {
      ret = fn(argc, argv, sout);
   }
   catch (...)
   {
      free(in);
      free(argv);
      throw;
   }
   
   free(in);
   free(argv);
   return (ret == 0) ? true : false;
}

boost::function<bool (std::ostream&, std::istream&)>
orcwrap(boost::function<int (int, char * [], std::ostream&)> fn)
{
   return boost::bind(orcwrap_call,"openrave_command",fn,_1,_2);
}

boost::function<bool (std::ostream&, std::istream&)>
orcwrap(const char * cmd, boost::function<int (int, char * [], std::ostream&)> fn)
{
   return boost::bind(orcwrap_call,cmd,fn,_1,_2);
}

