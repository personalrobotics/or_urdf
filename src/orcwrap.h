/** \file orcwrap.h
 * \brief Interface to orcwrap, an OpenRAVE interface command parser.
 * \author Christopher Dellin
 * \date 2012
 */

/* (C) Copyright 2012 Carnegie Mellon University */

/* requires:
 * #include <istream>
 * #include <ostream>
 * #include <boost/function.hpp>
 */

/* This is a tiny generic library which parses OpenRAVE interface command
 * strings into an argument array using shell-like quoting
 * (using libcd's util_shparse) */

boost::function<bool (std::ostream&, std::istream&)>
   orcwrap(boost::function<int (int, char * [], std::ostream&)> fn);

boost::function<bool (std::ostream&, std::istream&)>
   orcwrap(const char * cmd, boost::function<int (int, char * [], std::ostream&)> fn);
