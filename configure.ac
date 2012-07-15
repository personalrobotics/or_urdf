AC_INIT([orcdchomp], [r1], [cdellin@cs.cmu.edu])
AC_CONFIG_AUX_DIR([ac-scripts])
AM_INIT_AUTOMAKE
AC_PROG_CXX
AC_PROG_LIBTOOL

AC_ARG_WITH([dep-prefix],
   [AS_HELP_STRING([--with-dep-prefix],
      [path to dependency prefix (e.g. /usr/local; can be specified multiple times)])],
   [
      PATH="${withval}/bin$PATH_SEPARATOR$PATH";
      CFLAGS="$CFLAGS ${withval}/include";
      LDFLAGS="$LDFLAGS ${withval}/lib";
red      CXXFLAGS="$CXXFLAGS ${withval}/include";
   ])

# openrave include path from openrave-config
AC_CHECK_PROG(has_openrave, openrave-config, yes, no)
AS_IF([test "x$has_openrave" != xyes], [AC_MSG_ERROR([openrave-config not found])])
CXXFLAGS="$CXXFLAGS `openrave-config --cflags-only-I`";

# check for libcd chomp
AC_CHECK_HEADER(libcd/chomp.h, [], [AC_MSG_ERROR([libcd not found!])])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
