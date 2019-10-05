# Process this file with autoconf to produce a configure script.
AC_PREREQ([2.63])
AC_INIT([RVO],[1.1],[geom@cs.unc.edu])
AC_COPYRIGHT([Copyright (c) 2008 University of North Carolina at Chapel Hill. All rights reserved.])
AC_CONFIG_AUX_DIR([build-aux])
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([RVOLIB/RVOSimulator.cpp])
AC_CONFIG_HEADERS([config.h])
AM_INIT_AUTOMAKE([-Wall -Werror foreign])
AC_LANG([C++])

AC_ARG_ENABLE([build-docs],
  [AS_HELP_STRING([--enable-build-docs],
    [build Doxygen documentation @<:@default=no@:>@])],
  [try_doxygen=$enableval],
  [try_doxygen=no])

AC_ARG_ENABLE([openmp],
  [AS_HELP_STRING([--enable-openmp],
    [enable parallelization with OpenMP @<:@default=check@:>@])],
  [try_openmp=$enableval],
  [try_openmp=check])

AS_IF([test x"$try_openmp" != xno],
	    [AX_OPENMP([AC_SUBST([OPENMP_CXXFLAGS])], [AS_IF([test x"$try_openmp" != xcheck],
	    					       AC_MSG_ERROR([OpenMP explicitly requested but not found]),
						       [AC_MSG_WARN([can't find OpenMP])
						        HAVE_OPENMP=0])])],
	    [AC_MSG_WARN([skipping test for OpenMP])
	     HAVE_OPENMP=0])

# Checks for programs.
AM_PROG_AR
AC_PROG_CXX
AC_PROG_CXXCPP
AC_PROG_INSTALL
AC_PROG_MAKE_SET
AC_PROG_RANLIB

AC_PROG_TRY_DOXYGEN

if test -z "$DOXYGEN" -a x"$try_doxygen" != xno ; then
   echo "-----------------------------------------"
   echo " No Doxygen program found - continuing"
   echo " without Doxygen documentation support."
   echo "-----------------------------------------"
fi

AM_CONDITIONAL([HAVE_DOXYGEN], [test x"$try_doxygen" != xno -a -n "$DOXYGEN"])

# Checks for libraries.

# Checks for header files.
AC_HEADER_STDC

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.

AC_CONFIG_FILES([Makefile
		 doc/Doxyfile
		 doc/Makefile
		 RVOLIB/Makefile
		 example/Makefile])

AC_OUTPUT
echo "
  $PACKAGE_NAME version $PACKAGE_VERSION
  Prefix.........: $prefix"
if test -z $HAVE_OPENMP; then
echo "
  Build w/ OpenMP: yes
  OpenMP flag....: $OPENMP_CXXFLAGS"
else
echo "
  Build w/ OpenMP: no"
fi
if test x"$try_doxygen" != xno -a -n "$DOXYGEN"; then
echo "
  Building documentation: yes"
else
echo "
  Building documentation: no"
fi
echo "
  C++ Compiler...: $CXX $CXXFLAGS $CPPFLAGS $OPENMP_CXXFLAGS
  Linker.........: $LD $LDFLAGS $LIBS
"

