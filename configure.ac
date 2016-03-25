dnl Process this file with autogen.sh to produce a working configure
dnl script.

dnl Require at least automake 2.52
AC_PREREQ(2.52)

dnl Initialize the autoconf system (provide project name and version)
AC_INIT([nav-devel], [1.0])

dnl Sanity check of the distribution
AC_CONFIG_SRCDIR([src/include/globaldefs.h])

dnl Initialize the automake system
AM_INIT_AUTOMAKE

dnl Initialize libtool
LT_INIT
AC_CONFIG_MACRO_DIRS([m4])

dnl Checks for programs.
AC_PROG_MAKE_SET
AC_PROG_CC
AC_PROG_CPP
AC_PROG_CXX
dnl AC_PROG_RANLIB
AC_PROG_INSTALL
AC_PROG_LN_S

echo "AR is $AR"
if test -z "${AR+set}"; then
  AR=ar
fi
echo "AR is $AR"
AC_SUBST(AR)

dnl set the $host variable based on local machine/os
AC_CANONICAL_HOST

dnl check for default libraries
dnl AC_SEARCH_LIBS(clock_gettime, [rt])
dnl AC_SEARCH_LIBS(cos, [m])
dnl AC_SEARCH_LIBS(gzopen, [z])

AM_CONFIG_HEADER(src/include/nav_config.h)

dnl code configuration selections

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/include/Makefile \
        src/nav_orig/Makefile \
        src/nav_polarity/Makefile \
        src/nav_mag/Makefile \
        src/utils/Makefile \
        src/MadgwickAHRS/Makefile \
])
AC_OUTPUT

