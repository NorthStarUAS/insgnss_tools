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

AC_LANG_SAVE
AC_LANG_CPLUSPLUS
AC_CHECK_HEADERS([eigen3/Eigen/Core])
AC_LANG_RESTORE
if test "x$ac_cv_header_eigen3_Eigen_Core" != xyes; then
   echo "configure error: eigen3 matrix libraries are required"
   exit
fi

dnl find python primary
AM_PATH_PYTHON
AC_PATH_PROGS([PYTHON_CONFIG],
              [python$PYTHON_VERSION-config python-config],
              [no],
              [`dirname $PYTHON`])
AS_IF([test "$PYTHON_CONFIG" = no], [AC_MSG_ERROR([cannot find python-config for $PYTHON.])])
PYTHON_INCLUDES=`$PYTHON_CONFIG --includes`
AC_MSG_RESULT([$PYTHON_INCLUDES])
AC_SUBST(PYTHON_INCLUDES)
PYTHON_LIBS=`$PYTHON_CONFIG --libs`
AC_MSG_RESULT([$PYTHON_LIBS])
AC_SUBST(PYTHON_LIBS)

AM_CONFIG_HEADER(src/include/nav_config.h)

dnl code configuration selections

AC_CONFIG_FILES([ \
	Makefile \
        src/Makefile \
        src/include/Makefile \
        src/nav_orig/Makefile \
        src/nav_mag/Makefile \
        src/nav_eigen/Makefile \
        src/nav_eigen_mag/Makefile \
	src/nav_eigen_old/Makefile \
        src/nav_eigen_mag_old/Makefile \
        src/utils/Makefile \
        src/MadgwickAHRS/Makefile \
])
AC_OUTPUT

