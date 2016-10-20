AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_eigen_mag.la

libnav_eigen_mag_la_SOURCES = \
        EKF_15state_mag.cxx EKF_15state_mag.hxx \
        ../nav_core/nav_functions.cxx ../nav_core/nav_functions.hxx \
	../nav_core/coremag.c ../nav_core/coremag.h

AM_CPPFLAGS = @PYTHON_INCLUDES@

libnav_eigen_mag_la_LDFLAGS = -module -avoid-version -shared -lboost_python
