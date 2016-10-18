AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_eigen.la

libnav_eigen_la_SOURCES = \
        EKF_15state.cxx EKF_15state.hxx \
        nav_functions.cxx nav_functions.hxx \
	nav_state.hxx

AM_CPPFLAGS = @PYTHON_INCLUDES@

libnav_eigen_la_LDFLAGS = -module -avoid-version -shared -lboost_python
