AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_eigen_mag_old.la

libnav_eigen_mag_old_la_SOURCES = \
        EKF_15state_quat.cxx nav_interface.hxx \
        ../nav_core/nav_functions.cxx ../nav_core/nav_functions.hxx \
	../nav_core/coremag.c ../nav_core/coremag.h

libnav_eigen_mag_old_la_LDFLAGS = -module -avoid-version -shared
