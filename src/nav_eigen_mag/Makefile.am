AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_eigen_mag.la

libnav_eigen_mag_la_SOURCES = \
        EKF_15state_quat.cxx \
        nav_functions.cxx nav_functions.hxx nav_interface.hxx

libnav_eigen_mag_la_LDFLAGS = -module -avoid-version -shared
