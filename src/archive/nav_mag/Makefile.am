AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_mag.la

libnav_mag_la_SOURCES = \
        EKF_15state_quat.c \
        nav_functions.c nav_functions.h nav_interface.h \
	../nav_core/coremag.c ../nav_core/coremag.h \
	../utils/matrix.c ../utils/matrix.h

libnav_mag_la_LDFLAGS = -module -avoid-version -shared
