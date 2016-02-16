AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnavigation.la

libnavigation_la_SOURCES = \
        EKF_15state_quat.c \
        nav_functions.c nav_functions.h nav_interface.h \
	../utils/matrix.c ../utils/matrix.h

libnavigation_la_LDFLAGS = -module -avoid-version -shared
