AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_polarity.la

libnav_polarity_la_SOURCES = \
        EKF_15state_quat.c \
        nav_functions.c nav_functions.h nav_interface.h \
	../utils/matrix.c ../utils/matrix.h

libnav_polarity_la_LDFLAGS = -module -avoid-version -shared
