AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_openloop.la

libnav_openloop_la_SOURCES = \
        openloop.cxx openloop.hxx \
        ../nav_core/nav_functions.cxx ../nav_core/nav_functions.hxx

AM_CPPFLAGS = @PYTHON_INCLUDES@

libnav_openloop_la_LDFLAGS = -module -avoid-version -shared -lboost_python
