AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_core.la

libnav_core_la_SOURCES = \
        nav_structs.cxx nav_structs.hxx

AM_CPPFLAGS = @PYTHON_INCLUDES@

libnav_core_la_LDFLAGS = -module -avoid-version -shared -lboost_python
