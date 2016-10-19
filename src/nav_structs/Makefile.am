AUTOMAKE_OPTIONS = subdir-objects

lib_LTLIBRARIES = libnav_structs.la

libnav_structs_la_SOURCES = \
        nav_structs.cxx nav_structs.hxx

AM_CPPFLAGS = @PYTHON_INCLUDES@

libnav_structs_la_LDFLAGS = -module -avoid-version -shared -lboost_python
