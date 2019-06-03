#!/usr/bin/python3

from setuptools import setup, find_packages, Extension

setup(name='aurauas_navigation',
      version='1.3',
      description='Navigation Tools',
      author='Curtis L. Olson',
      author_email='curtolson@flightgear.org',
      url='https://github.com/AuraUAS',
      #py_modules=['props', 'props_json', 'props_xml'],
      #package_dir = {'': 'lib'},
      packages = find_packages(),
      ext_modules=[
          Extension('aurauas_navigation.structs',
                    define_macros=[('HAVE_PYBIND11', '1')],
                    sources=['src/nav_common/structs.cxx'],
                    depends=['src/nav_common/structs.hxx']),
          Extension('aurauas_navigation.filters',
                    define_macros=[('HAVE_PYBIND11', '1')],
                    sources=['src/filters.cxx',
                             'src/nav_ekf15/EKF_15state.cxx',
                             'src/nav_ekf15_mag/EKF_15state.cxx',
                             'src/nav_openloop/openloop.cxx',
                             'src/nav_openloop/glocal.cxx',
                             'src/nav_common/nav_functions_float.cxx',
                             'src/nav_common/coremag.c'],
                    depends=['src/nav_ekf15/EKF_15state.hxx',
                             'src/nav_ekf15_mag/EKF_15state.hxx',
                             'src/nav_openloop/openloop.hxx',
                             'src/nav_openloop/glocal.hxx',
                             'src/nav_common/constants.hxx',
                             'src/nav_common/nav_functions_float.hxx',
                             'src/nav_common/coremag.h'])
      ],
     )
