#!/usr/bin/env python

from distutils.core import setup, Extension

setup(name='navigation',
      version='1.0',
      description='Navigation Tools',
      author='Curtis L. Olson',
      author_email='curtolson@flightgear.org',
      url='https://github.com/AuraUAS',
      #py_modules=['props', 'props_json', 'props_xml'],
      package_dir = {'': 'lib'},
      packages=['nav', 'nav.data'],
      ext_package='nav',
      ext_modules=[
            Extension('structs', ['src/nav_core/structs.cxx'], libraries=['boost_python']),
            Extension('wgs84', ['src/nav_core/wgs84.cxx'], libraries=['boost_python']),
            Extension('EKF15', ['src/nav_eigen/EKF_15state.cxx', 'src/nav_core/nav_functions.cxx'], libraries=['boost_python']),
            Extension('EKF15_mag', ['src/nav_eigen_mag/EKF_15state_mag.cxx', 'src/nav_core/nav_functions.cxx', 'src/nav_core/coremag.c'], libraries=['boost_python']),
            Extension('openloop', ['src/nav_openloop/openloop.cxx', 'src/nav_openloop/glocal.cxx', 'src/nav_core/nav_functions.cxx'], libraries=['boost_python'])
      ],
     )
