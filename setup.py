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
      ext_modules=[Extension('nav_core', ['src/nav_core/nav_structs.cxx', 'src/nav_core/wgs84.cxx'], libraries=['boost_python'], aextra_link_args=['-module', '-avoid-version'])],
     )
