This directory is empty except for a lone/empty `__init__.py` file to 
satisfy the python module system.

All the real code is under src/ and is compiled as shared libraries and
wrapped with pybind11 interfaces so they can be imported into python and
used directly.

The setup.py file takes care of compiling all the code and bundling it up
properly as extensions to the module.

(The basic nav filter code is setup to also be useable in C++ applications
by dropping it into your project and making appropriate build system changes.)
