UAV Nav PyWrap
==============
Hamid M. and Trevor L.
September 25, 2014

This work wraps the navigation filter `.c` code and works with them in Python.

* **wrap_nav_filter.py**: wrap the entire nav filter C-Code
* **Csource/EKF_15state_quat.c**: 15-state INS/GPS filter function
* **Csource/nav_functions.c**: utility navigation functions
* **Csource/matrix.c**: matrix library

Getting Started
---------------

These instructions are for running **wrap_nav_filter.py*.  This script loads compiled INS/GPS C-code and runs the code using `.mat` flight data.


1. Create a folder `Cbuild`.  This is where the compiled code will go. 

2. The nav filter-code `EKF_15state_quat.c` or whatever nav filter has two dependencies: `matrix.c` and `nav_functions.c`.  All three source files should be compiled individually into `.o` *objects*, and then linked together into a final `EKF_15state_quat.so` *shared object*.  Assuming you are inside the `Csource` directory, open a terminal and run:
 
  ** Linux Machine ** 
    >gcc -o nav_functions.o -c nav_functions.c -fPIC
    
    >gcc -o matrix.o -c matrix.c -fPIC
      
    >gcc -o EKF_15state_quat.o -c EKF_15state_quat.c -fPIC

    >gcc -lm -shared -Wl,-soname,EKF_15state_quat -o EKF_15state_quat.so EKF_15state_quat.o  matrix.o nav_functions.o -lc

  ** Windows 8.1 Machine with Windows Powershell and IPython 2.7 from Anaconda 64 Bit Download **
    >gcc -o nav_functions.o -c nav_functions.c
    
    ***NOTE: You will need to comment termois.h file from matrix.c for Powershell to compile***
    >gcc -o matrix.o -c matrix.c
      
    >gcc -o EKF_15state_quat.o -c EKF_15state_quat.c

    >gcc -lm -shared -Wl",-soname,EKF_15state_quat" -o EKF_15state_quat.so EKF_15state_quat.o  matrix.o nav_functions.o

3. Copy `EKF_15state_quat.so` (other *object files* not needed) into `Cbuild` directory.

4. Open IPython and run `wrap_nav_filter.py`.  

Looking at the for-loop in `wrap_nav_filter.py` should make clear how to log extra terms.

Note on Original C-Code
-----------------------

The original c-code has dependencies which are needed when compiling the entire flight code.  However, in this case we are only running the navigation filter and hence these are not required.  Thus, the C-code is modified to exclude these additional dependencies.  A nominal list used in EKF_15state_quat.c is:

    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <math.h>

    #include "globaldefs.h"
    #include "matrix.h"

    #include "nav_functions.h"

Additionally the nav_functions.c file's dependencies may need to be slightly altered from the normal aircraft version depending on the location/directory of matrix.c.  The standard matrix.c dependencies should be fine.