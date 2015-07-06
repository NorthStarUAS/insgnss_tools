# UAV Nav PyWrap
Hamid M. and Trevor L.
September 25, 2014

This work wraps both a navigation and researchNavigation filter `.c` code and works with them in Python.  A `.mat` flight data file is used to play through the navigation code.

* **wrap_nav_filter.py**: wrap the entire nav filter C-Code
* **Csource/navigation/**: baseline navigation (15-state INS/GPS) filter code as well as navigation function library
* **Csource/researchNavigation**: research navigation code
* **Csource/utils/matrix.c**: matrix library
 
## Change Log

* June 19, 2014      *Hamid M.* 
    - Initial version for experimental AHRS/air-speed dead-reckoning filter.
* August 19, 2014    *Trevor L* 
    - Extended to work with nominal INS/GPS.
* September 25, 2014 *Hamid M.* 
    - Clean up comments and naming.
* March 31, 2014     *Hamid M.* 
    - Add commands to build .so on run.
* April 8, 2015      *Hamid M.* 
    - Fix bug in plotting altitude and ground track.
    - Add input flags.  Extend to work with research.
* April 9, 2015      *Hamid M.* 
    - Add way to affect mission->haveGPS.
* April 22, 2015     *Hamid M.* 
    - Modify architecture to run both nav and researchNav in parallel

## Getting Started

By running `wrap_nav_filter.py` the c-code will get compiled, wrapped, and the flight data will be played through the code.  Finally, plots will be generated.  This script should not be used in interactive mode and should instead by called from the terminal:
>python wrap_nav_filter.py

## Note on Original C-Code
The original c-code has dependencies which are needed when compiling the entire flight code.  However, in this case we are only running the navigation filter and hence these are not required.  Thus, the C-code is modified to exclude these additional dependencies.  A nominal list used in EKF_15state_quat.c is:

    #include <stdio.h>
    #include <string.h>
    #include <stdlib.h>
    #include <math.h>

    #include "globaldefs.h"
    #include "matrix.h"

    #include "nav_functions.h"

## Flight Data Parameters

The loaded `.mat` flight data file must have the following entries:

Parameter | Units | note
--- | --- | ---
time | sec | 
ias | m/s | indicated airspeed
h   | m   | baro-altimeter
p, q, r | rad/s | x, y, z-body axis gyros
ax, ay, az | m/s^2 | x, y, z-body axis accels
hx, hy, hz | Guass | x, y, z-body axis (note used nominally)
vn, ve, vd | m/s | GPS velocity north, east, down 
lat, lon, alt | deg, m | GPS position

where the GPS observations are simply repeated until the new measurement is available.  In this way all arrays are of the same length.

If `FLAG_FORCE_INIT` is used, then additionally the on-board computed navigation solution is required:


Parameter | Units | note
--- | --- | ---
psi, theta, phi | rad | yaw, pitch, roll
navlat, navlon, alt | radian, m | GPS position

*Note: navlat, navlon have different units than lat, lon.*

## Manual Compiling of C-Code

Although `wrap_nav_filter.py` automatically builds the code, these instructions are kept for documentation on how to manually compiles the C-code into shared objects..

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

The `.so` shared object can be loaded into Python and wrapped, as is done in `wrap_nav_filter.py`.

