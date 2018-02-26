# navigation

Navigation (EKF) toolbox with python wrappers.  Suitable for use in
small UAS applications.  The code is structured with dual C++ and
python interfaces.  It includes a plotting library for comparing
filters or configurations.

This code project was original put together by Hamid Mokhtarzadeh
mokh0006 at umn dot edu in support of the research performed by the
UAS and Control Systems groups at the Aerospace Engineering and
Mechanics Deptarment, University of Minnesota.

# Installation

* Requires the aura-props and aura-flightdata python modules (also available
  at https://github.com/AuraUAS)

* Run sudo ./setup.py install to compile the EKF modules and install them.

# Supported data log formats:

* Aura text format

* UMN .mat (matlab, hdf5)

* Ardupilot tlog (partial support, I would be happy to find a
  volunteer to improve this.)

* PX4 sdlog2_dump and ulog2csv formats (CSV).

* Sentera camera IMU format


# Available filters

Note that the eigen in the name denotes use of the Eigen3 matrix
library.  However, all the earlier non-eigen variants of the filter
have been depricated so this is rudundant.  It is something I will
clean up at some point as I continue to test and refine the newer
variants.

* nav_eigen_double: 15 state EKF using only inertial sensors and gps
  for input.  Converges to true heading without needing magnetometers.
  Uses double types exclusively for all internal math whether the
  precision is needed or not.

* nav_eigen_float: 15 state EKF using only intertial sensors and gps
  for input.  Does not need magnetometers.  Uses floating point data
  types as much as possible internally.  Produces the same (or
  practically) the same result as the double version of this
  algorithm.  For some platforms that done't support double precision
  floating point math in hardware, this version may run significantly
  faster.

* nav_eigen_sep: The same algorithm as nav_eigen_float, but with the
  code restructured a bit to enable separating the time update and the
  measurement update steps into separate functions.  Also adds
  trapazoidal numerical integration for slightly better results.

* nav_eigen_mag_sep: 15 state EKF that includes magnetometers in the
  measurement update.  Tends to converge to the same solution as the
  non-magnetometer variant over time.  Is more stable in attitude and
  holds the bias estimate more stable.  With a reasonable magnetometer
  calibration, this version should converge more quickly and drift
  less in low dynamic portions of a flight (however the drift that is
  there will be towards any magnetometer calibration error.)  With a
  bad magnetometer calibration, this algorithm can perform
  significantly worse than the non-mag version.  Time update and
  measurement update have been split into separate methods.

* nav_eigen_mag_unified: 15 state EKF that includes magnetometers in
  the measurement update.  Tends to converge to the exact same
  solution as the non-magnetometer variant over time.  Is more stable
  in attitude and holds the bias estimate more stable.  With a
  reasonable magnetometer calibration, this version should converge
  more quickly and drift less in low dynamic portions of a flight
  (however the drift that is there will be towards any magnetometer
  calibration error.) Time update and measurement update are rolled
  together into a single unified function.

* nav_openloop: Open-loop integrator.  Given an initial starting
  condition, will integrate position, velocity, and orientation from
  the inertial data alone.  Shis can be used with the piece-wise
  segment optimizer.


# Script Fron End Features:

* Run two filters (or the same filter with different noise settings)
  and plot the results side by side.

* Core filters are written in C/C++ but the infrastructure, data
  loading, and plotting is handled in python.

* Uses boost/python so that the same core C++ code can be used from
  either C++ or python applications.


# Calibration:

* Includes code that can import a flight data set and do a least
  squares fit of an ellipsoid to the magnetometer data to callibrate
  the magnetometer.

* Includes code that can compare the expected ideal mag vector (based
  on location, date, and aircraft orientation) versus the actual
  sensed mag vector and do a best fit (mag calibration) from flight data.