# navigation

Navigation libraries with a development infrastructure for EKF
development and evaluation.

This code project was original put together by Hamid Mokhtarzadeh
mokh0006 at umn dot edu in support of the research performed by the
UAS and Control Systems groups at the Aerospace Engineering and
Mechanics Deptarment, University of Minnesota.

# Supported data log formats:

* Aura text format

* UMN .mat (matlab, hdf5)

* Ardupilot tlog (partial support, I would be happy to find a
  volunteer to improve this.)

# Available filters

* 15 state EKF using only gyro, accels, and gps for input.  Converges
  to true heading without needing magnetometers.

* 15 state EKF that includes magnetometers in the measurement update.
  More stable in attitude, but assumes a quality magnetometer
  calibration.

* Piece-wise segment optimizer.

# Features:

* Run two filters (or the same filter with different noise settings)
  and plot the results side by side.

* Core filters are written in C/C++ but the infrastructure, data
  loading, and plotting is handled in python.

* Uses boost/python to create loadable python modules from the C/C++
  code.