# navigation

Navigation (EKF) libraries suitable for use in small UAS applications.
The code is structured with dual C++ and python interfaces.  It
includes a plotting librar for comparing filters or configurations.

This code project was original put together by Hamid Mokhtarzadeh
mokh0006 at umn dot edu in support of the research performed by the
UAS and Control Systems groups at the Aerospace Engineering and
Mechanics Deptarment, University of Minnesota.

# Supported data log formats:

* Aura text format

* UMN .mat (matlab, hdf5)

* Ardupilot tlog (partial support, I would be happy to find a
  volunteer to improve this.)

* PX4 "sdlog2_dump" (CSV) format

* Sentera camera IMU format

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

* Uses boost/python so that the same core C++ code can be used from
  either C++ or python applications.