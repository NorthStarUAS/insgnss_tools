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


# Calibration:

* Includes code that can import a flight data set and do a least
  squares fit of an ellipsoid to the magnetometer data to callibrate
  the magnetometer.

* Includes code that can compare the expected ideal mag vector (based
  on location, date, and aircraft orientation) versus the actual
  sensed mag vector and do a best fit (mag calibration) from flight data.