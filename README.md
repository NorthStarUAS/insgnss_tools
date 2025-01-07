# NorthStar UAS -- Navigation INS/GNSS EKF Toolbox

INS/GNSS, EKF, Sensor fusion toolbox with python wrappers.  Suitable for use in
small UAS applications.  The code is structured with dual C++ and python
interfaces.  It includes a plotting library for comparing filters and
configurations.

This code project was original put together by Hamid Mokhtarzadeh mokh0006 at
umn dot edu in support of the research performed by the UAS and Control Systems
groups at the Aerospace Engineering and Mechanics Deptarment, University of
Minnesota.

## Installation

* Requires the props and flightdata python modules (also available at
  <https://github.com/NorthStarUAS>)
* Requires C++ code compiling (and an integrated c++ compiler with your python
  environment.)

```bash
CC="ccache gcc" ./setup.py build
CC="ccache gcc" ./setup.py install --user
```

## Supported data log formats

* Native NorthStarUAS (AuraUAS) HDF5 and CSV formats
* UMN1 (.mat) and UMN3 (HDF5)
* Ardupilot tlog (partial support, I would be happy to find a volunteer to
  improve this.)
* PX4 sdlog2_dump and ulog2csv formats (CSV).
* Sentera camera IMU format

## Available filters

* nav_ekf15: 15 state EKF using only intertial sensors and gps for input.  Does
  not need magnetometers.  Uses floating point data types as much as possible
  internally.
* nav_ekf15_mag: 15 state EKF that includes magnetometers in the measurement
  update.  Tends to converge to the same solution as the non-magnetometer
  variant over time during dynamic motion.  In slow moving situations
  magnetometer error will dominate the computations. The quality of this filter
  is closely linked to the quality of the magnetometer calibration (or lack of
  magnetic field disturbances.)
* pyNavEKF15: a pure python/numpy version of the 15-state Kalman filter.
  Sacrifice some performance, gain simplicity and readability.
* nav_openloop: Open-loop integrator (forward propagation only.) Given an
  initial starting condition, will integrate position, velocity, and orientation
  from the inertial data alone.  This can be used with the piece-wise segment
  optimizer or used to forward propagate an EKF solution computed slightly in
  the past.

## Script Front End Features

* Run two filters (or the same filter with different noise settings) and plot
  the results side by side.
* Core filters are written in C/C++ but the infrastructure, data loading, and
  plotting is handled in python.
* Uses pybind11 so that the same core C++ code can be used from either C++ or
  python applications.
* Includes an example wrapper that demonstrates how to account for a known
  amount of GPS latency.
