#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "nav_ekf15/EKF_15state.hxx"
#include "nav_ekf15_mag/EKF_15state.hxx"
#include "nav_openloop/openloop.hxx"

PYBIND11_MODULE(filters, m) {
    py::class_<EKF15>(m, "EKF15")
        .def(py::init<>())
        .def("set_config", &EKF15::set_config)
        .def("init", &EKF15::init)
        .def("time_update", &EKF15::time_update)
        .def("measurement_update", &EKF15::measurement_update)
        .def("get_nav", &EKF15::get_nav)
    ;
    
    py::class_<EKF15_mag>(m, "EKF15_mag")
        .def(py::init<>())
        .def("set_config", &EKF15_mag::set_config)
        .def("init", &EKF15_mag::init)
        .def("time_update", &EKF15_mag::time_update)
        .def("measurement_update", &EKF15_mag::measurement_update)
        .def("get_nav", &EKF15_mag::get_nav)
    ;

    py::class_<OpenLoop>(m, "OpenLoop")
        .def(py::init<>())
        .def("init", &OpenLoop::init)
        .def("init_by_nav", &OpenLoop::init_by_nav)
        .def("set_pos", &OpenLoop::set_pos)
        .def("set_vel", &OpenLoop::set_vel)
        .def("set_att", &OpenLoop::set_att)
        .def("set_gyro_calib", &OpenLoop::set_gyro_calib)
        .def("set_accel_calib", &OpenLoop::set_accel_calib)
        .def("update", &OpenLoop::update)
    ;
  
}

#endif
