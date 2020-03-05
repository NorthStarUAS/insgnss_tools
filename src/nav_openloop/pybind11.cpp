#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "openloop.h"

PYBIND11_MODULE(openloop, m) {
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
