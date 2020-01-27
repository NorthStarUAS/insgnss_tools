#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "EKF_15state.hxx"

PYBIND11_MODULE(ekf15_mag, m) {
    py::class_<EKF15_mag>(m, "EKF15_mag")
        .def(py::init<>())
        .def("set_config", &EKF15_mag::set_config)
        .def("init", &EKF15_mag::init)
        .def("time_update", &EKF15_mag::time_update)
        .def("measurement_update", &EKF15_mag::measurement_update)
        .def("get_nav", &EKF15_mag::get_nav)
    ;
}

#endif
