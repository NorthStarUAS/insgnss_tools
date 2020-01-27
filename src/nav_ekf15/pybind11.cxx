#ifdef HAVE_PYBIND11

#include <pybind11/pybind11.h>
namespace py = pybind11;

#include "EKF_15state.hxx"

PYBIND11_MODULE(ekf15, m) {
    py::class_<EKF15>(m, "EKF15")
        .def(py::init<>())
        .def("set_config", &EKF15::set_config)
        .def("init", &EKF15::init)
        .def("time_update", &EKF15::time_update)
        .def("measurement_update", &EKF15::measurement_update)
        .def("get_nav", &EKF15::get_nav)
    ;
}

#endif
