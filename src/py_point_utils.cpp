// py_point_utils.cpp
#include <nanobind/nanobind.h>
#include "point_utils.h"

namespace nb = nanobind;

NB_MODULE(point_utils_py, m) {
    m.doc() = "Python bindings for PointUtils (nanobind_roscpp)";

    //–– Point class with docstring ––//
    nb::class_<Point>(m, "Point",
        "A 2D point with x and y coordinates.")
      .def(nb::init<double, double>(),
           "Construct a Point given x and y.",
           nb::arg("x") = 0.0,
           nb::arg("y") = 0.0)
      .def_rw("x", &Point::x, "X coordinate")
      .def_rw("y", &Point::y, "Y coordinate")
      .def("__repr__", [](const Point &p) {
          return "<Point x=" + std::to_string(p.x)
               + " y=" + std::to_string(p.y) + ">";
      });

    //–– distance function with docstring ––//
    m.def("distance",
          &PointUtils::distance,
          "Compute the Euclidean distance between two Points.",
          nb::arg("a"),
          nb::arg("b"));
}
