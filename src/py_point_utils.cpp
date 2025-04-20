// py_point_utils.cpp
#include <nanobind/nanobind.h>
#include "point_utils.h"

namespace nb = nanobind;

NB_MODULE(point_utils_py, m) {
    m.doc() = "Bindings for PointUtils";

    // Expose Point struct
    nb::class_<Point>(m, "Point")
        .def(nb::init<double, double>(),
             nb::arg("x") = 0.0,
             nb::arg("y") = 0.0)
        .def_rw("x", &Point::x)
        .def_rw("y", &Point::y)
        .def("__repr__", [](const Point &p) {
            return "<Point x=" + std::to_string(p.x)
                 + " y=" + std::to_string(p.y) + ">";
        });

    // Expose distance function
    m.def("distance",
          &PointUtils::distance,
          nb::arg("a"),
          nb::arg("b"));
}
