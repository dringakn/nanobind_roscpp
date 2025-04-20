# nanobind_roscpp

> Demonstrates how to write Python 3 bindings for ROS 1 C++ code using [nanobind](https://github.com/wjakob/nanobind).

## Features

- Exposes a simple C++ `Point` struct and `PointUtils::distance()` to Python  
- Builds a native C++ library (`point_utils`) and a Python module (`point_utils_py`)  
- Uses nanobind for zero‑overhead bindings  
- Installable into Python’s `dist-packages` for seamless `import nanobind_roscpp.point_utils_py`

## Prerequisites

- ROS 1 (e.g., Noetic) with catkin_tools  
- CMake ≥ 3.12  
- Python 3 development headers  
- nanobind (as a git submodule under `externals/nanobind`)

The nanobind is used which is more recent than pybind11. In order to create python bindings for your c++ code using nanobind, create a folder called externals inside your ROS package folder and download the nanobind repository and its submodules using following instructions:
```bash
cd ~/catkin_ws/src/PACKAGE_NAME/externals
git clone https://github.com/wjakob/nanobind.git
cd ~/catkin_ws/src/PACKAGE_NAME/externals/nanobind
git submodule update --init --recursive
```

## Directory Layout
Here is a sample folder structure of the package
```
nanobind_roscpp/
├── CMakeLists.txt
├── package.xml
├── externals/
│   └── nanobind/               # nanobind submodule
├── include/
│   └── point_utils.h
├── src/
│   ├── point_utils.cpp
│   └── py_point_utils.cpp
├── python/
│   └── nanobind_roscpp/
│       └── __init__.py
├── scripts/
│   └── test.py
└── launch/
    └── test.launch
```

## Build & Install

```bash
# 1. Enable install-space in your workspace
catkin config --install

# 2. Build and install
catkin build

# 3. Source the install-space
source install/setup.bash
```

## Usage

### From command line

```bash
rosrun nanobind_roscpp test.py
```

### As a ROS launch


```bash
roslaunch nanobind_roscpp test.launch
```

## Example Code

### C++ Header (`include/point_utils.h`)

```cpp
#include <cmath>

struct Point {
    double x, y;
};

class PointUtils {
public:
    static double distance(const Point &a, const Point &b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};
```

### Binding (`src/py_point_utils.cpp`)

```cpp
#include <nanobind/nanobind.h>
#include "point_utils.h"

namespace nb = nanobind;

NB_MODULE(point_utils_py, m) {
    m.doc() = "Bindings for PointUtils";

    nb::class_<Point>(m, "Point")
        .def(nb::init<double,double>(),
             nb::arg("x") = 0.0,
             nb::arg("y") = 0.0)
        .def_rw("x", &Point::x)
        .def_rw("y", &Point::y)
        .def("__repr__", [](const Point &p) {
            return "<Point x=" + std::to_string(p.x)
                 + " y=" + std::to_string(p.y) + ">";
        });

    m.def("distance",
          &PointUtils::distance,
          nb::arg("a"),
          nb::arg("b"));
}
```

### Test Script (`scripts/test.py`)

```python
#!/usr/bin/env python3
import rospy
from nanobind_roscpp.point_utils_py import Point, distance

def talker():
    rospy.init_node('distance_node')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        a = Point(0, 0)
        b = Point(1, 1)
        rospy.loginfo(f"Dist: {distance(a, b)}")
        rate.sleep()

if __name__ == "__main__":
    talker()
```

## License

BSD‑3-Clause © Dr.‑Ing. Ahmad Kamal Nasir