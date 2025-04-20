# 1. Require at least CMake 3.12 (for find_package(Python3 …) support)
cmake_minimum_required(VERSION 3.12)

# 2. Declare Project name
project(nanobind_roscpp)

# 3. Force all targets to compile under C++17
add_compile_options(-std=c++17)


# ────────────────────────────────────────────────────────────────────────────────
# 4. Locate the Python3 interpreter and development headers/libraries
#    - Interpreter: so we can run any helper scripts during build if needed
#    - Development: gives you Python3_INCLUDE_DIRS and Python3_LIBRARIES
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)


# ────────────────────────────────────────────────────────────────────────────────
# 5. Locate ROS’s build system (catkin) and the roscpp client lib
find_package(catkin REQUIRED COMPONENTS roscpp)


# ────────────────────────────────────────────────────────────────────────────────
# 6. Pull in nanobind as a subdirectory so we can use its CMake functions/macros
#   - ws/src/PACKAGE_NAME/externals: git clone https://github.com/wjakob/nanobind.git
#   - ws/src/PACKAGE_NAME/externals/nanobind: git submodule update --init --recursive
add_subdirectory(externals/nanobind)



# ────────────────────────────────────────────────────────────────────────────────
# 7. Declare what your catkin package provides to other catkin packages:
#    - INCLUDE_DIRS: headers to export
#    - LIBRARIES: libraries to export
#    - CATKIN_DEPENDS: runtime/package dependencies
catkin_package(
  INCLUDE_DIRS include ${Python3_INCLUDE_DIRS}
  LIBRARIES    point_utils point_utils_py
  CATKIN_DEPENDS roscpp
)


# ────────────────────────────────────────────────────────────────────────────────
# 8. Direct CMake where to find header files when compiling
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include     # Package public headers
  ${catkin_INCLUDE_DIRS}                  # headers from ROS packages
  ${Python3_INCLUDE_DIRS}                 # Python3 headers
  externals/nanobind/include              # nanobind’s headers
)


# ────────────────────────────────────────────────────────────────────────────────
# 9. Build the core C++ library “point_utils”
add_library(point_utils
  src/point_utils.cpp
)
target_include_directories(point_utils PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include     # expose package headers to dependents
)



# ────────────────────────────────────────────────────────────────────────────────
# 10. Build the Python extension module “point_utils_py” using nanobind
nanobind_add_module(point_utils_py
  src/py_point_utils.cpp
)
#    Ensure it can find package headers too
target_include_directories(point_utils_py PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)



# ────────────────────────────────────────────────────────────────────────────────
# 11. Link the Python module against:
#     - core C++ library
#     - the Python3 runtime libraries
#     - any ROS libraries (if the binding uses them)
target_link_libraries(point_utils_py PRIVATE
  point_utils
  Python3::Python
  ${catkin_LIBRARIES}
)



# ────────────────────────────────────────────────────────────────────────────────
# 12. Set properties of targets:
#     - enforce C++17 on this target
#     - ensure the linker knows it’s C++
set_target_properties(point_utils_py PROPERTIES
  CMAKE_CXX_STANDARD 17
  LINKER_LANGUAGE    CXX
)



# ────────────────────────────────────────────────────────────────────────────────
# 13. Install package Python extension so that after `make install` it lands under
#     - catkin config --install
#     - catkin build
#     install/lib/python3/dist-packages/PACKAGE_NAME/point_utils_py*.so
install(TARGETS point_utils_py
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# ────────────────────────────────────────────────────────────────────────────────
# 14. Install the __init__.py so Python recognizes PACKAGE_NAME as a package
install(FILES python/nanobind_roscpp/__init__.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# ────────────────────────────────────────────────────────────────────────────────
# 15. Install package C++ headers for other C++ users of this library
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# ────────────────────────────────────────────────────────────────────────────────
# 16. Install any helper scripts (e.g. test.py) into the ROS bin folder
catkin_install_python(
  PROGRAMS scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
