#!/usr/bin/env bash
#
# build_and_install.sh
#
# This script configures and builds your catkin workspace, copies the
# compiled Python extension into your package’s python folder, and
# installs the package in editable mode with pip.
#

set -euo pipefail

##
## User‑configurable variables
##

# ADD YOUR CATKIN WORKSPACE PATH AND PACKAGE NAME HERE
# Absolute path to your catkin workspace root
CATKIN_WS="${HOME}/delme_ws"
# Name of your package (must match the folder under src/)
PACKAGE_NAME="nanobind_roscpp"

##
## 1) Configure and build the workspace
##

echo "🔧 Configuring workspace to install into 'install/'"
cd "${CATKIN_WS}"
catkin config --install --workspace "${CATKIN_WS}" --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
echo "🏗️  Building all packages"
catkin build --workspace "${CATKIN_WS}"

##
## 2) Copy the compiled .so into your Python package
##

# Source .so location (wildcard to match the ABI tag)
SRC_SO="${CATKIN_WS}/install/lib/python3/dist-packages/${PACKAGE_NAME}/point_utils_py*.so"
# Destination folder inside your package
DST_DIR="${CATKIN_WS}/src/${PACKAGE_NAME}/python/${PACKAGE_NAME}"

echo "📋 Ensuring destination exists: ${DST_DIR}"
mkdir -p "${DST_DIR}"

echo "📤 Copying compiled extension:"
echo "    from: ${SRC_SO}"
echo "      to: ${DST_DIR}/"
cp ${SRC_SO} "${DST_DIR}/"

##
## 3) Install in editable (dev) mode
##

echo "🚀 Installing ${PACKAGE_NAME} in editable mode"
cd "${CATKIN_WS}/src/${PACKAGE_NAME}"
pip install -e .

echo "✅ Done! Your package is installed and ready for development."
