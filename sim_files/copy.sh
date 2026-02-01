#!/usr/bin/env bash

# Exit immediately if any command fails
set -e

echo "Starting copy process..."

cp _CMakeLists.txt ../../sim_ros2_interface/CMakeLists.txt
cp _package.xml ../../sim_ros2_interface/package.xml
cp _interfaces.txt ../../sim_ros2_interface/meta/interfaces.txt

echo "All copy operations completed successfully."
