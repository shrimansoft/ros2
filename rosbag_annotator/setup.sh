#!/bin/bash

# ROS2 Bag Annotator Setup Script
# This script sets up the environment and builds the annotation tool

echo "Setting up ROS2 Bag Annotator..."

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS2 not found. Please source your ROS2 installation first:"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "Found ROS2 distribution: $ROS_DISTRO"

# Install Python dependencies
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Create resource directory
mkdir -p resource
echo "rosbag_annotator" > resource/rosbag_annotator

# Build the package
echo "Building the package..."
cd ..
colcon build --packages-select rosbag_annotator

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "To use the annotation tool:"
    echo "1. Source the workspace: source install/setup.bash"
    echo "2. Run the GUI: ros2 run rosbag_annotator annotation_gui.py"
    echo "3. Or use launch file: ros2 launch rosbag_annotator annotation_gui.launch.py"
else
    echo "Build failed. Please check the error messages above."
    exit 1
fi
