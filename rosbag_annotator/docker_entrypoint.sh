#!/bin/bash

# Docker entrypoint script for ROS2 bag annotator

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Check if X11 forwarding is working
if ! xset q &>/dev/null; then
    echo "Warning: X11 forwarding may not be working properly"
    echo "Make sure you ran the Docker container with proper X11 forwarding"
fi

# Function to show usage
show_usage() {
    echo "ROS2 Bag Annotator - Docker Environment"
    echo "======================================="
    echo ""
    echo "Available commands:"
    echo "  annotate                 - Launch the enhanced annotation GUI"
    echo "  play <bag_path>         - Play a bag file"
    echo "  merge <bag> <ann> <out> - Merge annotations into bag"
    echo "  info <bag_path>         - Show bag file information"
    echo "  bash                    - Open bash shell"
    echo "  test                    - Test container functionality"
    echo ""
    echo "Examples:"
    echo "  docker run -it shriman1:a annotate"
    echo "  docker run -it shriman1:a play /data/my_bag"
    echo "  docker run -it shriman1:a info /data/my_bag"
    echo ""
    echo "Make sure to mount your data directory:"
    echo "  -v /host/path/to/data:/data"
}

# Handle different commands
case "$1" in
    "annotate")
        echo "Launching Enhanced ROS2 Bag Annotation GUI..."
        cd /ros2_ws/src/rosbag_annotator
        python3 -m rosbag_annotator.scripts.annotation_gui
        ;;
    "play")
        if [ -z "$2" ]; then
            echo "Error: Please specify bag path"
            echo "Usage: play <bag_path> [playback_rate]"
            exit 1
        fi
        echo "Playing bag file: $2"
        cd /ros2_ws/src/rosbag_annotator
        python3 -m rosbag_annotator.scripts.bag_player "$2" "${3:-1.0}"
        ;;
    "merge")
        if [ -z "$4" ]; then
            echo "Error: Please specify all parameters"
            echo "Usage: merge <input_bag> <annotations_file> <output_bag>"
            exit 1
        fi
        echo "Merging annotations..."
        ros2 run rosbag_annotator annotation_merger.py "$2" "$3" "$4"
        ;;
    "info")
        if [ -z "$2" ]; then
            echo "Error: Please specify bag path"
            echo "Usage: info <bag_path>"
            exit 1
        fi
        ros2 bag info "$2"
        ;;
    "bash")
        /bin/bash
        ;;
    "test")
        echo "Container test successful"
        exit 0
        ;;
    "")
        show_usage
        /bin/bash
        ;;
    *)
        echo "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac
