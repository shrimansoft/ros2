#!/bin/bash

# Docker run script for ROS2 bag annotator with data mounting
# Usage: 
#   ./run_annotator.bash annotate                           # Use default data directory
#   ./run_annotator.bash /path/to/data annotate             # Use custom data directory
#   ./run_annotator.bash /path/to/data bash                 # Open bash shell with custom mount
#   ./run_annotator.bash --debug annotate                   # Run in debug mode with detailed logging

# Check for debug mode
DEBUG_MODE=false
if [ "$1" = "--debug" ]; then
    DEBUG_MODE=true
    shift  # Remove debug flag from arguments
    echo "Debug mode enabled - detailed logging will be available"
fi

# Auto-detect DISPLAY if not set
if [ -z "$DISPLAY" ]; then
    # Try to find available X11 display
    if [ -d "/tmp/.X11-unix" ]; then
        for display_file in /tmp/.X11-unix/X*; do
            if [ -e "$display_file" ]; then
                display_num=$(basename "$display_file" | sed 's/X//')
                export DISPLAY=":$display_num"
                echo "Auto-detected DISPLAY=$DISPLAY"
                break
            fi
        done
    fi
    
    # Fallback to :0 if nothing found
    if [ -z "$DISPLAY" ]; then
        export DISPLAY=":0"
        echo "Using fallback DISPLAY=:0"
    fi
fi

echo "Using DISPLAY=$DISPLAY"

# Fix X11 forwarding permissions
echo "Setting up X11 forwarding..."
xhost +local:root > /dev/null 2>&1 || {
    echo "Warning: xhost command failed. Trying alternative method..."
    xhost +local:docker > /dev/null 2>&1 || {
        echo "Warning: Could not configure X11 permissions. GUI may not work."
    }
}

XAUTH=/tmp/.docker.xauth

# Create X11 auth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    echo "Creating X11 auth file..."
    touch $XAUTH
    chmod 644 $XAUTH
fi

# Add X11 forwarding permissions with better error handling
if command -v xauth >/dev/null 2>&1; then
    echo "Configuring X11 authentication..."
    xauth nlist $DISPLAY 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge - 2>/dev/null || {
        echo "Warning: xauth configuration failed. Trying to continue anyway..."
    }
else
    echo "Warning: xauth not found. X11 forwarding may not work properly."
fi

# Remove existing container if it exists
docker rm -f ros2_annotator 2>/dev/null || true

# Check if NVIDIA runtime is available
if docker info | grep -q "nvidia"; then
    echo "NVIDIA runtime detected - using GPU acceleration"
    RUNTIME_FLAG="--runtime=nvidia"
else
    echo "NVIDIA runtime not found - running without GPU acceleration"
    RUNTIME_FLAG=""
fi

# Get the current directory to mount as data volume
CURRENT_DIR=$(pwd)
DATA_DIR="$CURRENT_DIR/data"

# Create data directory if it doesn't exist
mkdir -p "$DATA_DIR"

# Check if user specified a custom data directory as first argument
if [ "$1" != "" ] && [ -d "$1" ] && [ "$1" != "annotate" ] && [ "$1" != "bash" ] && [ "$1" != "play" ] && [ "$1" != "info" ] && [ "$1" != "merge" ]; then
    CUSTOM_DATA_DIR=$(realpath "$1")
    echo "Using custom data directory: $CUSTOM_DATA_DIR -> /data"
    DATA_MOUNT="$CUSTOM_DATA_DIR:/data"
    shift  # Remove the directory argument
else
    echo "Using default data directory: $DATA_DIR -> /data"
    echo "Also mounting user home directory: $HOME -> /home_data (read-only)"
    DATA_MOUNT="$DATA_DIR:/data"
fi

echo "Starting ROS2 Bag Annotator container..."
echo "Access your rosbag files through the /data directory in file dialogs"
echo "User home directory is available at /home_data (read-only)"

# Create logs directory for debug mode
LOGS_DIR="$CURRENT_DIR/logs"
mkdir -p "$LOGS_DIR"

# Set debug environment variables
DEBUG_ENV=""
if [ "$DEBUG_MODE" = true ]; then
    echo "Debug logs will be saved to: $LOGS_DIR"
    DEBUG_ENV="--env=ANNOTATOR_DEBUG=1 --env=ANNOTATOR_LOG_DIR=/logs --volume=$LOGS_DIR:/logs"
fi

echo "Running Docker container..."
docker run -it \
    --name=ros2_annotator \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$DATA_MOUNT" \
    --volume="$HOME:/home_data:ro" \
    --volume="$CURRENT_DIR/rosbag_annotator/config:/ros2_ws/config" \
    $DEBUG_ENV \
    --net=host \
    --privileged \
    $RUNTIME_FLAG \
    shriman1:a \
    "$@"

echo "Container stopped."
