#!/bin/bash

# Smart Docker run script that auto-detects NVIDIA runtime availability
# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If it still doesn't work, try running the script as root.

## Build the image first:
### docker build -t shriman1:a .

## Then run this script:
xhost +local:root

XAUTH=/tmp/.docker.xauth

# Create X11 auth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    chmod a+r $XAUTH
fi

# Add X11 forwarding permissions
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

# Remove existing container if it exists
docker rm -f ros2os 2>/dev/null || true

# Check if NVIDIA runtime is available
if docker info | grep -q "nvidia"; then
    echo "NVIDIA runtime detected - using GPU acceleration"
    RUNTIME_FLAG="--runtime=nvidia"
else
    echo "NVIDIA runtime not found - running without GPU acceleration"
    RUNTIME_FLAG=""
fi

docker run -it \
    --name=ros2os \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --device=/dev/bus/usb \
    $RUNTIME_FLAG \
    shriman1:a \
    bash

echo "Done."
