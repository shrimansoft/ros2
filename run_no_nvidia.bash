#!/bin/bash

# Alternative script without NVIDIA runtime for systems without NVIDIA GPU
# If not working, first do: sudo rm -rf /tmp/.docker.xauth

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
docker rm -f ros2os_no_nvidia 2>/dev/null || true

docker run -it \
    --name=ros2os_no_nvidia \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --device=/dev/bus/usb \
    shriman1:a \
    bash

echo "Done."