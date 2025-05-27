# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# If it still doesn't work, try running the script as root.

## Build the image first:
### docker build -t r2_path_planning .

## Then run this script:
xhost local:root

XAUTH=/tmp/.docker.xauth

## remover rm

##docker run -it --rm\
##    --name=swapnil_m \
##    --env="DISPLAY=$DISPLAY" \
##    --env="QT_X11_NO_MITSHM=1" \
##    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
##    --env="XAUTHORITY=$XAUTH" \
##    --volume="$XAUTH:$XAUTH" \
#    --volume="$HOME/ppg_ws:/root/Crazyflie" \
#    --net=host \
#    --privileged \
#    --device=/dev/bus/usb \
#    --runtime=nvidia \
#    crazeflieexp:swap \
#    bash
    
docker run -it \
    --name=crazyflie_test \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --device=/dev/bus/usb \
    --runtime=nvidia \
    crazynew:version1 \
    bash

echo "Done."


# if fails gui then xhost +local:docker
