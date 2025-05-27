FROM osrf/ros:humble-desktop

# Define arguments for NVIDIA GPU support
ARG NVIDIA_VISIBLE_DEVICES=all
ARG NVIDIA_DRIVER_CAPABILITIES=graphics

# Set environment variables correctly
ENV NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES}"
ENV NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES}"

SHELL [ "/bin/bash" , "-c" ]

# Update package lists and install required dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-joint-state-publisher-gui \
    ros-humble-gazebo-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-gazebo-ros \
    ros-humble-rviz2
    # python3 -m pip install --upgrade pip \
    # pip3 install cfclient

# Install Python dependencies
# RUN python3 -m pip install opencv-contrib-python==4.6.0.66 pygame

# Final message
RUN echo "Setup completed"
