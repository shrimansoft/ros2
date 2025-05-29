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
    ros-humble-rviz2 \
    ros-humble-rosbag2-py \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    python3-tk \
    python3-yaml \
    sqlite3 \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for annotation tool
RUN python3 -m pip install --upgrade pip && \
    pip3 install \
    opencv-python==4.8.1.78 \
    Pillow==10.0.1 \
    numpy==1.24.3 \
    PyYAML==6.0.1

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy the annotation tool package
COPY rosbag_annotator src/rosbag_annotator

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select rosbag_annotator

# Copy and set up entrypoint script
COPY rosbag_annotator/docker_entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Set workspace as default directory
WORKDIR /ros2_ws

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

# Final message
RUN echo "Setup completed"
