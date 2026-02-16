# Use official ROS 2 Humble base image
FROM ros:humble

# Set environment variables to avoid interactive installs
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools including pip, pytest, Git and colcon
RUN apt-get update && \
    apt-get install -y \
    python3-pip \
    git \
    python3-colcon-common-extensions \
    libx11-xcb1 \
    libxcb1 \
    libx11-dev \
    libglu1-mesa \
    libgl1-mesa-glx \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages using pip
RUN pip3 install --no-cache-dir pytest pytest-cov

# Source ROS 2 setup in every shell
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Set the default shell to bash
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /ros2_ws

# Entry point to source ROS 2 by default
ENTRYPOINT ["/bin/bash"]
