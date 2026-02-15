FROM osrf/ros:humble-desktop-full

# Install tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    vim \
    git \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /root

CMD ["/bin/bash"]
