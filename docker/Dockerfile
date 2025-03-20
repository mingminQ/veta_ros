# Base Image
FROM osrf/ros:noetic-desktop-full

# Set Workspace Directory
WORKDIR /home/veta_ws

# Set Environment Variables
ENV DEBIAN_FRONTEND=noninteractive

# Install Basic Ubuntu Essentials
RUN apt-get update && apt-get install -y \
    software-properties-common \
    build-essential \
    cmake \
    git \
    vim

# C++20 for GNU 11.X.X
RUN add-apt-repository ppa:ubuntu-toolchain-r/test
RUN apt-get update && apt-get install -y \
    g++-11

# Install Rosdep
RUN apt-get update && apt-get install -y \
    python3-rosdep

# Install Python Packages
RUN apt-get update && apt-get install -y \
    python3-pip

RUN pip3 install --upgrade \
    numpy \
    pandas \
    pillow \
    matplotlib

# Required Libraries
RUN apt-get update && apt-get install -y \
    valgrind \
    libeigen3-dev


# Source ROS setup.bash when the container starts
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && exec bash"]

