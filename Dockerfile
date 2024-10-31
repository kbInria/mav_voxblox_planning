# Start from the ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /opt/catkin_ws/

# Install necessary packages
RUN apt-get update && apt-get install -y \
    git \
    vim \
    wget \
    ros-noetic-cmake-modules \
    libyaml-cpp-dev \
    protobuf-compiler \
    autoconf \
    libompl-dev \
    build-essential \
    python3-rosdep \
    python3-catkin-tools \
    libtool \
    && rm -rf /var/lib/apt/lists/*

RUN catkin init \
    && catkin config --extend /opt/ros/noetic \
    && catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && catkin config --merge-devel

# Set the working directory
WORKDIR /opt/catkin_ws/src/

# Clone the ROS package from GitHub
RUN git clone --single-branch --branch dev/avenue https://github.com/kbInria/mav_voxblox_planning.git \
    && wstool init . /opt/catkin_ws/src/mav_voxblox_planning/install/install_https.rosinstall \
    && wstool update \
    && rm /opt/catkin_ws/src/mav_voxblox_planning/voxblox_rrt_planner/CATKIN_IGNORE
# Removing CATKIN_IGNORE because -DCMAKE_CXX_STANDARD=14 fixed voxblox_rrt_planner build

# # Go back to the workspace root
WORKDIR /opt/catkin_ws/

# Initialize and build the Catkin workspace
# TODO: need to remove the catkin_ignore in mav_voxblox_planning/voxblox_rrt_planner before this build
RUN catkin build protobuf_catkin \
    && catkin build mav_voxblox_planning voxblox_rrt_planner -DCMAKE_CXX_STANDARD=14

# # Source the setup.bash so that the package is available in the environment
RUN echo "source /opt/catkin_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# # Set the entrypoint
CMD ["/bin/bash", "-c", "source /opt/ros/noetic/setup.bash && source /opt/catkin_ws/devel/setup.bash && exec bash"]
