FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV VEHICLE_WS=/workspace/vehicle/02mapping/catkin_ws
ENV LIVOX_WS=/workspace/livox_ws

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    libapt-pkg-dev \
    python3-rosdep \
    python3-wstool \
    python3-serial \
    python3-pyproj \
    python3-numpy \
    python3-matplotlib \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libgtsam-dev \
    ros-noetic-catkin \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-octomap-server \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd /workspace/Livox-SDK2 && \
    mkdir -p build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    make install

RUN mkdir -p ${LIVOX_WS}/src && \
    cd ${LIVOX_WS}/src && \
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git && \
    /bin/bash -lc "source /opt/ros/noetic/setup.bash && cd ${LIVOX_WS}/src/livox_ros_driver2 && ./build.sh ROS1"

COPY . /workspace/vehicle

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && source ${LIVOX_WS}/devel/setup.bash && cd ${VEHICLE_WS} && catkin_make --pkg fast_lio2 data_listener2 clouder_gps -j4"

RUN chmod +x /workspace/vehicle/docker/mapping/two-images/car/entrypoint.sh

ENTRYPOINT ["/workspace/vehicle/docker/mapping/two-images/car/entrypoint.sh"]
