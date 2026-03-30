FROM ros:noetic-ros-base-focal

ENV DEBIAN_FRONTEND=noninteractive
ENV CLOUD_WS=/workspace/cloud_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ca-certificates \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    ros-noetic-cv-bridge \
    ros-noetic-message-generation \
    ros-noetic-message-runtime \
    ros-noetic-pcl-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
RUN mkdir -p ${CLOUD_WS}/src
COPY 02mapping/catkin_ws/src/data_sender ${CLOUD_WS}/src/data_sender

RUN /bin/bash -lc "source /opt/ros/noetic/setup.bash && cd ${CLOUD_WS} && catkin_make --pkg data_listener2 --make-args cloud_recv -j4"

COPY docker/mapping/two-images/cloud/entrypoint.sh /workspace/entrypoint.sh
RUN chmod +x /workspace/entrypoint.sh

ENTRYPOINT ["/workspace/entrypoint.sh"]
