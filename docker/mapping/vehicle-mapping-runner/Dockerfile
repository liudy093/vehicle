FROM arm64v8/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    bash \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    python3 \
    python3-pip \
    software-properties-common \
    util-linux \
 && curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list \
 && echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list \
 && apt-get update \
 && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base \
    ros-noetic-ros-base \
 && pip3 install --no-cache-dir easydict python-can==3.3.4 PyMySQL \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/vehicle

COPY scripts/ /opt/vehicle/scripts/
COPY config/ /opt/vehicle/config/
COPY deploy/ /opt/vehicle/deploy/
COPY docker/mapping/vehicle-mapping-runner/entrypoint.sh /usr/local/bin/vehicle-mapping-runner-entrypoint

RUN chmod +x /usr/local/bin/vehicle-mapping-runner-entrypoint \
 && find /opt/vehicle/scripts -type f -name '*.sh' -exec chmod +x {} + \
 && find /opt/vehicle/deploy/shared/k8s -type f -name '*.sh' -exec chmod +x {} +

ENTRYPOINT ["/usr/local/bin/vehicle-mapping-runner-entrypoint"]
