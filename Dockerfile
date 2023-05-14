FROM osrf/ros:melodic-desktop-full as base-image

RUN set -ex \
    && apt-get update -yq \
    && apt-get install -yq --no-install-recommends --no-upgrade \
    python-serial \
    ros-melodic-moveit \
    ros-melodic-rqt-joint-trajectory-controller \
    && apt-get clean

COPY . /catkin_ws

RUN git clone https://github.com/arebgun/dynamixel_motor.git --branch master /dynamixel_motor

RUN bash /ros_entrypoint.sh catkin_make install --source /dynamixel_motor -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic

RUN rosdep install --from-paths /catkin_ws --ignore-src --rosdistro=melodic --default-yes

COPY <<EOF /root/.ignition/fuel/config.yaml
---
# The list of servers.
servers:
  -
    name: osrf
    url: https://api.ignitionrobotics.org

  # -
    # name: another_server
    # url: https://myserver

# Where are the assets stored in disk.
# cache:
#   path: /tmp/ignition/fuel

EOF

WORKDIR /catkin_ws

RUN rm -rf build devel && bash /ros_entrypoint.sh catkin_make

FROM base-image as cyton-gamma-300

