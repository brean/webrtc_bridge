ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
      qtwayland5 \
      python3-colcon-ros \
      python3-pip \
      python3-pkg-resources \
      python3-opencv \
      python3-venv \
      python3-aiortc \
      python3-aiohttp \
      ros-$ROS_DISTRO-cv-bridge \
  && rm -rf /var/lib/apt/lists/*

ARG COLCON_WS=/root/ws/

ENV PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"

COPY ./docker/entrypoint.sh /ros2_ws_entrypoint.sh
RUN chmod a+x /ros2_ws_entrypoint.sh
ENTRYPOINT [ "/ros2_ws_entrypoint.sh" ]

RUN mkdir -p ${COLCON_WS}/src

COPY ./webrtc_bridge/requirements.txt /tmp/requirements.txt
# install requirements.txt
RUN pip3 install -r /tmp/requirements.txt --break-system-packages

COPY ./webrtc_bridge ${COLCON_WS}/src/webrtc_bridge

WORKDIR ${COLCON_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
  colcon build

CMD [ "ros2", "launch", "webrtc_bridge", "webrtc_bridge.launch.py" ]