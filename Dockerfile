FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-numpy python3-opencv \
    ros-humble-rosidl-default-generators ros-humble-cv-bridge ros-humble-mavros ros-humble-mavros-extras \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY src /ws/src
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install
CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch start autoland.launch.py"]
