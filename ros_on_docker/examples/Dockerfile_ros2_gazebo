FROM osrf/ros:humble-desktop

# Install vim and tmux
RUN apt-get update && apt-get install -y \
    vim \
    tmux \
    ros-humble-turtlebot3-simulations \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    && rm -rf /var/lib/apt/lists/*

# Set up the workspace
WORKDIR /ros2_ws

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH
ENV GAZEBO_RESOURCE_PATH=/opt/ros/humble/share/turtlebot3_gazebo:$GAZEBO_RESOURCE_PATH

# Setup entrypoint
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"] 