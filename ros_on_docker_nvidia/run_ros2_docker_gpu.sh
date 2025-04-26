#!/bin/bash

# Build the Docker image if not already built
docker build -t ros2-dev-gpu-s1 -f Dockerfile_ros_gpu_base .
docker build -t ros2-dev-gpu-kiss-icp -f Dockerfile_ros_kiss_icp ../..
docker build -t ros2-dev-gpu-liosam -f Dockerfile_ros_lio_sam .
docker build -t ros1-ros2-dev -f Dockerfile_ros1_bridge .

# Run the container with X11 forwarding and NVIDIA GPU access
docker run -it --rm \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="`pwd`:/ros2_ws/src_wip" \
    --volume="/opt/mnt/data/10_slam/:/ros2_ws/data/" \
    --network=host \
    --name ros2-dev-gpu-container \
    ros2-dev-gpu-liosam
