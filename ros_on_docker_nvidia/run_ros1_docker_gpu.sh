#!/bin/bash

# Build the Docker image if not already built
docker build -t ros1-dev-gpu-base -f Dockerfile_ros1_gpu_base .
# docker build -t ros1-dev-gpu-liosam -f Dockerfile_ros1_gpu_lio_sam .

# Run the container with X11 forwarding and NVIDIA GPU access
docker run -it --rm \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="`pwd`:/root/src_wip" \
    --volume="/opt/mnt/data/10_slam/:/data/" \
    --network=host \
    --name ros1-dev-gpu-container \
    ros1-dev-gpu-base
