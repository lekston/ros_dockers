#!/bin/bash

# Build the Docker image if not already built
docker build -t ros1-dev-liosam -f Dockerfile_ros1_lio_sam .

# Run the container with X11 forwarding and NVIDIA GPU access
sudo docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="`pwd`:/root/src_wip" \
    --volume="/opt/mnt/data/10_slam/:/data/" \
    --network=host \
    --name ros1-dev-liosam \
    ros1-dev-liosam