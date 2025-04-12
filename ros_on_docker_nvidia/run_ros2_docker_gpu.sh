#!/bin/bash

# Build the Docker image if not already built
docker build -t ros2-dev-nvidia .

# Run the container with X11 forwarding and NVIDIA GPU access
docker run -it --rm \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    --name ros2-dev-gpu-container \
    ros2-dev-nvidia 