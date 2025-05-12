#!/bin/bash

# Build the Docker image if not already built
docker build -t ros2-dev .

# Run the container with X11 forwarding
docker run -it --rm \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --network=host \
    --name ros2-dev-container \
    ros2-dev 