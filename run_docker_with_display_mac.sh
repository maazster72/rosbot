#!/bin/bash

# Set the image name
IMAGE_NAME="rosbot-image"

# Check if the Docker image exists
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "Docker image '$IMAGE_NAME' does not exist. Please build it first."
    exit 1
fi

# Allow access to the X11 server for local Docker containers
echo "Allowing Docker container access to X11 server..."
xhost +local:docker

# Run the Docker container with display access
echo "Starting $IMAGE_NAME Docker container..."
docker run -it \
    --rm \
    --env DISPLAY=$DISPLAY \
    --env XAUTHORITY=$HOME/.Xauthority \
    --env XDG_RUNTIME_DIR=/tmp/runtime-root \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $HOME/.Xauthority:$HOME/.Xauthority \
    $IMAGE_NAME

# Revoke access to X11 server after the container stops
echo "Revoking access to X11 server..."
xhost -local:docker

