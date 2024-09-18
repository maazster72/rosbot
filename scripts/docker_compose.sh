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

# Start the Docker container using docker-compose
echo "Starting $IMAGE_NAME Docker container with docker-compose..."
docker-compose -f ../docker/docker-compose.yml up

# Revoke access to X11 server after the container stops
echo "Revoking access to X11 server..."
xhost -local:docker

