#!/bin/bash

echo "Starting ROS2 Container..."
echo ""

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "arm64" ]; then
    PLATFORM="linux/arm64"
elif [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux/amd64"
else
    echo "Unknown architecture: $ARCH"
    exit 1
fi

# Get repository directory
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Shared directories
BRIDGE_DIR="$HOME/webots_ros2_bridge"
WORKSPACE_DIR="$REPO_DIR/ros2_workspace"

# Create bridge directory if not exists
mkdir -p "$BRIDGE_DIR"

# Check if container exists
if [ "$(docker ps -aq -f name=ros2_webots)" ]; then
    echo "Container 'ros2_webots' already exists"
    read -p "Start existing container? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        docker start ros2_webots
        docker exec -it ros2_webots bash
        exit 0
    else
        echo "Removing existing container..."
        docker stop ros2_webots 2>/dev/null
        docker rm ros2_webots 2>/dev/null
    fi
fi

echo "Creating new container..."
echo ""

docker run -it \
    --platform $PLATFORM \
    -e DISPLAY=host.docker.internal:0 \
    -v "$BRIDGE_DIR":/shared \
    -v "$WORKSPACE_DIR":/root/ros2_ws \
    --name ros2_webots \
    ros2_humble_webots

echo ""
echo "Container stopped."
