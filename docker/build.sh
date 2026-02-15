#!/bin/bash

echo "🐳 Building Docker Image for ROS2 Humble..."
echo ""

# Detect architecture
ARCH=$(uname -m)

if [ "$ARCH" = "arm64" ]; then
    PLATFORM="linux/arm64"
    echo "🍎 Detected Apple Silicon (ARM64)"
elif [ "$ARCH" = "x86_64" ]; then
    PLATFORM="linux/amd64"
    echo "💻 Detected Intel/AMD (x86_64)"
else
    echo "❌ Unknown architecture: $ARCH"
    exit 1
fi

echo "🔨 Building for $PLATFORM..."
echo ""

cd "$(dirname "$0")"

docker build \
    --platform $PLATFORM \
    -t ros2_humble_webots \
    -f Dockerfile \
    .

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Docker image built successfully!"
    echo ""
    echo "Next step: Run ../setup.sh to configure workspace"
else
    echo ""
    echo "❌ Build failed!"
    exit 1
fi
