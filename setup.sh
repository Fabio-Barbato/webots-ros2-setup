#!/bin/bash

echo "🛠️  Setting up Webots-ROS2 Environment..."
echo ""

# Get repository directory
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Create shared bridge directory
BRIDGE_DIR="$HOME/webots_ros2_bridge"
echo "📁 Creating shared directory: $BRIDGE_DIR"
mkdir -p "$BRIDGE_DIR"

# Create Webots project directory
WEBOTS_DIR="$HOME/Desktop/webots_robot_project"
echo "📁 Creating Webots project: $WEBOTS_DIR"
mkdir -p "$WEBOTS_DIR/controllers/bridge_controller"
mkdir -p "$WEBOTS_DIR/worlds"

# Copy controller
echo "📄 Copying Webots controller..."
cp "$REPO_DIR/webots_controller/bridge_controller.py" \
   "$WEBOTS_DIR/controllers/bridge_controller/"

# Update path in controller
USERNAME=$(whoami)
sed -i.bak "s|Path.home()|Path(\"/Users/$USERNAME/webots_ros2_bridge\")|g" \
    "$WEBOTS_DIR/controllers/bridge_controller/bridge_controller.py"

echo "✅ Webots setup complete!"
echo ""
echo "Next steps:"
echo "  1. Run: ./start.sh"
echo "  2. Inside container, build workspace:"
echo "     cd ~/ros2_ws"
echo "     source /opt/ros/humble/setup.bash"
echo "     colcon build"
echo "     source install/setup.bash"
