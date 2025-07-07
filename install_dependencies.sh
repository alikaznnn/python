#!/bin/bash

# ROS2 + PyTorch Demo - Dependency Installation Script
# This script helps install all necessary dependencies for the project

set -e  # Exit on any error

echo "🚀 Installing dependencies for ROS2 + PyTorch Demo..."

# Check if running on Ubuntu
if ! grep -q "Ubuntu" /etc/os-release; then
    echo "⚠️  This script is designed for Ubuntu. Please install dependencies manually on other systems."
    exit 1
fi

# Update package list
echo "📦 Updating package list..."
sudo apt update

# Install Python3 and pip if not already installed
echo "🐍 Installing Python3 and pip..."
sudo apt install -y python3 python3-pip python3-venv

# Install ROS2 dependencies
echo "🤖 Installing ROS2 system dependencies..."
sudo apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# Check if ROS2 is already installed
if ! command -v ros2 &> /dev/null; then
    echo "📥 ROS2 not found. Installing ROS2 Humble..."
    
    # Add ROS2 apt repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    # Update and install ROS2
    sudo apt update
    sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
    
    echo "✅ ROS2 Humble installed successfully!"
else
    echo "✅ ROS2 is already installed."
fi

# Install PyTorch and other Python dependencies
echo "🔥 Installing PyTorch and Python dependencies..."

# Create virtual environment (optional but recommended)
if [ "$1" == "--venv" ]; then
    echo "🌟 Creating virtual environment..."
    python3 -m venv ros2_pytorch_env
    source ros2_pytorch_env/bin/activate
    echo "Virtual environment created and activated."
fi

# Install Python packages
pip3 install --upgrade pip
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install numpy matplotlib

# Install additional ROS2 Python packages
pip3 install rclpy

echo "🎯 Testing PyTorch installation..."
python3 -c "import torch; print(f'PyTorch version: {torch.__version__}'); print(f'CUDA available: {torch.cuda.is_available()}')"

echo "🎯 Testing ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "ROS2 command found: $(which ros2)"
else
    echo "⚠️  ROS2 command not found. You may need to source the setup file:"
    echo "    source /opt/ros/humble/setup.bash"
fi

# Create workspace if it doesn't exist
WORKSPACE_DIR="$HOME/ros2_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "📁 Creating ROS2 workspace at $WORKSPACE_DIR..."
    mkdir -p "$WORKSPACE_DIR/src"
fi

echo ""
echo "🎉 Installation completed successfully!"
echo ""
echo "📋 Next steps:"
echo "1. Source ROS2 environment:"
echo "   source /opt/ros/humble/setup.bash"
echo ""
echo "2. Copy this project to your ROS2 workspace:"
echo "   cp -r ros2_pytorch_demo $WORKSPACE_DIR/src/"
echo ""
echo "3. Build the project:"
echo "   cd $WORKSPACE_DIR"
echo "   colcon build --packages-select ros2_pytorch_demo"
echo ""
echo "4. Source the workspace:"
echo "   source install/setup.bash"
echo ""
echo "5. Launch the demo:"
echo "   ros2 launch ros2_pytorch_demo demo_launch.py"
echo ""

if [ "$1" == "--venv" ]; then
    echo "💡 Remember to activate the virtual environment:"
    echo "   source ros2_pytorch_env/bin/activate"
    echo ""
fi

echo "🔗 For more information, see the README.md file."
echo "Happy coding! 🚀🤖"