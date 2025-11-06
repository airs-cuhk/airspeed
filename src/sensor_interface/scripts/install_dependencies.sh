#!/bin/bash
"""
Realsense Camera Dependencies Installation Script
Realsense相机依赖安装脚本
"""

set -e

echo "Installing Realsense Camera Dependencies..."

# 检查是否为root用户
if [ "$EUID" -eq 0 ]; then
    echo "Please do not run this script as root"
    exit 1
fi

# 更新包列表
echo "Updating package list..."
sudo apt-get update

# 安装基础依赖
echo "Installing basic dependencies..."
sudo apt-get install -y \
    curl \
    wget \
    gnupg \
    lsb-release \
    software-properties-common

# 注册Intel Realsense公钥
echo "Registering Intel Realsense public key..."
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || \
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# 添加Intel Realsense存储库
echo "Adding Intel Realsense repository..."
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# 更新包列表
echo "Updating package list with new repository..."
sudo apt-get update

# 安装Realsense SDK
echo "Installing Realsense SDK..."
sudo apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    librealsense2-dbg

# 安装Python依赖
echo "Installing Python dependencies..."
pip3 install --user \
    pyrealsense2 \
    opencv-python \
    numpy

# 设置USB权限
echo "Setting up USB permissions..."
sudo usermod -a -G video $USER

# 创建udev规则
echo "Creating udev rules for Realsense devices..."
sudo tee /etc/udev/rules.d/99-realsense-libusb.rules > /dev/null <<EOF
# Intel RealSense D435i
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3a", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3c", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b3d", MODE="0666", GROUP="plugdev"
EOF

# 重新加载udev规则
echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger

# 验证安装
echo "Verifying installation..."
if command -v realsense-viewer &> /dev/null; then
    echo "✓ Realsense SDK installed successfully"
else
    echo "✗ Realsense SDK installation failed"
    exit 1
fi

if python3 -c "import pyrealsense2" &> /dev/null; then
    echo "✓ Python pyrealsense2 package installed successfully"
else
    echo "✗ Python pyrealsense2 package installation failed"
    exit 1
fi

if python3 -c "import cv2" &> /dev/null; then
    echo "✓ OpenCV Python package installed successfully"
else
    echo "✗ OpenCV Python package installation failed"
    exit 1
fi

echo ""
echo "Installation completed successfully!"
echo ""
echo "Next steps:"
echo "1. Log out and log back in to apply group changes"
echo "2. Connect your Realsense D435i camera"
echo "3. Test the camera with: realsense-viewer"
echo "4. Build your ROS2 workspace: colcon build"
echo "5. Source the workspace: source install/setup.bash"
echo "6. Launch the camera node: ros2 launch sensor_interface realsense_camera.launch.py"
echo ""
echo "If you encounter permission issues, try:"
echo "sudo chmod 666 /dev/video*"




