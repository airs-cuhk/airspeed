#!/bin/bash

# Sense Interface 一键启动脚本
# 用于快速启动Realsense相机节点

set -e  # 遇到错误时退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的消息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否在正确的目录
check_workspace() {
    if [ ! -f "install/setup.bash" ]; then
        print_error "未找到install/setup.bash文件，请确保在ROS2工作空间根目录下运行此脚本"
        print_info "当前目录: $(pwd)"
        print_info "请切换到工作空间根目录: cd /home/airspeedbox/code/vr_robot_control_ws"
        exit 1
    fi
}

# 检查conda环境
check_conda_env() {
    if ! command -v conda &> /dev/null; then
        print_warning "未找到conda命令，跳过conda环境激活"
        return 0
    fi
    
    if conda info --envs | grep -q "ros2_env"; then
        print_info "激活conda环境: ros2_env"
        source $(conda info --base)/etc/profile.d/conda.sh
        conda activate ros2_env
        print_success "conda环境激活成功"
    else
        print_warning "未找到ros2_env conda环境，继续使用当前环境"
    fi
}

# 检查ROS2环境
check_ros2_env() {
    if [ -z "$ROS_DISTRO" ]; then
        print_info "设置ROS2环境"
        source /opt/ros/humble/setup.bash
    fi
    print_success "ROS2环境设置完成"
}

# 检查相机设备
check_camera() {
    print_info "检查Realsense相机设备..."
    if lsusb | grep -q "Intel"; then
        print_success "检测到Intel Realsense设备"
    else
        print_warning "未检测到Intel Realsense设备，请检查USB连接"
    fi
}

# 检查依赖
check_dependencies() {
    print_info "检查Python依赖..."
    if python3 -c "import pyrealsense2" 2>/dev/null; then
        print_success "pyrealsense2已安装"
    else
        print_error "pyrealsense2未安装，请运行: pip install pyrealsense2 opencv-python"
        exit 1
    fi
}

# 检查构建依赖
check_build_dependencies() {
    print_info "检查构建依赖..."
    
    # 检查catkin_pkg（ROS2构建必需）
    if ! python3 -c "import catkin_pkg" 2>/dev/null; then
        print_warning "catkin_pkg未安装，正在安装..."
        pip install catkin_pkg
        if [ $? -ne 0 ]; then
            print_error "catkin_pkg安装失败，请手动安装: pip install catkin_pkg"
            exit 1
        fi
        print_success "catkin_pkg安装成功"
    else
        print_info "构建依赖检查通过"
    fi
}

# 检查包是否完整安装
is_package_complete() {
    # 检查关键文件和目录
    if [ ! -d "install/sensor_interface" ]; then
        return 1
    fi
    
    # 检查 package.xml
    if [ ! -f "install/sensor_interface/share/sensor_interface/package.xml" ]; then
        return 1
    fi
    
    # 检查 launch 目录
    if [ ! -d "install/sensor_interface/share/sensor_interface/launch" ]; then
        return 1
    fi
    
    # 检查 launch 文件
    if [ ! -f "install/sensor_interface/share/sensor_interface/launch/realsense_camera.launch.py" ]; then
        return 1
    fi
    
    return 0
}

# 构建包
build_package() {
    print_info "构建sensor_interface包..."
    
    # 检查包是否完整安装
    if is_package_complete; then
        print_info "sensor_interface包已完整安装，跳过构建"
        return 0
    else
        print_warning "sensor_interface包不完整，需要重新构建"
    fi
    
    # 检查构建依赖
    check_build_dependencies
    
    # 构建包
    colcon build --packages-select sensor_interface
    
    # 检查构建是否成功
    if [ $? -ne 0 ]; then
        print_error "构建失败，请检查错误信息"
        exit 1
    fi
    
    # 验证安装是否完整
    if ! is_package_complete; then
        print_error "构建完成但包不完整，请检查错误信息"
        exit 1
    fi
    
    print_success "sensor_interface包构建成功"
    
    # 重新加载环境
    source install/setup.bash
}

# 启动相机节点
launch_camera() {
    print_info "启动Realsense相机节点..."
    
    # 检查launch文件是否存在
    if [ ! -f "src/sensor_interface/launch/realsense_camera.launch.py" ]; then
        print_error "未找到launch文件: src/sensor_interface/launch/realsense_camera.launch.py"
        exit 1
    fi
    
    # 启动节点
    print_info "执行命令: ros2 launch sensor_interface realsense_camera.launch.py"
    ros2 launch sensor_interface realsense_camera.launch.py
}

# 主函数
main() {
    print_info "=== Sensor Interface 一键启动脚本 ==="
    print_info "开始启动Realsense相机节点..."
    
    # 检查工作空间
    check_workspace
    
    # 激活conda环境
    check_conda_env
    
    # 设置ROS2环境
    check_ros2_env
    
    # 源工作空间
    print_info "源工作空间环境..."
    source install/setup.bash
    print_success "工作空间环境设置完成"
    
    # 检查相机设备
    check_camera
    
    # 检查依赖
    check_dependencies
    
    # 构建包
    build_package
    
    # 启动相机节点
    launch_camera
}

# 捕获中断信号
trap 'print_info "收到中断信号，正在退出..."; exit 0' INT TERM

# 运行主函数
main "$@"
