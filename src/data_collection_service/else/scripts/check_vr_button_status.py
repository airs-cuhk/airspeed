#!/usr/bin/env python3
"""
VR Button Control Status Check
VR按钮控制状态检查

检查VR按钮控制功能的状态和数据存储情况
"""

import os
import json
import time
import subprocess
import sys

def check_vr_data_file():
    """检查VR数据文件"""
    vr_data_file = "/home/airspeedbox/code/vr_robot_control_ws/src/vr_teleoperation/vr_data.json"
    
    print("🔍 检查VR数据文件...")
    print(f"文件路径: {vr_data_file}")
    
    if not os.path.exists(vr_data_file):
        print("❌ VR数据文件不存在")
        return False
    
    try:
        with open(vr_data_file, 'r', encoding='utf-8') as f:
            vr_data = json.load(f)
        
        print("✅ VR数据文件存在")
        
        # 检查左手柄按钮数据
        if 'left' in vr_data and 'button' in vr_data['left']:
            buttons = vr_data['left']['button']
            print(f"📊 左手柄按钮数量: {len(buttons)}")
            
            if len(buttons) >= 6:
                button_5 = buttons[4].get('value', 0) if len(buttons) > 4 else 0
                button_6 = buttons[5].get('value', 0) if len(buttons) > 5 else 0
                print(f"🔘 第五个按钮状态: {button_5}")
                print(f"🔘 第六个按钮状态: {button_6}")
                return True
            else:
                print("❌ 按钮数量不足")
                return False
        else:
            print("❌ 缺少左手柄按钮数据")
            return False
            
    except Exception as e:
        print(f"❌ 读取VR数据文件失败: {e}")
        return False

def check_data_storage_directory():
    """检查数据存储目录"""
    data_dir = "/home/airspeedbox/code/vr_robot_control_ws/src/data_storage/data"
    
    print("\n📁 检查数据存储目录...")
    print(f"目录路径: {data_dir}")
    
    if not os.path.exists(data_dir):
        print("❌ 数据存储目录不存在")
        return False
    
    print("✅ 数据存储目录存在")
    
    # 列出文件
    files = os.listdir(data_dir)
    print(f"📄 文件数量: {len(files)}")
    
    if files:
        print("📋 文件列表:")
        for file in sorted(files):
            file_path = os.path.join(data_dir, file)
            size = os.path.getsize(file_path)
            mtime = os.path.getmtime(file_path)
            mtime_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))
            print(f"  - {file} ({size:,} bytes, {mtime_str})")
    
    return True

def check_ros_nodes():
    """检查ROS节点状态"""
    print("\n🤖 检查ROS节点状态...")
    
    try:
        # 检查data_storage节点
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            data_storage_node = [node for node in nodes if 'data_storage' in node]
            
            if data_storage_node:
                print(f"✅ 数据存储节点运行中: {data_storage_node[0]}")
            else:
                print("❌ 数据存储节点未运行")
                return False
        else:
            print("❌ 无法获取ROS节点列表")
            return False
        
        # 检查话题
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            data_storage_topics = [topic for topic in topics if 'data_storage' in topic]
            
            if data_storage_topics:
                print(f"📡 数据存储话题: {data_storage_topics}")
            else:
                print("⚠️  未找到数据存储话题")
        
        return True
        
    except Exception as e:
        print(f"❌ 检查ROS节点失败: {e}")
        return False

def check_vr_sessions_directory():
    """检查VR会话目录"""
    sessions_dir = "/home/airspeedbox/code/vr_robot_control_ws/src/data_storage/data/vr_sessions"
    
    print("\n📂 检查VR会话目录...")
    print(f"目录路径: {sessions_dir}")
    
    if os.path.exists(sessions_dir):
        print("✅ VR会话目录存在")
        files = os.listdir(sessions_dir)
        print(f"📄 会话文件数量: {len(files)}")
        
        if files:
            print("📋 会话文件列表:")
            for file in sorted(files):
                file_path = os.path.join(sessions_dir, file)
                size = os.path.getsize(file_path)
                mtime = os.path.getmtime(file_path)
                mtime_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(mtime))
                print(f"  - {file} ({size:,} bytes, {mtime_str})")
    else:
        print("📁 VR会话目录不存在 (将在首次使用时创建)")

def main():
    """主函数"""
    print("🚀 VR按钮控制状态检查")
    print("=" * 50)
    
    # 检查各项状态
    vr_data_ok = check_vr_data_file()
    storage_ok = check_data_storage_directory()
    ros_ok = check_ros_nodes()
    check_vr_sessions_directory()
    
    print("\n" + "=" * 50)
    print("📊 状态总结:")
    print(f"VR数据文件: {'✅ 正常' if vr_data_ok else '❌ 异常'}")
    print(f"数据存储: {'✅ 正常' if storage_ok else '❌ 异常'}")
    print(f"ROS节点: {'✅ 正常' if ros_ok else '❌ 异常'}")
    
    if vr_data_ok and storage_ok and ros_ok:
        print("\n🎉 VR按钮控制功能状态正常!")
        print("💡 现在可以通过VR手柄按钮控制数据存储:")
        print("   - 第六个按钮: 开始/停止记录")
        print("   - 第五个按钮: 存储数据")
    else:
        print("\n⚠️  部分功能异常，请检查上述问题")
        sys.exit(1)

if __name__ == '__main__':
    main()

