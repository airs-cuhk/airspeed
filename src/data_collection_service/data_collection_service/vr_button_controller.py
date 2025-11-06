#!/usr/bin/env python3
"""
VR Button Controller Module
VR按钮控制器模块

该模块负责处理VR左手柄按钮数据，实现手动控制数据存储功能：
- 第六个按钮：开始/停止记录数据
- 第五个按钮：存储当前记录的数据
"""

import json
import os
import time
import threading
from collections import deque
from std_msgs.msg import Float32MultiArray


class VRButtonController:
    """VR按钮控制器类"""
    
    def __init__(self, config, parent_node):
        self.node = parent_node
        self.config = config
        
        # VR按钮话题配置
        self.vr_button_topic = config['vr_button_control']['vr_button_topic']
        self.vr_button_message_type = config['vr_button_control']['vr_button_message_type']
        
        # 按钮状态跟踪
        self.button_5_prev = 0  # 第五个按钮前一次状态
        self.button_6_prev = 0  # 第六个按钮前一次状态
        
        # 记录状态
        self.recording_enabled = False
        self.recording_start_time = None
        self.session_data = []  # 当前会话的数据
        
        # 按钮事件回调
        self.on_button_5_press = None  # 第五个按钮按下回调
        self.on_button_6_press = None  # 第六个按钮按下回调
        
        # 数据统计
        self.button_stats = {
            'button_5_presses': 0,
            'button_6_presses': 0,
            'recording_sessions': 0,
            'total_recording_time': 0.0
        }
        
        # 创建ROS话题订阅器
        self.vr_button_subscription = self.node.create_subscription(
            Float32MultiArray,
            self.vr_button_topic,
            self.vr_button_callback,
            10
        )
        
        self.get_logger().info("VR Button Controller initialized")
    
    def vr_button_callback(self, msg):
        """VR按钮话题回调函数"""
        try:
            # 验证按钮数据
            if not self.validate_button_data(msg.data):
                return
            
            # 提取按钮数据（假设前6个元素是按钮状态）
            button_data = msg.data[:6] if len(msg.data) >= 6 else msg.data + [0] * (6 - len(msg.data))
            
            # 获取第五个和第六个按钮状态
            button_5 = button_data[4] if len(button_data) > 4 else 0
            button_6 = button_data[5] if len(button_data) > 5 else 0
            
            # 检测按钮按下事件（从0到1的跳变）
            if button_5 == 1 and self.button_5_prev == 0:
                self.handle_button_5_press()
            
            if button_6 == 1 and self.button_6_prev == 0:
                self.handle_button_6_press()
            
            # 更新按钮状态
            self.button_5_prev = button_5
            self.button_6_prev = button_6
            
        except Exception as e:
            self.get_logger().error(f"Error processing VR button callback: {e}")
    
    def validate_button_data(self, button_data):
        """验证按钮数据"""
        if not button_data or len(button_data) < 2:
            return False
        
        # 检查按钮值是否在有效范围内 (0-1)
        for button_value in button_data:
            if not isinstance(button_value, (int, float)) or button_value < 0 or button_value > 1:
                return False
        
        return True
    
    
    def handle_button_5_press(self):
        """处理第五个按钮按下事件 - 存储数据"""
        try:
            self.button_stats['button_5_presses'] += 1
            
            if self.recording_enabled:
                # 如果正在记录，立即存储当前会话数据
                self.save_current_session()
                self.get_logger().info("Button 5 pressed: Current session data saved")
            else:
                # 如果没有在记录，提示用户先开始记录
                self.get_logger().warn("Button 5 pressed: No active recording session to save")
                
        except Exception as e:
            self.get_logger().error(f"Error handling button 5 press: {e}")
    
    def handle_button_6_press(self):
        """处理第六个按钮按下事件 - 开始/停止记录"""
        try:
            self.button_stats['button_6_presses'] += 1
            
            if not self.recording_enabled:
                # 开始记录
                self.start_recording()
            else:
                # 停止记录
                self.stop_recording()
                
        except Exception as e:
            self.get_logger().error(f"Error handling button 6 press: {e}")
    
    def start_recording(self):
        """开始记录数据"""
        try:
            self.recording_enabled = True
            self.recording_start_time = time.time()
            self.session_data = []
            self.button_stats['recording_sessions'] += 1
            
            self.get_logger().info("VR Button Control: Recording started")
            
            # 调用回调函数
            if self.on_button_6_press is not None:
                self.on_button_6_press('start')
                
        except Exception as e:
            self.get_logger().error(f"Error starting recording: {e}")
    
    def stop_recording(self):
        """停止记录数据"""
        try:
            if self.recording_enabled:
                # 计算记录时间
                if self.recording_start_time is not None:
                    session_time = time.time() - self.recording_start_time
                    self.button_stats['total_recording_time'] += session_time
                
                self.recording_enabled = False
                self.get_logger().info(f"VR Button Control: Recording stopped (Session time: {session_time:.2f}s)")
                
                # 调用回调函数
                if self.on_button_6_press is not None:
                    self.on_button_6_press('stop')
            else:
                self.get_logger().warn("VR Button Control: No active recording to stop")
                
        except Exception as e:
            self.get_logger().error(f"Error stopping recording: {e}")
    
    def save_current_session(self):
        """保存当前会话数据"""
        try:
            if not self.recording_enabled:
                self.get_logger().warn("No active recording session to save")
                return
            
            if not self.session_data:
                self.get_logger().warn("No data in current session to save")
                return
            
            # 创建会话数据记录
            session_record = {
                'session_id': f"session_{int(time.time())}",
                'start_time': self.recording_start_time,
                'end_time': time.time(),
                'duration': time.time() - self.recording_start_time if self.recording_start_time else 0,
                'data_count': len(self.session_data),
                'data': self.session_data.copy()
            }
            
            # 保存到文件
            self.save_session_to_file(session_record)
            
            # 清空当前会话数据
            self.session_data.clear()
            
            self.get_logger().info(f"Session saved: {session_record['data_count']} data points, duration: {session_record['duration']:.2f}s")
            
        except Exception as e:
            self.get_logger().error(f"Error saving current session: {e}")
    
    def save_session_to_file(self, session_record):
        """保存会话数据到文件"""
        try:
            # 创建会话数据目录
            session_dir = os.path.join(self.config['storage']['storage_directory'], 'vr_sessions')
            os.makedirs(session_dir, exist_ok=True)
            
            # 生成文件名
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"vr_session_{timestamp}_{session_record['session_id']}.json"
            filepath = os.path.join(session_dir, filename)
            
            # 保存会话数据
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(session_record, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f"Session data saved to: {filepath}")
            
        except Exception as e:
            self.get_logger().error(f"Error saving session to file: {e}")
    
    def add_data(self, data_record):
        """添加数据到当前会话"""
        try:
            if self.recording_enabled:
                # 添加时间戳
                data_record['session_timestamp'] = time.time()
                data_record['session_elapsed'] = time.time() - self.recording_start_time if self.recording_start_time else 0
                
                self.session_data.append(data_record)
                
                if len(self.session_data) % 100 == 0:  # 每100个数据点记录一次
                    self.get_logger().debug(f"Session data count: {len(self.session_data)}")
                    
        except Exception as e:
            self.get_logger().error(f"Error adding data to session: {e}")
    
    def get_recording_status(self):
        """获取记录状态"""
        return {
            'recording_enabled': self.recording_enabled,
            'recording_start_time': self.recording_start_time,
            'session_data_count': len(self.session_data),
            'session_duration': time.time() - self.recording_start_time if self.recording_start_time and self.recording_enabled else 0,
            'button_stats': self.button_stats.copy()
        }
    
    def get_button_stats(self):
        """获取按钮统计信息"""
        return self.button_stats.copy()
    
    def set_button_callbacks(self, button_5_callback=None, button_6_callback=None):
        """设置按钮回调函数"""
        self.on_button_5_press = button_5_callback
        self.on_button_6_press = button_6_callback
        self.get_logger().info("Button callbacks set")
    
    def read_vr_button_data(self):
        """读取VR按钮数据"""
        try:
            # 返回当前按钮状态
            return {
                'timestamp': time.time(),
                'button_5': self.button_5_prev,
                'button_6': self.button_6_prev
            }
        except Exception as e:
            self.get_logger().error(f"Error reading VR button data: {e}")
            return None
    
    def get_logger(self):
        """获取日志记录器"""
        return self.node.get_logger()

