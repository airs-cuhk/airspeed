#!/usr/bin/env python3
"""
Data Subscriber Module
数据订阅器模块

该模块负责订阅robot_interface发布的话题，接收机器人数据
"""

import rclpy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
import threading
import time
import json
import os
import numpy as np
from collections import deque
from cv_bridge import CvBridge


class DataSubscriber:
    """数据订阅器类"""
    
    def __init__(self, config, parent_node):
        self.node = parent_node
        self.config = config
        self.data_sources = config['data_sources']
        
        # 数据缓冲区
        self.data_buffers = {}
        self.buffer_locks = {}
        
        # 订阅器字典
        self.subscribers = {}
        
        # 数据统计
        self.data_stats = {
            'joint_angles': {'count': 0, 'last_received': None},
            'cartesian_pose': {'count': 0, 'last_received': None},
            'vr_left_buttons': {'count': 0, 'last_received': None},
            'camera_rgb': {'count': 0, 'last_received': None},
            'camera_depth': {'count': 0, 'last_received': None},
            'camera_pointcloud': {'count': 0, 'last_received': None}
        }
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 初始化订阅器
        self.init_subscribers()
        
        self.get_logger().info("Data subscriber initialized")
    
    def init_subscribers(self):
        """初始化数据订阅器"""
        # 订阅关节角度数据
        if self.data_sources['joint_angles']['enabled']:
            topic_name = self.data_sources['joint_angles']['topic_name']
            self.subscribers['joint_angles'] = self.node.create_subscription(
                Float32MultiArray,
                topic_name,
                self.joint_angles_callback,
                10
            )
            self.data_buffers['joint_angles'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['joint_angles'] = threading.Lock()
            self.get_logger().info(f"Subscribed to joint angles: {topic_name}")
        
        # 订阅笛卡尔位姿数据
        if self.data_sources['cartesian_pose']['enabled']:
            topic_name = self.data_sources['cartesian_pose']['topic_name']
            self.subscribers['cartesian_pose'] = self.node.create_subscription(
                PoseStamped,
                topic_name,
                self.cartesian_pose_callback,
                10
            )
            self.data_buffers['cartesian_pose'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['cartesian_pose'] = threading.Lock()
            self.get_logger().info(f"Subscribed to cartesian pose: {topic_name}")
        
        # 订阅VR左手柄按钮数据
        if self.data_sources['vr_left_buttons']['enabled']:
            topic_name = self.data_sources['vr_left_buttons']['topic_name']
            self.subscribers['vr_left_buttons'] = self.node.create_subscription(
                Float32MultiArray,
                topic_name,
                self.vr_left_buttons_callback,
                10
            )
            self.data_buffers['vr_left_buttons'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['vr_left_buttons'] = threading.Lock()
            self.get_logger().info(f"Subscribed to VR left buttons: {topic_name}")
        
        # 订阅相机RGB图像数据
        if self.data_sources['camera_rgb']['enabled']:
            topic_name = self.data_sources['camera_rgb']['topic_name']
            self.subscribers['camera_rgb'] = self.node.create_subscription(
                Image,
                topic_name,
                self.camera_rgb_callback,
                10
            )
            self.data_buffers['camera_rgb'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['camera_rgb'] = threading.Lock()
            self.get_logger().info(f"Subscribed to camera RGB: {topic_name}")
        
        # 订阅相机深度图像数据
        if self.data_sources['camera_depth']['enabled']:
            topic_name = self.data_sources['camera_depth']['topic_name']
            self.subscribers['camera_depth'] = self.node.create_subscription(
                Image,
                topic_name,
                self.camera_depth_callback,
                10
            )
            self.data_buffers['camera_depth'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['camera_depth'] = threading.Lock()
            self.get_logger().info(f"Subscribed to camera depth: {topic_name}")
        
        # 订阅相机点云数据
        if self.data_sources['camera_pointcloud']['enabled']:
            topic_name = self.data_sources['camera_pointcloud']['topic_name']
            self.subscribers['camera_pointcloud'] = self.node.create_subscription(
                PointCloud2,
                topic_name,
                self.camera_pointcloud_callback,
                10
            )
            self.data_buffers['camera_pointcloud'] = deque(maxlen=self.config['storage']['buffer']['buffer_size'])
            self.buffer_locks['camera_pointcloud'] = threading.Lock()
            self.get_logger().info(f"Subscribed to camera pointcloud: {topic_name}")
    
    def joint_angles_callback(self, msg):
        """关节角度数据回调函数 - 接收来自robot_interface的数据"""
        try:
            # 提取数据
            joint_angles = list(msg.data)
            timestamp = time.time()
            
            # 数据验证
            if not self.validate_joint_angles(joint_angles):
                self.get_logger().warn(f"Invalid joint angles data: {joint_angles}")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': joint_angles,
                'source': 'joint_angles',
                'raw_data': joint_angles.copy(),
                'message_header': {
                    'frame_id': getattr(msg, 'header', {}).frame_id if hasattr(msg, 'header') else 'robot_base',
                    'stamp': getattr(msg, 'header', {}).stamp if hasattr(msg, 'header') else None
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['joint_angles']:
                self.data_buffers['joint_angles'].append(data_record)
            
            # 更新统计信息
            self.data_stats['joint_angles']['count'] += 1
            self.data_stats['joint_angles']['last_received'] = timestamp
            
            # 轻量验证日志（临时便于联调）
            self.get_logger().info("Joint angles received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing joint angles: {e}")
    
    def cartesian_pose_callback(self, msg):
        """笛卡尔位姿数据回调函数 - 接收来自robot_interface的数据"""
        try:
            # 提取数据
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orientation = [
                msg.pose.orientation.x, 
                msg.pose.orientation.y, 
                msg.pose.orientation.z, 
                msg.pose.orientation.w
            ]
            timestamp = time.time()
            
            # 数据验证
            if not self.validate_cartesian_pose(position, orientation):
                self.get_logger().warn(f"Invalid cartesian pose data: pos={position}, ori={orientation}")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': {
                    'position': position,
                    'orientation': orientation
                },
                'source': 'cartesian_pose',
                'raw_data': {
                    'position': position.copy(),
                    'orientation': orientation.copy()
                },
                'message_header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['cartesian_pose']:
                self.data_buffers['cartesian_pose'].append(data_record)
            
            # 更新统计信息
            self.data_stats['cartesian_pose']['count'] += 1
            self.data_stats['cartesian_pose']['last_received'] = timestamp
            
            # 轻量验证日志（临时便于联调）
            self.get_logger().info("Cartesian pose received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing cartesian pose: {e}")
    
    def vr_left_buttons_callback(self, msg):
        """VR左手柄按钮数据回调函数 - 接收来自vr_teleoperation的数据"""
        try:
            timestamp = time.time()
            
            # 验证按钮数据
            if not self.validate_button_data(msg.data):
                return
            
            # 提取按钮数据（假设前6个元素是按钮状态）
            button_data = msg.data[:6] if len(msg.data) >= 6 else msg.data + [0] * (6 - len(msg.data))
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': {
                    'button_5': button_data[4] if len(button_data) > 4 else 0,
                    'button_6': button_data[5] if len(button_data) > 5 else 0
                },
                'source': 'vr_left_buttons',
                'raw_data': {
                    'button_5': button_data[4] if len(button_data) > 4 else 0,
                    'button_6': button_data[5] if len(button_data) > 5 else 0,
                    'all_buttons': button_data
                },
                'message_header': {
                    'frame_id': 'vr_left_controller',
                    'stamp': msg.header.stamp if hasattr(msg, 'header') else None
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['vr_left_buttons']:
                self.data_buffers['vr_left_buttons'].append(data_record)
            
            # 更新统计信息
            self.data_stats['vr_left_buttons']['count'] += 1
            self.data_stats['vr_left_buttons']['last_received'] = timestamp
            
            self.get_logger().debug("VR left buttons received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing VR left buttons: {e}")
    
    
    def validate_button_data(self, button_data):
        """验证按钮数据"""
        if not button_data or len(button_data) < 2:
            return False
        
        # 检查按钮值是否在有效范围内 (0-1)
        for button_value in button_data:
            if not isinstance(button_value, (int, float)) or button_value < 0 or button_value > 1:
                return False
        
        return True
    
    def validate_joint_angles(self, joint_angles):
        """验证关节角度数据"""
        if not joint_angles or len(joint_angles) != 6:
            return False
        
        # 检查数据范围
        range_limits = self.data_sources['joint_angles']['processing']['range_limits']
        for i, angle in enumerate(joint_angles):
            if not isinstance(angle, (int, float)):
                return False
            if angle < range_limits['min'][i] or angle > range_limits['max'][i]:
                return False
        
        return True
    
    def validate_cartesian_pose(self, position, orientation):
        """验证笛卡尔位姿数据"""
        # 验证位置数据
        if not position or len(position) != 3:
            return False
        
        for coord in position:
            if not isinstance(coord, (int, float)):
                return False
        
        # 验证四元数数据
        if not orientation or len(orientation) != 4:
            return False
        
        for quat in orientation:
            if not isinstance(quat, (int, float)):
                return False
        
        # 检查四元数归一化
        norm = sum(q**2 for q in orientation)**0.5
        if abs(norm - 1.0) > 0.1:  # 允许一定的误差
            return False
        
        return True
    
    def validate_vr_button_data(self, button_data):
        """验证VR按钮数据"""
        if not button_data or 'button_5' not in button_data or 'button_6' not in button_data:
            return False
        
        # 检查按钮数据范围 (0-1)
        button_5 = button_data['button_5']
        button_6 = button_data['button_6']
        
        if not isinstance(button_5, (int, float)) or not isinstance(button_6, (int, float)):
            return False
        
        if button_5 < 0 or button_5 > 1 or button_6 < 0 or button_6 > 1:
            return False
        
        return True
    
    def get_data_buffer(self, data_type):
        """获取指定类型的数据缓冲区"""
        if data_type not in self.data_buffers:
            return []
        
        with self.buffer_locks[data_type]:
            return list(self.data_buffers[data_type])
    
    def clear_data_buffer(self, data_type):
        """清空指定类型的数据缓冲区"""
        if data_type in self.data_buffers:
            with self.buffer_locks[data_type]:
                self.data_buffers[data_type].clear()
    
    def get_data_stats(self):
        """获取数据统计信息"""
        return self.data_stats.copy()
    
    def get_available_data_types(self):
        """获取可用的数据类型"""
        return list(self.data_buffers.keys())

    def camera_rgb_callback(self, msg):
        """相机RGB图像数据回调函数"""
        try:
            # 转换图像数据
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            timestamp = time.time()
            
            # 数据验证
            if not self.validate_camera_image(cv_image, 'rgb'):
                self.get_logger().warn("Invalid RGB image data")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': {
                    'image': cv_image,
                    'width': cv_image.shape[1],
                    'height': cv_image.shape[0],
                    'channels': cv_image.shape[2] if len(cv_image.shape) == 3 else 1,
                    'encoding': msg.encoding
                },
                'source': 'camera_rgb',
                'raw_data': {
                    'image': cv_image.copy(),
                    'encoding': msg.encoding,
                    'width': msg.width,
                    'height': msg.height
                },
                'message_header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['camera_rgb']:
                self.data_buffers['camera_rgb'].append(data_record)
            
            # 更新统计信息
            self.data_stats['camera_rgb']['count'] += 1
            self.data_stats['camera_rgb']['last_received'] = timestamp
            
            self.get_logger().debug("Camera RGB image received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing camera RGB: {e}")
    
    def camera_depth_callback(self, msg):
        """相机深度图像数据回调函数"""
        try:
            # 转换深度图像数据
            depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            timestamp = time.time()
            
            # 数据验证
            if not self.validate_camera_image(depth_image, 'depth'):
                self.get_logger().warn("Invalid depth image data")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': {
                    'image': depth_image,
                    'width': depth_image.shape[1],
                    'height': depth_image.shape[0],
                    'encoding': msg.encoding,
                    'depth_unit': 'mm'  # 默认毫米
                },
                'source': 'camera_depth',
                'raw_data': {
                    'image': depth_image.copy(),
                    'encoding': msg.encoding,
                    'width': msg.width,
                    'height': msg.height
                },
                'message_header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['camera_depth']:
                self.data_buffers['camera_depth'].append(data_record)
            
            # 更新统计信息
            self.data_stats['camera_depth']['count'] += 1
            self.data_stats['camera_depth']['last_received'] = timestamp
            
            self.get_logger().debug("Camera depth image received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing camera depth: {e}")
    
    def camera_pointcloud_callback(self, msg):
        """相机点云数据回调函数"""
        try:
            timestamp = time.time()
            
            # 数据验证
            if not self.validate_pointcloud_data(msg):
                self.get_logger().warn("Invalid pointcloud data")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'data': {
                    'width': msg.width,
                    'height': msg.height,
                    'point_step': msg.point_step,
                    'row_step': msg.row_step,
                    'is_dense': msg.is_dense,
                    'fields': [{'name': field.name, 'offset': field.offset, 'datatype': field.datatype, 'count': field.count} for field in msg.fields]
                },
                'source': 'camera_pointcloud',
                'raw_data': {
                    'width': msg.width,
                    'height': msg.height,
                    'point_step': msg.point_step,
                    'row_step': msg.row_step,
                    'is_dense': msg.is_dense,
                    'data': msg.data,
                    'fields': [{'name': field.name, 'offset': field.offset, 'datatype': field.datatype, 'count': field.count} for field in msg.fields]
                },
                'message_header': {
                    'frame_id': msg.header.frame_id,
                    'stamp': msg.header.stamp
                }
            }
            
            # 存储到缓冲区
            with self.buffer_locks['camera_pointcloud']:
                self.data_buffers['camera_pointcloud'].append(data_record)
            
            # 更新统计信息
            self.data_stats['camera_pointcloud']['count'] += 1
            self.data_stats['camera_pointcloud']['last_received'] = timestamp
            
            self.get_logger().debug("Camera pointcloud received")
                
        except Exception as e:
            self.get_logger().error(f"Error processing camera pointcloud: {e}")
    
    def validate_camera_image(self, image, image_type):
        """验证相机图像数据"""
        if image is None:
            return False
        
        if len(image.shape) < 2:
            return False
        
        # 检查图像尺寸
        height, width = image.shape[:2]
        if height <= 0 or width <= 0:
            return False
        
        # 根据图像类型进行特定验证
        if image_type == 'rgb':
            if len(image.shape) != 3 or image.shape[2] != 3:
                return False
        elif image_type == 'depth':
            if len(image.shape) != 2:
                return False
        
        return True
    
    def validate_pointcloud_data(self, msg):
        """验证点云数据"""
        if not msg:
            return False
        
        if msg.width <= 0 or msg.height <= 0:
            return False
        
        if msg.point_step <= 0 or msg.row_step <= 0:
            return False
        
        if not msg.fields:
            return False
        
        return True

    def get_logger(self):
        """获取日志记录器（复用父节点）"""
        return self.node.get_logger()
