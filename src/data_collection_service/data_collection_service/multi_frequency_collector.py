#!/usr/bin/env python3
"""
Multi-Frequency Data Collector
多频率数据采集器

该模块实现多频率分层采集架构，支持不同频率的数据源
专门针对robot_interface的关节角度和笛卡尔位姿话题
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import threading
import time
import numpy as np
from collections import deque
from collections import defaultdict
import math


class MultiFrequencyCollector(Node):
    """多频率数据采集器"""
    
    def __init__(self, config):
        super().__init__('multi_frequency_collector')
        self.config = config
        self.multi_freq_config = config.get('multi_frequency', {})
        
        # 数据源配置
        self.data_sources = {
            'joint_angles': {
                'topic': '/robot_joint_angles',
                'msg_type': Float32MultiArray,
                'frequency': self.multi_freq_config.get('joints_fps', 10),
                'enabled': self.multi_freq_config.get('enable_joints', True),
                'buffer': deque(maxlen=1000),
                'last_timestamp': None,
                'data_count': 0
            },
            'cartesian_pose': {
                'topic': '/robot_cartesian_pose', 
                'msg_type': PoseStamped,
                'frequency': self.multi_freq_config.get('cartesian_pose_fps', 10),
                'enabled': self.multi_freq_config.get('enable_cartesian_pose', True),
                'buffer': deque(maxlen=1000),
                'last_timestamp': None,
                'data_count': 0
            }
        }
        
        # 同步配置
        self.sync_config = config.get('data_synchronization', {})
        self.sync_tolerance = self.sync_config.get('sync_tolerance', 0.01)
        self.max_time_diff = self.sync_config.get('max_time_diff', 0.1)
        self.base_frequency = self.multi_freq_config.get('base_frequency', 30)
        
        # 数据缓存
        self.sync_buffer = deque(maxlen=1000)
        self.sync_lock = threading.Lock()
        
        # 统计信息
        self.stats = {
            'total_collected': 0,
            'sync_events': 0,
            'sync_errors': 0,
            'frequency_stats': defaultdict(list)
        }
        
        # 初始化订阅器
        self.init_subscribers()
        
        # 启动同步处理线程
        self.start_sync_processor()
        
        # 启动统计报告
        self.start_stats_timer()
        
        self.get_logger().info("Multi-frequency collector initialized")
        self.get_logger().info(f"Joint angles frequency: {self.data_sources['joint_angles']['frequency']} Hz")
        self.get_logger().info(f"Cartesian pose frequency: {self.data_sources['cartesian_pose']['frequency']} Hz")
    
    def init_subscribers(self):
        """初始化数据订阅器"""
        # 关节角度订阅器
        if self.data_sources['joint_angles']['enabled']:
            self.joint_angles_sub = self.create_subscription(
                Float32MultiArray,
                self.data_sources['joint_angles']['topic'],
                self.joint_angles_callback,
                10
            )
            self.get_logger().info(f"Subscribed to joint angles: {self.data_sources['joint_angles']['topic']}")
        
        # 笛卡尔位姿订阅器
        if self.data_sources['cartesian_pose']['enabled']:
            self.cartesian_pose_sub = self.create_subscription(
                PoseStamped,
                self.data_sources['cartesian_pose']['topic'],
                self.cartesian_pose_callback,
                10
            )
            self.get_logger().info(f"Subscribed to cartesian pose: {self.data_sources['cartesian_pose']['topic']}")
    
    def joint_angles_callback(self, msg):
        """关节角度数据回调"""
        try:
            timestamp = time.time()
            
            # 提取数据
            joint_angles = list(msg.data)
            
            # 数据验证
            if not self.validate_joint_angles(joint_angles):
                self.get_logger().warn(f"Invalid joint angles data: {joint_angles}")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'source': 'joint_angles',
                'data': joint_angles,
                'raw_data': joint_angles.copy(),
                'message_header': {
                    'frame_id': getattr(msg, 'header', {}).frame_id if hasattr(msg, 'header') else 'robot_base',
                    'stamp': getattr(msg, 'header', {}).stamp if hasattr(msg, 'header') else None
                }
            }
            
            # 存储到缓冲区
            self.data_sources['joint_angles']['buffer'].append(data_record)
            self.data_sources['joint_angles']['last_timestamp'] = timestamp
            self.data_sources['joint_angles']['data_count'] += 1
            
            # 更新统计信息
            self.update_frequency_stats('joint_angles', timestamp)
            
            # 尝试同步数据
            self.attempt_sync('joint_angles', data_record)
            
            if self.multi_freq_config.get('debug_mode', False):
                self.get_logger().debug(f"Received joint angles: {joint_angles}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing joint angles: {e}")
    
    def cartesian_pose_callback(self, msg):
        """笛卡尔位姿数据回调"""
        try:
            timestamp = time.time()
            
            # 提取数据
            position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
            orientation = [
                msg.pose.orientation.x,
                msg.pose.orientation.y, 
                msg.pose.orientation.z,
                msg.pose.orientation.w
            ]
            
            # 数据验证
            if not self.validate_cartesian_pose(position, orientation):
                self.get_logger().warn(f"Invalid cartesian pose data: pos={position}, ori={orientation}")
                return
            
            # 创建数据记录
            data_record = {
                'timestamp': timestamp,
                'source': 'cartesian_pose',
                'data': {
                    'position': position,
                    'orientation': orientation
                },
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
            self.data_sources['cartesian_pose']['buffer'].append(data_record)
            self.data_sources['cartesian_pose']['last_timestamp'] = timestamp
            self.data_sources['cartesian_pose']['data_count'] += 1
            
            # 更新统计信息
            self.update_frequency_stats('cartesian_pose', timestamp)
            
            # 尝试同步数据
            self.attempt_sync('cartesian_pose', data_record)
            
            if self.multi_freq_config.get('debug_mode', False):
                self.get_logger().debug(f"Received cartesian pose: pos={position}, ori={orientation}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing cartesian pose: {e}")
    
    def validate_joint_angles(self, joint_angles):
        """验证关节角度数据"""
        if not joint_angles or len(joint_angles) != 6:
            return False
        
        # 检查数据范围
        for angle in joint_angles:
            if not isinstance(angle, (int, float)):
                return False
            if angle < -180.0 or angle > 180.0:
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
        if abs(norm - 1.0) > 0.1:
            return False
        
        return True
    
    def update_frequency_stats(self, source, timestamp):
        """更新频率统计信息"""
        if source in self.stats['frequency_stats']:
            self.stats['frequency_stats'][source].append(timestamp)
            
            # 保持最近100个时间戳
            if len(self.stats['frequency_stats'][source]) > 100:
                self.stats['frequency_stats'][source] = self.stats['frequency_stats'][source][-100:]
    
    def attempt_sync(self, source, data_record):
        """尝试同步数据"""
        try:
            with self.sync_lock:
                # 检查是否有足够的数据进行同步
                if len(self.data_sources['joint_angles']['buffer']) < 1 or \
                   len(self.data_sources['cartesian_pose']['buffer']) < 1:
                    return
                
                # 获取最新的数据
                joint_data = self.data_sources['joint_angles']['buffer'][-1]
                pose_data = self.data_sources['cartesian_pose']['buffer'][-1]
                
                # 检查时间同步
                time_diff = abs(joint_data['timestamp'] - pose_data['timestamp'])
                
                if time_diff <= self.sync_tolerance:
                    # 数据同步成功
                    sync_record = {
                        'timestamp': (joint_data['timestamp'] + pose_data['timestamp']) / 2,
                        'joint_angles': joint_data,
                        'cartesian_pose': pose_data,
                        'sync_quality': 1.0 - (time_diff / self.sync_tolerance)
                    }
                    
                    self.sync_buffer.append(sync_record)
                    self.stats['sync_events'] += 1
                    self.stats['total_collected'] += 1
                    
                    if self.multi_freq_config.get('debug_mode', False):
                        self.get_logger().debug(f"Data synchronized: time_diff={time_diff:.4f}s")
                        
                elif time_diff <= self.max_time_diff:
                    # 数据在容忍范围内，但质量较低
                    sync_record = {
                        'timestamp': (joint_data['timestamp'] + pose_data['timestamp']) / 2,
                        'joint_angles': joint_data,
                        'cartesian_pose': pose_data,
                        'sync_quality': max(0.0, 1.0 - (time_diff / self.max_time_diff))
                    }
                    
                    self.sync_buffer.append(sync_record)
                    self.stats['sync_events'] += 1
                    self.stats['total_collected'] += 1
                    
                else:
                    # 数据同步失败
                    self.stats['sync_errors'] += 1
                    if self.multi_freq_config.get('debug_mode', False):
                        self.get_logger().warn(f"Sync failed: time_diff={time_diff:.4f}s > {self.max_time_diff}s")
                        
        except Exception as e:
            self.get_logger().error(f"Error in sync attempt: {e}")
    
    def start_sync_processor(self):
        """启动同步处理线程"""
        self.sync_processor_thread = threading.Thread(target=self.sync_processor_loop, daemon=True)
        self.sync_processor_thread.start()
        self.get_logger().info("Sync processor thread started")
    
    def sync_processor_loop(self):
        """同步处理循环"""
        while rclpy.ok():
            try:
                with self.sync_lock:
                    if self.sync_buffer:
                        # 处理同步数据
                        sync_record = self.sync_buffer.popleft()
                        self.process_sync_data(sync_record)
                
                time.sleep(0.01)  # 100Hz处理频率
                
            except Exception as e:
                self.get_logger().error(f"Error in sync processor: {e}")
                time.sleep(0.1)
    
    def process_sync_data(self, sync_record):
        """处理同步数据"""
        try:
            # 这里可以添加数据预处理逻辑
            # 例如：数据融合、滤波、特征提取等
            
            # 计算数据质量指标
            quality_score = self.calculate_data_quality(sync_record)
            sync_record['quality_score'] = quality_score
            
            # 发布同步数据（如果需要）
            if self.multi_freq_config.get('publish_sync_data', False):
                self.publish_sync_data(sync_record)
            
            if self.multi_freq_config.get('debug_mode', False):
                self.get_logger().debug(f"Processed sync data: quality={quality_score:.3f}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing sync data: {e}")
    
    def calculate_data_quality(self, sync_record):
        """计算数据质量分数"""
        try:
            # 基于同步质量和数据一致性计算质量分数
            sync_quality = sync_record.get('sync_quality', 0.0)
            
            # 检查数据一致性
            joint_data = sync_record['joint_angles']['data']
            pose_data = sync_record['cartesian_pose']['data']
            
            # 简单的数据一致性检查
            consistency_score = 1.0
            
            # 检查关节角度是否在合理范围内
            for angle in joint_data:
                if abs(angle) > 180.0:
                    consistency_score *= 0.8
            
            # 检查位置数据是否在合理范围内
            position = pose_data['position']
            for coord in position:
                if abs(coord) > 2.0:  # 2米范围
                    consistency_score *= 0.9
            
            # 综合质量分数
            quality_score = (sync_quality + consistency_score) / 2.0
            
            return min(1.0, max(0.0, quality_score))
            
        except Exception as e:
            self.get_logger().error(f"Error calculating data quality: {e}")
            return 0.0
    
    def publish_sync_data(self, sync_record):
        """发布同步数据"""
        # 这里可以实现发布同步数据到其他话题
        # 例如：发布到 /data_storage/sync_data
        pass
    
    def start_stats_timer(self):
        """启动统计报告定时器"""
        timer_period = 10.0  # 10秒
        self.stats_timer = self.create_timer(timer_period, self.stats_timer_callback)
        self.get_logger().info("Stats timer started")
    
    def stats_timer_callback(self):
        """统计报告定时器回调"""
        try:
            self.log_statistics()
        except Exception as e:
            self.get_logger().error(f"Error in stats timer: {e}")
    
    def log_statistics(self):
        """记录统计信息"""
        try:
            current_time = time.time()
            
            # 计算各数据源的频率
            freq_stats = {}
            for source, timestamps in self.stats['frequency_stats'].items():
                if len(timestamps) > 1:
                    time_span = timestamps[-1] - timestamps[0]
                    if time_span > 0:
                        freq = (len(timestamps) - 1) / time_span
                        freq_stats[source] = freq
            
            self.get_logger().info(
                f"Multi-frequency stats - "
                f"Total collected: {self.stats['total_collected']}, "
                f"Sync events: {self.stats['sync_events']}, "
                f"Sync errors: {self.stats['sync_errors']}"
            )
            
            for source, freq in freq_stats.items():
                self.get_logger().info(f"{source} frequency: {freq:.2f} Hz")
                
        except Exception as e:
            self.get_logger().error(f"Error logging statistics: {e}")
    
    def get_sync_data(self):
        """获取同步数据"""
        with self.sync_lock:
            return list(self.sync_buffer)
    
    def clear_sync_buffer(self):
        """清空同步缓冲区"""
        with self.sync_lock:
            self.sync_buffer.clear()
    
    def get_collection_stats(self):
        """获取采集统计信息"""
        return {
            'total_collected': self.stats['total_collected'],
            'sync_events': self.stats['sync_events'],
            'sync_errors': self.stats['sync_errors'],
            'frequency_stats': dict(self.stats['frequency_stats']),
            'buffer_sizes': {
                source: len(data['buffer']) for source, data in self.data_sources.items()
            }
        }

