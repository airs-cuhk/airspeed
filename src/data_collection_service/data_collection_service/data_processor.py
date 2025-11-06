#!/usr/bin/env python3
"""
Data Processor Module
数据处理器模块

该模块负责根据配置对接收到的数据进行处理，包括字段提取、数据转换、过滤等
"""

import numpy as np
import math
from collections import deque
import threading
import time


class DataProcessor:
    """数据处理器类"""
    
    def __init__(self, config):
        self.config = config
        self.data_sources = config['data_sources']
        
        # 数据历史记录（用于平滑和速度计算）
        self.data_history = {
            'joint_angles': deque(maxlen=10),
            'cartesian_pose': deque(maxlen=10),
            'vr_left_buttons': deque(maxlen=10),
            'camera_rgb': deque(maxlen=5),
            'camera_depth': deque(maxlen=5),
            'camera_pointcloud': deque(maxlen=5)
        }
        
        # 历史数据锁
        self.history_locks = {
            'joint_angles': threading.Lock(),
            'cartesian_pose': threading.Lock(),
            'vr_left_buttons': threading.Lock(),
            'camera_rgb': threading.Lock(),
            'camera_depth': threading.Lock(),
            'camera_pointcloud': threading.Lock()
        }
        
        self.get_logger().info("Data processor initialized")
    
    def process_data(self, data_record):
        """处理数据记录 - 支持来自robot_interface的数据"""
        data_type = data_record['source']
        processed_data = data_record.copy()
        
        try:
            if data_type == 'joint_angles':
                processed_data = self.process_joint_angles(processed_data)
            elif data_type == 'cartesian_pose':
                processed_data = self.process_cartesian_pose(processed_data)
            elif data_type == 'vr_left_buttons':
                processed_data = self.process_vr_left_buttons(processed_data)
            elif data_type == 'camera_rgb':
                processed_data = self.process_camera_rgb(processed_data)
            elif data_type == 'camera_depth':
                processed_data = self.process_camera_depth(processed_data)
            elif data_type == 'camera_pointcloud':
                processed_data = self.process_camera_pointcloud(processed_data)
            else:
                self.get_logger().warn(f"Unknown data type: {data_type}")
                return None
            
            # 添加处理元数据
            source_system = 'vr_teleoperation' if data_type == 'vr_left_buttons' else 'robot_interface'
            processed_data['processing_metadata'] = {
                'processed_at': time.time(),
                'source_system': source_system,
                'message_header': data_record.get('message_header', {}),
                'multi_frequency_enabled': True
            }
            
            # 如果包含同步信息，添加同步元数据
            if 'sync_info' in data_record:
                processed_data['sync_metadata'] = data_record['sync_info']
            
            # 更新历史记录
            self.update_data_history(data_type, processed_data)
            
            return processed_data
            
        except Exception as e:
            self.get_logger().error(f"Error processing {data_type} data: {e}")
            return None
    
    def process_joint_angles(self, data_record):
        """处理关节角度数据"""
        config = self.data_sources['joint_angles']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        joint_angles = np.array(data_record['data'])
        processed_angles = joint_angles.copy()
        
        # 数据过滤
        if processing_config['enable_filtering']:
            processed_angles = self.filter_joint_angles(processed_angles, processing_config)
        
        # 异常值检测和移除
        if processing_config['remove_outliers']:
            processed_angles = self.remove_outliers(processed_angles, processing_config['outlier_threshold'])
        
        # 数据转换
        if transformation_config['convert_to_radians']:
            processed_angles = np.deg2rad(processed_angles)
        
        # 数据平滑
        if transformation_config['enable_smoothing']:
            processed_angles = self.smooth_data(processed_angles, 'joint_angles', transformation_config['smoothing_window'])
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = processed_angles.tolist()
        processed_record['processing_info'] = {
            'filtered': processing_config['enable_filtering'],
            'outliers_removed': processing_config['remove_outliers'],
            'converted_to_radians': transformation_config['convert_to_radians'],
            'smoothed': transformation_config['enable_smoothing']
        }
        
        return processed_record
    
    def process_cartesian_pose(self, data_record):
        """处理笛卡尔位姿数据"""
        config = self.data_sources['cartesian_pose']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        position = np.array(data_record['data']['position'])
        orientation = np.array(data_record['data']['orientation'])
        
        processed_position = position.copy()
        processed_orientation = orientation.copy()
        
        # 位置数据处理
        if processing_config['enable_filtering']:
            processed_position = self.filter_cartesian_position(processed_position, processing_config)
        
        # 异常值检测和移除
        if processing_config['remove_outliers']:
            processed_position = self.remove_outliers(processed_position, processing_config['outlier_threshold'])
            processed_orientation = self.remove_outliers(processed_orientation, processing_config['outlier_threshold'])
        
        # 单位转换
        if config['fields']['position']['convert_to_mm']:
            processed_position = processed_position * 1000.0  # 米转毫米
        
        # 旋转数据转换
        if config['fields']['orientation']['convert_to_euler']:
            euler_angles = self.quaternion_to_euler(processed_orientation)
            if config['fields']['orientation']['euler_units'] == 'degrees':
                euler_angles = np.rad2deg(euler_angles)
            processed_orientation = euler_angles
        
        # 数据平滑
        if transformation_config['enable_smoothing']:
            processed_position = self.smooth_data(processed_position, 'cartesian_pose', transformation_config['smoothing_window'])
            if not config['fields']['orientation']['convert_to_euler']:
                processed_orientation = self.smooth_data(processed_orientation, 'cartesian_pose', transformation_config['smoothing_window'])
        
        # 计算速度和加速度
        velocity = None
        acceleration = None
        if transformation_config['calculate_velocity']:
            velocity = self.calculate_velocity('cartesian_pose')
        if transformation_config['calculate_acceleration']:
            acceleration = self.calculate_acceleration('cartesian_pose')
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = {
            'position': processed_position.tolist(),
            'orientation': processed_orientation.tolist()
        }
        
        if velocity is not None:
            processed_record['processed_data']['velocity'] = velocity
        if acceleration is not None:
            processed_record['processed_data']['acceleration'] = acceleration
        
        processed_record['processing_info'] = {
            'filtered': processing_config['enable_filtering'],
            'outliers_removed': processing_config['remove_outliers'],
            'converted_to_mm': config['fields']['position']['convert_to_mm'],
            'converted_to_euler': config['fields']['orientation']['convert_to_euler'],
            'smoothed': transformation_config['enable_smoothing'],
            'velocity_calculated': transformation_config['calculate_velocity'],
            'acceleration_calculated': transformation_config['calculate_acceleration']
        }
        
        return processed_record
    
    def process_vr_left_buttons(self, data_record):
        """处理VR左手柄按钮数据"""
        config = self.data_sources['vr_left_buttons']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        button_data = data_record['data']
        button_5 = button_data['button_5']
        button_6 = button_data['button_6']
        
        processed_button_5 = button_5
        processed_button_6 = button_6
        
        # 数据过滤
        if processing_config['enable_filtering']:
            processed_button_5 = self.filter_button_data(processed_button_5, processing_config)
            processed_button_6 = self.filter_button_data(processed_button_6, processing_config)
        
        # 异常值检测和移除
        if processing_config['remove_outliers']:
            processed_button_5 = self.remove_button_outliers(processed_button_5, processing_config['outlier_threshold'])
            processed_button_6 = self.remove_button_outliers(processed_button_6, processing_config['outlier_threshold'])
        
        # 数据平滑
        if transformation_config['enable_smoothing']:
            processed_button_5 = self.smooth_button_data(processed_button_5, 'vr_left_buttons', transformation_config['smoothing_window'], 'button_5')
            processed_button_6 = self.smooth_button_data(processed_button_6, 'vr_left_buttons', transformation_config['smoothing_window'], 'button_6')
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = {
            'button_5': processed_button_5,
            'button_6': processed_button_6
        }
        processed_record['processing_info'] = {
            'filtered': processing_config['enable_filtering'],
            'outliers_removed': processing_config['remove_outliers'],
            'smoothed': transformation_config['enable_smoothing']
        }
        
        return processed_record
    
    def filter_joint_angles(self, joint_angles, processing_config):
        """过滤关节角度数据"""
        # 简单的范围过滤
        range_limits = processing_config['range_limits']
        filtered_angles = joint_angles.copy()
        
        for i in range(len(filtered_angles)):
            if filtered_angles[i] < range_limits['min'][i]:
                filtered_angles[i] = range_limits['min'][i]
            elif filtered_angles[i] > range_limits['max'][i]:
                filtered_angles[i] = range_limits['max'][i]
        
        return filtered_angles
    
    def filter_cartesian_position(self, position, processing_config):
        """过滤笛卡尔位置数据"""
        # 位置范围过滤
        position_limits = processing_config['position_limits']
        filtered_position = position.copy()
        
        for i, coord in enumerate(['x', 'y', 'z']):
            limits = position_limits[coord]
            if filtered_position[i] < limits[0]:
                filtered_position[i] = limits[0]
            elif filtered_position[i] > limits[1]:
                filtered_position[i] = limits[1]
        
        return filtered_position
    
    def filter_button_data(self, button_value, processing_config):
        """过滤按钮数据"""
        # 按钮数据范围限制 (0-1)
        button_limits = processing_config['button_limits']
        filtered_value = button_value
        
        if filtered_value < button_limits['min']:
            filtered_value = button_limits['min']
        elif filtered_value > button_limits['max']:
            filtered_value = button_limits['max']
        
        return filtered_value
    
    def remove_button_outliers(self, button_value, threshold):
        """移除按钮数据异常值"""
        # 按钮数据通常是0或1，异常值检测相对简单
        if button_value < 0 or button_value > 1:
            # 如果超出范围，使用最近的有效值
            return 0.0 if button_value < 0.5 else 1.0
        return button_value
    
    def smooth_button_data(self, button_value, data_type, window_size, button_name):
        """按钮数据平滑"""
        with self.history_locks[data_type]:
            # 创建临时数据记录用于历史记录
            temp_record = {
                'timestamp': time.time(),
                'processed_data': {button_name: button_value}
            }
            self.data_history[data_type].append(temp_record)
            
            if len(self.data_history[data_type]) < window_size:
                return button_value
            
            # 使用移动平均进行平滑
            recent_data = list(self.data_history[data_type])[-window_size:]
            button_values = [record['processed_data'][button_name] for record in recent_data]
            smoothed_value = np.mean(button_values)
            
            return smoothed_value
    
    def remove_outliers(self, data, threshold):
        """移除异常值"""
        if len(data) == 0:
            return data
        
        # 使用Z-score方法检测异常值
        mean = np.mean(data)
        std = np.std(data)
        
        if std == 0:
            return data
        
        z_scores = np.abs((data - mean) / std)
        outlier_mask = z_scores > threshold
        
        if np.any(outlier_mask):
            # 用均值替换异常值
            data[outlier_mask] = mean
        
        return data
    
    def smooth_data(self, data, data_type, window_size):
        """数据平滑"""
        with self.history_locks[data_type]:
            self.data_history[data_type].append(data.copy())
            
            if len(self.data_history[data_type]) < window_size:
                return data
            
            # 使用移动平均进行平滑
            recent_data = list(self.data_history[data_type])[-window_size:]
            smoothed_data = np.mean(recent_data, axis=0)
            
            return smoothed_data
    
    def calculate_velocity(self, data_type):
        """计算速度"""
        with self.history_locks[data_type]:
            if len(self.data_history[data_type]) < 2:
                return None
            
            # 获取最近两个数据点
            current_data = self.data_history[data_type][-1]
            previous_data = self.data_history[data_type][-2]
            
            # 计算时间差
            time_diff = current_data['timestamp'] - previous_data['timestamp']
            if time_diff == 0:
                return None
            
            # 计算位置差
            if data_type == 'cartesian_pose':
                current_pos = np.array(current_data['processed_data']['position'])
                previous_pos = np.array(previous_data['processed_data']['position'])
                velocity = (current_pos - previous_pos) / time_diff
            else:
                # 关节角度速度
                current_angles = np.array(current_data['processed_data'])
                previous_angles = np.array(previous_data['processed_data'])
                velocity = (current_angles - previous_angles) / time_diff
            
            return velocity.tolist()
    
    def calculate_acceleration(self, data_type):
        """计算加速度"""
        with self.history_locks[data_type]:
            if len(self.data_history[data_type]) < 3:
                return None
            
            # 获取最近三个数据点
            current_data = self.data_history[data_type][-1]
            previous_data = self.data_history[data_type][-2]
            before_previous_data = self.data_history[data_type][-3]
            
            # 计算时间差
            time_diff1 = current_data['timestamp'] - previous_data['timestamp']
            time_diff2 = previous_data['timestamp'] - before_previous_data['timestamp']
            
            if time_diff1 == 0 or time_diff2 == 0:
                return None
            
            # 计算速度差
            if data_type == 'cartesian_pose':
                current_pos = np.array(current_data['processed_data']['position'])
                previous_pos = np.array(previous_data['processed_data']['position'])
                before_previous_pos = np.array(before_previous_data['processed_data']['position'])
                
                velocity1 = (current_pos - previous_pos) / time_diff1
                velocity2 = (previous_pos - before_previous_pos) / time_diff2
            else:
                current_angles = np.array(current_data['processed_data'])
                previous_angles = np.array(previous_data['processed_data'])
                before_previous_angles = np.array(before_previous_data['processed_data'])
                
                velocity1 = (current_angles - previous_angles) / time_diff1
                velocity2 = (previous_angles - before_previous_angles) / time_diff2
            
            # 计算加速度
            acceleration = (velocity1 - velocity2) / ((time_diff1 + time_diff2) / 2)
            
            return acceleration.tolist()
    
    def quaternion_to_euler(self, quaternion):
        """四元数转欧拉角"""
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])
    
    def update_data_history(self, data_type, processed_data):
        """更新数据历史记录"""
        with self.history_locks[data_type]:
            self.data_history[data_type].append(processed_data)
    
    def process_camera_rgb(self, data_record):
        """处理相机RGB图像数据"""
        config = self.data_sources['camera_rgb']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        image_data = data_record['data']
        image = image_data['image']
        width = image_data['width']
        height = image_data['height']
        encoding = image_data['encoding']
        
        processed_image = image.copy()
        
        # 图像处理
        if transformation_config['enable_resize']:
            target_size = transformation_config['target_size']
            processed_image = self.resize_image(processed_image, target_size)
        
        if transformation_config['convert_to_grayscale']:
            processed_image = self.convert_to_grayscale(processed_image)
        
        if transformation_config['enable_enhancement']:
            processed_image = self.enhance_image(processed_image)
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = {
            'image': processed_image,
            'width': processed_image.shape[1],
            'height': processed_image.shape[0],
            'channels': processed_image.shape[2] if len(processed_image.shape) == 3 else 1,
            'encoding': encoding
        }
        processed_record['processing_info'] = {
            'resized': transformation_config['enable_resize'],
            'grayscale': transformation_config['convert_to_grayscale'],
            'enhanced': transformation_config['enable_enhancement']
        }
        
        return processed_record
    
    def process_camera_depth(self, data_record):
        """处理相机深度图像数据"""
        config = self.data_sources['camera_depth']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        depth_data = data_record['data']
        depth_image = depth_data['image']
        width = depth_data['width']
        height = depth_data['height']
        encoding = depth_data['encoding']
        depth_unit = depth_data['depth_unit']
        
        processed_depth = depth_image.copy()
        
        # 深度图像处理
        if transformation_config['enable_resize']:
            target_size = transformation_config['target_size']
            processed_depth = self.resize_image(processed_depth, target_size)
        
        if transformation_config['enable_depth_filtering']:
            processed_depth = self.filter_depth_image(processed_depth, transformation_config)
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = {
            'image': processed_depth,
            'width': processed_depth.shape[1],
            'height': processed_depth.shape[0],
            'encoding': encoding,
            'depth_unit': depth_unit
        }
        processed_record['processing_info'] = {
            'resized': transformation_config['enable_resize'],
            'filtered': transformation_config['enable_depth_filtering']
        }
        
        return processed_record
    
    def process_camera_pointcloud(self, data_record):
        """处理相机点云数据"""
        config = self.data_sources['camera_pointcloud']
        processing_config = config['processing']
        transformation_config = config['transformation']
        
        # 提取原始数据
        pointcloud_data = data_record['data']
        
        processed_pointcloud = pointcloud_data.copy()
        
        # 点云处理
        if transformation_config['enable_downsampling']:
            voxel_size = transformation_config['voxel_size']
            processed_pointcloud = self.downsample_pointcloud(processed_pointcloud, voxel_size)
        
        if transformation_config['enable_filtering']:
            processed_pointcloud = self.filter_pointcloud(processed_pointcloud, transformation_config)
        
        # 创建处理后的数据记录
        processed_record = data_record.copy()
        processed_record['processed_data'] = processed_pointcloud
        processed_record['processing_info'] = {
            'downsampled': transformation_config['enable_downsampling'],
            'filtered': transformation_config['enable_filtering']
        }
        
        return processed_record
    
    def resize_image(self, image, target_size):
        """调整图像大小"""
        import cv2
        return cv2.resize(image, (target_size[0], target_size[1]))
    
    def convert_to_grayscale(self, image):
        """转换为灰度图"""
        import cv2
        if len(image.shape) == 3:
            return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return image
    
    def enhance_image(self, image):
        """图像增强"""
        import cv2
        # 简单的直方图均衡化
        if len(image.shape) == 3:
            # 彩色图像
            yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
            yuv[:,:,0] = cv2.equalizeHist(yuv[:,:,0])
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
        else:
            # 灰度图像
            return cv2.equalizeHist(image)
    
    def filter_depth_image(self, depth_image, config):
        """深度图像滤波"""
        import cv2
        filter_type = config['filter_type']
        kernel_size = config['filter_kernel_size']
        
        if filter_type == 'median':
            return cv2.medianBlur(depth_image, kernel_size)
        elif filter_type == 'gaussian':
            return cv2.GaussianBlur(depth_image, (kernel_size, kernel_size), 0)
        elif filter_type == 'bilateral':
            return cv2.bilateralFilter(depth_image, kernel_size, 75, 75)
        else:
            return depth_image
    
    def downsample_pointcloud(self, pointcloud, voxel_size):
        """点云下采样"""
        # 这里需要实现点云下采样逻辑
        # 由于点云数据格式复杂，这里只是占位符
        return pointcloud
    
    def filter_pointcloud(self, pointcloud, config):
        """点云滤波"""
        # 这里需要实现点云滤波逻辑
        # 由于点云数据格式复杂，这里只是占位符
        return pointcloud
    
    def get_logger(self):
        """获取日志记录器"""
        import rclpy.logging
        return rclpy.logging.get_logger('data_processor')
