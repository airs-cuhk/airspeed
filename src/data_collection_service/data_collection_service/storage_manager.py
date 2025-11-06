#!/usr/bin/env python3
"""
Storage Manager Module
存储管理器模块

该模块负责将处理后的数据存储到指定格式的文件中，支持HDF5、CSV、JSON、Pickle等格式
"""

import os
import time
import threading
import h5py
import csv
import json
import pickle
import numpy as np
from datetime import datetime
from collections import deque
from rclpy.time import Time


class ROSJSONEncoder(json.JSONEncoder):
    """自定义JSON编码器，处理ROS Time对象"""
    
    def default(self, obj):
        if isinstance(obj, Time):
            # 将ROS Time对象转换为秒数
            return obj.nanoseconds / 1e9
        elif hasattr(obj, 'nanoseconds'):
            # 处理其他时间对象
            return obj.nanoseconds / 1e9
        return super().default(obj)


class StorageManager:
    """存储管理器类"""
    
    def __init__(self, config):
        self.config = config
        self.storage_config = config['storage']
        self.storage_formats = config['storage_formats']
        
        # 存储目录（规范化为绝对路径并展开 ~）
        self.storage_directory = os.path.abspath(
            os.path.expanduser(self.storage_config['storage_directory'])
        )
        self.storage_format = self.storage_config['storage_format']
        
        # 数据缓冲区
        self.data_buffer = deque(maxlen=self.storage_config['buffer']['buffer_size'])
        self.buffer_lock = threading.Lock()
        
        # 存储文件信息
        self.current_file = None
        self.file_handle = None
        self.file_created_time = None
        
        # 性能监控
        self.performance_stats = {
            'total_records': 0,
            'buffer_flushes': 0,
            'file_operations': 0,
            'last_flush_time': None
        }
        
        # 创建存储目录
        self.create_storage_directory()
        
        # 启动自动刷新定时器
        if self.storage_config['buffer']['auto_flush']:
            self.start_auto_flush_timer()
        
        self.get_logger().info(f"Storage manager initialized with format: {self.storage_format}")
    
    def create_storage_directory(self):
        """创建存储目录"""
        if self.storage_config['auto_create_directory']:
            os.makedirs(self.storage_directory, exist_ok=True)
            self.get_logger().info(f"Storage directory created: {self.storage_directory}")
    
    def start_auto_flush_timer(self):
        """启动自动刷新定时器"""
        flush_interval = self.storage_config['buffer']['flush_interval']
        self.flush_timer = threading.Timer(flush_interval, self.auto_flush_callback)
        self.flush_timer.daemon = True
        self.flush_timer.start()
    
    def auto_flush_callback(self):
        """自动刷新回调函数"""
        try:
            self.flush_buffer()
            # 重新启动定时器
            if self.storage_config['buffer']['auto_flush']:
                self.start_auto_flush_timer()
        except Exception as e:
            self.get_logger().error(f"Error in auto flush: {e}")
    
    def store_data(self, processed_data):
        """存储处理后的数据"""
        try:
            with self.buffer_lock:
                self.data_buffer.append(processed_data)
                self.performance_stats['total_records'] += 1
            
            # 检查缓冲区是否已满
            if len(self.data_buffer) >= self.storage_config['buffer']['buffer_size']:
                self.flush_buffer()
            
        except Exception as e:
            self.get_logger().error(f"Error storing data: {e}")
    
    def flush_buffer(self, force_new_file=False):
        """刷新缓冲区，将数据写入文件"""
        try:
            with self.buffer_lock:
                if not self.data_buffer:
                    return
                
                # 获取缓冲区数据
                buffer_data = list(self.data_buffer)
                self.data_buffer.clear()
            
            # 根据存储格式写入数据
            if self.storage_format == 'hdf5':
                self.store_hdf5_data(buffer_data, force_new_file)
            elif self.storage_format == 'csv':
                self.store_csv_data(buffer_data)
            elif self.storage_format == 'json':
                self.store_json_data(buffer_data)
            elif self.storage_format == 'pickle':
                self.store_pickle_data(buffer_data)
            else:
                self.get_logger().error(f"Unsupported storage format: {self.storage_format}")
                return
            
            self.performance_stats['buffer_flushes'] += 1
            self.performance_stats['last_flush_time'] = time.time()
            
            if self.config['logging']['verbose_logging']:
                self.get_logger().info(f"Flushed {len(buffer_data)} records to storage")
                
        except Exception as e:
            self.get_logger().error(f"Error flushing buffer: {e}")
    
    def store_hdf5_data(self, data_records, force_new_file=False):
        """存储HDF5格式数据"""
        try:
            # 创建或打开HDF5文件
            if self.current_file is None or self.should_create_new_file(force_new_file):
                self.create_new_hdf5_file()
            
            # 获取HDF5配置
            hdf5_config = self.storage_formats['hdf5']
            
            with h5py.File(self.current_file, 'a') as f:
                for record in data_records:
                    data_type = record['source']
                    timestamp = record['timestamp']
                    
                    # 创建数据集路径
                    dataset_path = f"/{data_type}/{timestamp}"
                    
                    if data_type == 'joint_angles':
                        # 存储关节角度数据
                        data = np.array(record['processed_data'])
                        f.create_dataset(dataset_path, data=data, compression=hdf5_config['compression_algorithm'])
                        
                    elif data_type == 'cartesian_pose':
                        # 存储笛卡尔位姿数据
                        pos_data = np.array(record['processed_data']['position'])
                        ori_data = np.array(record['processed_data']['orientation'])
                        
                        f.create_dataset(f"{dataset_path}/position", data=pos_data, 
                                        compression=hdf5_config['compression_algorithm'])
                        f.create_dataset(f"{dataset_path}/orientation", data=ori_data, 
                                        compression=hdf5_config['compression_algorithm'])
                        
                        # 存储速度和加速度（如果存在）
                        if 'velocity' in record['processed_data']:
                            vel_data = np.array(record['processed_data']['velocity'])
                            f.create_dataset(f"{dataset_path}/velocity", data=vel_data, 
                                            compression=hdf5_config['compression_algorithm'])
                        
                        if 'acceleration' in record['processed_data']:
                            acc_data = np.array(record['processed_data']['acceleration'])
                            f.create_dataset(f"{dataset_path}/acceleration", data=acc_data, 
                                            compression=hdf5_config['compression_algorithm'])
                    
                    elif data_type == 'camera_rgb':
                        # 存储相机RGB图像数据
                        image_data = record['processed_data']['image']
                        if isinstance(image_data, np.ndarray):
                            f.create_dataset(f"{dataset_path}/image", data=image_data, 
                                            compression=hdf5_config['compression_algorithm'])
                        
                        # 存储图像元数据
                        metadata = {
                            'width': record['processed_data']['width'],
                            'height': record['processed_data']['height'],
                            'channels': record['processed_data']['channels'],
                            'encoding': record['processed_data']['encoding']
                        }
                        f.attrs.update({f"{dataset_path}_metadata": json.dumps(metadata, cls=ROSJSONEncoder)})
                    
                    elif data_type == 'camera_depth':
                        # 存储相机深度图像数据
                        depth_data = record['processed_data']['image']
                        if isinstance(depth_data, np.ndarray):
                            f.create_dataset(f"{dataset_path}/image", data=depth_data, 
                                            compression=hdf5_config['compression_algorithm'])
                        
                        # 存储深度图像元数据
                        metadata = {
                            'width': record['processed_data']['width'],
                            'height': record['processed_data']['height'],
                            'encoding': record['processed_data']['encoding'],
                            'depth_unit': record['processed_data']['depth_unit']
                        }
                        f.attrs.update({f"{dataset_path}_metadata": json.dumps(metadata, cls=ROSJSONEncoder)})
                    
                    elif data_type == 'camera_pointcloud':
                        # 存储相机点云数据
                        pointcloud_data = record['processed_data']
                        
                        # 存储点云元数据
                        metadata = {
                            'width': pointcloud_data['width'],
                            'height': pointcloud_data['height'],
                            'point_step': pointcloud_data['point_step'],
                            'row_step': pointcloud_data['row_step'],
                            'is_dense': pointcloud_data['is_dense'],
                            'fields': pointcloud_data['fields']
                        }
                        f.attrs.update({f"{dataset_path}_metadata": json.dumps(metadata, cls=ROSJSONEncoder)})
                        
                        # 注意：点云数据本身很大，这里只存储元数据
                        # 如果需要存储完整点云数据，需要特殊处理
                    
                    # 存储元数据
                    metadata = {
                        'timestamp': timestamp,
                        'source': data_type,
                        'processing_info': record.get('processing_info', {}),
                        'processing_metadata': record.get('processing_metadata', {}),
                        'message_header': record.get('message_header', {})
                    }
                    f.attrs[f"{dataset_path}_metadata"] = json.dumps(metadata, cls=ROSJSONEncoder)
            
            self.performance_stats['file_operations'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error storing HDF5 data: {e}")
    
    def store_csv_data(self, data_records):
        """存储CSV格式数据"""
        try:
            # 创建或打开CSV文件
            if self.current_file is None or self.should_create_new_file():
                self.create_new_csv_file()
            
            # 获取CSV配置
            csv_config = self.storage_formats['csv']
            
            with open(self.current_file, 'a', newline='', encoding=csv_config['encoding']) as f:
                writer = csv.writer(f, delimiter=csv_config['delimiter'])
                
                for record in data_records:
                    data_type = record['source']
                    timestamp = record['timestamp']
                    
                    if data_type == 'joint_angles':
                        # 关节角度数据行
                        row = [timestamp, data_type] + record['processed_data']
                        writer.writerow(row)
                        
                    elif data_type == 'cartesian_pose':
                        # 笛卡尔位姿数据行
                        pos_data = record['processed_data']['position']
                        ori_data = record['processed_data']['orientation']
                        row = [timestamp, data_type] + pos_data + ori_data
                        writer.writerow(row)
            
            self.performance_stats['file_operations'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error storing CSV data: {e}")
    
    def store_json_data(self, data_records):
        """存储JSON格式数据"""
        try:
            # 创建或打开JSON文件
            if self.current_file is None or self.should_create_new_file():
                self.create_new_json_file()
            
            # 获取JSON配置
            json_config = self.storage_formats['json']
            
            # 读取现有数据
            try:
                with open(self.current_file, 'r', encoding='utf-8') as f:
                    existing_data = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                existing_data = []
            
            # 添加新数据
            for record in data_records:
                json_record = {
                    'timestamp': record['timestamp'],
                    'source': record['source'],
                    'data': record['processed_data'],
                    'processing_info': record.get('processing_info', {})
                }
                existing_data.append(json_record)
            
            # 写入文件
            with open(self.current_file, 'w', encoding='utf-8') as f:
                if json_config['pretty_print']:
                    json.dump(existing_data, f, indent=2, ensure_ascii=False)
                else:
                    json.dump(existing_data, f, ensure_ascii=False)
            
            self.performance_stats['file_operations'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error storing JSON data: {e}")
    
    def store_pickle_data(self, data_records):
        """存储Pickle格式数据"""
        try:
            # 创建或打开Pickle文件
            if self.current_file is None or self.should_create_new_file():
                self.create_new_pickle_file()
            
            # 获取Pickle配置
            pickle_config = self.storage_formats['pickle']
            
            # 读取现有数据
            try:
                with open(self.current_file, 'rb') as f:
                    existing_data = pickle.load(f)
            except (FileNotFoundError, pickle.UnpicklingError):
                existing_data = []
            
            # 添加新数据
            existing_data.extend(data_records)
            
            # 写入文件
            with open(self.current_file, 'wb') as f:
                pickle.dump(existing_data, f, protocol=pickle_config['protocol_version'])
            
            self.performance_stats['file_operations'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error storing Pickle data: {e}")
    
    def should_create_new_file(self, force_new=False):
        """判断是否应该创建新文件"""
        if force_new:
            return True
            
        if self.current_file is None:
            return True
        
        # 检查文件大小
        if os.path.exists(self.current_file):
            file_size = os.path.getsize(self.current_file)
            max_size = self.config['performance']['cleanup_strategy']['max_file_size'] * 1024 * 1024  # MB to bytes
            if file_size > max_size:
                return True
        
        # 检查文件创建时间
        if self.file_created_time is not None:
            time_diff = time.time() - self.file_created_time
            if time_diff > 3600:  # 1小时
                return True
        
        return False
    
    def create_new_hdf5_file(self):
        """创建新的HDF5文件"""
        self.close_current_file()
        self.current_file = self.generate_filename('h5')
        self.file_created_time = time.time()
        self.get_logger().info(f"Created new HDF5 file: {self.current_file}")
    
    def create_new_csv_file(self):
        """创建新的CSV文件"""
        self.close_current_file()
        self.current_file = self.generate_filename('csv')
        self.file_created_time = time.time()
        
        # 写入CSV头部
        csv_config = self.storage_formats['csv']
        if csv_config['include_header']:
            with open(self.current_file, 'w', newline='', encoding=csv_config['encoding']) as f:
                writer = csv.writer(f, delimiter=csv_config['delimiter'])
                writer.writerow(['timestamp', 'data_type', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        
        self.get_logger().info(f"Created new CSV file: {self.current_file}")
    
    def create_new_json_file(self):
        """创建新的JSON文件"""
        self.close_current_file()
        self.current_file = self.generate_filename('json')
        self.file_created_time = time.time()
        self.get_logger().info(f"Created new JSON file: {self.current_file}")
    
    def create_new_pickle_file(self):
        """创建新的Pickle文件"""
        self.close_current_file()
        self.current_file = self.generate_filename('pkl')
        self.file_created_time = time.time()
        self.get_logger().info(f"Created new Pickle file: {self.current_file}")
    
    def generate_filename(self, extension):
        """生成文件名"""
        naming_config = self.storage_config['file_naming']
        timestamp = datetime.now().strftime(naming_config['timestamp_format'])
        
        filename = f"{naming_config['prefix']}_{timestamp}"
        if naming_config['suffix']:
            filename += f"_{naming_config['suffix']}"
        
        filename += f".{extension}"
        
        return os.path.join(self.storage_directory, filename)
    
    def close_current_file(self):
        """关闭当前文件"""
        if self.file_handle is not None:
            try:
                self.file_handle.close()
            except Exception as e:
                self.get_logger().error(f"Error closing file: {e}")
            finally:
                self.file_handle = None
    
    def get_performance_stats(self):
        """获取性能统计信息"""
        return self.performance_stats.copy()
    
    def cleanup_old_files(self):
        """清理旧文件"""
        try:
            cleanup_config = self.config['performance']['cleanup_strategy']
            if not cleanup_config['auto_cleanup']:
                return
            
            # 获取存储目录中的所有文件
            files = []
            for filename in os.listdir(self.storage_directory):
                if filename.startswith(self.storage_config['file_naming']['prefix']):
                    filepath = os.path.join(self.storage_directory, filename)
                    files.append((filepath, os.path.getmtime(filepath)))
            
            # 按修改时间排序
            files.sort(key=lambda x: x[1], reverse=True)
            
            # 删除超出限制的文件
            max_files = cleanup_config['max_files']
            if len(files) > max_files:
                for filepath, _ in files[max_files:]:
                    try:
                        os.remove(filepath)
                        self.get_logger().info(f"Removed old file: {filepath}")
                    except Exception as e:
                        self.get_logger().error(f"Error removing file {filepath}: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Error in cleanup: {e}")
    
    def force_new_hdf5_file(self):
        """强制创建新的HDF5文件（用于VR按钮控制）"""
        try:
            # 先刷新当前缓冲区到现有文件
            self.flush_buffer()
            
            # 强制创建新文件
            self.create_new_hdf5_file()
            
            self.get_logger().info("Forced creation of new HDF5 file for VR button control")
            
        except Exception as e:
            self.get_logger().error(f"Error forcing new HDF5 file: {e}")
    
    def get_logger(self):
        """获取日志记录器"""
        import rclpy.logging
        return rclpy.logging.get_logger('storage_manager')
