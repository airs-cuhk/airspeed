#!/usr/bin/env python3
"""
Data Storage Node
数据存储节点

该节点负责整合所有功能模块，实现完整的数据存储功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml
import os
import sys
import threading
import time
from datetime import datetime

try:
    # 尝试相对导入（当作为包的一部分运行时）
    from .data_subscriber import DataSubscriber
    from .data_processor import DataProcessor
    from .storage_manager import StorageManager
    from .multi_frequency_collector import MultiFrequencyCollector
    from .vr_button_controller import VRButtonController
except ImportError:
    # 如果相对导入失败，尝试绝对导入（当作为独立脚本运行时）
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from data_subscriber import DataSubscriber
    from data_processor import DataProcessor
    from storage_manager import StorageManager
    from multi_frequency_collector import MultiFrequencyCollector
    from vr_button_controller import VRButtonController


class DataStorageNode(Node):
    """数据存储节点类"""
    
    def __init__(self):
        super().__init__('data_storage_node')
        
        # 加载配置文件
        self.load_config()
        
        # 检查是否启用多频率采集
        self.multi_frequency_enabled = self.config.get('multi_frequency', {}).get('enable_multi_frequency', False)
        self.get_logger().info(f"Multi-frequency mode: {self.multi_frequency_enabled}")
        
        # 检查是否启用VR按钮控制
        self.vr_button_control_enabled = self.config.get('vr_button_control', {}).get('enable_vr_button_control', False)
        self.get_logger().info(f"VR Button Control mode: {self.vr_button_control_enabled}")
        
        # 初始化各个模块
        if self.multi_frequency_enabled:
            # 使用多频率采集器
            self.multi_frequency_collector = MultiFrequencyCollector(self.config)
            self.data_processor = DataProcessor(self.config)
            self.storage_manager = StorageManager(self.config)
            self.data_subscriber = None  # 不使用传统订阅器
        else:
            # 使用传统订阅器（复用当前节点创建订阅）
            self.data_subscriber = DataSubscriber(self.config, self)
            self.data_processor = DataProcessor(self.config)
            self.storage_manager = StorageManager(self.config)
            self.multi_frequency_collector = None  # 不使用多频率采集器
        
        # 初始化VR按钮控制器
        if self.vr_button_control_enabled:
            self.vr_button_controller = VRButtonController(self.config, self)
            # 设置按钮回调
            self.vr_button_controller.set_button_callbacks(
                button_5_callback=self.handle_vr_button_5,
                button_6_callback=self.handle_vr_button_6
            )
            self.get_logger().info("VR Button Controller initialized")
        else:
            self.vr_button_controller = None
        
        # 数据记录控制
        self.recording_enabled = self.config['recording']['auto_record']
        self.recording_stats = {
            'start_time': time.time(),
            'total_records': 0,
            'processed_records': 0,
            'stored_records': 0,
            'error_count': 0
        }
        
        # 初始化控制话题
        self.init_control_publishers()
        self.init_control_subscribers()
        
        # 启动数据处理线程
        self.start_data_processing_thread()
        
        # 启动VR按钮数据读取定时器
        self.start_vr_button_timer()
        
        # 启动统计报告定时器
        self.start_statistics_timer()
        
        # 启动清理定时器
        self.start_cleanup_timer()
        
        self.get_logger().info("Data storage node initialized successfully")
    
    def load_config(self):
        """加载配置文件"""
        # 从ROS参数中获取配置文件路径
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果参数为空，使用默认路径
        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            package_share_directory = get_package_share_directory('data_storage')
            config_path = os.path.join(package_share_directory, 'config', 'data_storage_config.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
                self.config = config_data['data_storage']['ros__parameters']
                self.get_logger().info(f"Configuration loaded from: {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            sys.exit(1)
        
        # 允许通过启动参数覆盖 YAML 中的关键配置
        try:
            self.declare_parameter('storage_format', '')
            self.declare_parameter('storage_directory', '')
            override_format = self.get_parameter('storage_format').get_parameter_value().string_value
            override_dir = self.get_parameter('storage_directory').get_parameter_value().string_value
            if override_format:
                self.config['storage']['storage_format'] = override_format
            if override_dir:
                self.config['storage']['storage_directory'] = override_dir
        except Exception as e:
            # 覆盖失败不应导致退出，仅记录
            self.get_logger().warn(f"Failed to apply parameter overrides: {e}")
    
    def init_control_publishers(self):
        """初始化控制话题发布器"""
        # 状态发布器
        self.status_publisher = self.create_publisher(
            String,
            self.config['recording']['control']['status_topic'],
            10
        )
        
        # 统计信息发布器
        self.stats_publisher = self.create_publisher(
            String,
            '/data_storage/statistics',
            10
        )
    
    def init_control_subscribers(self):
        """初始化控制话题订阅器"""
        # 控制订阅器
        self.control_subscription = self.create_subscription(
            String,
            self.config['recording']['control']['control_topic'],
            self.control_callback,
            10
        )
    
    def control_callback(self, msg):
        """控制命令回调函数"""
        try:
            command = msg.data.strip().lower()
            
            if command == 'start':
                self.start_recording()
            elif command == 'stop':
                self.stop_recording()
            elif command == 'pause':
                self.pause_recording()
            elif command == 'resume':
                self.resume_recording()
            elif command == 'flush':
                self.flush_data()
            elif command == 'status':
                self.publish_status()
            elif command == 'stats':
                self.publish_statistics()
            else:
                self.get_logger().warn(f"Unknown control command: {command}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing control command: {e}")
    
    def start_recording(self):
        """开始记录数据"""
        self.recording_enabled = True
        self.recording_stats['start_time'] = time.time()
        self.get_logger().info("Data recording started")
        self.publish_status()
    
    def stop_recording(self):
        """停止记录数据"""
        self.recording_enabled = False
        self.flush_data()  # 刷新缓冲区
        # 强制创建新文件，为下次记录做准备
        if self.storage_manager.storage_format == 'hdf5':
            self.storage_manager.force_new_hdf5_file()
        self.get_logger().info("Data recording stopped")
        self.publish_status()
    
    def pause_recording(self):
        """暂停记录数据"""
        self.recording_enabled = False
        self.get_logger().info("Data recording paused")
        self.publish_status()
    
    def resume_recording(self):
        """恢复记录数据"""
        self.recording_enabled = True
        self.get_logger().info("Data recording resumed")
        self.publish_status()
    
    def flush_data(self):
        """刷新数据缓冲区"""
        try:
            self.storage_manager.flush_buffer()
            self.get_logger().info("Data buffer flushed")
        except Exception as e:
            self.get_logger().error(f"Error flushing data: {e}")
    
    def handle_vr_button_5(self, action):
        """处理VR第五个按钮事件 - 存储数据"""
        try:
            if action == 'press':
                if self.vr_button_controller and self.vr_button_controller.recording_enabled:
                    # 保存当前会话数据
                    self.vr_button_controller.save_current_session()
                    self.get_logger().info("VR Button 5: Session data saved")
                else:
                    self.get_logger().warn("VR Button 5: No active recording session to save")
        except Exception as e:
            self.get_logger().error(f"Error handling VR button 5: {e}")
    
    def handle_vr_button_6(self, action):
        """处理VR第六个按钮事件 - 开始/停止记录"""
        try:
            if action == 'start':
                # 开始记录
                self.start_recording()
                self.get_logger().info("VR Button 6: Recording started")
            elif action == 'stop':
                # 停止记录
                self.stop_recording()
                self.get_logger().info("VR Button 6: Recording stopped")
        except Exception as e:
            self.get_logger().error(f"Error handling VR button 6: {e}")
    
    def start_data_processing_thread(self):
        """启动数据处理线程"""
        self.data_processing_thread = threading.Thread(target=self.data_processing_loop, daemon=True)
        self.data_processing_thread.start()
        self.get_logger().info("Data processing thread started")
    
    def start_vr_button_timer(self):
        """启动VR按钮数据读取定时器"""
        if not self.multi_frequency_enabled and self.data_subscriber is not None:
            # 检查是否启用VR按钮数据
            if 'vr_left_buttons' in self.config['data_sources'] and self.config['data_sources']['vr_left_buttons']['enabled']:
                timer_period = 0.1  # 10Hz读取频率
                self.vr_button_timer = self.create_timer(timer_period, self.vr_button_timer_callback)
                self.get_logger().info("VR button data timer started")
    
    def vr_button_timer_callback(self):
        """VR按钮数据定时器回调"""
        try:
            if self.vr_button_controller is not None:
                # 读取VR按钮数据
                vr_data = self.vr_button_controller.read_vr_button_data()
                if vr_data is not None:
                    self.get_logger().debug("VR button data read successfully")
        except Exception as e:
            self.get_logger().error(f"Error in VR button timer callback: {e}")
    
    def data_processing_loop(self):
        """数据处理循环"""
        while rclpy.ok():
            try:
                if not self.recording_enabled:
                    time.sleep(0.1)
                    continue
                
                if self.multi_frequency_enabled:
                    # 使用多频率采集器
                    self.process_multi_frequency_data()
                else:
                    # 使用传统订阅器
                    self.process_traditional_data()
                
                # 如果启用VR按钮控制，处理VR按钮数据
                if self.vr_button_control_enabled and self.vr_button_controller:
                    self.process_vr_button_data()
                
                # 短暂休眠避免过度占用CPU
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f"Error in data processing loop: {e}")
                time.sleep(0.1)
    
    def process_multi_frequency_data(self):
        """处理多频率数据"""
        try:
            # 获取同步数据
            sync_data = self.multi_frequency_collector.get_sync_data()
            
            for sync_record in sync_data:
                try:
                    # 处理关节角度数据
                    if 'joint_angles' in sync_record:
                        joint_data = sync_record['joint_angles']
                        processed_joint_data = self.data_processor.process_data(joint_data)
                        
                        if processed_joint_data is not None:
                            # 添加同步信息
                            processed_joint_data['sync_info'] = {
                                'sync_quality': sync_record.get('sync_quality', 0.0),
                                'quality_score': sync_record.get('quality_score', 0.0),
                                'sync_timestamp': sync_record['timestamp']
                            }
                            
                            # 存储数据
                            self.storage_manager.store_data(processed_joint_data)
                            
                            # 更新统计信息
                            self.recording_stats['processed_records'] += 1
                            self.recording_stats['stored_records'] += 1
                        else:
                            self.recording_stats['error_count'] += 1
                    
                    # 处理笛卡尔位姿数据
                    if 'cartesian_pose' in sync_record:
                        pose_data = sync_record['cartesian_pose']
                        processed_pose_data = self.data_processor.process_data(pose_data)
                        
                        if processed_pose_data is not None:
                            # 添加同步信息
                            processed_pose_data['sync_info'] = {
                                'sync_quality': sync_record.get('sync_quality', 0.0),
                                'quality_score': sync_record.get('quality_score', 0.0),
                                'sync_timestamp': sync_record['timestamp']
                            }
                            
                            # 存储数据
                            self.storage_manager.store_data(processed_pose_data)
                            
                            # 更新统计信息
                            self.recording_stats['processed_records'] += 1
                            self.recording_stats['stored_records'] += 1
                        else:
                            self.recording_stats['error_count'] += 1
                    
                    self.recording_stats['total_records'] += 1
                    
                except Exception as e:
                    self.get_logger().error(f"Error processing sync data: {e}")
                    self.recording_stats['error_count'] += 1
            
            # 清空已处理的同步数据
            self.multi_frequency_collector.clear_sync_buffer()
            
        except Exception as e:
            self.get_logger().error(f"Error in multi-frequency processing: {e}")
    
    def process_traditional_data(self):
        """处理传统数据"""
        try:
            # 获取所有可用数据类型
            available_types = self.data_subscriber.get_available_data_types()
            
            for data_type in available_types:
                # 获取数据缓冲区
                buffer_data = self.data_subscriber.get_data_buffer(data_type)
                
                if buffer_data:
                    # 处理每个数据记录
                    for data_record in buffer_data:
                        try:
                            # 数据预处理
                            processed_data = self.data_processor.process_data(data_record)
                            
                            if processed_data is not None:
                                # 存储数据
                                self.storage_manager.store_data(processed_data)
                                
                                # 更新统计信息
                                self.recording_stats['processed_records'] += 1
                                self.recording_stats['stored_records'] += 1
                                
                            else:
                                self.recording_stats['error_count'] += 1
                            
                            self.recording_stats['total_records'] += 1
                            
                        except Exception as e:
                            self.get_logger().error(f"Error processing {data_type} data: {e}")
                            self.recording_stats['error_count'] += 1
                    
                    # 清空已处理的数据缓冲区
                    self.data_subscriber.clear_data_buffer(data_type)
                    
        except Exception as e:
            self.get_logger().error(f"Error in traditional data processing: {e}")
    
    def process_vr_button_data(self):
        """处理VR按钮数据"""
        try:
            # 读取VR按钮数据
            vr_data = self.vr_button_controller.read_vr_button_data()
            if vr_data is None:
                return
            
            # 如果正在记录，将数据添加到会话中
            if self.vr_button_controller.recording_enabled:
                # 创建数据记录
                data_record = {
                    'timestamp': vr_data['timestamp'],
                    'data': {
                        'button_5': vr_data['button_5'],
                        'button_6': vr_data['button_6']
                    },
                    'source': 'vr_left_buttons',
                    'raw_data': {
                        'button_5': vr_data['button_5'],
                        'button_6': vr_data['button_6']
                    },
                    'message_header': {
                        'frame_id': 'vr_left_controller',
                        'stamp': None
                    }
                }
                
                # 处理数据
                processed_data = self.data_processor.process_data(data_record)
                if processed_data is not None:
                    # 添加到VR按钮控制器的会话中
                    self.vr_button_controller.add_data(processed_data)
                    
                    # 同时存储到常规存储管理器
                    self.storage_manager.store_data(processed_data)
                    
                    # 更新统计信息
                    self.recording_stats['processed_records'] += 1
                    self.recording_stats['stored_records'] += 1
                else:
                    self.recording_stats['error_count'] += 1
                
                self.recording_stats['total_records'] += 1
                
        except Exception as e:
            self.get_logger().error(f"Error in VR button data processing: {e}")
    
    def start_statistics_timer(self):
        """启动统计报告定时器"""
        timer_period = 10.0  # 10秒
        self.statistics_timer = self.create_timer(timer_period, self.statistics_timer_callback)
        self.get_logger().info("Statistics timer started")
    
    def statistics_timer_callback(self):
        """统计报告定时器回调"""
        try:
            # 发布统计信息
            self.publish_statistics()
            
            # 记录日志统计信息
            if self.config['logging']['log_statistics']:
                self.log_statistics()
                
        except Exception as e:
            self.get_logger().error(f"Error in statistics timer: {e}")
    
    def start_cleanup_timer(self):
        """启动清理定时器"""
        cleanup_interval = self.config['performance']['cleanup_strategy']['cleanup_interval']
        self.cleanup_timer = self.create_timer(cleanup_interval, self.cleanup_timer_callback)
        self.get_logger().info("Cleanup timer started")
    
    def cleanup_timer_callback(self):
        """清理定时器回调"""
        try:
            self.storage_manager.cleanup_old_files()
        except Exception as e:
            self.get_logger().error(f"Error in cleanup: {e}")
    
    def publish_status(self):
        """发布状态信息"""
        try:
            status_msg = String()
            status_data = {
                'recording_enabled': self.recording_enabled,
                'timestamp': time.time(),
                'uptime': time.time() - self.recording_stats['start_time']
            }
            status_msg.data = str(status_data)
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def publish_statistics(self):
        """发布统计信息"""
        try:
            stats_msg = String()
            
            # 获取统计信息
            if self.multi_frequency_enabled:
                # 多频率采集器统计信息
                collector_stats = self.multi_frequency_collector.get_collection_stats()
                subscriber_stats = collector_stats
            else:
                # 传统订阅器统计信息
                subscriber_stats = self.data_subscriber.get_data_stats()
            
            # 获取存储管理器统计信息
            storage_stats = self.storage_manager.get_performance_stats()
            
            # 获取VR按钮控制统计信息
            vr_button_stats = {}
            if self.vr_button_control_enabled and self.vr_button_controller:
                vr_button_stats = self.vr_button_controller.get_button_stats()
            
            # 合并统计信息
            combined_stats = {
                'recording_stats': self.recording_stats,
                'subscriber_stats': subscriber_stats,
                'storage_stats': storage_stats,
                'multi_frequency_enabled': self.multi_frequency_enabled,
                'vr_button_control_enabled': self.vr_button_control_enabled,
                'vr_button_stats': vr_button_stats,
                'timestamp': time.time()
            }
            
            stats_msg.data = str(combined_stats)
            self.stats_publisher.publish(stats_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing statistics: {e}")
    
    def log_statistics(self):
        """记录统计信息到日志"""
        try:
            current_time = time.time()
            uptime = current_time - self.recording_stats['start_time']
            
            # 计算处理速率
            if uptime > 0:
                processing_rate = self.recording_stats['processed_records'] / uptime
                storage_rate = self.recording_stats['stored_records'] / uptime
            else:
                processing_rate = 0
                storage_rate = 0
            
            # 获取订阅/采集统计信息（根据模式选择来源）
            if self.multi_frequency_enabled and self.multi_frequency_collector is not None:
                subscriber_stats = self.multi_frequency_collector.get_collection_stats()
            elif self.data_subscriber is not None:
                subscriber_stats = self.data_subscriber.get_data_stats()
            else:
                subscriber_stats = {}
            
            self.get_logger().info(
                f"Statistics - "
                f"Uptime: {uptime:.1f}s, "
                f"Total Records: {self.recording_stats['total_records']}, "
                f"Processed: {self.recording_stats['processed_records']}, "
                f"Stored: {self.recording_stats['stored_records']}, "
                f"Errors: {self.recording_stats['error_count']}, "
                f"Processing Rate: {processing_rate:.2f} Hz, "
                f"Storage Rate: {storage_rate:.2f} Hz"
            )
            
            # 记录各数据源的统计信息（若可用）
            if isinstance(subscriber_stats, dict):
                for data_type, stats in subscriber_stats.items():
                    try:
                        count_val = stats.get('count') if isinstance(stats, dict) else None
                        last_received_val = stats.get('last_received') if isinstance(stats, dict) else None
                        self.get_logger().info(
                            f"{data_type} - "
                            f"Count: {count_val}, "
                            f"Last Received: {last_received_val}"
                        )
                    except Exception:
                        # 保护性日志，避免统计结构差异导致异常
                        self.get_logger().info(f"{data_type} - {stats}")
                
        except Exception as e:
            self.get_logger().error(f"Error logging statistics: {e}")
    
    def shutdown(self):
        """节点关闭时的清理工作"""
        try:
            # 停止记录
            self.recording_enabled = False
            
            # 刷新所有缓冲区
            self.flush_data()
            
            # 关闭存储管理器
            if hasattr(self.storage_manager, 'close_current_file'):
                self.storage_manager.close_current_file()
            
            self.get_logger().info("Data storage node shutdown completed")
            
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        data_storage_node = DataStorageNode()
        rclpy.spin(data_storage_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'data_storage_node' in locals():
            data_storage_node.shutdown()
            data_storage_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
