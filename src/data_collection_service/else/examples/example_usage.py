#!/usr/bin/env python3
"""
Data Storage Usage Example
数据存储使用示例

展示如何使用数据存储包的基本功能
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped
import time
import threading


class DataStorageExample(Node):
    """数据存储使用示例"""
    
    def __init__(self):
        super().__init__('data_storage_example')
        
        # 创建发布器用于测试
        self.joint_publisher = self.create_publisher(
            Float32MultiArray,
            '/robot_joint_angles',
            10
        )
        
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/robot_cartesian_pose',
            10
        )
        
        # 创建控制订阅器
        self.control_subscription = self.create_subscription(
            String,
            '/data_storage/control',
            self.control_callback,
            10
        )
        
        # 创建状态订阅器
        self.status_subscription = self.create_subscription(
            String,
            '/data_storage/status',
            self.status_callback,
            10
        )
        
        # 创建统计信息订阅器
        self.stats_subscription = self.create_subscription(
            String,
            '/data_storage/statistics',
            self.stats_callback,
            10
        )
        
        # 启动数据发布线程
        self.start_data_publishing()
        
        self.get_logger().info("Data storage example started")
    
    def start_data_publishing(self):
        """启动数据发布线程"""
        self.publishing_thread = threading.Thread(target=self.publish_test_data, daemon=True)
        self.publishing_thread.start()
    
    def publish_test_data(self):
        """发布测试数据"""
        counter = 0
        
        while rclpy.ok():
            try:
                # 发布关节角度数据
                joint_msg = Float32MultiArray()
                joint_msg.data = [
                    10.0 + counter * 0.1,
                    20.0 + counter * 0.1,
                    30.0 + counter * 0.1,
                    40.0 + counter * 0.1,
                    50.0 + counter * 0.1,
                    60.0 + counter * 0.1
                ]
                self.joint_publisher.publish(joint_msg)
                
                # 发布笛卡尔位姿数据
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "robot_base"
                
                # 位置数据
                pose_msg.pose.position.x = 0.1 + counter * 0.01
                pose_msg.pose.position.y = 0.2 + counter * 0.01
                pose_msg.pose.position.z = 0.3 + counter * 0.01
                
                # 四元数数据
                pose_msg.pose.orientation.x = 0.0
                pose_msg.pose.orientation.y = 0.0
                pose_msg.pose.orientation.z = 0.0
                pose_msg.pose.orientation.w = 1.0
                
                self.pose_publisher.publish(pose_msg)
                
                counter += 1
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.get_logger().error(f"Error publishing test data: {e}")
                time.sleep(0.1)
    
    def control_callback(self, msg):
        """控制命令回调"""
        self.get_logger().info(f"Received control command: {msg.data}")
    
    def status_callback(self, msg):
        """状态信息回调"""
        self.get_logger().info(f"Data storage status: {msg.data}")
    
    def stats_callback(self, msg):
        """统计信息回调"""
        self.get_logger().info(f"Data storage statistics: {msg.data}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        example = DataStorageExample()
        rclpy.spin(example)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'example' in locals():
            example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
