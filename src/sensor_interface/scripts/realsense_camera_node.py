#!/usr/bin/env python3
"""
Realsense Camera Node
Realsense相机节点

该节点负责：
1. 初始化Realsense D435i相机
2. 发布RGB图像和深度图像数据
3. 提供相机参数配置
4. 处理相机数据流
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
import cv2
import numpy as np
import pyrealsense2 as rs
import yaml
import os
import sys
import threading
import time
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs


class RealsenseCameraNode(Node):
    """Realsense相机节点类"""
    
    def __init__(self):
        super().__init__('realsense_camera_node')
        
        # 加载配置文件
        self.load_config()
        
        # 初始化相机
        self.init_camera()
        
        # 初始化ROS2发布器
        self.init_publishers()
        
        # 初始化TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化CV桥接器
        self.bridge = CvBridge()
        
        # 数据流控制
        self.streaming_enabled = True
        self.frame_count = 0
        
        # 启动数据流线程
        self.start_streaming_thread()
        
        # 启动TF广播定时器
        self.start_tf_timer()
        
        self.get_logger().info("Realsense camera node initialized successfully")
    
    def load_config(self):
        """加载配置文件"""
        # 从ROS参数中获取配置文件路径
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果参数为空，使用默认路径
        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            package_share_directory = get_package_share_directory('sensor_interface')
            config_path = os.path.join(package_share_directory, 'config', 'realsense_config.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
                self.config = config_data['sensor_interface']['ros__parameters']
                self.get_logger().info(f"Configuration loaded from: {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # 使用默认配置
            self.config = self.get_default_config()
            self.get_logger().warn("Using default configuration")
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'camera': {
                'width': 640,
                'height': 480,
                'fps': 30,
                'depth_format': 'Z16',
                'color_format': 'BGR8'
            },
            'topics': {
                'rgb_image': '/camera/color/image_raw',
                'depth_image': '/camera/depth/image_raw',
                'rgb_camera_info': '/camera/color/camera_info',
                'depth_camera_info': '/camera/depth/camera_info',
                'pointcloud': '/camera/depth/points'
            },
            'frame_id': 'camera_link',
            'child_frame_id': 'camera_depth_optical_frame',
            'publish_tf': True,
            'tf_transform': {
                'translation': [0.0, 0.0, 0.0],
                'rotation': [0.0, 0.0, 0.0, 1.0]
            }
        }
    
    def init_camera(self):
        """初始化Realsense相机"""
        try:
            # 创建管道
            self.pipeline = rs.pipeline()
            
            # 创建配置
            self.config_rs = rs.config()
            
            # 配置流
            self.config_rs.enable_stream(
                rs.stream.depth,
                self.config['camera']['width'],
                self.config['camera']['height'],
                rs.format.z16,
                self.config['camera']['fps']
            )
            
            self.config_rs.enable_stream(
                rs.stream.color,
                self.config['camera']['width'],
                self.config['camera']['height'],
                rs.format.bgr8,
                self.config['camera']['fps']
            )
            
            # 启动管道
            self.profile = self.pipeline.start(self.config_rs)
            
            # 获取相机内参
            self.depth_intrinsics = None
            self.color_intrinsics = None
            
            for stream in self.profile.get_streams():
                if stream.stream_type() == rs.stream.depth:
                    self.depth_intrinsics = stream.as_video_stream_profile().get_intrinsics()
                elif stream.stream_type() == rs.stream.color:
                    self.color_intrinsics = stream.as_video_stream_profile().get_intrinsics()
            
            self.get_logger().info("Realsense camera initialized successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            sys.exit(1)
    
    def init_publishers(self):
        """初始化ROS2发布器"""
        # RGB图像发布器
        self.rgb_publisher = self.create_publisher(
            Image,
            self.config['topics']['rgb_image'],
            10
        )
        
        # 深度图像发布器
        self.depth_publisher = self.create_publisher(
            Image,
            self.config['topics']['depth_image'],
            10
        )
        
        # RGB相机信息发布器
        self.rgb_camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.config['topics']['rgb_camera_info'],
            10
        )
        
        # 深度相机信息发布器
        self.depth_camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.config['topics']['depth_camera_info'],
            10
        )
        
        # 点云发布器
        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            self.config['topics']['pointcloud'],
            10
        )
        
        self.get_logger().info("ROS2 publishers initialized")
    
    def start_streaming_thread(self):
        """启动数据流线程"""
        self.streaming_thread = threading.Thread(target=self.streaming_loop, daemon=True)
        self.streaming_thread.start()
        self.get_logger().info("Streaming thread started")
    
    def start_tf_timer(self):
        """启动TF广播定时器"""
        if self.config['publish_tf']:
            timer_period = 0.1  # 10Hz
            self.tf_timer = self.create_timer(timer_period, self.publish_tf)
            self.get_logger().info("TF timer started")
    
    def streaming_loop(self):
        """数据流循环"""
        while rclpy.ok() and self.streaming_enabled:
            try:
                # 等待帧
                frames = self.pipeline.wait_for_frames()
                
                # 获取深度帧和彩色帧
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # 转换帧数据
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # 发布图像数据
                self.publish_images(depth_image, color_image, frames.get_timestamp())
                
                # 发布相机信息
                self.publish_camera_info()
                
                # 发布点云数据
                self.publish_pointcloud(depth_frame, color_frame)
                
                self.frame_count += 1
                
            except Exception as e:
                self.get_logger().error(f"Error in streaming loop: {e}")
                time.sleep(0.1)
    
    def publish_images(self, depth_image, color_image, timestamp):
        """发布图像数据"""
        try:
            # 创建消息头
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.config['frame_id']
            
            # 发布RGB图像
            rgb_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            rgb_msg.header = header
            self.rgb_publisher.publish(rgb_msg)
            
            # 发布深度图像
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            depth_msg.header = header
            self.depth_publisher.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing images: {e}")
    
    def publish_camera_info(self):
        """发布相机信息"""
        try:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.config['frame_id']
            
            # RGB相机信息
            if self.color_intrinsics:
                rgb_camera_info = CameraInfo()
                rgb_camera_info.header = header
                rgb_camera_info.width = self.color_intrinsics.width
                rgb_camera_info.height = self.color_intrinsics.height
                rgb_camera_info.distortion_model = "plumb_bob"
                rgb_camera_info.d = list(self.color_intrinsics.coeffs)
                
                # 确保K矩阵的值是有效的浮点数
                fx = float(self.color_intrinsics.fx) if not np.isnan(self.color_intrinsics.fx) else 0.0
                fy = float(self.color_intrinsics.fy) if not np.isnan(self.color_intrinsics.fy) else 0.0
                ppx = float(self.color_intrinsics.ppx) if not np.isnan(self.color_intrinsics.ppx) else 0.0
                ppy = float(self.color_intrinsics.ppy) if not np.isnan(self.color_intrinsics.ppy) else 0.0
                
                rgb_camera_info.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
                rgb_camera_info.p = [fx, 0.0, ppx, 0.0, 0.0, fy, ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
                self.rgb_camera_info_publisher.publish(rgb_camera_info)
            
            # 深度相机信息
            if self.depth_intrinsics:
                depth_camera_info = CameraInfo()
                depth_camera_info.header = header
                depth_camera_info.width = self.depth_intrinsics.width
                depth_camera_info.height = self.depth_intrinsics.height
                depth_camera_info.distortion_model = "plumb_bob"
                depth_camera_info.d = list(self.depth_intrinsics.coeffs)
                
                # 确保K矩阵的值是有效的浮点数
                fx = float(self.depth_intrinsics.fx) if not np.isnan(self.depth_intrinsics.fx) else 0.0
                fy = float(self.depth_intrinsics.fy) if not np.isnan(self.depth_intrinsics.fy) else 0.0
                ppx = float(self.depth_intrinsics.ppx) if not np.isnan(self.depth_intrinsics.ppx) else 0.0
                ppy = float(self.depth_intrinsics.ppy) if not np.isnan(self.depth_intrinsics.ppy) else 0.0
                
                depth_camera_info.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
                depth_camera_info.p = [fx, 0.0, ppx, 0.0, 0.0, fy, ppy, 0.0, 0.0, 0.0, 1.0, 0.0]
                self.depth_camera_info_publisher.publish(depth_camera_info)
                
        except Exception as e:
            self.get_logger().error(f"Error publishing camera info: {e}")
    
    def publish_pointcloud(self, depth_frame, color_frame):
        """发布点云数据"""
        try:
            # 创建点云
            pc = rs.pointcloud()
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)
            
            # 获取顶点数据
            vertices = np.asanyarray(points.get_vertices())
            if len(vertices) == 0:
                return
                
            # 创建点云消息
            pointcloud_msg = PointCloud2()
            pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
            pointcloud_msg.header.frame_id = self.config['frame_id']
            
            # 设置点云数据
            pointcloud_msg.height = 1
            pointcloud_msg.width = len(vertices)
            pointcloud_msg.is_dense = False
            
            # 创建字段
            from sensor_msgs.msg import PointField
            pointcloud_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            
            pointcloud_msg.point_step = 12
            pointcloud_msg.row_step = pointcloud_msg.point_step * pointcloud_msg.width
            
            # 正确处理结构化数组
            if vertices.dtype.names is not None:
                # 如果是结构化数组，提取坐标
                vertices_array = np.column_stack([vertices[name] for name in vertices.dtype.names])
            else:
                vertices_array = vertices
            
            pointcloud_msg.data = vertices_array.astype(np.float32).tobytes()
            
            self.pointcloud_publisher.publish(pointcloud_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing pointcloud: {e}")
    
    def publish_tf(self):
        """发布TF变换"""
        try:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.config['frame_id']
            t.child_frame_id = self.config['child_frame_id']
            
            # 设置变换
            t.transform.translation.x = self.config['tf_transform']['translation'][0]
            t.transform.translation.y = self.config['tf_transform']['translation'][1]
            t.transform.translation.z = self.config['tf_transform']['translation'][2]
            
            t.transform.rotation.x = self.config['tf_transform']['rotation'][0]
            t.transform.rotation.y = self.config['tf_transform']['rotation'][1]
            t.transform.rotation.z = self.config['tf_transform']['rotation'][2]
            t.transform.rotation.w = self.config['tf_transform']['rotation'][3]
            
            self.tf_broadcaster.sendTransform(t)
            
        except Exception as e:
            self.get_logger().error(f"Error publishing TF: {e}")
    
    def shutdown(self):
        """节点关闭时的清理工作"""
        try:
            self.streaming_enabled = False
            if hasattr(self, 'pipeline'):
                self.pipeline.stop()
            self.get_logger().info("Realsense camera node shutdown completed")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        realsense_node = RealsenseCameraNode()
        rclpy.spin(realsense_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'realsense_node' in locals():
            realsense_node.shutdown()
            realsense_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



