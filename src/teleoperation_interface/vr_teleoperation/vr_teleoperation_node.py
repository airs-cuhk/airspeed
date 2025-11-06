#!/usr/bin/env python3.10
"""
VR Teleoperation Integrated Node
VR遥操作集成节点

该节点订阅原始VR数据ROS2话题，提取所需字段，进行坐标转换，
并发布处理后的ROS2话题供机器人接口使用。

Dependencies:
- Python 3.10+
- PyYAML (for YAML config parsing)
- numpy (for coordinate transformation)
- rclpy (for ROS2)
- geometry_msgs (for PoseStamped message)
- std_msgs (for String and Float32MultiArray message types)
"""

import json
import os
import yaml
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray, String
from typing import Dict, Any, Optional, List


class VRTeleoperationNode(Node):
    """VR遥操作集成节点"""
    
    def __init__(self, config_path: str = "src/teleoperation_interface/config/teleoperation_config.yaml"):
        """
        初始化VR遥操作节点
        
        Args:
            config_path: 配置文件路径
        """
        super().__init__('vr_teleoperation_node')
        
        # 声明ROS2参数
        self.declare_parameter('config_file', config_path)
        
        # 获取参数值
        self.config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        self.config = self._load_config()
        
        # 获取数据提取配置
        self.useful_fields = self.config.get("useful_fields", {})
        self.include_timestamp = self.config.get("extraction", {}).get("include_timestamp", True)
        
        # 获取坐标转换配置
        self.transform_config = self.config.get("coordinate_transformation", {})
        matrix_config = self.transform_config.get("transformation_matrix", [])
        self.transform_matrix = np.array(matrix_config)
        self.transform_position = self.transform_config.get("transform_position", True)
        self.transform_rotation = self.transform_config.get("transform_rotation", True)
        
        # 获取ROS2发布配置
        self.ros_config = self.config.get("ros_publish", {})
        self.topics_config = self.ros_config.get("topics", {})
        self.publish_rate = self.ros_config.get("publish_rate", 10.0)
        self.publish_enabled = self.ros_config.get("enabled", True)
        
        # 创建ROS2发布器
        self.device_publishers = {}
        self.button_publishers = {}
        if self.publish_enabled:
            for device in ["head", "left", "right"]:
                if device in self.topics_config:
                    topic_config = self.topics_config[device]
                    # 创建姿态发布器
                    pose_topic_name = topic_config.get("topic_name", f"/target_{device}_pose")
                    self.device_publishers[device] = self.create_publisher(PoseStamped, pose_topic_name, 10)
                    self.get_logger().info(f"Created pose publisher for {device}: {pose_topic_name}")
                    
                    # 为左手柄和右手柄创建按钮发布器
                    if device in ["left", "right"]:
                        button_topic_name = topic_config.get("button_topic_name", f"/target_{device}_buttons")
                        self.button_publishers[device] = self.create_publisher(Float32MultiArray, button_topic_name, 10)
                        self.get_logger().info(f"Created button publisher for {device}: {button_topic_name}")
        
        # 创建订阅器：订阅原始VR数据（仅缓存最新数据，不直接发布）
        self.vr_data_subscription = self.create_subscription(
            String,
            '/vr_raw_data',
            self.vr_data_update_callback,
            10
        )
        self.get_logger().info("Subscribed to topic: /vr_raw_data")
        
        # 最新VR数据缓存
        self.latest_vr_data = None
        self.latest_vr_data_lock = threading.Lock()
        
        # 创建定时器 - 像旧版本一样以固定频率发布
        if self.publish_enabled:
            self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)
            self.get_logger().info(f"Created timer with rate: {self.publish_rate} Hz")
        
        self.get_logger().info("VR Teleoperation Node initialized successfully")
        self.get_logger().info(f"Using TIMER mode (like old JSON version) at {self.publish_rate} Hz for stable timing")
        self.get_logger().info("Waiting for VR data from /vr_raw_data topic...")
    
    def _load_config(self) -> Dict[str, Any]:
        """加载配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # 如果配置在tele_data_transformer.ros__parameters下，提取出来
            if 'tele_data_transformer' in config and 'ros__parameters' in config['tele_data_transformer']:
                return config['tele_data_transformer']['ros__parameters']
            
            return config
        except FileNotFoundError:
            self.get_logger().error(f"Config file {self.config_path} not found")
            return self._get_default_config()
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing config file: {e}")
            return self._get_default_config()
    
    def _get_default_config(self) -> Dict[str, Any]:
        """获取默认配置"""
        return {
            "useful_fields": {
                "head": {
                    "position": {"x": True, "y": True, "z": True},
                    "rotation": {"x": True, "y": True, "z": True, "w": True}
                },
                "left": {
                    "position": {"x": True, "y": True, "z": True},
                    "rotation": {"x": True, "y": True, "z": True, "w": True},
                    "button": {"enabled": True}
                },
                "right": {
                    "position": {"x": True, "y": True, "z": True},
                    "rotation": {"x": True, "y": True, "z": True, "w": True},
                    "button": {"enabled": True}
                }
            },
            "extraction": {
                "include_timestamp": True
            },
            "coordinate_transformation": {
                "transformation_matrix": [[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
                "transform_position": True,
                "transform_rotation": True
            },
            "ros_publish": {
                "topics": {
                    "head": {
                        "topic_name": "/target_head_pose",
                        "frame_id": "robot_world"
                    },
                    "left": {
                        "topic_name": "/target_left_arm_pose",
                        "button_topic_name": "/target_left_buttons",
                        "frame_id": "robot_world"
                    },
                    "right": {
                        "topic_name": "/target_right_arm_pose",
                        "button_topic_name": "/target_right_buttons",
                        "frame_id": "robot_world"
                    }
                },
                "publish_rate": 10.0,
                "enabled": True
            }
        }
    
    def _extract_field_data(self, data: Dict[str, Any], field_path: str) -> Optional[Any]:
        """
        根据字段路径提取数据
        
        Args:
            data: 原始数据字典
            field_path: 字段路径，如 "head.position.x"
            
        Returns:
            提取的数据值，如果不存在则返回 None
        """
        try:
            keys = field_path.split('.')
            current = data
            for key in keys:
                current = current[key]
            return current
        except (KeyError, TypeError):
            return None
    
    def _extract_device_data(self, vr_data: Dict[str, Any], device: str) -> Dict[str, Any]:
        """
        提取单个设备的数据
        
        Args:
            vr_data: VR原始数据
            device: 设备名称 (head, left, right)
            
        Returns:
            提取的设备数据
        """
        device_data = {}
        
        # 检查设备是否存在
        if device not in vr_data:
            return device_data
        
        device_config = self.useful_fields.get(device, {})
        
        # 提取 position 数据
        if "position" in device_config:
            position_data = {}
            for axis, enabled in device_config["position"].items():
                if enabled:
                    value = self._extract_field_data(vr_data, f"{device}.position.{axis}")
                    if value is not None:
                        position_data[axis] = value
            if position_data:
                device_data["position"] = position_data
        
        # 提取 rotation 数据
        if "rotation" in device_config:
            rotation_data = {}
            for axis, enabled in device_config["rotation"].items():
                if enabled:
                    value = self._extract_field_data(vr_data, f"{device}.rotation.{axis}")
                    if value is not None:
                        rotation_data[axis] = value
            if rotation_data:
                device_data["rotation"] = rotation_data
        
        # 提取 button 数据（仅对左手柄和右手柄）
        if device in ["left", "right"] and "button" in device_config and device_config["button"].get("enabled", False):
            button_data = self._extract_button_data(vr_data, device)
            if button_data:
                device_data["button"] = button_data
        
        return device_data
    
    def _extract_button_data(self, vr_data: Dict[str, Any], device: str) -> Optional[List[float]]:
        """
        提取按钮数据
        
        Args:
            vr_data: VR原始数据
            device: 设备名称 (left, right)
            
        Returns:
            按钮数据列表 [button0_value, button1_value, ..., button5_value]
        """
        try:
            if device not in vr_data or "button" not in vr_data[device]:
                return None
            
            buttons = vr_data[device]["button"]
            if not isinstance(buttons, list) or len(buttons) < 6:
                return None
            
            # 提取所有6个按钮的value值
            button_values = []
            for i in range(6):
                if i < len(buttons):
                    button_value = buttons[i].get("value", 0.0)
                    button_values.append(float(button_value))
                else:
                    button_values.append(0.0)
            
            return button_values
            
        except Exception as e:
            self.get_logger().error(f"Error extracting button data for {device}: {e}")
            return None
    
    def _transform_position(self, position: List[float]) -> List[float]:
        """
        转换位置坐标
        
        Args:
            position: 原始位置 [x, y, z]
            
        Returns:
            转换后的位置 [x, y, z]
        """
        if not self.transform_position:
            return position
        
        # 将位置向量转换为numpy数组
        pos_array = np.array(position)
        
        # 应用变换矩阵
        transformed_pos = self.transform_matrix @ pos_array
        
        return transformed_pos.tolist()
    
    def _transform_rotation(self, rotation: List[float]) -> List[float]:
        """
        转换旋转四元数
        
        Args:
            rotation: 原始四元数 [x, y, z, w]
            
        Returns:
            转换后的四元数 [x, y, z, w]
        """
        if not self.transform_rotation:
            return rotation
        
        # 将四元数转换为numpy数组
        quat_array = np.array(rotation)
        
        # 四元数转旋转矩阵
        rotation_matrix = self._quaternion_to_rotation_matrix(quat_array)
        
        # 应用坐标变换
        transformed_matrix = self.transform_matrix @ rotation_matrix
        
        # 旋转矩阵转四元数
        transformed_quat = self._rotation_matrix_to_quaternion(transformed_matrix)
        
        return transformed_quat.tolist()
    
    def _quaternion_to_rotation_matrix(self, quat: np.ndarray) -> np.ndarray:
        """
        四元数转旋转矩阵
        
        Args:
            quat: 四元数 [x, y, z, w]
            
        Returns:
            3x3旋转矩阵
        """
        # 归一化四元数
        quat = quat / np.linalg.norm(quat)
        x, y, z, w = quat
        
        # 四元数转旋转矩阵
        rotation_matrix = np.array([
            [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
            [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
            [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
        ])
        
        return rotation_matrix
    
    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        旋转矩阵转四元数
        
        Args:
            rotation_matrix: 3x3旋转矩阵
            
        Returns:
            四元数 [x, y, z, w]
        """
        trace = np.trace(rotation_matrix)
        
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
            y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
            z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                S = np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2]) * 2
                w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / S
                x = 0.25 * S
                y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
                z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                S = np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2]) * 2
                w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / S
                x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / S
                y = 0.25 * S
                z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
            else:
                S = np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1]) * 2
                w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / S
                x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / S
                y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / S
                z = 0.25 * S
        
        return np.array([x, y, z, w])
    
    def _transform_device_data(self, device_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        转换单个设备的数据
        
        Args:
            device_data: 设备数据，包含position和rotation
            
        Returns:
            转换后的设备数据
        """
        transformed_data = {}
        
        # 转换位置数据
        if "position" in device_data:
            position = device_data["position"]
            if isinstance(position, dict):
                # 如果是字典格式，转换为列表
                pos_list = [position.get("x", 0.0), position.get("y", 0.0), position.get("z", 0.0)]
            else:
                pos_list = position
            
            transformed_position = self._transform_position(pos_list)
            transformed_data["position"] = {
                "x": transformed_position[0],
                "y": transformed_position[1],
                "z": transformed_position[2]
            }
        
        # 转换旋转数据
        if "rotation" in device_data:
            rotation = device_data["rotation"]
            if isinstance(rotation, dict):
                # 如果是字典格式，转换为列表
                rot_list = [rotation.get("x", 0.0), rotation.get("y", 0.0), 
                           rotation.get("z", 0.0), rotation.get("w", 1.0)]
            else:
                rot_list = rotation
            
            transformed_rotation = self._transform_rotation(rot_list)
            transformed_data["rotation"] = {
                "x": transformed_rotation[0],
                "y": transformed_rotation[1],
                "z": transformed_rotation[2],
                "w": transformed_rotation[3]
            }
        
        # 保留按钮数据（不需要转换）
        if "button" in device_data:
            transformed_data["button"] = device_data["button"]
        
        return transformed_data
    
    def _create_pose_stamped(self, device_data: Dict[str, Any], device_name: str) -> PoseStamped:
        """
        创建PoseStamped消息
        
        Args:
            device_data: 设备数据
            device_name: 设备名称
            
        Returns:
            PoseStamped消息
        """
        msg = PoseStamped()
        
        # 设置消息头
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 获取对应设备的frame_id
        if device_name in self.topics_config:
            frame_id = self.topics_config[device_name].get("frame_id", "robot_world")
        else:
            frame_id = "robot_world"
        msg.header.frame_id = frame_id
        
        # 设置位置
        if "position" in device_data:
            pos = device_data["position"]
            if isinstance(pos, dict):
                msg.pose.position.x = float(pos.get("x", 0.0))
                msg.pose.position.y = float(pos.get("y", 0.0))
                msg.pose.position.z = float(pos.get("z", 0.0))
            else:
                msg.pose.position.x = float(pos[0])
                msg.pose.position.y = float(pos[1])
                msg.pose.position.z = float(pos[2])
        
        # 设置旋转
        if "rotation" in device_data:
            rot = device_data["rotation"]
            if isinstance(rot, dict):
                msg.pose.orientation.x = float(rot.get("x", 0.0))
                msg.pose.orientation.y = float(rot.get("y", 0.0))
                msg.pose.orientation.z = float(rot.get("z", 0.0))
                msg.pose.orientation.w = float(rot.get("w", 1.0))
            else:
                msg.pose.orientation.x = float(rot[0])
                msg.pose.orientation.y = float(rot[1])
                msg.pose.orientation.z = float(rot[2])
                msg.pose.orientation.w = float(rot[3])
        
        return msg
    
    def _create_button_message(self, button_data: List[float], device_name: str) -> Float32MultiArray:
        """
        创建按钮消息
        
        Args:
            button_data: 按钮数据列表 [button0_value, button1_value, ..., button5_value]
            device_name: 设备名称
            
        Returns:
            Float32MultiArray消息
        """
        msg = Float32MultiArray()
        msg.data = button_data
        return msg
    
    def _process_vr_data(self, vr_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        处理VR数据：提取、转换并返回结果
        
        Args:
            vr_data: VR原始数据
            
        Returns:
            处理后的数据
        """
        # 初始化处理结果
        processed_data = {}
        
        # 添加时间戳（如果配置要求）
        if self.include_timestamp:
            processed_data["timestamp"] = time.time()
        
        # 处理各设备数据
        devices = ["head", "left", "right"]
        for device in devices:
            # 提取设备数据
            device_data = self._extract_device_data(vr_data, device)
            if device_data:  # 只有当设备数据不为空时才处理
                # 转换设备数据
                transformed_device = self._transform_device_data(device_data)
                if transformed_device:
                    processed_data[device] = transformed_device
        
        return processed_data
    
    def vr_data_update_callback(self, msg):
        """轻量级回调：仅更新最新VR数据缓存（快速返回，不阻塞）
        
        Args:
            msg: String消息，包含JSON格式的VR数据
        """
        try:
            # 快速更新最新数据（线程安全）
            with self.latest_vr_data_lock:
                self.latest_vr_data = msg.data
        except Exception as e:
            self.get_logger().error(f"Error updating VR data: {e}")
    
    def publish_callback(self):
        """定时器回调：以固定频率处理并发布VR数据（像旧版本一样）"""
        if not self.publish_enabled:
            return
        
        try:
            # 获取最新VR数据（线程安全）
            with self.latest_vr_data_lock:
                if self.latest_vr_data is None:
                    return
                vr_data_str = self.latest_vr_data
            
            # 解析JSON字符串
            vr_data = json.loads(vr_data_str)
            
            # 处理VR数据（提取+转换）
            processed_data = self._process_vr_data(vr_data)
            
            # 发布所有设备数据
            for device in ["head", "left", "right"]:
                if device in processed_data and device in self.device_publishers:
                    # 发布姿态数据
                    pose_msg = self._create_pose_stamped(processed_data[device], device)
                    self.device_publishers[device].publish(pose_msg)
                    self.get_logger().debug(f"Published pose for {device}: pos=({pose_msg.pose.position.x:.3f}, {pose_msg.pose.position.y:.3f}, {pose_msg.pose.position.z:.3f})")
                    
                    # 发布按钮数据（仅对左手柄和右手柄）
                    if device in ["left", "right"] and device in self.button_publishers:
                        if "button" in processed_data[device]:
                            button_msg = self._create_button_message(processed_data[device]["button"], device)
                            self.button_publishers[device].publish(button_msg)
                            self.get_logger().info(f"Published buttons for {device}: {processed_data[device]['button']}")
                        else:
                            self.get_logger().warn(f"No button data found for {device} in processed_data")
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse VR data JSON: {e}")
        except KeyError as e:
            self.get_logger().error(f"Missing required field in VR data: {e}")
        except Exception as e:
            self.get_logger().error(f"Unexpected error in publish_callback: {e}")


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        node = VRTeleoperationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("收到中断信号，正在关闭节点...")
    except Exception as e:
        print(f"节点运行出错: {e}")
    finally:
        try:
            rclpy.shutdown()
        except Exception as e:
            print(f"关闭ROS2上下文时出错: {e}")


if __name__ == "__main__":
    main()
