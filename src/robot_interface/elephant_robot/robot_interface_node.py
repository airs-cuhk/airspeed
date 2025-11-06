#!/usr/bin/env python3
"""
Robot Interface Node
机器人接口节点

该节点负责：
1. 订阅遥操作节点发布的ROS2话题/target_right_arm_pose
2. 数据提取模块：根据配置文件提取指定字段数据
3. 数据转换模块：进行坐标转换和参数转换
4. 机器人控制：发送运动指令控制机器人运动
5. 数据回传模块：实时获取机器人状态数据并通过ROS2话题发布
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
import sys
import yaml
import os
import threading
import time
import queue
from pymycobot.elephantrobot import ElephantRobot


class RobotInterfaceNode(Node):
    """机器人接口节点类"""
    
    def __init__(self):
        super().__init__('robot_interface_node')
        
        # 加载配置文件
        self.load_config()
        
        # 初始化通用API命令映射
        self.init_robot_commands()
        
        # 初始化机器人连接
        self.init_robot_connection()
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            PoseStamped,
            '/target_right_arm_pose',
            self.pose_callback,
            10
        )
        
        # 创建按钮数据订阅者
        self.button_subscription = self.create_subscription(
            Float32MultiArray,
            '/target_right_buttons',
            self.button_callback,
            10
        )
        
        # 初始化数据提取模块
        self.data_extractor = DataExtractor(self.config['useful_fields'])
        
        # 初始化数据转换模块
        self.data_transformer = DataTransformer(self.config['coordinate_transformation'])
        
        # 初始化机器人控制器（传入通用命令配置）
        self.robot_controller = RobotController(
            self.config['robot_control'], 
            self.config['data_processing'],
            self.robot_commands  # 传入通用命令映射
        )
        
        # 启动命令工作线程（异步执行机械臂命令）
        self.robot_controller.start_command_worker(self.mc)
        
        # 初始化数据回传模块（传入通用命令配置）
        self.data_feedback = DataFeedbackModule(
            self.config['data_feedback'], 
            self.mc,
            self.robot_commands  # 传入通用命令映射
        )
        
        # 初始化ROS2话题发布器
        self.init_feedback_publishers()
        
        # 初始化状态变量
        self.initial_hand_position = None
        self.initial_hand_rotation = None
        self.log_counter = 0
        
        # 启动数据回传定时器
        self.start_feedback_timer()
        
        self.get_logger().info("Robot Interface Node initialized successfully")
    
    def load_config(self):
        """加载机器人配置文件"""
        # 从ROS参数中获取配置文件路径
        self.declare_parameter('config_file', '')
        config_path = self.get_parameter('config_file').get_parameter_value().string_value
        
        # 如果参数为空，使用默认路径
        if not config_path:
            from ament_index_python.packages import get_package_share_directory
            package_share_directory = get_package_share_directory('robot_interface')
            config_path = os.path.join(package_share_directory, 'robot_config.yaml')
        
        try:
            with open(config_path, 'r', encoding='utf-8') as file:
                config_data = yaml.safe_load(file)
                self.config = config_data['robot_interface']['ros__parameters']
                self.get_logger().info(f"Configuration loaded from: {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            sys.exit(1)
    
    def init_robot_commands(self):
        """初始化通用机器人命令映射
        
        从配置文件读取命令名称，实现与具体机器人API的解耦。
        更换机器人时只需修改配置文件中的命令映射即可。
        """
        self.robot_commands = self.config['robot_control']['commands']
        self.get_logger().info("=" * 60)
        self.get_logger().info("通用机器人API命令映射已加载:")
        self.get_logger().info(f"  - 电源开启: {self.robot_commands['power_on_command']}")
        self.get_logger().info(f"  - 启动机器人: {self.robot_commands['start_robot_command']}")
        self.get_logger().info(f"  - 位置控制: {self.robot_commands['position_command']}")
        self.get_logger().info(f"  - 角度控制: {self.robot_commands['angle_command']}")
        self.get_logger().info(f"  - 夹爪模式: {self.robot_commands['gripper_mode_command']}")
        self.get_logger().info(f"  - 夹爪状态: {self.robot_commands['gripper_state_command']}")
        self.get_logger().info(f"  - 获取角度: {self.robot_commands['get_angles_command']}")
        self.get_logger().info(f"  - 获取坐标: {self.robot_commands['get_coords_command']}")
        self.get_logger().info("更换机器人时只需修改robot_config.yaml中的commands配置")
        self.get_logger().info("=" * 60)
    
    def call_robot_api(self, command_key, *args):
        """通用机器人API调用方法
        
        Args:
            command_key: 命令键名（在robot_commands中定义）
            *args: API调用参数
        
        Returns:
            API调用返回值，失败返回None
        
        Example:
            # 调用位置控制命令
            self.call_robot_api('position_command', target_position, speed)
            
            # 调用获取角度命令
            angles = self.call_robot_api('get_angles_command')
        """
        if self.mc is None:
            return None
        
        try:
            # 从配置获取实际的API方法名
            api_method_name = self.robot_commands.get(command_key)
            
            if api_method_name is None:
                self.get_logger().error(f"Command key '{command_key}' not found in configuration")
                return None
            
            # 使用getattr动态获取机器人对象的方法
            api_method = getattr(self.mc, api_method_name, None)
            
            if api_method is None:
                self.get_logger().error(f"API method '{api_method_name}' not found in robot object")
                return None
            
            # 调用API方法
            if callable(api_method):
                result = api_method(*args)
                self.get_logger().debug(f"Called {api_method_name}{args} -> {result}")
                return result
            else:
                self.get_logger().error(f"'{api_method_name}' is not callable")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error calling robot API '{command_key}': {e}")
            return None
    
    def init_robot_connection(self):
        """初始化机器人连接"""
        # 检查是否启用测试模式
        test_mode = self.config.get('test_mode', True)
        
        if test_mode:
            self.get_logger().info("Running in TEST MODE - No actual robot connection")
            self.mc = None
            self.base_position = self.config['robot_control']['motion']['base_position']
            self.get_logger().info(f"Test mode initialized with base position: {self.base_position}")
            return
        
        try:
            # 获取连接参数
            ip = self.config['robot_control']['connection']['ip']
            port = self.config['robot_control']['connection']['port']
            
            # 创建机器人连接
            self.mc = ElephantRobot(ip, port)
            
            # 启动客户端连接
            res = self.mc.start_client()
            if not res:
                self.get_logger().error(f"Failed to start robot client: {res}")
                sys.exit(1)
            
            # 电源开启和启动机器人（使用通用API）
            self.call_robot_api('power_on_command')
            self.call_robot_api('start_robot_command')
            
            # 获取机器人状态（使用通用API）
            state = self.call_robot_api('get_state_command')
            self.get_logger().info(f"Robot state: {state}")
            
            # 设置运动速度（使用通用API）
            speed = self.config['robot_control']['motion']['speed']
            self.call_robot_api('speed_command', speed)
            
            # 设置到初始位置（使用通用API）
            initial_angles = self.config['robot_control']['motion']['initial_angles']
            initial_speed = self.config['robot_control']['motion']['initial_speed']
            self.call_robot_api('angle_command', initial_angles, initial_speed)
            self.call_robot_api('wait_command')
            
            # 获取基座位置
            self.base_position = self.config['robot_control']['motion']['base_position']
            self.get_logger().info(f"Robot initialized with base position: {self.base_position}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize robot connection: {e}")
            sys.exit(1)
    
    def init_feedback_publishers(self):
        """初始化数据回传话题发布器"""
        self.feedback_publishers = {}
        
        # 检查是否启用数据回传
        if not self.config['data_feedback']['enable_feedback']:
            self.get_logger().info("Data feedback is disabled")
            return
        
        feedback_config = self.config['data_feedback']['feedback_data_types']
        
        # 初始化关节角度发布器
        if feedback_config['joint_angles']['enabled']:
            topic_name = feedback_config['joint_angles']['topic_name']
            self.feedback_publishers['joint_angles'] = self.create_publisher(
                Float32MultiArray,
                topic_name,
                10
            )
            self.get_logger().info(f"Joint angles publisher created: {topic_name}")
        
        # 初始化笛卡尔位姿发布器
        if feedback_config['cartesian_pose']['enabled']:
            topic_name = feedback_config['cartesian_pose']['topic_name']
            self.feedback_publishers['cartesian_pose'] = self.create_publisher(
                PoseStamped,
                topic_name,
                10
            )
            self.get_logger().info(f"Cartesian pose publisher created: {topic_name}")
    
    def start_feedback_timer(self):
        """启动数据回传定时器"""
        if not self.config['data_feedback']['enable_feedback']:
            return
        
        feedback_rate = self.config['data_feedback']['feedback_rate']
        timer_period = 1.0 / feedback_rate  # 转换为秒
        
        self.feedback_timer = self.create_timer(
            timer_period,
            self.feedback_timer_callback
        )
        
        self.get_logger().info(f"Data feedback timer started with rate: {feedback_rate} Hz")
    
    def feedback_timer_callback(self):
        """数据回传定时器回调函数"""
        try:
            # 获取机器人数据
            feedback_data = self.data_feedback.get_robot_data()
            
            # 发布关节角度数据
            if 'joint_angles' in self.feedback_publishers and 'joint_angles' in feedback_data:
                self.publish_joint_angles(feedback_data['joint_angles'])
            
            # 发布笛卡尔位姿数据
            if 'cartesian_pose' in self.feedback_publishers and 'cartesian_pose' in feedback_data:
                self.publish_cartesian_pose(feedback_data['cartesian_pose'])
                
        except Exception as e:
            self.get_logger().error(f"Error in feedback timer callback: {e}")
    
    def publish_joint_angles(self, joint_angles):
        """发布关节角度数据"""
        try:
            msg = Float32MultiArray()
            msg.data = joint_angles
            self.feedback_publishers['joint_angles'].publish(msg)
            
            # 记录调试信息
            if self.config['data_processing']['debug_mode']:
                self.get_logger().info(f"Published joint angles: {joint_angles}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing joint angles: {e}")
    
    def publish_cartesian_pose(self, cartesian_pose):
        """发布笛卡尔位姿数据"""
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "robot_base"
            
            # 设置位置 (转换为米)
            msg.pose.position.x = cartesian_pose[0] / 1000.0  # mm to m
            msg.pose.position.y = cartesian_pose[1] / 1000.0  # mm to m
            msg.pose.position.z = cartesian_pose[2] / 1000.0  # mm to m
            
            # 设置旋转 (欧拉角转四元数)
            rx, ry, rz = cartesian_pose[3], cartesian_pose[4], cartesian_pose[5]
            quat = self.euler_to_quaternion(rx, ry, rz)
            msg.pose.orientation.x = quat[0]
            msg.pose.orientation.y = quat[1]
            msg.pose.orientation.z = quat[2]
            msg.pose.orientation.w = quat[3]
            
            self.feedback_publishers['cartesian_pose'].publish(msg)
            
            # 记录调试信息
            if self.config['data_processing']['debug_mode']:
                self.get_logger().info(f"Published cartesian pose: {cartesian_pose}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing cartesian pose: {e}")
    
    def euler_to_quaternion(self, rx, ry, rz):
        """欧拉角转四元数"""
        # 转换为弧度
        rx_rad = math.radians(rx)
        ry_rad = math.radians(ry)
        rz_rad = math.radians(rz)
        
        # 计算四元数
        cy = math.cos(rz_rad * 0.5)
        sy = math.sin(rz_rad * 0.5)
        cp = math.cos(ry_rad * 0.5)
        sp = math.sin(ry_rad * 0.5)
        cr = math.cos(rx_rad * 0.5)
        sr = math.sin(rx_rad * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return [x, y, z, w]
    
    def pose_callback(self, msg):
        """处理接收到的姿态消息 - 集成两阶段控制逻辑"""
        try:
            # 1. 数据提取模块：提取指定字段数据
            extracted_data = self.data_extractor.extract_pose_data(msg)
            
            # 2. 数据转换模块：进行坐标转换
            transformed_data = self.data_transformer.transform_pose_data(extracted_data)
            
            # 3. 机器人控制：发送运动指令（包含状态管理）
            # 优先使用ROS2话题接收的按钮数据
            button_data = getattr(self, 'current_button_data', None)
            if button_data is None:
                # 如果没有ROS2按钮数据，初始化为默认值
                button_data = {
                    'h_button': 0,
                    'a_button': 0,
                    'b_button': 0,
                    'second_button': 0
                }
                self.get_logger().warn("No button data from ROS2 topic, using default values")
            
            updated_initial_position, updated_initial_rotation = self.robot_controller.control_robot_motion(
                transformed_data, 
                self.mc, 
                self.base_position,
                self.initial_hand_position,
                self.initial_hand_rotation,
                button_data
            )
            
            # 更新初始位置（由控制器管理）
            self.initial_hand_position = updated_initial_position
            self.initial_hand_rotation = updated_initial_rotation
            
            # 记录调试信息
            self.log_counter += 1
            log_frequency = self.config['data_processing']['log_frequency']
            
            if self.log_counter % log_frequency == 0:
                self.get_logger().info(f"Processed pose data #{self.log_counter}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing pose data: {e}")
    
    def button_callback(self, msg):
        """处理接收到的按钮消息"""
        try:
            # 提取按钮数据
            button_data = list(msg.data)
            
            # 验证按钮数据
            if len(button_data) != 6:
                self.get_logger().warn(f"Invalid button data length: {len(button_data)}, expected 6")
                return
            
            # 更新按钮状态
            self.update_button_state(button_data)
            
            # 记录调试信息
            if self.config['data_processing']['debug_mode']:
                self.get_logger().info(f"Received button data: {button_data}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing button data: {e}")
    
    def update_button_state(self, button_data):
        """更新按钮状态"""
        try:
            # 将按钮数据存储到类变量中，供机器人控制器使用
            self.current_button_data = {
                'h_button': button_data[0],      # 第1个按钮 (索引0) - H键夹爪控制
                'a_button': button_data[4],      # 第5个按钮 (索引4)
                'b_button': button_data[5],      # 第6个按钮 (索引5)
                'second_button': button_data[1]  # 第2个按钮 (索引1) - 运动控制按钮
            }
            
            # 记录按钮状态变化
            if hasattr(self, 'previous_button_data'):
                for button_name, current_value in self.current_button_data.items():
                    previous_value = self.previous_button_data.get(button_name, 0)
                    if current_value != previous_value:
                        self.get_logger().info(f"Button {button_name} changed: {previous_value} -> {current_value}")
            
            # 保存当前状态作为下次比较的基准
            self.previous_button_data = self.current_button_data.copy()
            
        except Exception as e:
            self.get_logger().error(f"Error updating button state: {e}")


class DataExtractor:
    """数据提取模块"""
    
    def __init__(self, useful_fields_config):
        self.useful_fields = useful_fields_config
        self.get_logger().info("Data extractor initialized")
    
    def extract_pose_data(self, pose_msg):
        """从PoseStamped消息中提取指定字段数据"""
        extracted_data = {}
        
        # 提取位置数据
        if self.useful_fields['right_arm']['position']['x']:
            extracted_data['position'] = {
                'x': pose_msg.pose.position.x,
                'y': pose_msg.pose.position.y,
                'z': pose_msg.pose.position.z
            }
        
        # 提取旋转数据
        if self.useful_fields['right_arm']['rotation']['x']:
            extracted_data['rotation'] = {
                'x': pose_msg.pose.orientation.x,
                'y': pose_msg.pose.orientation.y,
                'z': pose_msg.pose.orientation.z,
                'w': pose_msg.pose.orientation.w
            }
        
        # 按钮数据现在通过ROS2话题单独处理，不再从JSON文件读取
        # button数据通过button_callback方法处理，存储在self.current_button_data中
        
        return extracted_data
    
    def get_logger(self):
        """获取日志记录器"""
        return rclpy.logging.get_logger('data_extractor')


class DataTransformer:
    """数据转换模块"""
    
    def __init__(self, transformation_config):
        self.transformation_matrix = np.array(transformation_config['transformation_matrix'])
        self.transform_position = transformation_config['transform_position']
        self.transform_rotation = transformation_config['transform_rotation']
        self.get_logger().info("Data transformer initialized")
    
    def transform_pose_data(self, pose_data):
        """转换姿态数据坐标系统"""
        transformed_data = pose_data.copy()
        
        # 转换位置数据
        if self.transform_position and 'position' in pose_data:
            position = np.array([
                pose_data['position']['x'],
                pose_data['position']['y'],
                pose_data['position']['z']
            ])
            transformed_position = self.transformation_matrix @ position
            transformed_data['position'] = {
                'x': transformed_position[0],
                'y': transformed_position[1],
                'z': transformed_position[2]
            }
        
        # 转换旋转数据
        if self.transform_rotation and 'rotation' in pose_data:
            # 四元数坐标转换
            quaternion = np.array([
                pose_data['rotation']['x'],
                pose_data['rotation']['y'],
                pose_data['rotation']['z'],
                pose_data['rotation']['w']
            ])
            
            # 应用坐标转换矩阵到四元数的虚部
            quat_xyz = quaternion[:3]
            transformed_xyz = self.transformation_matrix @ quat_xyz
            quaternion[0] = transformed_xyz[0]
            quaternion[1] = transformed_xyz[1]
            quaternion[2] = transformed_xyz[2]
            
            transformed_data['rotation'] = {
                'x': quaternion[0],
                'y': quaternion[1],
                'z': quaternion[2],
                'w': quaternion[3]
            }
        
        return transformed_data
    
    def get_logger(self):
        """获取日志记录器"""
        return rclpy.logging.get_logger('data_transformer')


class RobotController:
    """机器人控制器"""
    
    def __init__(self, control_config, processing_config, robot_commands):
        self.control_config = control_config
        self.processing_config = processing_config
        self.robot_commands = robot_commands  # 通用命令映射
        self.position_scale = control_config['motion']['position_scale']
        
        # 控制状态管理
        self.control_state = processing_config['control_states']['waiting_for_calibration']
        self.previous_a_button = 0
        self.previous_b_button = 0
        self.previous_h_button = 0
        self.previous_second_button = 0
        
        # 命令限流控制
        self.robot_control_rate = processing_config.get('robot_control_rate', 15.0)  # Hz
        self.min_command_interval = 1.0 / self.robot_control_rate  # 秒
        self.last_command_time = 0.0
        
        # 位置变化阈值（米）
        self.position_threshold = processing_config.get('position_change_threshold', 0.001)
        self.last_sent_position = None
        
        # 异步命令队列 - 解决write_coords阻塞问题
        self.command_queue = queue.Queue(maxsize=2)  # 最多保留2个命令，避免堆积
        self.worker_thread = None
        self.worker_running = False
        
        self.get_logger().info("Robot controller initialized")
        self.get_logger().info(f"Initial control state: {self.control_state}")
        self.get_logger().info(f"Robot control rate limit: {self.robot_control_rate} Hz")
        self.get_logger().info(f"Position change threshold: {self.position_threshold} m")
        self.get_logger().info("Async command queue enabled - write_coords will not block main thread")
    
    def call_robot_api(self, mc, command_key, *args):
        """通用机器人API调用方法（RobotController版本）
        
        Args:
            mc: 机器人连接对象
            command_key: 命令键名
            *args: API调用参数
        
        Returns:
            API调用返回值
        """
        if mc is None:
            return None
        
        try:
            api_method_name = self.robot_commands.get(command_key)
            if api_method_name is None:
                self.get_logger().error(f"Command key '{command_key}' not found")
                return None
            
            api_method = getattr(mc, api_method_name, None)
            if api_method is None:
                self.get_logger().error(f"API method '{api_method_name}' not found")
                return None
            
            if callable(api_method):
                return api_method(*args)
            else:
                self.get_logger().error(f"'{api_method_name}' is not callable")
                return None
        except Exception as e:
            self.get_logger().error(f"Error calling robot API '{command_key}': {e}")
            return None
    
    def start_command_worker(self, mc):
        """启动命令工作线程"""
        if self.worker_thread is None or not self.worker_thread.is_alive():
            self.worker_running = True
            self.worker_thread = threading.Thread(target=self._command_worker, args=(mc,), daemon=True)
            self.worker_thread.start()
            self.get_logger().info("Command worker thread started")
    
    def stop_command_worker(self):
        """停止命令工作线程"""
        self.worker_running = False
        if self.worker_thread is not None:
            # 放入None信号停止线程
            try:
                self.command_queue.put(None, block=False)
            except queue.Full:
                pass
            self.worker_thread.join(timeout=1.0)
            self.get_logger().info("Command worker thread stopped")
    
    def _command_worker(self, mc):
        """后台工作线程：从队列取命令并执行"""
        self.get_logger().info("Command worker thread running")
        
        while self.worker_running:
            try:
                # 从队列获取命令（阻塞等待）
                command = self.command_queue.get(timeout=0.1)
                
                if command is None:
                    # 收到停止信号
                    break
                
                cmd_type, args = command
                
                if cmd_type == 'coords':
                    # 执行坐标命令（使用通用API）
                    target_position, speed = args
                    if mc is not None:
                        try:
                            self.call_robot_api(mc, 'position_command', target_position, speed)
                        except Exception as e:
                            self.get_logger().error(f"Error executing coords command: {e}")
                
                elif cmd_type == 'gripper':
                    # 执行夹爪命令（使用通用API）
                    state, gripper_speed, gripper_mode = args
                    if mc is not None:
                        try:
                            self.call_robot_api(mc, 'gripper_mode_command', gripper_mode)
                            self.call_robot_api(mc, 'gripper_state_command', state, gripper_speed)
                        except Exception as e:
                            self.get_logger().error(f"Error executing gripper command: {e}")
                
                self.command_queue.task_done()
                
            except queue.Empty:
                # 队列空，继续等待
                continue
            except Exception as e:
                self.get_logger().error(f"Error in command worker: {e}")
        
        self.get_logger().info("Command worker thread exiting")
    
    def _send_command_async(self, cmd_type, args):
        """异步发送命令到队列"""
        try:
            # 如果队列满了，清空旧命令（保持最新）
            if self.command_queue.full():
                try:
                    self.command_queue.get_nowait()
                    self.command_queue.task_done()
                except queue.Empty:
                    pass
            
            # 放入新命令
            self.command_queue.put((cmd_type, args), block=False)
            
        except queue.Full:
            self.get_logger().warn("Command queue full, command dropped")
    
    def control_robot_motion(self, transformed_data, mc, base_position, initial_hand_position, initial_hand_rotation, button_data=None):
        """控制机器人运动 - 两阶段控制逻辑"""
        try:
            # 获取当前按钮状态
            if button_data is not None:
                # 使用传入的按钮数据
                current_a_button = button_data.get('a_button', 0)
                current_b_button = button_data.get('b_button', 0)
                current_h_button = button_data.get('h_button', 0)
                current_second_button = button_data.get('second_button', 0)
            else:
                # 使用从transformed_data中提取的按钮数据
                current_a_button = transformed_data.get('button', {}).get('a_button', 0)
                current_b_button = transformed_data.get('button', {}).get('b_button', 0)
                current_h_button = transformed_data.get('button', {}).get('h_button', 0)
                current_second_button = transformed_data.get('button', {}).get('second_button', 0)
            
            # 添加调试信息
            if current_a_button != self.previous_a_button or current_b_button != self.previous_b_button or current_h_button != self.previous_h_button or current_second_button != self.previous_second_button:
                self.get_logger().info(f"Button states - H: {current_h_button}, A: {current_a_button}, B: {current_b_button}, Second: {current_second_button}, State: {self.control_state}")
            
            # 检测按钮状态变化
            a_button_pressed = (current_a_button == 1 and self.previous_a_button == 0)
            a_button_released = (current_a_button == 0 and self.previous_a_button == 1)
            b_button_pressed = (current_b_button == 1 and self.previous_b_button == 0)
            b_button_released = (current_b_button == 0 and self.previous_b_button == 1)
            h_button_pressed = (current_h_button == 1 and self.previous_h_button == 0)
            h_button_released = (current_h_button == 0 and self.previous_h_button == 1)
            
            # H键夹爪控制 - 基于持续状态控制，无论机器人处于什么状态都可以控制夹爪
            if current_h_button == 1:
                # H键持续按下 - 夹爪闭合
                should_log = not hasattr(self, '_last_gripper_state') or self._last_gripper_state != 1
                self.control_gripper(mc, 1, log_state_change=should_log)  # 1表示闭合
                if should_log:
                    self.get_logger().info("H button held - Gripper closing")
                    self._last_gripper_state = 1
            else:
                # H键未按下 - 夹爪张开
                should_log = not hasattr(self, '_last_gripper_state') or self._last_gripper_state != 0
                self.control_gripper(mc, 0, log_state_change=should_log)  # 0表示张开
                if should_log:
                    self.get_logger().info("H button released - Gripper opening")
                    self._last_gripper_state = 0
            
            # 全局B键逻辑：无论当前处于哪个状态，释放B键都立即回到基座并重新标定初始手柄姿态
            if b_button_released:
                # 标定当前手柄姿态为新的初始姿态
                initial_hand_position = np.array([
                    transformed_data['position']['x'],
                    transformed_data['position']['y'],
                    transformed_data['position']['z']
                ])
                initial_hand_rotation = np.array([
                    transformed_data['rotation']['x'],
                    transformed_data['rotation']['y'],
                    transformed_data['rotation']['z'],
                    transformed_data['rotation']['w']
                ])

                # 异步发送机械臂回到基座位置的命令
                if mc is not None:
                    speed = self.control_config['motion']['speed']
                    self._send_command_async('coords', (base_position, speed))
                    self.get_logger().info(f"B button released - Robot returning to base position: {base_position}")
                else:
                    # 测试模式：只记录目标位置
                    self.get_logger().info(f"TEST MODE - B button released: target base position {base_position}")

                # 重置限流状态
                self.last_sent_position = None
                import time
                self.last_command_time = time.time()
                
                # 切换为等待A键开始控制的状态
                self.control_state = self.processing_config['control_states']['waiting_for_control']
                self.get_logger().info("Calibration done. Waiting for A button to start control from base.")

                # 更新按钮状态并返回更新后的初始姿态
                self.previous_a_button = current_a_button
                self.previous_b_button = current_b_button
                return initial_hand_position, initial_hand_rotation

            # 状态机逻辑
            if self.control_state == self.processing_config['control_states']['waiting_for_calibration']:
                # 等待B键标定
                if b_button_released:  # B键按下并松开
                    # 标定初始位置
                    initial_hand_position = np.array([
                        transformed_data['position']['x'],
                        transformed_data['position']['y'],
                        transformed_data['position']['z']
                    ])
                    initial_hand_rotation = np.array([
                        transformed_data['rotation']['x'],
                        transformed_data['rotation']['y'],
                        transformed_data['rotation']['z'],
                        transformed_data['rotation']['w']
                    ])
                    
                    # 重置限流状态
                    self.last_sent_position = None
                    import time
                    self.last_command_time = time.time()
                    
                    self.control_state = self.processing_config['control_states']['waiting_for_control']
                    self.get_logger().info("B button released - Initial position calibrated")
                    self.get_logger().info(f"Calibrated position: {initial_hand_position}")
                    self.get_logger().info("Waiting for A button to start control...")
                    
            elif self.control_state == self.processing_config['control_states']['waiting_for_control']:
                # 等待A键开始控制
                if a_button_released:  # A键按下并松开
                    self.control_state = self.processing_config['control_states']['ready_to_control']
                    self.get_logger().info("A button released - Ready to control, waiting for second button to be held")
                    
            elif self.control_state == self.processing_config['control_states']['ready_to_control']:
                # 准运动状态 - 等待第二个按钮持续按下
                if current_second_button == 1:
                    # 第二个按钮被按下，进入控制状态
                    self.control_state = self.processing_config['control_states']['controlling']
                    self.get_logger().info("Second button held - Starting robot control")
                # 在准运动状态下，如果第二个按钮没有被按下，不执行任何运动
                
            elif self.control_state == self.processing_config['control_states']['controlling']:
                # 正在控制状态 - 检查第二个按钮状态
                if current_second_button == 0:
                    # 第二个按钮松开，回到准运动状态
                    self.control_state = self.processing_config['control_states']['ready_to_control']
                    self.get_logger().info("Second button released - Stopping robot control, back to ready state")
                elif initial_hand_position is not None:
                    # 第二个按钮持续按下，执行相对位移控制
                    current_position = np.array([
                        transformed_data['position']['x'],
                        transformed_data['position']['y'],
                        transformed_data['position']['z']
                    ])
                    
                    # 计算相对位移
                    displacement = current_position - initial_hand_position
                    
                    # 检查位置变化是否超过阈值（减少微小抖动）
                    if self.last_sent_position is not None:
                        position_change = np.linalg.norm(current_position - self.last_sent_position)
                        if position_change < self.position_threshold:
                            # 位置变化太小，跳过本次命令
                            return initial_hand_position, initial_hand_rotation
                    
                    # 命令限流：检查时间间隔
                    import time
                    current_time = time.time()
                    time_since_last_command = current_time - self.last_command_time
                    
                    if time_since_last_command < self.min_command_interval:
                        # 距离上次命令时间太短，跳过本次命令
                        return initial_hand_position, initial_hand_rotation
                    
                    # 处理旋转数据 - 始终使用基座位置的固定旋转角度
                    rx_deg = base_position[3]  # -178.9
                    ry_deg = base_position[4]  # 0.7
                    rz_deg = base_position[5]  # 69.1
                    
                    # 转换为毫米并加到基座位置
                    target_position = [
                        base_position[0] + displacement[0] * self.position_scale,
                        base_position[1] + displacement[1] * self.position_scale,
                        base_position[2] + displacement[2] * self.position_scale,
                        rx_deg,  # rx
                        ry_deg,  # ry
                        rz_deg   # rz
                    ]
                    
                    # 异步发送运动指令（不阻塞）
                    if mc is not None:
                        speed = self.control_config['motion']['speed']
                        # 使用异步队列发送命令
                        self._send_command_async('coords', (target_position, speed))
                        
                        # 更新限流状态
                        self.last_command_time = current_time
                        self.last_sent_position = current_position.copy()
                        
                        self.get_logger().debug(f"Robot command queued: {target_position}")
                    else:
                        # 测试模式：只记录目标位置
                        self.last_command_time = current_time
                        self.last_sent_position = current_position.copy()
                        self.get_logger().debug(f"TEST MODE - Target position: {target_position}")
            
            # 更新按钮状态
            self.previous_a_button = current_a_button
            self.previous_b_button = current_b_button
            self.previous_h_button = current_h_button
            self.previous_second_button = current_second_button
            
            # 返回更新后的初始位置（用于状态传递）
            return initial_hand_position, initial_hand_rotation
            
        except Exception as e:
            self.get_logger().error(f"Error controlling robot motion: {e}")
            return initial_hand_position, initial_hand_rotation
    
    def quaternion_to_euler(self, x, y, z, w):
        """四元数转欧拉角"""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def control_gripper(self, mc, state, log_state_change=True):
        """控制夹爪开合（异步）
        
        Args:
            mc: 机器人连接对象
            state: 夹爪状态 (0=张开, 1=闭合)
            log_state_change: 是否记录状态变化日志
        """
        try:
            if mc is None:
                # 测试模式
                if log_state_change:
                    gripper_state = "张开" if state == 0 else "闭合"
                    self.get_logger().info(f"TEST MODE - Gripper {gripper_state}")
                return
            
            # 获取夹爪配置参数
            gripper_speed = self.control_config['gripper']['speed']
            gripper_mode = self.control_config['gripper']['mode']
            
            # 异步发送夹爪命令
            self._send_command_async('gripper', (state, gripper_speed, gripper_mode))
            
            if log_state_change:
                gripper_action = "closing" if state == 1 else "opening"
                self.get_logger().info(f"Gripper command queued: {gripper_action}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to queue gripper command: {e}")
    
    def get_logger(self):
        """获取日志记录器"""
        return rclpy.logging.get_logger('robot_controller')


class DataFeedbackModule:
    """数据回传模块"""
    
    def __init__(self, feedback_config, robot_connection, robot_commands):
        self.feedback_config = feedback_config
        self.robot_connection = robot_connection
        self.robot_commands = robot_commands  # 通用命令映射
        self.get_logger().info("Data feedback module initialized")
    
    def call_robot_api(self, command_key, *args):
        """通用机器人API调用方法（DataFeedbackModule版本）
        
        Args:
            command_key: 命令键名
            *args: API调用参数
        
        Returns:
            API调用返回值
        """
        if self.robot_connection is None:
            return None
        
        try:
            api_method_name = self.robot_commands.get(command_key)
            if api_method_name is None:
                self.get_logger().error(f"Command key '{command_key}' not found")
                return None
            
            api_method = getattr(self.robot_connection, api_method_name, None)
            if api_method is None:
                self.get_logger().error(f"API method '{api_method_name}' not found")
                return None
            
            if callable(api_method):
                return api_method(*args)
            else:
                self.get_logger().error(f"'{api_method_name}' is not callable")
                return None
        except Exception as e:
            self.get_logger().error(f"Error calling robot API '{command_key}': {e}")
            return None
    
    def get_robot_data(self):
        """获取机器人数据"""
        feedback_data = {}
        
        try:
            # 获取关节角度数据
            if self.feedback_config['feedback_data_types']['joint_angles']['enabled']:
                joint_angles = self.get_joint_angles()
                if joint_angles is not None:
                    feedback_data['joint_angles'] = joint_angles
            
            # 获取笛卡尔位姿数据
            if self.feedback_config['feedback_data_types']['cartesian_pose']['enabled']:
                cartesian_pose = self.get_cartesian_pose()
                if cartesian_pose is not None:
                    feedback_data['cartesian_pose'] = cartesian_pose
                    
        except Exception as e:
            self.get_logger().error(f"Error getting robot data: {e}")
        
        return feedback_data
    
    def get_joint_angles(self):
        """获取关节角度数据"""
        try:
            if self.robot_connection is None:
                # 测试模式：返回模拟数据
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 调用机器人API获取关节角度（使用通用API）
            result = self.call_robot_api('get_angles_command')
            
            if result and len(result) == 6:
                # 检查是否为错误数据格式
                error_data = self.feedback_config['error_handling']['error_joint_angles']
                if result == error_data:
                    self.get_logger().warn("Received error data from get_angles()")
                    if self.feedback_config['error_handling']['publish_error_data']:
                        return result
                    else:
                        return None
                else:
                    return result
            else:
                self.get_logger().warn(f"Invalid joint angles data: {result}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting joint angles: {e}")
            return None
    
    def get_cartesian_pose(self):
        """获取笛卡尔位姿数据"""
        try:
            if self.robot_connection is None:
                # 测试模式：返回模拟数据
                return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            
            # 调用机器人API获取笛卡尔位姿（使用通用API）
            result = self.call_robot_api('get_coords_command')
            
            if result and len(result) == 6:
                # 检查是否为错误数据格式
                error_data = self.feedback_config['error_handling']['error_cartesian_pose']
                if result == error_data:
                    self.get_logger().warn("Received error data from get_coords()")
                    if self.feedback_config['error_handling']['publish_error_data']:
                        return result
                    else:
                        return None
                else:
                    return result
            else:
                self.get_logger().warn(f"Invalid cartesian pose data: {result}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error getting cartesian pose: {e}")
            return None
    
    def get_logger(self):
        """获取日志记录器"""
        return rclpy.logging.get_logger('data_feedback')


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        robot_interface = RobotInterfaceNode()
        rclpy.spin(robot_interface)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'robot_interface' in locals():
            # 停止命令工作线程
            print("Stopping command worker thread...")
            robot_interface.robot_controller.stop_command_worker()
            robot_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
