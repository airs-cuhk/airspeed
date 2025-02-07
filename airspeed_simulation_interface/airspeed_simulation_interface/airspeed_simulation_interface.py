from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from .utility_ik import UtilityIk
from time import sleep
import numpy as np


class JointCommandSubscriber(Node):
    def __init__(self):
        super().__init__('joints_command_subscriber')

        # JointState 发布者
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_command', 10)

        # 初始位置
        self.initial_position = [0.0, -0.4, 0.7, -1.7, -1.57, 1.9, 0.0]
        self.gripper_position = 0.0  # 初始夹爪状态
        self.target_joint = self.initial_position

        self.config_path = "/home/airspeed/airspeed/src/airspeed/airspeed_simulation_interface/config/airspeed_grasp_config.yaml"
        self.ik = UtilityIk(self.config_path)

        self.create_subscription(String, 'joints_pub_random', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received data: {msg.data}')
       
        # 提取目标点和夹爪状态
        data = eval(msg.data)
        # if isinstance(data, list) and len(data) > 3:
        target_coordinates = data[0]
        target_coordinates = [(coord / 1000.0) for coord in target_coordinates]
#             self.get_logger().info(f"target_coordinates: {target_coordinates}")
        target_x, target_y, target_z = target_coordinates[:3] 
        if data[-1]:
            finger_distance = data[-1] / 1000 # 夹爪状态
        else:
            finger_distance = 0.05
        target_size = 0.045
            # 计算逆运动学
        joint_angles = self.ik.inverse(target_x, target_y, target_z)
        if joint_angles:
            self.target_joint = joint_angles
            if finger_distance:
                if finger_distance < target_size:
            
                    self.gripper_position = -0.3  # 限制夹爪范围
                    self.get_logger().info("finger distance < thresold: closing gripper!!")
                else:
                    self.gripper_position = 0.0
                    self.get_logger().info("Keeping gripper open..")
                    # 发布关节状态
            else:
                self.gripper_position = 0.0
            self.publish_joint_state()
        else:
            self.get_logger().warning("Target out of reach!")
            

    def publish_joint_state(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [
            'joint_link1_to_base',
            'joint_link2_to_link1',
            'joint_link3_to_link2',
            'joint_link4_to_link3',
            'joint_link5_to_link4',
            'joint_link6_to_link5',
            'gripper_controller'
        ]
        joint_state_msg.position = self.target_joint + [self.gripper_position]
        joint_state_msg.velocity = [0.0] * len(joint_state_msg.name)
        joint_state_msg.effort = [0.0] * len(joint_state_msg.name)

        self.joint_state_publisher.publish(joint_state_msg)
        self.get_logger().info(f"Published joint state: {joint_state_msg.position}")


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = JointCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
