import socket
import re
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
import math
import time
import yaml

class JointPublisher(Node):
    """
    Teleoperation Interface node.
    """
    def __init__(self):
        super().__init__('joint_publisher')
        # Built-in Parameters of BVH Files
        self.joints = [
            'Hips', 'Spine', 'Spine1', 'Spine2', 'RightShoulder',
            'RightArm', 'RightForeArm', 'RightHand', 'RightHandThumb1',
            'RightHandThumb2', 'RightHandThumb3', 'RightInHandIndex',
            'RightHandIndex1', 'RightHandIndex2', 'RightHandIndex3'
        ]
        self.joint_translations = {
            'Spine': np.array([0.0, 8.12, 0.0]),
            'Spine1': np.array([0.0, 17.98, 0.0]),
            'Spine2': np.array([0.0, 12.76, 0.0]),
            'RightShoulder': np.array([-2.9, 13.34, 0.0]),
            'RightArm': np.array([-16.1, 0.0, 0.0]),
            'RightForeArm': np.array([-28.0, 0.0, 0.0]),
            'RightHand': np.array([-26.0, 0.0, 0.0]),
            'RightHandThumb1': np.array([-2.702, 0.206, 3.388]),
            'RightHandThumb2': np.array([-3.998, 0.0, 0.0]),
            'RightHandThumb3': np.array([-2.778, 0.0, 0.0]),
            'RightInHandIndex': np.array([-3.5, 0.552, 2.148]),
            'RightHandIndex1': np.array([-5.664001, -0.099, 1.085]),
            'RightHandIndex2': np.array([-3.93, 0.0, 0.0]),
            'RightHandIndex3': np.array([-2.228, 0.0, 0.0])
        }
        self.shared_data = {
            'position_queue': deque(maxlen=2),  
            'distance_queue': deque(maxlen=2)   
        }

        # Load configuration
        self.config_path = self.declare_and_get_parameter('config', 'config/config.yaml').value
        self.config = self.load_config(self.config_path)
        self.get_logger().info(f"Loaded Config: {self.config}")
        self.teleoperation_ip = self.config.get('teleoperation_ip')
        self.teleoperation_port = self.config.get('teleoperation_port')
        self.teleoperation_fre = self.config.get('teleoperation_fre')
        self.tcp_ip = self.config.get('tcp_ip')
        self.tcp_port = self.config.get('tcp_port')

        # Waiting for data conversion
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.start()
        time.sleep(0.1)
        # Starting data publishing
        self.publisher = self.create_publisher(String, 'joint_data', 1)
        self.receive_thread2 = threading.Thread(target=self.tcp_server)
        self.receive_thread2.start()
        self.timer = self.create_timer(self.teleoperation_fre, self.timer_callback) 
    
    def declare_and_get_parameter(self, name, default_value):
        """
        Declare and get a parameter with a default value.
        """
        self.declare_parameter(name, default_value)
        return self.get_parameter(name)
    
    def load_config(self, config_path):
        """
        Load configuration from a yaml file.
        """
        try:
            with open(config_path, 'r') as file:
                if config_path.endswith('.yaml') or config_path.endswith('.yml'):
                    return yaml.safe_load(file)  # 解析 YAML
                else:
                    return json.load(file)  # 解析 JSON
        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {e}")
            return {}

    def tcp_server(self):
        """
        Directly establish a TCP connection with the robotic arm.
        """
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((self.tcp_ip, self.tcp_port))
        server_socket.listen(5)
        self.get_logger().info(f"TCP 服务器正在监听端口 {self.tcp_port}...")
        while True:
            client_socket, addr = server_socket.accept()
            try:
                while True:
                    if self.shared_data['position_queue'] and self.shared_data['distance_queue']:
                        position = self.shared_data['position_queue'][0]
                        distance = self.shared_data['distance_queue'][0]
                        data = f"[{position}, {distance}]\n"
                        client_socket.sendall(data.encode()) 
                        # self.get_logger().info(f"发送数据: {data}")
            except (ConnectionResetError, BrokenPipeError):
                self.get_logger().info(f"客户端 {addr} 已断开连接")
            finally:
                client_socket.close()
                self.get_logger().info(f"关闭与客户端 {addr} 的连接")
    
    def timer_callback(self):
        """
        Publish data at regular intervals
        """
        msg = String()
        msg_data = self.shared_data['position_queue'][0]
        msg_ditance_data =  self.shared_data['distance_queue'][0]
        if msg_data :
            msg.data = f"[[{','.join(map(str, msg_data))}], {str(msg_ditance_data)}]"
            self.publisher.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
        else:
            self.get_logger().info('No data to publish')
    @staticmethod
    def rotation_matrix(ry, rx, rz):
        """
        Construction of a rotation matrix
        """
        ry = np.radians(ry)
        rx = np.radians(rx)
        rz = np.radians(rz)
        Ry = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        Rz = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        return Ry @ Rx @ Rz

    @staticmethod
    def calculate_distance(coord1, coord2):
        """
        Calculate the distance between fingers
        """
        return math.sqrt(sum((abs(c1 - c2)) ** 2 for c1, c2 in zip(coord1[:], coord2[:])))

    @staticmethod
    def create_transformation_matrix(translation, rotation):
        """
        Translation matrix
        """
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation
        return T
    
    def calculate_position(self, frames, frame_count):
        """
        Relative position calculation
        """
        pattern = r"([A-Za-z0-9]+):\(([-+]?[0-9]*\.?[0-9]+(?:,[ ]?[-+]?[0-9]*\.?[0-9]+){2})\)\(([-+]?[0-9]*\.?[0-9]+(?:,[ ]?[-+]?[0-9]*\.?[0-9]+){2})\)"
        for i, frame in enumerate(frames):
                    frame_count += 1
                    joints_data = re.findall(pattern, frame)
                    joint_rotations = {}
                    position_dict = {}
                    trtr = {}
                    for joint in joints_data:
                        name = joint[0]
                        translation = list(map(float, joint[1].split(',')))
                        rotation = list(map(float, joint[2].split(',')))
                        if name in self.joints:
                            joint_rotations[name] = rotation
                            rotation_matrix_values = joint_rotations[name]
                            rot_matrix = self.rotation_matrix(rotation_matrix_values[0], rotation_matrix_values[1], rotation_matrix_values[2])
                            if name == "Hips":
                                position_dict[name] = translation
                                trtr[name] = self.create_transformation_matrix(position_dict[name], rot_matrix)
                            else:
                                trtr[name] = self.create_transformation_matrix(self.joint_translations[name], rot_matrix)
                            if name == "RightHand":
                                current_transform = self.create_transformation_matrix(self.joint_translations[name], np.eye(3))
                                world_pos = (
                                    trtr.get("Hips") @ trtr.get("Spine") @ trtr.get("Spine1") @ trtr.get("Spine2")
                                    @ trtr.get("RightShoulder") @ trtr.get("RightArm") @ trtr.get("RightForeArm")
                                    @ current_transform
                                )
                                position_dict[name] = world_pos[:3, 3]
                                Hips_pos = position_dict["Hips"]
                                right_hand_pos = position_dict["RightHand"]
                                temp_x = right_hand_pos[0] - Hips_pos[0]
                                temp_y = right_hand_pos[1] - Hips_pos[1]
                                temp_z = right_hand_pos[2] - Hips_pos[2]
                                new_x = temp_z * 10 - 230
                                new_y = -temp_x * 10 + 350
                                new_z = temp_y * 10 + 50
                                coord = [-new_x, new_y, new_z, 179.99, 0, 90]
                                self.shared_data['position_queue'].append(coord)
                            if name == "RightHandThumb3":
                                current_transform = self.create_transformation_matrix(self.joint_translations[name], np.eye(3))
                                world_pos = (
                                    trtr.get("Hips") @ trtr.get("Spine") @ trtr.get("Spine1") @ trtr.get("Spine2") 
                                    @ trtr.get("RightShoulder") @ trtr.get("RightArm") @ trtr.get("RightForeArm") 
                                    @ trtr.get("RightHand") @ trtr.get("RightHandThumb1") @ trtr.get("RightHandThumb2") 
                                    @ current_transform
                                )
                                position_dict[name]=world_pos[:3, 3]
                                right_hand_thumb3_pos=position_dict["RightHandThumb3"]
                                temp_x = right_hand_thumb3_pos[0] 
                                temp_y = right_hand_thumb3_pos[1] 
                                temp_z = right_hand_thumb3_pos[2] 
                                new_x = temp_z * 10 
                                new_y = -temp_x * 10 
                                new_z = temp_y * 10 
                                coord1=[-new_x, new_y, new_z]
                            if name == "RightHandIndex3":
                                current_transform = self.create_transformation_matrix(self.joint_translations[name], np.eye(3))
                                world_pos = (
                                    trtr.get("Hips") @ trtr.get("Spine") @ trtr.get("Spine1") @ trtr.get("Spine2")
                                    @ trtr.get("RightShoulder") @ trtr.get("RightArm") @ trtr.get("RightForeArm")
                                    @ trtr.get("RightHand") @ trtr.get("RightInHandIndex") @ trtr.get("RightHandIndex1")
                                    @ trtr.get("RightHandIndex2") @ current_transform
                                )
                                position_dict[name]=world_pos[:3, 3]
                                right_hand_index3_pos=position_dict["RightHandIndex3"]
                                temp_x = right_hand_index3_pos[0] 
                                temp_y = right_hand_index3_pos[1] 
                                temp_z = right_hand_index3_pos[2] 
                                new_x = temp_z * 10 
                                new_y = -temp_x * 10 
                                new_z = temp_y * 10 
                                coord2=[-new_x, new_y, new_z]
                                distance= self.calculate_distance(coord1, coord2)
                                self.shared_data['distance_queue'].append(distance)
                    joint_rotations.clear()
                    position_dict.clear()
                    trtr.clear()

    def receive_data(self):
        """
        Receive Noitom data
        """
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (self.teleoperation_ip, self.teleoperation_port)
        try:
            client_socket.connect(server_address)
            self.get_logger().info(f"Connected to server at {server_address}")
            buffer_size = 1024
            remaining_data = ''
            frame_count = 0
            while True:
                position_dicts = []
                data = client_socket.recv(buffer_size).decode('utf-8')
                if not data:
                    self.get_logger().info("Connection closed by server")
                    break
                full_data = remaining_data + data
                frames = []
                start_index = 0
                while True:
                    start_index = full_data.find('Avatar', start_index)
                    if start_index == -1:
                        break
                    end_index = full_data.find('Avatar', start_index + len('Avatar'))
                    if end_index == -1:
                        remaining_data = full_data[start_index:]
                        break
                    else:
                        frame = full_data[start_index:end_index]
                        frames.append(frame)
                        start_index = end_index
                self.calculate_position(frames,frame_count)
        except Exception as e:
            self.get_logger().error(f"Error in receive_data: {e}")
        finally:
            client_socket.close()

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
