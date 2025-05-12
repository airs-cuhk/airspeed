import os
import json
import socket
import copy
import rclpy
import yaml
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 
from std_msgs.msg import String
from airspeed_services.srv import DataCollection
from concurrent.futures import ThreadPoolExecutor


class AirspeedDataCollection(Node):
    """
    Data collection node for real word.
    """
    def __init__(self):
        super().__init__("data_collection_node")
        self.get_logger().info("Data collection service started!")
        
        # Load configuration
        self.data_settings_path = self.declare_and_get_parameter('setting', 'config/data_settings.json').value
        self.config = self.declare_and_get_parameter('config', 'config/config.yaml').value
        self.DCS_ip = self.config.get('DCS_ip')
        self.DCS_port = self.config.get('DCS_port')
        self.Data_save_type = self.config.get('Data_save_type')
        self.sock = self.build_connection(self.DCS_ip, self.DCS_port)
        self.hdf5_path = self.config.get('hdf5_path')

        # Initialize utilities
        self.bridge = CvBridge()
        self.thread_pool = ThreadPoolExecutor(max_workers=5)

        # Create the service
        self._collection_srv = self.create_service(
            DataCollection,
            '/airspeed_data_collection/collection_server',
            self.instruct_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        if self.Data_save_type == '0':
            # Send the dataset configuration to the Data Construction Service
            self.send_settings()
        elif self.Data_save_type == '1':
            self.initialize_hdf5(self.hdf5_path, self.data_settings_path)

    def declare_and_get_parameter(self, name, default_value):
        """
        Declare and get a parameter with a default value.

        Parameters:
            name (str): The name of the parameter to be declared.
            default_value (any): The default value for the parameter if it does not already exist.

        Returns:
            Parameter: The parameter object that was declared and retrieved. This object contains the parameter's name, type, and value.
        """
        self.declare_parameter(name, default_value)
        return self.get_parameter(name)

    def build_connection(self, Data_Construction_ip, port):
        """
        Build a TCP connection with the Data Construction Service.

        Parameters:
            data_construction_ip (str): The IP address of the Data Construction Service.
            port (int): The port number of the Data Construction Service.

        Returns:
            socket.socket: A socket object if the connection is successfully established.
            None: If the connection cannot be established after the maximum number of attempts.
        
        """
        max_attempts = 5
        for i in range(1, max_attempts + 1):
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((Data_Construction_ip, port))
                return sock
            except Exception as e:
                self.get_logger().error(f"Failed to build connection with Data Construction.")
                if i < max_attempts:
                    self.get_logger().info(f"retrying: {i}/{max_attempts}")
                    time.sleep(5)
                else:
                    self.get_logger().error("Data Collection service has been stop. Please check the Data Construction service")
                    return None
         

    def send_settings(self):
        """
        Load dataset configuration from the json file and send it to the Data Construction Service.
        """
        try:
            with open(self.data_settings_path, 'r') as file:
                settings = json.load(file)
            if settings is None:
                self.get_logger().error("The dataset configuration load failed!")
                return
            self.get_logger().info("The dataset configuration loaded successfully!")
            self.send_data(settings, "settings", is_json=True)
        except Exception as e:
            self.get_logger().error(f"Failed to send dataset configuration: {e}")

    def send_data(self, data, identifier, is_json=False):
        """
        Send data (json or image) over a socket.

        Parameters:
            data: json(str) or image data(numpy array)
            identifier: A string that identifies the type of data being sent. Special identifiers include "settings" and "end".
            is_json: A boolean flag indicating whether the data should be treated as JSON. Defaults to False.
        """
        try:
            identifier_length = len(identifier).to_bytes(4, byteorder='big')
            self.sock.sendall(identifier_length + identifier.encode('utf-8'))

            if is_json:
                data_json = json.dumps(data) if not isinstance(data, str) else data
                data_bytes = data_json.encode('utf-8')
            else:
                if identifier == "settings" or identifier == "end":
                    if isinstance(data, str):
                        data_bytes = data.encode('utf-8')
                    elif isinstance(data, bytes):
                        data_bytes = data
                    else:
                        raise TypeError("Flag must be either a string or bytes")
                else:
                    data_bytes = data

            size = len(data_bytes).to_bytes(4, byteorder='big')
            self.sock.sendall(size + data_bytes)

            self.sock.settimeout(10.0)
            ack = self.sock.recv(1024)
            if ack:
                self.get_logger().info(f"{identifier} data sent successfully!")
            else:
                self.get_logger().error(f"Sending {identifier} data failed with response: {ack.decode('utf-8', errors='replace')}")
        except Exception as e:
            self.get_logger().error(f"Error sending {identifier} data: {e}")

    def process_request(self, request, response):
        """
        Process the request in a separate thread.

        Parameters:
            request: The incoming request object sent by a ros2 client to the service.
            response: The response object to be populated with the result or error message.

        Returns:
            response: The response object containing the result of the processing or an error message.
        """
        if self.save_static_data = "0":
            try:
                flag = request.flag
                if flag == "end":
                    self.send_data("end", "end", is_json=False)
                    response.response_message = "Succeed!"
                    return response

                frame_data = request.frame_data
                rgb_image_msg = request.rgb_image
                depth_image_msg = request.depth_image

                if not frame_data or not rgb_image_msg or not depth_image_msg:
                    self.get_logger().error("Missing one or more required data fields.")
                    response.response_message = "Fail!"
                    return response
                else:
                    self.get_logger().info(f"Data frame: {flag} received.")

                self.send_data(frame_data, "frame", is_json=True)

                try:
                    cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding='bgr8')
                    cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
                    compressed_rgb = self.bridge.cv2_to_compressed_imgmsg(cv_rgb_image, dst_format='jpg').data
                    compressed_depth = self.bridge.cv2_to_compressed_imgmsg(cv_depth_image, dst_format='jpg').data

                    self.send_data(copy.deepcopy(compressed_rgb), "RGB_image", is_json=False)
                    self.send_data(copy.deepcopy(compressed_depth), "depth_image", is_json=False)
                    self.get_logger().info(f"Data frame: {flag} sent successfully!")
                    response.response_message = "Succeed!"
                except Exception as e:
                    self.get_logger().error(f"Failed to send data: {e}")
                    response.response_message = "Fail!"
                return response 
            except Exception as e:
                self.get_logger().error(f"Failed to process request: {e}")
                response.response_message = "Fail!"
                return response
        elif self.save_static_data = "1":
            try:
                flag = request.flag
                if flag == "end":
                    return "end"

                frame_data = request.frame_data
                rgb_image_msg = request.rgb_image
                depth_image_msg = request.depth_image

                if not frame_data or not rgb_image_msg or not depth_image_msg:
                    self.get_logger().error("Missing one or more required data fields.")
                    return "Fail"
                else:
                    self.get_logger().info(f"Data frame: {flag} received.")


                try:
                    cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding='bgr8')
                    cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')
                    compressed_rgb = self.bridge.cv2_to_compressed_imgmsg(cv_rgb_image, dst_format='jpg').data
                    compressed_depth = self.bridge.cv2_to_compressed_imgmsg(cv_depth_image, dst_format='jpg').data

                    joint_motion_data = frame_data["execution_data"]["motion_execution_data"]["joint_motion_data"]
                    gripper_status = frame_data["execution_data"]["motion_execution_data"]["gripper_status"]
                    timestamp = frame_data["timestamp"]

                    self.append_real_time_data(self.hdf5_path, timestamp, joint_motion_data, gripper_status, compressed_rgb, compressed_depth)

                    self.get_logger().info(f"Data frame: {flag} save successfully!")
                except Exception as e:
                    self.get_logger().error(f"Failed to save data: {e}")
                return "Sucess" 
            except Exception as e:
                self.get_logger().error(f"Failed to process request: {e}")
                return "Fail"


    def instruct_callback(self, request, response):
        """
        Handles the instruction callback by processing the request asynchronously using a thread pool.
        
        Parameters:
            request: The incoming request object sent by a ros2 client to the service.
            response: The response object to be populated with the result or error message.

        Returns:
            response: The response object containing the result of the processing or an error message.
        """
        try:
            future = self.thread_pool.submit(self.process_request, request, response)
            result = future.result()
            return result
        except Exception as e:
            self.get_logger().error(f"Exception occurred while processing request: {e}")
            response.response_message = "Fail!"
            return response
    def initialize_hdf5(self, hdf5_path, data_path):
        """
        初始化 HDF5 文件，存储 JSON 元数据，并创建用于存储实时数据的组
        """
        output_dir = os.path.dirname(hdf5_path)
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f" 目录 {output_dir} 已创建")
        
        try:
            with open(data_path, 'r') as file:
                data = json.load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load data: {e}")
            return

        with h5py.File(hdf5_path, "w") as f:
            def save_static_data(group, data):
                """递归存储 JSON 数据"""
                for key, value in data.items():
                    if isinstance(value, dict):
                        subgroup = group.create_group(key)  # 创建字典对应的子组
                        save_static_data(subgroup, value)
                    elif isinstance(value, list):
                        subgroup = group.create_group(key)  # 处理列表，存为 0,1,2...
                        for i, item in enumerate(value):
                            sub_group = subgroup.create_group(str(i))
                            save_static_data(sub_group, item)
                    else:
                        group.attrs[key] = value if isinstance(value, str) else str(value)

        #  存储属性
        root_group = f.create_group("robot_motion_sample")
        save_static_data(root_group, data["robot_motion_sample"])

        # 创建存储实时数据的组
        f.create_group("dataset/execution_data/motion_execution_data/joint_motion_data")
        f.create_group("dataset/execution_data/motion_execution_data/gripper_status")
        f.create_group("dataset/execution_data/perception_data/rgb_image")
        f.create_group("dataset/execution_data/perception_data/depth_image")

        print(f" HDF5 文件已初始化: {hdf5_path}")
    
    def append_real_time_data(self, hdf5_path, timestamp, joint_motion_data, gripper_status, rgb_image, depth_image):
        """
        追加实时数据到 HDF5
        """
        with h5py.File(hdf5_path, "a") as f:  #  （追加数据）
            # 追加关节运动数据
            jm_group = f["dataset/execution_data/motion_execution_data/joint_motion_data"]
            jm_group.create_dataset(timestamp, data=joint_motion_data)

            #  追加夹爪状态
            gs_group = f["dataset/execution_data/motion_execution_data/gripper_status"]
            gs_group.create_dataset(timestamp, data=gripper_status)

            #  追加 RGB 图像
            rgb_group = f["dataset/execution_data/perception_data/rgb_image"]
            rgb_group.create_dataset(timestamp, data=rgb_image)

            # 追加深度图像
            depth_group = f["dataset/execution_data/perception_data/depth_image"]
            depth_group.create_dataset(timestamp, data=depth_image)

def main(args=None):
    rclpy.init(args=args)
    collection = AirspeedDataCollection()

    executor = MultiThreadedExecutor()
    executor.add_node(collection)

    try:
        collection.get_logger().info('Beginning data collection, shut down with CTRL-C')
        executor.spin()
    except KeyboardInterrupt:
        collection.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()