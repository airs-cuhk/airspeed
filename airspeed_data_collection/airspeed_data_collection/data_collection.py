import os
import json
import socket
import rclpy
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
        self.config_path = self.declare_and_get_parameter('config', 'config/config.json').value
        self.config = self.load_config(self.config_path)
        self.main_data = self.config.get('main_data')
        if self.main_data is None:
            self.get_logger().warning("Main data not found!")
        else:
            self.get_logger().info("Loaded main data!")

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

    def declare_and_get_parameter(self, name, default_value):
        """
        Declare and get a parameter with a default value.
        """
        self.declare_parameter(name, default_value)
        return self.get_parameter(name)

    def load_config(self, config_path):
        """
        Load configuration from a JSON file.
        """
        try:
            with open(config_path, 'r') as file:
                return json.load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {e}")
            return {}

    def send_data(self, sock, data, identifier, is_json=False):
        """
        Send data (either JSON or image) over a socket.
        """
        try:
            # 发送标识符
            identifier_length = len(identifier).to_bytes(4, byteorder='big')
            sock.sendall(identifier_length + identifier.encode('utf-8'))

            # 处理数据部分
            if is_json:
                data_json = json.dumps(data) if not isinstance(data, str) else data
                data_bytes = data_json.encode('utf-8')
            else:
                # 确保 data 是字节串或可转换为字节串的字符串
                if isinstance(data, str):
                    data_bytes = data.encode('utf-8')  # 将字符串编码为字节串
                elif isinstance(data, bytes):
                    data_bytes = data  # 如果已经是字节串，则直接使用
                else:
                    raise TypeError("Data must be either a string or bytes")

            # 发送数据大小和数据本身
            size = len(data_bytes).to_bytes(4, byteorder='big')
            sock.sendall(size + data_bytes)

            # 设置超时并接收确认消息
            sock.settimeout(10.0)
            ack = sock.recv(1024)
            if ack.startswith(b'{"status":"success"'):
                self.get_logger().info(f"Sent {identifier} data successfully!")
            else:
                self.get_logger().error(f"Sending {identifier} data failed with response: {ack.decode('utf-8', errors='replace')}")

        except socket.timeout:
            self.get_logger().error("Timeout waiting for acknowledgment.")
        except Exception as e:
            self.get_logger().error(f"Error sending {identifier} data: {e}")

    def process_request(self, request, response):
        """
        Process the request in a separate thread.
        """
        try:
            flag = request.flag
            if flag != "end":
                frame_data = request.frame_data
                rgb_image_msg = request.rgb_image
                depth_image_msg = request.depth_image
                self.get_logger().info(flag)

                if not frame_data or not rgb_image_msg or not depth_image_msg:
                    self.get_logger().error("Missing one or more required data fields.")
                    response.response_message = "Fail!"
                    return response

                # 尝试建立连接
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)  # 设置超时时间
                
                try:
                    # 尝试连接到服务端
                    # sock.connect(('10.60.170.131', 6078))  # 替换为接收者的 IP 和端口
                    # 或者使用备用地址
                    sock.connect(('10.60.2.182', 6078))
                    
                    # 如果连接成功，开始发送数据
                    try:
                        self.send_data(sock, flag, "flag", is_json=False)
                        frame_data_dict = json.loads(frame_data)
                        self.send_data(sock, frame_data, "frame", is_json=True)

                        if flag == "start":
                            self.send_data(sock, self.main_data, "main", is_json=True)

                        cv_rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, desired_encoding='bgr8')
                        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding='passthrough')

                        compressed_rgb = self.bridge.cv2_to_compressed_imgmsg(cv_rgb_image, dst_format='jpg').data
                        compressed_depth = self.bridge.cv2_to_compressed_imgmsg(cv_depth_image, dst_format='jpg').data

                        self.send_data(sock, compressed_rgb, "rgb", is_json=False)
                        self.send_data(sock, compressed_depth, "depth", is_json=False)

                        self.get_logger().info("All data sent successfully!")
                        response.response_message = "Succeed!"

                    except Exception as e:
                        self.get_logger().error(f"Failed to send data: {e}")
                        response.response_message = "Fail!"

                except socket.error as e:
                    self.get_logger().error(f"Failed to connect to server: {e}")
                    response.response_message = "Server disconnect!"
                
                finally:
                    sock.close()

            else:
                # 处理结束标志
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.settimeout(5.0)
                    sock.connect(('10.60.170.131', 6078))  # 替换为接收者的 IP 和端口
                    self.send_data(sock, "end", "flag", is_json=False)
                response.response_message = "Succeed!"
            
            return response
        
        except Exception as e:
            self.get_logger().error(f"Failed to process request: {e}")
            response.response_message = "Fail!"
            return response

    def instruct_callback(self, request, response):
        """
        Handle incoming requests by submitting them to the thread pool.
        """
        future = self.thread_pool.submit(self.process_request, request, response)

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Exception occurred while processing request: {e}")
            response.response_message = "Fail!"
            return response

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