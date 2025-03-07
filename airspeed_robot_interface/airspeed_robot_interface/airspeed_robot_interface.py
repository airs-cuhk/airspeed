import cv2
import zlib
import copy
import time
import json
import queue
import rclpy
import requests
import numpy as np
import pyrealsense2 as rs
from rclpy.node import Node
from threading import Thread
from datetime import datetime
from std_msgs.msg import String
from flask import Flask, request, jsonify
from sensor_msgs.msg import CompressedImage
from airspeed_services.srv import DataCollection
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class MessageProcessorNode(Node):
    def __init__(self):
        """
        Initialize the MessageProcessorNode.

        This method sets up the ROS node, connects to the DataCollection service, initializes the RealSense camera,
        sets up the subscription to receive joint positions, and starts the Flask server to receive data from the robot.

        Returns:
            None
        """

        super().__init__('message_processor')
        # Connect to DataCollection
        self.collection_server_ready = False
        self._client = self.create_client(DataCollection, '/airspeed_data_collection/collection_server', callback_group=MutuallyExclusiveCallbackGroup())
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.collection_server_ready = False
            self.get_logger().error('Collection service not available!')
        else:
            self.collection_server_ready = True
            self.get_logger().info('Collection service is ready!')
            
        self.req = DataCollection.Request()


        # Display the real time image
        # self.rgb_image_queue = queue.Queue(maxsize=10) # 显示图像的队列
        # self.display_thread = Thread(target=self.display_image)
        # self.display_thread.daemon = True  # 设置为守护线程，程序退出时会自动结束
        # self.display_thread.start()

        # realsense init
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.flag = 'start'
        self.num = 0
        self.config.enable_stream(
            rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(
            rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(self.config)
        self.device = self.pipeline.get_active_profile().get_device()
        self.color_sensor = self.device.first_color_sensor()
        self.color_sensor.set_option(
            rs.option.enable_auto_exposure, 1)
        self.color_sensor.set_option(
            rs.option.enable_auto_white_balance, 1)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # create the subscription
        # self.subscription = self.create_subscription(
        #     String,
        #     'joint_data',
        #     self.listener_callback,
        #     1)
        
        self.publisher_ = self.create_publisher(
            String,
            'joints_pub_random',
            1)
        
        # create the connetion to the robot, in order to receive the data.
        self.http_endpoint = "http://10.60.77.187:65432"
        self.flask_app = Flask(__name__)
        self.flask_app.add_url_rule(
            '/receive_message', 'receive_message', self.receive_message, methods=['POST'])

        # start the app to receive the data from the robot
        self.flask_thread = Thread(target=self.run_flask_server)
        self.flask_thread.start()
        
        self.ros2_node = False  # Whether to start node ros2
        self.http_node = True # Whether to start node http_node
        
        # set the interval of sending message to the robot
        if self.ros2_node:
            self.interval = 0.2
        elif self.http_node:
            self.interval = 0.2

    def listener_callback(self, msg):
        '''
        Receive the data from human's manipulation or other way.

        This method takes a JSON string as input, parses it into a dictionary, and then sends a POST request
        to the specified HTTP endpoint with the message and gripper data. It logs the HTTP response status code
        and text for debugging purposes.

        Args:
            msg (str): A JSON string containing the message and gripper data.

        Returns:
            None
        '''
        self.get_logger().info(f'Received message: "{msg.data}"')
        self.send_message_via_http_or_ros(msg.data)
        time.sleep(self.interval)

    def send_message_via_http_or_ros(self, data):
        """
        Send a message to a remote server via HTTP POST request.

        This method takes a JSON string as input, parses it into a dictionary, and then sends a POST request
        to the specified HTTP endpoint with the message and gripper data. It logs the HTTP response status code
        and text for debugging purposes.

        Args:
            data (str): A JSON string containing the message and gripper data.

        Returns:
            requests.Response: The HTTP response object if the request is successful, otherwise None.
        """

        try:
            if self.ros2_node:
                msg = String()
                msg.data = data
                self.publisher_.publish(msg)
            elif self.https_node:
                data = json.loads(data)
                response = requests.post(
                    self.http_endpoint, json={"message": data[0], 'gripper':data[1]})
                self.get_logger().info(
                    f'HTTP response: {response.status_code}, {response.text}')
                return response
        except Exception as e:
            self.get_logger().error(f'HTTP request failed: {str(e)}')
            return None
    
    def compress_rgb_image(self, color_image: np.ndarray):
        """
        Compress the RGB image using JPEG encoding.

        This method takes an RGB image as input, encodes it using JPEG format, and returns a CompressedImage message.

        Args:
            color_image (np.ndarray): The RGB image to be compressed.

        Returns:
            CompressedImage: The compressed RGB image message.
        """
        
        color_image_jpeg = cv2.imencode('.jpg', color_image)[1]
        color_image_send_compressed = CompressedImage()
        color_image_send_compressed.format = "jpeg"
        color_image_send_compressed.data = np.array(color_image_jpeg).tostring()
        
        return color_image_send_compressed
    
    def compress_depth_image(self, depth_image: np.ndarray):
        """
        Compress the depth image using zlib compression.

        This method takes a depth image as input, converts it to bytes, compresses it using zlib, and returns a CompressedImage message.

        Args:
            depth_image (np.ndarray): The depth image to be compressed.

        Returns:
            CompressedImage: The compressed depth image message.
        """
        
        depth_image_bytes = depth_image.tobytes()
        compressed_depth_data = zlib.compress(depth_image_bytes)
        depth_image_send_compressed = CompressedImage()
        depth_image_send_compressed.format = "16UC1;zlib"
        depth_image_send_compressed.data = compressed_depth_data
        
        return depth_image_send_compressed
    
    def get_time_stamp(self):
        """
        Generate a timestamp in the format of 'YYYY-MM-DD-HH-MM-SS-SSS'.

        This method gets the current ROS time, converts it to a datetime object, and formats it into a string with milliseconds.

        Returns:
            str: The formatted timestamp.
        """
        
        ros_time = self.get_clock().now().to_msg()
        dt = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)
        milliseconds = int(dt.microsecond // 1000)
        formatted_time = dt.strftime('%Y-%m-%d-%H-%M-%S') + f'-{milliseconds:03d}'
        
        return formatted_time
    
    def get_frame_data(self, joint_angles: list, gripper: float):
        """
        Generate a JSON message containing the timestamp, joint angles, and gripper status.

        This method takes joint angles and gripper status as input, generates a timestamp, and creates a JSON message with the execution data.

        Args:
            joint_angles (list): The joint angles of the robot.
            gripper (float): The gripper status of the robot.

        Returns:
            String: The JSON message containing the frame data.
        """
        
        formatted_time = self.get_time_stamp()
    
        frame_data_raw = {
            "timestamp": formatted_time,
            "execution_data": {
                "motion_execution_data": {
                    "joint_motion_data": joint_angles,
                    "gripper_status": gripper
                }
            }
        }
        
        frame_data_str =json.dumps(frame_data_raw, indent=4)
        json_msg = String()
        json_msg.data = frame_data_str
        
        return json_msg
    
    def get_images(self):
        """
        Retrieve depth and color images from the RealSense camera.

        This method waits for frames from the camera, aligns them, and extracts the depth and color frames.
        It then converts the frames to numpy arrays and returns them as depth and color images.

        Returns:
            tuple: A tuple containing the depth image and color image as numpy arrays.
        """
        try:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            return depth_image, color_image
        
        except Exception as e:
            return jsonify({"status": "failed", "message": f"Color and Deep img has missed, error is {e}"})
            
        
            
        
    def send_request(self, depth_image: np.ndarray, color_image: np.ndarray, flag: str, joint_angles:list, gripper:float):
        """
        Send a request to the DataCollection service with the provided data.

        This method takes depth and color images, a flag, joint angles, and gripper status as input. It compresses the images,
        generates a JSON message with the frame data, and sends a request to the DataCollection service. If the flag is not 'end',
        it includes the compressed images and frame data in the request.

        Args:
            depth_image (np.ndarray): The depth image to be sent.
            color_image (np.ndarray): The color image to be sent.
            flag (str): The flag indicating the type of request.
            joint_angles (list): The joint angles of the robot.
            gripper (float): The gripper status of the robot.

        Returns:
            None
        """
        
        json_msg = self.get_frame_data(joint_angles, gripper)
        color_image_send_compressed = self.compress_rgb_image(color_image)
        depth_image_send_compressed = self.compress_depth_image(depth_image)


        if flag != 'end':
            self.req.flag = str(flag)
            self.req.frame_data = json_msg.data
            self.req.rgb_image = copy.deepcopy(color_image_send_compressed)
            self.req.depth_image = copy.deepcopy(depth_image_send_compressed)
        else:
            self.req.flag = str(flag)
        
        self.num += 1
        future = self._client.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """
        Callback function for handling the response from the DataCollection service.

        This method is called when the service call is completed. It logs the response if successful,
        or logs the error if the service call fails.

        Args:
            future (rclpy.task.Future): The future object representing the service call.

        Returns:
            None
        """

        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def receive_message(self):
        """
        Receive and process HTTP POST requests containing joint angles and gripper data.

        This method extracts joint angles and gripper data from the incoming JSON request,
        retrieves depth and color images from the camera, and sends a request to the DataCollection service.
        It logs the received data and the status of the image sending process.

        Returns:
            tuple: A tuple containing the JSON response and the HTTP status code.
        """

        try:
            data = request.json
            joint_angles = json.loads(data)['joint_angles']
            gripper = json.loads(data['gripper'])
            self.get_logger().info(f'Received HTTP POST data: {joint_angles}.')
            
            # Get the rgb_image and depth_image
            depth_image, color_image = self.get_images()
            
            # Put the rgb_image into the queue
            # self.rgb_image_queue.put(color_image)
            
            try:
               if not self._client.wait_for_service(timeout_sec=0.01):
                   self.get_logger().error('Collection service not available!')
                   return jsonify({"status": "failed", "message": "Collection service not available!"})
                
               if self.num != 0:
                   self.flag = self.num
               self.send_request(depth_image, color_image, self.flag, joint_angles, gripper)
               self.get_logger().info(f'Depth_image and color_image has been sent')
                
            except Exception as e:
                self.get_logger().info(f' Depth_image and color_image has not been saved, {e}')
                return jsonify({"status": "failed", "message": "DataCollection broken"}), 400
            
            return jsonify({"status": "success", "message": "Data received"}), 200
        
        except Exception as e:
            self.get_logger().error(f'Error processing HTTP POST request: {str(e)}')
            
            return jsonify({"status": "error", "message": str(e)}), 400

    def run_flask_server(self):
        '''
        Run the Flask server to receive HTTP POST requests.
        '''

        self.flask_app.run(host='0.0.0.0', port=61740)
        
        
    def display_image(self):
        """
        Continuously display RGB images from the queue.

        This method runs in a loop while the ROS node is running. It checks if the RGB image queue is not empty,
        retrieves an image from the queue, and displays it using OpenCV. The loop continues until the 'q' key is pressed.

        Returns:
            None
        """

        while rclpy.ok():
            if not self.rgb_image_queue.empty():
                img = self.rgb_image_queue.get()
                if img is not None:
                    cv2.imshow('RGB Image', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
        cv2.destroyAllWindows()
        

def main(args=None):
    '''
    The main function to initialize the ROS node and start the message processing loop.
    '''
    rclpy.init(args=args)
    node = MessageProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
