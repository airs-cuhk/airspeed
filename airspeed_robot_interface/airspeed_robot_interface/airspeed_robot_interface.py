import rclpy
import cv2
import pyrealsense2 as rs
from rclpy.node import Node
from std_msgs.msg import String
import requests
from cv_bridge import CvBridge
import time
import json
from flask import Flask, request, jsonify
from threading import Thread
import numpy as np
from airspeed_services.srv import DataCollection
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from datetime import datetime
import queue
from std_msgs.msg import String

class MessageProcessorNode(Node):
    def __init__(self):
        super().__init__('message_processor')
        
        self.collection_server_ready = False
        self._client = self.create_client(DataCollection, '/airspeed_data_collection/collection_server', callback_group=MutuallyExclusiveCallbackGroup())
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.collection_server_ready = False
            self.get_logger().error('Collection service not available!')
        else:
            self.collection_server_ready = True
            self.get_logger().info('Collection service is ready!')
            
        self.req = DataCollection.Request()
        
        self.rgb_image_queue = queue.Queue(maxsize=10) # 显示图像的队列
        
        self.display_thread = Thread(target=self.display_image)
        self.display_thread.daemon = True  # 设置为守护线程，程序退出时会自动结束
        self.display_thread.start()

        # 配置管道和配置文件
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.flag = 'start'
        self.num = 0

        # 启动 RealSense 设备
        self.config.enable_stream(
            rs.stream.depth, 1280, 720, rs.format.z16, 30)
        self.config.enable_stream(
            rs.stream.color, 1280, 720, rs.format.bgr8, 30)

        # 开始流
        self.profile = self.pipeline.start(self.config)

        # 获取设备对象并设置自动曝光
        self.device = self.pipeline.get_active_profile().get_device()
        self.color_sensor = self.device.first_color_sensor()
        self.color_sensor.set_option(
            rs.option.enable_auto_exposure, 1)  # 开启自动曝光
        self.color_sensor.set_option(
            rs.option.enable_auto_white_balance, 1)  # 开启自动白平衡

        # 对齐
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        self.interval = 0.2

        self.subscription = self.create_subscription(
            String,
            'joint_positions',
            self.listener_callback,
            1)

        self.http_endpoint = "http://10.60.77.187:65432"

        self.flask_app = Flask(__name__)

        # 注册 HTTP 接口
        self.flask_app.add_url_rule(
            '/receive_message', 'receive_message', self.receive_message, methods=['POST'])

        self.get_logger().info('MessageProcessorNode has been started.')

        # 启动 Flask 应用线程
        self.flask_thread = Thread(target=self.run_flask_server)
        self.flask_thread.start()

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: "{msg.data}"')

        self.send_message_via_http(msg.data)

        time.sleep(self.interval)

    def send_message_via_http(self, data):
        try:
            data = json.loads(data)
            response = requests.post(
                self.http_endpoint, json={"message": data[0], 'gripper':data[1]})
            self.get_logger().info(
                f'HTTP response: {response.status_code}, {response.text}')
            return response
        except Exception as e:
            self.get_logger().error(f'HTTP request failed: {str(e)}')
            return None

    def send_request(self, depth_image: np.ndarray, color_image: np.ndarray, flag: str, joint_angles:list, gripper:float):
        
        ros_time = self.get_clock().now().to_msg()
        dt = datetime.fromtimestamp(ros_time.sec + ros_time.nanosec / 1e9)
        milliseconds = int(dt.microsecond // 1000)
        formatted_time = dt.strftime('%Y-%m-%d-%H-%M-%S') + f'-{milliseconds:03d}'

        frame_data_raw = {
            "timestamp": formatted_time,
            "execution_data": {
                "motion_execution_data": {
                    "joint_motion_data": joint_angles,
                    "gripper_status": gripper
                }
            }
        }
        
        frame_data_str =json.dumps(frame_data_raw,indent=4)
        json_msg = String()
        json_msg.data = frame_data_str
        
        
        print(frame_data_raw['execution_data']['motion_execution_data']['joint_motion_data'])
        
        color_image = self.bridge.cv2_to_imgmsg(color_image, encoding='rgb8')
        depth_image = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
        
        
        if flag != 'end':
            self.req.flag = str(flag)
            self.req.frame_data = json_msg.data
            self.req.rgb_image = color_image
            self.req.depth_image = depth_image
            if self.req.rgb_image is not None:
                print('============+++++++++++')
            else:
                print('+++++++++++============')
        else:
            self.req.flag = str(flag)
            
        # 设置请求消息的内容
        self.num += 1
        future = self._client.call_async(self.req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response: {response}')
        except Exception as e:
            self.get_logger().error(f'Service call failed {e}')

    def receive_message(self):
        try:
            data = request.json
            joint_angles = json.loads(data)['joint_angles']
            self.get_logger().info(f'Received HTTP POST data: {joint_angles}.')
            self.get_logger().info(f'=======================================')
            
            frames = self.pipeline.wait_for_frames()

            aligned_frames = self.align.process(frames)

            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not aligned_depth_frame or not color_frame:
                return jsonify({"status": "failed", "message": "Color and Deep img has missed"})

            # Convert images to numpy arrays
            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # 将接收到的 RGB 图像放入队列
            self.rgb_image_queue.put(color_image)
            
            if color_image is not None:
                print('!!!!!!!!@@@@@@@@@@@@#############')
            else:
                print('$$$$$$$$$$$%^^^^^^^^^^^^')
            
            # 调用服务并传递数据
            try:
               if not self._client.wait_for_service(timeout_sec=0.01):
                   self.get_logger().error('Collection service not available!')
                   return jsonify({"status": "failed", "message": "Collection service not available!"})
                
               if self.num != 0:
                   self.flag = self.num
               self.send_request(depth_image, color_image, self.flag, joint_angles, gripper=0.01)
               self.get_logger().info(f' Depth_image and color_image has been sent')
                
            except Exception as e:
                self.get_logger().info(f' Depth_image and color_image has not been saved, {e}')
                return jsonify({"status": "failed", "message": "DataCollection broken"}), 400
            
            return jsonify({"status": "success", "message": "Data received"}), 200
        
        except Exception as e:
            
            self.send_request(None, None, 'end', None)
            self.num = 0
            self.get_logger().info(f' The last Depth_image and color_image has been saved')
            
            self.get_logger().error(f'Error processing HTTP POST request: {str(e)}')
            
            return jsonify({"status": "error", "message": str(e)}), 400

    def run_flask_server(self):
        self.flask_app.run(host='0.0.0.0', port=61740)
        
        
    def display_image(self):
        # 在一个独立的线程中显示图像
        while rclpy.ok():
            if not self.rgb_image_queue.empty():
                print('+++++++++++++++++++++++!!!!!!!!!!!!!!!!!!!!')
                img = self.rgb_image_queue.get()
                if img is not None:
                    cv2.imshow('RGB Image', img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 'q' 键退出
                        break
        cv2.destroyAllWindows()
        

def main(args=None):
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
