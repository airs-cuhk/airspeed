import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from grasp_interface_pkg.srv import Grasp
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
import cv2 as cv
from time import sleep

def inverse(x, y, z):
    rx = np.sqrt(x**2 + y**2) - 0.095
    rz = z + 0.1693 - 0.22934
    l = np.sqrt(rx**2 + rz**2)
    if l > 0.27 + 0.267:
        print('Out of reach')
        return None
    dz = 0.0725
    theta1 = np.arctan2(y, x) - np.arcsin(dz/l)
    alpha = np.arctan2(rz, rx)
    beta = np.arccos((0.27**2 + l**2 - 0.267**2) / (2*0.27*l))
    theta2 = -alpha - beta
    theta3 = np.pi - np.arccos((0.27**2 + 0.267**2 - l**2) / (2*0.27*0.267))
    theta4 = - theta2 - theta3 - np.pi/2.0
    theta5 = - np.pi/2
    theta6 =  theta1 + np.pi - (67.5/180.0)*np.pi
    return [theta1, theta2, theta3, theta4, theta5, theta6, 0.0]

class teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.pub = self.create_publisher(JointState, 'joint_command', 10)
        self.msg = JointState()
        self.msg.name = [
            'joint_link1_to_base',
            'joint_link2_to_link1',
            'joint_link3_to_link2',
            'joint_link4_to_link3',
            'joint_link5_to_link4',
            'joint_link6_to_link5',
            'gripper_controller'
        ]
        self.msg.position = [0.0, -0.5, 0.7, -1.7, -1.57, 1.9, 0.0]
        self.msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.target_x = 0.4
        self.target_y = 0
        self.target_z = 0
        self.gripper_distance = 0.0
        self.control_pannel = np.zeros((500, 500, 3), np.uint8)

        my_callback_group = ReentrantCallbackGroup()

        self.timer = self.create_timer(0.025, self.timer_callback, callback_group=my_callback_group)
        # self.update_joint = self.create_timer(0.01, self.update_target_joint)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10,callback_group=my_callback_group)
        self.target_joint = inverse(self.target_x, self.target_y, self.target_z)
        self.srv = self.create_service(Grasp, 'grasp', self.grasp_callback, callback_group=my_callback_group)

        self.gripper_command = 0
        self.gripper_bool = 0
        self.pos_bool = 1

    def update_target_joint(self):
        self.target_x = np.clip(self.target_x, 0.24, 0.6)
        self.target_y = np.clip(self.target_y, -0.4, 0.4)
        self.target_z = np.clip(self.target_z, -0.55, 0.3)
        tmp_joint = inverse(self.target_x, self.target_y, self.target_z)
        if tmp_joint is not None:
            self.target_joint = tmp_joint
        else :
            print('Out of reach')

    def grasp_callback(self, request, response):
        target_x = request.x
        target_y = request.y
        target_z = request.z
        if(inverse(target_x, target_y, target_z) is None):
            response.ret = 1 # Out of reach
            return response

        self.target_x = target_x
        self.target_y = target_y
        self.target_z = target_z
        
        self.pos_bool = 1

        if(request.g == 0 or request.g == 1): #遥操作任务
            self.target_joint = inverse(self.target_x, self.target_y, self.target_z)
            self.gripper_command = request.g
            response.ret = 0
            return response

        self.gripper_command = 0
        self.gripper_bool = 1

        # while self.gripper_bool == 1:
            # print("Gripper Opening")
        sleep(1.0) # 等待夹爪打开
        print ("Gripper Opened")

        self.target_joint = inverse(self.target_x, self.target_y, self.target_z)

        while self.pos_bool == 1:
            print(self.now_joint)
            print("Moving to target position")
        
        print("Arrived at target position")


        self.gripper_command = 1
        # while self.gripper_bool == 0:
        #     print("Grasping")
        # sleep for 0.5s
        sleep(1.0)
        print("Grasper Closed")

        response.ret = 0
        return response

    def joint_callback(self, msg):
        self.now_joint = msg.position[6:12]
        error = np.linalg.norm(np.array(self.now_joint) - np.array(self.target_joint[0:6]))
        if error < 0.05:
            self.pos_bool = 0
        else:
            self.pos_bool = 1
        
        gripper_target = 0.0 if self.gripper_command == 0 else -1.0
        gripper_error = np.abs(msg.position[15] - gripper_target)

        if gripper_error < 0.1: #到达之后切换夹爪状态
            self.gripper_bool = self.gripper_command

        self.msg.position = self.target_joint
        self.msg.position[6] = 0.0 if self.gripper_command == 0 else -1.0
        self.pub.publish(self.msg)

    def timer_callback(self):
        control_pannel = np.zeros((500, 500, 3), np.uint8)
        cv.putText(control_pannel, 'Target Position', (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(control_pannel, 'X: ' + str(self.target_x), (10, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(control_pannel, 'Y: ' + str(self.target_y), (10, 90), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(control_pannel, 'Z: ' + str(self.target_z), (10, 120), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.putText(control_pannel, 'Gripper: ' + str(self.gripper_distance), (10, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J1: ' + str(joint_list[0]), (10, 150), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J2: ' + str(joint_list[1]), (10, 180), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J3: ' + str(joint_list[2]), (10, 210), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J4: ' + str(joint_list[3]), (10, 240), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J5: ' + str(joint_list[4]), (10, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        # cv.putText(control_pannel, 'J6: ' + str(joint_list[5]), (10, 300), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv.imshow('Control Pannel', control_pannel)
        key = cv.waitKey(10)
        if(key == ord('w')):
            self.target_x += 0.01
        if(key == ord('s')):
            self.target_x -= 0.01
        if(key == ord('a')):
            self.target_y += 0.01
        if(key == ord('d')):
            self.target_y -= 0.01
        if(key == ord(' ')):
            self.target_z += 0.01
        if(key == ord('c')):
            self.target_z -= 0.01
        if(key == ord('o')):
            self.gripper_distance = 0.0
        if(key == ord('l')):
            self.gripper_distance = -1.0
        if key == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = teleop()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    rclpy.shutdown()
    cv.destroyAllWindows()

