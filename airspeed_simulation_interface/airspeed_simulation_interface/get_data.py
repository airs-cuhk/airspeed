import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class JointCommandSubscriber(Node):


    def __init__(self):
        super().__init__('joints_command_subscriber')

        self.subscription = self.create_subscription(
            String,
            'joints_pub_random',
            self.listener_callback,
            10
        )
        self.subscriptions
    
    def listener_callback(self, msg):
        self.get_logger().info('The real time coordinates are: %s' % msg.data)

    

def main(args = None):
    rclpy.init(args=args)
    joint_command_subscriber = JointCommandSubscriber()
    rclpy.spin(joint_command_subscriber)

    joint_command_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
