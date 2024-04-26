import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info('port: %s, baud: %d' % (port, baud))
        self.mycobot = MyCobot(port, str(baud))
    
    def listener_callback(self, msg):
        self.get_logger().info('I heard: %s' % msg)
        self.mycobot.send_angles(msg.position, 80)
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)

    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()