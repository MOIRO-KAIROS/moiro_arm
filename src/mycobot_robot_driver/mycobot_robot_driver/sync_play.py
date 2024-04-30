import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot

tm = 0

class MyCobotListener(Node):

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

        self.get_logger().info(f'port: {port}, baud: {baud}')
        self.mycobot = MyCobot(port, str(baud))
        time.sleep(0.05)
        self.mycobot.set_free_mode(1)
        time.sleep(0.05)


    def listener_callback(self, msg):
        data_list = []
        joint_data = sorted(zip(msg.name, msg.position), key=lambda x: x[0])
        msg.name, msg.position = zip(*joint_data)
        msg.position = list(msg.position)
        msg.position[2] = -1 * msg.position[2]

        data_list = [round(math.degrees(pos), 2) for pos in msg.position]
        global tm
        tm += 1
        self.mycobot.send_angles(data_list, 80)
        self.get_logger().info(f'joint angles: {data_list}, {tm}')

def main(args=None):
    rclpy.init(args=args)

    mycobot_listener = MyCobotListener()

    rclpy.spin(mycobot_listener)

    mycobot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()