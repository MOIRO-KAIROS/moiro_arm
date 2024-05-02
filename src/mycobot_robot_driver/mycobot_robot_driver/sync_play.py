import time
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot
tm = 0

# mc = MyCobot('/dev/ttyACM0', str(115200))
# # gripper initailization
# mc.set_gripper_calibration()
# mc.set_gripper_mode(0)
# mc.init_eletric_gripper()
# time.sleep(1)
# mc.set_free_mode(1)
# time.sleep(0.05)

class MyCobotListener(Node):

    def __init__(self):
        super().__init__('listener')
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'port: {port}, baud: {baud}')
        self.mycobot = MyCobot(port, str(baud))
        # # gripper initailization
        self.mycobot.set_gripper_calibration()
        self.mycobot.set_gripper_mode(0)
        self.get_logger().info(f'============================================================\ngripper initialized\n')
        time.sleep(1)
        self.mycobot.set_free_mode(1)
        time.sleep(0.05)

        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        data_list = []
        # print(msg.name)
        joint_data = sorted(zip(msg.name, msg.position), key=lambda x: x[0])
        msg.name, msg.position = zip(*joint_data)
        msg.position = list(msg.position)
        msg.position[3] = -1 * msg.position[3]
        # print(msg.name)
        print("===="*3)

        data_list = [round(math.degrees(pos), 2) for pos in msg.position[1:]] + [msg.position[0]]
        global tm
        tm += 1
        data_list[6] = round(abs(-0.7-data_list[6])*117)
        if data_list[6] > 100:
            data_list[6] = 100
        data_list[6] = 100 - data_list[6]
        self.get_logger().info(f'data list: {data_list}, {tm}')
        self.mycobot.send_angles(data_list[:6], 80)
        time.sleep(0.1)
        self.mycobot.set_gripper_value(data_list[6], 80, 1)
        time.sleep(0.1)
        # self.mycobot.set_gripper_value("as", 80)
        # self.get_logger().info(f'joint angles: {msg}, {tm}')

def main(args=None):
    rclpy.init(args=args)

    mycobot_listener = MyCobotListener()

    rclpy.spin(mycobot_listener)

    mycobot_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()