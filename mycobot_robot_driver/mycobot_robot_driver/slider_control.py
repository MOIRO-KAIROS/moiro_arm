import time
import math
import rclpy
from pymycobot.mycobot import MyCobot
from rclpy.node import Node
from sensor_msgs.msg import JointState

tm = 0

class Slider_Subscriber(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.subscription = self.create_subscription(
            JointState,
            "joint_states",
            self.listener_callback,
            10
        )
        self.subscription

        self.mc = MyCobot("/dev/ttyACM0", 115200)
        self.mc.set_gripper_calibration()
        self.mc.set_gripper_state(0, 20) # gripper open
        self.mc.set_gripper_value(100, 20) # gripper close
        time.sleep(1)
        time.sleep(0.05)
        self.mc.set_free_mode(1)
        time.sleep(0.05)

    def listener_callback(self, msg):
        data_list = []
        # print(msg.name)
        joint_data = sorted(zip(msg.name, msg.position), key=lambda x: x[0])
        msg.name, msg.position = zip(*joint_data)
        msg.position = list(msg.position)
        msg.position[3] = -1 * msg.position[3]
        # print(msg.name)
        print("===="*3)

        data_list = [round(math.degrees(pos), 2) for pos in msg.position]
        global tm
        tm += 1
        self.mc.send_angles(data_list[1:], 25)
        print(f"gripper angle msg {data_list[0]}")
        data_list[0] = 100 if abs((data_list[0] / 0.6) * 100 - 100) >= 100 else abs(((data_list[0] / 0.6) * 100) - 100) 
        if tm % 10 == 0:
            self.mc.set_gripper_value(int(data_list[0]), 20, 1) # int(data_list[0])
            tm = 0
        self.get_logger().info(f'joint angles: {msg}, {tm}')


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
