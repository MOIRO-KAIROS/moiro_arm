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
        # print(msg.name)
        print("===="*3)

        data_list = [round(math.degrees(pos), 2) for pos in msg.position]
        self.mc.send_angles(data_list, 80)
        self.get_logger().info(f'joint angles: {msg}')
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    slider_subscriber = Slider_Subscriber()
    
    rclpy.spin(slider_subscriber)
    
    slider_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
