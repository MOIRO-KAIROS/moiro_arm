import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose,PoseStamped
# from tf2_geometry_msgs import 

import time
import tf2_ros
import transforms3d.quaternions as txq
import numpy as np
from cube_msg.msg import DetectInfo

class CameraPosition(Node):

    def __init__(self):
        super().__init__('camera_position')
        self.get_logger().info('========================') 
        self.get_logger().info('Start to set Camera world pose!')
        self.get_logger().info('========================') 

        self.camera_link = "camera_link"
        # self.serial_port = serial.Serial('/dev/ttyACM0', 115200)  # Modify port and baudrate as needed

        # Create subscriber for YOLO detection info
        self.point_sub = self.create_subscription(
            DetectInfo,
            '/cube_info',
            self.detection_info_callback,
            10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # pub
        self.pose_publisher = self.create_publisher(PoseStamped, 'object_pose', 10)

        self.cube_x = 0
        self.cube_y = 0
        self.cube_depth = 0
        self.color = ""
        self.non_detected = True

    def pixel_to_camera_coordinates(self, x_pixel, y_pixel, depth, focal_length, image_center):
        x_camera = (x_pixel - image_center[0]) * depth / focal_length
        y_camera = (y_pixel - image_center[1]) * depth / focal_length
        z_camera = depth

        return np.array([x_camera, y_camera, z_camera])

    def detection_info_callback(self, point_msg: DetectInfo):
        self.get_logger().info('Callback')
        self.cube_x = point_msg.center_x
        self.cube_y = point_msg.center_y
        self.cube_depth = point_msg.average_depth
        self.color = point_msg.color
        if self.cube_x != 0:
            self.get_logger().info('state change')
            self.non_detected = False

    def run(self):
        while rclpy.ok():
            # self.get_logger().info('rclpy ok')
            rclpy.spin_once(self)
            if not self.non_detected:
                # self.get_logger().info('start to publish pose')
                # Convert pixel coordinates to camera coordinates
                image_center = [640, 360]  # Assuming image center at (640, 360)
                focal_length = 1000  # Focal length in pixels (example value)
                camera_coords = self.pixel_to_camera_coordinates(self.cube_x, self.cube_y, self.cube_depth,
                                                                 focal_length, image_center)

                # Get the transform from camera_link to world
                try:
                    ##transform = self.tf_buffer.lookup_transform('base_link', self.camera_link, rclpy.time.Time())
                    transform = self.tf_buffer.lookup_transform('camera_link', self.camera_link, rclpy.time.Time())
                    camera_position = np.array([transform.transform.translation.x,
                                                transform.transform.translation.y,
                                                transform.transform.translation.z])
                    camera_orientation = transform.transform.rotation
                    # self.get_logger().info('finish camera_position')

                    # Convert quaternion to rotation matrix
                    R = self.quaternion_to_rotation_matrix(camera_orientation)

                    # Transform object position from camera to world coordinates
                    object_position_camera_frame = camera_coords
                    object_position_world_frame = np.dot(R, object_position_camera_frame) + camera_position

                    # Publish the object position in world coordinates
                    pose_msg = Pose()
                    pose_msg.position.x = object_position_world_frame[0]
                    pose_msg.position.y = object_position_world_frame[1]
                    pose_msg.position.z = object_position_world_frame[2]
                    self.get_logger().info(f'world x:{pose_msg.position.x} | y:{pose_msg.position.y} | z:{pose_msg.position.z} ')

                    self.publish_pose(pose_msg)  # Publish the transformed pose

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    print(f"Failed to lookup transform: {e}")

                self.non_detected = True  # Reset the detection flag after processing

            time.sleep(1)

    def quaternion_to_rotation_matrix(self, quaternion):
        # Convert quaternion to rotation matrix
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        R = txq.quat2mat(q)
        return R

    def publish_pose(self, pose_msg):
        # Publish the pose in world coordinates
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.pose = pose_msg
        pose_stamped_msg.header.frame_id = 'cube_frame'
        # self.get_logger().info('publish cube frame')
        self.pose_publisher.publish(pose_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_position = CameraPosition()
    try:
        camera_position.run()
    except KeyboardInterrupt:
        pass

    camera_position.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()