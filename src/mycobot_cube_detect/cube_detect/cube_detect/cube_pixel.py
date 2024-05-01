import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
import message_filters
from sensor_msgs.msg import CameraInfo,Image
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from cube_msg.msg import DetectInfo
import tf2_ros 

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.get_logger().info('========================') 
        self.get_logger().info('Start Cube Pixel Detect!')
        self.get_logger().info('========================') 

        calibrationParams = cv2.FileStorage(
            "calibrationFileName.xml", cv2.FILE_STORAGE_READ
        )
        self.dist_coeffs = calibrationParams.getNode("distCoeffs").mat()
        self.camera_matrix = None

        # params
        self.declare_parameter("input_image_topic", "/camera/camera/color/image_raw")
        input_image_topic = self.get_parameter("input_image_topic").get_parameter_value().string_value

        self.declare_parameter("depth_image_topic", "/camera/camera/depth/image_rect_raw")
        depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value

        self.declare_parameter("image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        image_qos_profile = QoSProfile(
            reliability=self.get_parameter(
                "image_reliability").get_parameter_value().integer_value,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # subs    
        self.image_sub = message_filters.Subscriber(
            self, Image, input_image_topic,  # 웹캠 이미지 토픽을 구독
            # '/camera2/image_raw',
            qos_profile=image_qos_profile)
        self.depth_sub = message_filters.Subscriber(
            self, Image, depth_image_topic,
            qos_profile=image_qos_profile)
        
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.image_sub, self.depth_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.image_callback)
        
        #pubs
        self.cube_info_publisher = self.create_publisher(DetectInfo,'cube_info',10)
        self.img_publisher = self.create_publisher(Image,'cube_img',10)

        self.cv_bridge = CvBridge()

    def image_callback(
        self,
        color_img_msg: Image,
        depth_img_msg: Image,
    ) -> None:
    
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환
            frame = self.cv_bridge.imgmsg_to_cv2(color_img_msg, desired_encoding='bgr8')
            depth_frame = self.cv_bridge.imgmsg_to_cv2(depth_img_msg)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        size = frame.shape
        focal_length = size[1]
        center = [size[1] / 2, size[0] / 2]
        if self.camera_matrix is None:
            # calc the camera matrix, if don't have.
            self.camera_matrix = np.array(
                [
                    [focal_length, 0, center[0]],
                    [0, focal_length, center[1]],
                    [0, 0, 1],
                ],
                dtype=np.float32,
            )

        # 큐브 감지 및 중심점 계산
        frame = self.detect_cubes(frame,depth_frame,'red')
        frame = self.detect_cubes(frame,depth_frame,'blue')
        frame = self.detect_cubes(frame,depth_frame,'yellow')
        frame = self.detect_cubes(frame,depth_frame,'purple')
        color_img_msg = self.cv_bridge.cv2_to_imgmsg(frame,encoding='bgr8')
        self.img_publisher.publish(color_img_msg)
    
    def detect_cube(self,frame,cube_corners,depth_frame,idx,color):
        xyz = [(cube_corners[0][0][0]+cube_corners[1][0][0]+cube_corners[2][0][0]+cube_corners[3][0][0])/4,
               (cube_corners[0][0][1]+cube_corners[1][0][1]+cube_corners[2][0][1]+cube_corners[3][0][1])/4]
        # print(cube_corners)
        # print(xyz)
        xyz.append(float(depth_frame[int(xyz[1])][int(xyz[0])]))

        cv2.putText(frame, f"{color} {idx}", (cube_corners[0][0][0] - 10 ,cube_corners[0][0][1] + 10),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),1)
        cube_corners = (np.array([[[cube_corners[0][0][0],cube_corners[0][0][1]],
                                    [cube_corners[1][0][0],cube_corners[1][0][1]],
                                    [cube_corners[2][0][0],cube_corners[2][0][1]],
                                    [cube_corners[3][0][0],cube_corners[3][0][1]]]],dtype=np.float32),)

        # 근사화된 외곽선을 사각형으로 그리기
        ret = cv2.aruco.estimatePoseSingleMarkers(
            cube_corners, 0.03, self.camera_matrix, self.dist_coeffs
        )
        (rvec, tvec) = (ret[0], ret[1])
        (rvec - tvec).any()

        for i in range(rvec.shape[0]):
            cv2.aruco.drawDetectedMarkers(frame, cube_corners)
            cv2.aruco.drawAxis(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec[i, :, :],
                tvec[i, :, :],
                0.03,
            )
        
        self.cube_info_publisher.publish(DetectInfo(center_x=xyz[0], center_y=xyz[1], average_depth=xyz[2],color=color))
        idx+=1
        self.get_logger().info(f"color : {color}, x: {(xyz[0])}, y: {(xyz[1])},depth: {(xyz[2])}")
        return idx,frame

    def detect_cubes(self, frame, depth_frame,color):
        # 색상 범위 지정
        if color == 'blue':
            low = np.array([78, 43, 46])
            up = np.array([110, 255, 255])
        elif color == 'red':
            low = np.array([0, 43, 46])
            up = np.array([20, 255, 255])
        elif color == 'yellow':
            low = np.array([11, 85, 70])
            up = np.array([59, 255, 245])
        else:  # Purple
            low = np.array([100, 50, 50])
            up = np.array([140, 255, 255])

        # 프레임을 HSV 형식으로 변환
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 색상에 따라 마스크 생성
        mask = cv2.inRange(hsv, low, up)
        
        # 마스크를 사용하여 큐브 영역 추출
        cube_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if cube_contours:
            idx = 0
            for cube_contour in cube_contours:
                area = cv2.contourArea(cube_contour)
                epsilon = 0.1 * cv2.arcLength(cube_contour, True)
                cube_corners = cv2.approxPolyDP(cube_contour, epsilon, True)
                if area >= 1000 and len(cube_corners) == 4:
                    idx,frame = self.detect_cube(frame,cube_corners,depth_frame,idx,color)
        return frame
        
def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
