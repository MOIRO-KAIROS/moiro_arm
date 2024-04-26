import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster

class ImageConverter(Node):
    def __init__(self):
        super().__init__("detect_color_box")
        self.br = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.HSV = {
            "blue": [[100, 43, 46], [124, 255, 255]],
            "red": [[160, 100, 100], [180, 255, 255]],
            "yellow": [[20, 20, 100], [32, 255, 245]],
            "purple": [[120, 50, 50], [160, 255, 255]],
        }
        self.screen_coords = []
        self.real_world_coords = []
        self.coords_set = False
        self.transformation_matrix = None
        # subscriber, listen wether has img come in.
        self.image_sub = self.create_subscription(
            msg_type=Image,
            topic="/image_raw",
            callback=self.color_detect,
            qos_profile=1
        )
        self.pose_data_server = self.create_service(
            GetPoseData,
            "get_pose_data",
            self.get_pose_data_callback
        )
        
    def color_detect(self, data):
        """Callback function.

        Process image with OpenCV, detect Color Box to get the pose. Then acccording the
        pose to transforming.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        size = cv_image.shape
        
        # coords 설정
        if not self.coords_set:
            self.screen_coords = [
                [0, 0],
                [size[1], 0],
                [size[1], size[0]],
                [0, size[0]],
            ]
            self.real_world_coords = [
                [0, 0],
                [50, 0],
                [50, 50],
                [0, 50],
            ]
            self.transformation_matrix = cv2.getPerspectiveTransform(
                np.float32(self.screen_coords), np.float32(self.real_world_coords)
            )

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # detect color box.
        x = y = 0
        for mycolor, item in self.HSV.items():
            lower = np.array(item[0])
            upper = np.array(item[1])
            mask = cv2.inRange(hsv, lower, upper)
            erosion = cv2.erode(mask, None, iterations=2)
            dilation = cv2.dilate(erosion, None, iterations=2)
            target = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            ret, binary = cv2.threshold(dilation, 127, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                boxes = []
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 1000:
                        boxes.append(cv2.boundingRect(cnt))
                if boxes:
                    for box in boxes:
                        x, y, w, h = box
                        x_center = x + w / 2
                        y_center = y + h / 2
                        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 0), 2)
                        cv2.circle(cv_image, (int(x_center), int(y_center)), 2, (0, 0, 255), -1)
                        real_x, real_y = cv2.perspectiveTransform(
                            np.array([[[x_center, y_center]]], dtype=np.float32),
                            self.transformation_matrix
                        )[0][0]
                        cv2.putText(cv_image, f"{mycolor} : {int(real_x)}, {int(real_y)}", (int(x_center), int(y_center)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 5, 255), 2)

        cv2.imshow("color box", cv_image)
        cv2.waitKey(3)
        try:
            pass
        except Exception as e:
            print(e)
    
def main(args=None):
    try:
        rclpy.init(args=args)
        print("Starting detect_color_box")
        i = ImageConverter()
        rclpy.spin(i)
    except KeyboardInterrupt:
        print("detect_color_box stopped cleanly")
        cv2.destroyAllWindows()

    i.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
