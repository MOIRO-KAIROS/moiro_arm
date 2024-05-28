# defines a YOLOv8 detection result

# class probability
int32 class_id
string class_name
float64 score

# ID for tracking
string id
# is bbox
bool istrack

# 2D bounding box surrounding the object in pixels
yolov8_msgs/BoundingBox2D bbox

# yolo format | 2D bounding box surrounding the object in pixels
yolov8_msgs/BBoxYolo bboxyolo

# 2D bounding box for face recognition
yolov8_msgs/FaceBox2D facebox

# segmentation mask of the detected object
# it is only the boundary of the segmented object
yolov8_msgs/Mask mask

# keypoints for human pose estimation
yolov8_msgs/KeyPoint2DArray keypoints