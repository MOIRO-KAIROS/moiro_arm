# defines a Face recogntion result

# ID for tracking
string name # for face recognition
float64 score

bool isdetect

# YOLO format's 2D bounding box surrounding the object in pixels
yolov8_msgs/BBoxYolo bbox
