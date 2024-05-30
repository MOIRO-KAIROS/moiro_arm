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
moiro_interfaces/BoundingBox2D bbox

# yolo format | 2D bounding box surrounding the object in pixels
moiro_interfaces/BBoxYolo bboxyolo

# 2D bounding box for face recognition
moiro_interfaces/FaceBox2D facebox

# segmentation mask of the detected object
# it is only the boundary of the segmented object
moiro_interfaces/Mask mask

# keypoints for human pose estimation
moiro_interfaces/KeyPoint2DArray keypoints