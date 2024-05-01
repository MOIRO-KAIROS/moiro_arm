#include <Eigen/Geometry>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <unistd.h>     // UNIX Standard Definitions
#include <fcntl.h>      // File Control Definitions
#include <termios.h>    // POSIX Terminal Control Definitions
#include <string>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <vector>
#include <cctype>
#include <cstdlib>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <sensor_msgs/msg/camera_info.hpp>
#include "yolov8_msgs/msg/detection_info.hpp"


#define PI 3.141592

using namespace std::chrono_literals; // For using milliseconds
using namespace std;

// pixel 좌표를 camera coordinate로 변환하는 함수
Eigen::Vector3d pixel_to_camera_coordinates(int x_pixel, int y_pixel, double depth, double focal_length, const Eigen::Vector2d& image_center) {
    double x_camera = (x_pixel - image_center.x()) * depth / focal_length;
    double y_camera = (y_pixel - image_center.y()) * depth / focal_length;
    double z_camera = depth;

    return Eigen::Vector3d(x_camera, y_camera, z_camera);
}

// 쿼터니언을 rpy로 변환해주는 함수
Eigen::Vector3d quaternion_to_rpy(const Eigen::Quaterniond& quaternion) {
    Eigen::Vector3d rpy = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
    return rpy;
}

// 쿼터니언에 해당하는 rotation matrix를 생성하는 함수
Eigen::Matrix3d quaternion_to_rotation_matrix(const Eigen::Quaterniond& quaternion) {
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    return rotation_matrix;
}

// rpy에 해당하는 rotation matrix를 생성하는 함수
Eigen::Matrix3d rpy_to_rotation_matrix(const Eigen::Vector3d& rpy) {
    Eigen::AngleAxisd rollAngle(rpy.x(), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(rpy.y(), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(rpy.z(), Eigen::Vector3d::UnitZ());

    Eigen::Quaternion<double> q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix3d rotation_matrix = q.matrix();
    return rotation_matrix;
}

// rotation matrix와 camera position, camera coordintate를 이용하여 real coordinate 위치로 변환하는 함수  
Eigen::Vector3d translate_to_world(const Eigen::Vector3d& camera_position, const Eigen::Quaterniond& camera_orientation_quaternion, const Eigen::Vector3d& object_position_camera_frame) {

    Eigen::Matrix3d R = quaternion_to_rotation_matrix(camera_orientation_quaternion);

    Eigen::Vector3d object_position_world_frame = R * object_position_camera_frame + camera_position;
    //Eigen::Vector3d object_position_world_frame = R * object_position_camera_frame;
    return object_position_world_frame;
}

/* ------------------------------------ Gripper Function -------------------------------------*/

// 입력된 속도에 대해 16진수 명령을 유도하는 함수
string VelToHex(int velocity) {
    stringstream ss;
    if (-127 <= velocity && velocity <= 127) {
        int adjusted_val = velocity;
        
        if (velocity < 0) {
            adjusted_val = 128 + abs(velocity); // 음수 값을 변환
        }
        ss << setw(2) << setfill('0') << hex << adjusted_val;
    }
    else {
        throw invalid_argument("Wrong Velocity");
    }
    return ss.str();
}

// 입력된 각도에 대해 16진수 명령을 유도하는 함수
string AngToHex(int angle) {
    double increment_per_degree = (0x00000C80 - 0x00000000) / 360.0;
    int hex_value = static_cast<int>(increment_per_degree * angle);
    stringstream ss;
    ss << setw(8) << setfill('0') << uppercase << hex << hex_value;
    return ss.str();
}

// 16진수 문자열을 바이트 배열로 변환하는 함수
vector<unsigned char> HexStringToBytes(const string& hex) {
    vector<unsigned char> bytes;

    for (size_t i = 0; i < hex.length(); i += 2) {
        string byteString = hex.substr(i, 2);
        unsigned char byte = static_cast<unsigned char>(strtol(byteString.c_str(), nullptr, 16));
        bytes.push_back(byte);
    }

    return bytes;
}

// 16진수 데이터의 check Sum을 계산하는 함수
unsigned char CheckSum(const vector<unsigned char>& data) {
    unsigned int sum = 0;
    for (size_t i = 0; i < data.size(); i++) {
        sum += data[i];
    }
    return static_cast<unsigned char>(sum % 256);
}

// 시리얼 포트로 데이터를 전송하는 함수
void TxSerial(int serial_port, const string& data_hex) {
    cout << data_hex << "\n";
    vector<unsigned char> data = HexStringToBytes(data_hex);
    unsigned char checksum = CheckSum(data);
    data.push_back(checksum); // 체크섬 바이트 추가
    write(serial_port, data.data(), data.size());
}

/*
class CameraInfoListener : public rclcpp::Node
{
public:
    CameraInfoListener() : Node("camera_info_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera/depth/camera_info", 10,
            std::bind(&CameraInfoListener::camera_info_callback, this, std::placeholders::_1));
    }
    // pixel 좌표를 camera coordinate로 변환하는 함수
    Eigen::Vector3d pixel_to_camera_coordinates(int x_pixel, int y_pixel, double depth) {
        double x_camera = (x_pixel - image_center_.x()) * depth / focal_length_x_;
        double y_camera = (y_pixel - image_center_.y()) * depth / focal_length_y_;
        double z_camera = depth;
        return Eigen::Vector3d(x_camera, y_camera, z_camera);
    }
private:
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        focal_length_x_ = msg->k[0]; // X 축 초점 거리
        focal_length_y_ = msg->k[4]; // Y 축 초점 거리
        image_center_ = Eigen::Vector2d(msg->k[2], msg->k[5]); // 이미지 중심 (cx, cy)
    }
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_;
    double focal_length_x_, focal_length_y_;
    Eigen::Vector2d image_center_; // 이미지 중심 (cx, cy)
};
*/

/* ------------------------------------ Main Function -------------------------------------*/

int main(int argc, char * argv[])
{

  const char* portname = "/dev/ttyUSB0";
  int serial_port = open(portname, O_RDWR | O_NOCTTY);

  if (serial_port < 0) {
      cerr << "Error " << errno << " opening " << portname << ": " << strerror(errno) << endl;
      return 1;
  }

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(serial_port, &tty) != 0) {
      cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << endl;
      return 1;
  }

  cfsetospeed(&tty, B38400);
  cfsetispeed(&tty, B38400);

  tty.c_cflag |= (CLOCAL | CREAD);   // Enable reading and ignore control lines
  tty.c_cflag &= ~CSIZE;             // Clear bit mask for data bits
  tty.c_cflag |= CS8;                // 8 data bits
  tty.c_cflag &= ~PARENB;            // No parity bit
  tty.c_cflag &= ~CSTOPB;            // 1 stop bit
  tty.c_cflag &= ~CRTSCTS;           // No hardware flow control

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;
  tty.c_lflag &= ~ECHOE;
  tty.c_lflag &= ~ECHONL;
  tty.c_lflag &= ~ISIG;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_oflag &= ~ONLCR;

  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    cerr << "Error " << errno << " from tcsetattr" << endl;
    return 1;
  }

  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("ur_camera_position");
  
  int img_center_x, img_center_y;
  double img_depth;
  double height;

  double box_center_x, box_center_y, box_center_z;
  bool non_detected = true;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("ur_camera_position", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);


  // ---------------------- Make plane to appropriate path planning --------------------------------------

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_arm.getPlanningFrame();
  collision_object.id = "ground";

  shape_msgs::msg::Plane plane_z;
  //shape_msgs::msg::Plane plane_x, plane_y, plane_z;
  //plane_x.coef = {1, 0, 0, -0.2}; // Equation of plane x = -0.1
  //plane_z.coef = {0, 1, 0, -0.2}; // Equation of plane y = -0.1
  plane_z.coef = {0, 0, 1, 0}; // Equation of plane z = 0

  geometry_msgs::msg::Pose ground_pose;
  ground_pose.orientation.w = 1.0;
  ground_pose.position.z = -0.01; // z position

  //collision_object.planes.push_back(plane_x);
  //collision_object.planes.push_back(plane_y);
  collision_object.planes.push_back(plane_z);
  collision_object.plane_poses.push_back(ground_pose);

  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  // ------------------------------- End of plane making ---------------------------------------------------


  // ------------------------------- Go to initial state ---------------------------------------------------

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();



  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm1;
  bool success = (move_group_arm.plan(my_plan_arm1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Pregrasp - Move to Picking Point
  RCLCPP_INFO(LOGGER, "Pregrasp Position");
  current_state_arm = move_group_arm.getCurrentState();
  geometry_msgs::msg::Pose start_pose = move_group_arm.getCurrentPose().pose;

  // Make waypoint vector
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose); // Push start position

  // Set Target Position
  tf2::Quaternion orientation;
  orientation.setRPY(0, -PI, 0.75*PI); // Set Rotation Roll Pitch Yaw
  geometry_msgs::msg::Quaternion ros_orientation;
  ros_orientation = tf2::toMsg(orientation);  // tf2 Quaternion -> ROS msg
  geometry_msgs::msg::Pose target_pose;   // Assign Quaternion
  target_pose.orientation = ros_orientation; 

  target_pose.position.x = -0.230; // X 좌표
  target_pose.position.y = 0.230;  // Y 좌표
  target_pose.position.z = 0.350;  // Z 좌표
  waypoints.push_back(target_pose); // Push target position

  // Cartesian Path planning
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.01;  // 엔드 이펙터가 한 번에 이동할 최대 거리 (미터)
  const double jump_threshold = 0.0;  // 조인트가 허용하는 최대 거리, 0은 비활성화
  double fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  if (fraction > 0.0) {
    // 계획된 경로를 실행합니다.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_ = trajectory;
    move_group_arm.execute(my_plan);
    
  }
  rclcpp::sleep_for(std::chrono::seconds(5));


  static const std::string CAMERA_LINK = "camera_link";

  //yolo node subscription
  auto detection_info_subscriber = move_group_node->create_subscription<yolov8_msgs::msg::DetectionInfo>(
    "/yolo/detection_info", 10, [&LOGGER, &img_center_x, &img_center_y, &img_depth, &non_detected](const yolov8_msgs::msg::DetectionInfo::SharedPtr msg) {
        
        img_center_x = msg->center_x;
        img_center_y = msg->center_y;
        img_depth = msg->average_depth;

        if (img_center_x != 0) non_detected = false;

    });
    
  while (rclcpp::ok()) {
  // Get position and quaternion of camera
    while (rclcpp::ok()) {

      if (!non_detected){
        geometry_msgs::msg::PoseStamped current_pose = move_group_arm.getCurrentPose(CAMERA_LINK);
        RCLCPP_INFO(move_group_node->get_logger(), "Camera Position: (%.6f, %.6f, %.6f)",
                    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
        RCLCPP_INFO(move_group_node->get_logger(), "Camera Quaternion : (%.6f, %.6f, %.6f, %.6f)",
                    current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                    current_pose.pose.orientation.z, current_pose.pose.orientation.w);
        
        // From Camera Intrinsic Matrix -- Need modifying
        Eigen::Vector2d image_center(321.72564697, 241.86355591); // 예제 값
        double focal_length = 384.64208984; // 예제 값
        Eigen::Vector3d camera_position(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z); // 예제 값
        Eigen::Quaterniond camera_orientation(current_pose.pose.orientation.w, -current_pose.pose.orientation.x, -current_pose.pose.orientation.y,
                    -current_pose.pose.orientation.z); // 예제 값

        Eigen::Vector3d object_position_camera = pixel_to_camera_coordinates(img_center_x, img_center_y, img_depth, focal_length, image_center);
        Eigen::Vector3d object_position_world = translate_to_world(camera_position, camera_orientation, object_position_camera);
        std::cout << "\n\nObject Camera coordinate: " << object_position_camera <<  std::endl;

        height = img_depth;

        // Carry - Move to Placing Point

        RCLCPP_INFO(LOGGER, "Move to Placing Point");

        std::vector<geometry_msgs::msg::Pose> carry_waypoints;

        // Camera 보정 ( D435 카메라 중심과 Color 카메라의 위치가 일치하지 않음 )
        if (object_position_camera[0] >= 0.1) object_position_camera[0] *= 0.6;
        else if (object_position_camera[0] >= 0.070) object_position_camera[0] -= 0.070;
        else if (object_position_camera[0] >= 0.035) object_position_camera[0] -= 0.035;
        else if (object_position_camera[0] > 0) object_position_camera[0] = 0.000;
        else if (object_position_camera[0] <= -0.1) object_position_camera[0] *= 0.8;
        //else if (object_position_camera[0] < 0) object_position_camera[0] = 0.000;

        if (abs(object_position_camera[1]) >= 0.1) object_position_camera[1] *= 0.8;

        target_pose.position.x += object_position_camera[0];
        target_pose.position.y -= object_position_camera[1] - 0.06;
        target_pose.position.z = 0.300;
        //std::cout << "\n\nObject Position in World Frame: " << target_pose.position.x << " | " << target_pose.position.y << " | " << object_position_world.transpose()[2] << std::endl;

        // 실제 coordinate로 이동
        if (true){
          carry_waypoints.push_back(target_pose);

          moveit_msgs::msg::RobotTrajectory trajectory_carry;


          fraction = move_group_arm.computeCartesianPath(
          carry_waypoints, eef_step, jump_threshold, trajectory_carry);

          move_group_arm.execute(trajectory_carry);

          /*
          //rclcpp::spin_some(move_group_node);
          rclcpp::sleep_for(std::chrono::seconds(4));
          target_pose.position.x = -0.230; // X 좌표
          target_pose.position.y = 0.230;  // Y 좌표
          target_pose.position.z = 0.350;  // Z 좌표
          waypoints.push_back(target_pose); // Push target position
          fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
          move_group_arm.execute(trajectory);
          */
          rclcpp::sleep_for(std::chrono::seconds(4));
          break;
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
        non_detected = true;
      }
    }

    // Approach - Get close to object along the Cartesian Path
    RCLCPP_INFO(LOGGER, "Approach to object!");

    std::vector<geometry_msgs::msg::Pose> approach_waypoints1;
    target_pose.position.z -= 0.09;
    approach_waypoints1.push_back(target_pose);

    target_pose.position.z -= 0.09;
    approach_waypoints1.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_approach1;

    fraction = move_group_arm.computeCartesianPath(
        approach_waypoints1, eef_step, jump_threshold, trajectory_approach1);

    move_group_arm.execute(trajectory_approach1);

    rclcpp::sleep_for(std::chrono::seconds(3));

    // Grasp - Add Grasping action here
    string data_hex = "e0fd" + VelToHex(-15) + AngToHex(1440);
    TxSerial(serial_port, data_hex);
    sleep(2);

    // Retreat - Lift up the object along the Cartesian Path
    RCLCPP_INFO(LOGGER, "Retreat from object!");

    std::vector<geometry_msgs::msg::Pose> retreat_waypoints1;
    target_pose.position.z += 0.09;
    retreat_waypoints1.push_back(target_pose);

    target_pose.position.z += 0.09;
    retreat_waypoints1.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat1;

    fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints1, eef_step, jump_threshold, trajectory_retreat1);

    move_group_arm.execute(trajectory_retreat1);
    
    rclcpp::sleep_for(std::chrono::seconds(3));

    // Carry - Move to Placing Point

    // Make waypoint vector

    RCLCPP_INFO(LOGGER, "Move to Placing Point");

    std::vector<geometry_msgs::msg::Pose> end_waypoints;

    // Set Target Position
    target_pose.position.x = -0.480; // X 좌표
    target_pose.position.y = 0.100;  // Y 좌표
    target_pose.position.z = 0.250;  // Z 좌표
    end_waypoints.push_back(target_pose); // Push target position

    // Cartesian Path planning
    moveit_msgs::msg::RobotTrajectory trajectory_end;
    fraction = move_group_arm.computeCartesianPath(end_waypoints, eef_step, jump_threshold, trajectory_end);

    move_group_arm.execute(trajectory_end);

    rclcpp::sleep_for(std::chrono::seconds(5));


      // Approach - Get close to object along the Cartesian Path
    RCLCPP_INFO(LOGGER, "Approach to object!");

    std::vector<geometry_msgs::msg::Pose> approach_waypoints2;
    target_pose.position.z -= 0.05;
    approach_waypoints2.push_back(target_pose);

    target_pose.position.z -= 0.05;
    approach_waypoints2.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_approach2;

    fraction = move_group_arm.computeCartesianPath(
        approach_waypoints2, eef_step, jump_threshold, trajectory_approach2);

    move_group_arm.execute(trajectory_approach2);

    rclcpp::sleep_for(std::chrono::seconds(3));

    // Dropping - Add Dropping action here
    data_hex = "e0fd" + VelToHex(15) + AngToHex(1440);
    TxSerial(serial_port, data_hex);
    sleep(2);

    // Retreat - Lift up the object along the Cartesian Path
    RCLCPP_INFO(LOGGER, "Retreat from object!");

    std::vector<geometry_msgs::msg::Pose> retreat_waypoints2;
    target_pose.position.z += 0.05;
    retreat_waypoints2.push_back(target_pose);

    target_pose.position.z += 0.05;
    retreat_waypoints2.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_retreat2;

    fraction = move_group_arm.computeCartesianPath(
        retreat_waypoints2, eef_step, jump_threshold, trajectory_retreat2);

    move_group_arm.execute(trajectory_retreat2);
    
    rclcpp::sleep_for(std::chrono::seconds(3));


    // home 위치로 복귀
    target_pose.position.x = -0.230; // X 좌표
    target_pose.position.y = 0.230;  // Y 좌표
    target_pose.position.z = 0.350;  // Z 좌표
    waypoints.push_back(target_pose); // Push target position

    fraction = move_group_arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    move_group_arm.execute(trajectory);
    rclcpp::sleep_for(std::chrono::seconds(4));
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
  
}